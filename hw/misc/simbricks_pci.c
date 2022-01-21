/*
 * Co-simulation PCI device
 *
 * Copyright (c) 2020 Max Planck Institute for Software Systems
 * Copyright (c) 2020 National University of Singapore
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/qdev-properties.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "chardev/char-fe.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "sysemu/cpus.h"
#include "hw/core/cpu.h"

#include <simbricks/proto/base.h>
#include <simbricks/proto/pcie.h>

//#define DEBUG_PRINTS

#define SIMBRICKS_CLOCK QEMU_CLOCK_VIRTUAL

#define TYPE_PCI_SIMBRICKS_DEVICE "simbricks-pci"
#define SIMBRICKS_PCI(obj) \
    OBJECT_CHECK(SimbricksPciState, obj, TYPE_PCI_SIMBRICKS_DEVICE)

typedef struct SimbricksPciBarInfo {
    struct SimbricksPciState *simbricks;
    uint8_t index;
    bool is_io;
    bool is_dummy;
} SimbricksPciBarInfo;

typedef struct SimbricksPciRequest {
    CPUState *cpu;          /* CPU associated with this request */
    QemuCond cond;
    uint64_t addr;
    uint8_t bar;
    uint64_t value;         /* value read/to be written */
    unsigned size;
    bool processing;
    bool requested;
} SimbricksPciRequest;

#define SYNC_MODES   0
#define SYNC_BARRIER 1

typedef struct SimbricksPciState {
    PCIDevice pdev;

    QemuThread thread;
    volatile bool stopping;

    CharBackend sim_chr;

    MemoryRegion mmio_bars[6];
    SimbricksPciBarInfo bar_info[6];

    struct SimbricksProtoPcieDevIntro dev_intro;

    void *shm_base;
    size_t shm_len;

    /* dev to host queue (managed by worker thread) */
    uint8_t *d2h_base;
    size_t d2h_nentries;
    size_t d2h_elen;
    size_t d2h_pos;

    /* host to dev queue (managed by worker thread) */
    uint8_t *h2d_base;
    size_t h2d_nentries;
    size_t h2d_elen;
    size_t h2d_pos;

    /* communication bewteen main io thread and worker thread
     * (protected by thr_mutex). */
    size_t reqs_len;
    SimbricksPciRequest *reqs;

    /* timers for synchronization etc. */
    bool sync;
    int sync_mode;
    uint64_t pci_latency;
    uint64_t sync_period;
    int64_t ts_base;
    QEMUTimer *timer_dummy;
    QEMUTimer *timer_sync;
    int64_t sync_ts;
    bool sync_ts_bumped;
    QEMUTimer *timer_poll;
    int64_t poll_ts;
} SimbricksPciState;

void QEMU_NORETURN cpu_loop_exit(CPUState *cpu);

static void panic(const char *msg, ...) __attribute__((noreturn));

static void panic(const char *msg, ...)
{
    va_list ap;

    va_start(ap, msg);
    error_vreport(msg, ap);
    va_end(ap);

    abort();
}

static inline uint64_t ts_to_proto(SimbricksPciState *simbricks,
                                   int64_t qemu_ts)
{
    return (qemu_ts - simbricks->ts_base) * 1000;
}

static inline int64_t ts_from_proto(SimbricksPciState *simbricks,
                                    uint64_t proto_ts)
{
    return (proto_ts / 1000) + simbricks->ts_base;
}

/******************************************************************************/
/* Worker thread */

static volatile union SimbricksProtoPcieH2D *simbricks_comm_h2d_alloc(
        SimbricksPciState *simbricks, int64_t ts)
{
    uint8_t *pos;
    volatile union SimbricksProtoPcieH2D *msg;

    pos = simbricks->h2d_base + (simbricks->h2d_elen * simbricks->h2d_pos);
    msg = (volatile union SimbricksProtoPcieH2D *) pos;

    while ((msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_H2D_OWN_MASK) !=
            SIMBRICKS_PROTO_PCIE_H2D_OWN_HOST)
    {
#ifdef DEBUG_PRINTS
        warn_report("simbricks_comm_h2d_alloc: ran into non-owned entry in h2d"
                    " queue");
#endif
    }

    /* tag message with timestamp */
    msg->dummy.timestamp = ts_to_proto(simbricks, ts + simbricks->pci_latency);

    /* re-arm sync timer */
    if (simbricks->sync_mode == SIMBRICKS_PROTO_SYNC_SIMBRICKS) {
        simbricks->sync_ts = ts + simbricks->sync_period;
        simbricks->sync_ts_bumped = true;
    }

#ifdef DEBUG_PRINTS
    warn_report("simbricks_comm_h2d_alloc: ts=%lu msg_ts=%lu next=%ld", ts,
            msg->dummy.timestamp, ts + simbricks->sync_period);
#endif

    /* advance position to next entry */
    simbricks->h2d_pos = (simbricks->h2d_pos + 1) % simbricks->h2d_nentries;

    return msg;
}

static void simbricks_comm_d2h_dma_read(
        SimbricksPciState *simbricks,
        int64_t ts,
        volatile struct SimbricksProtoPcieD2HRead *read)
{
    volatile union SimbricksProtoPcieH2D *h2d;
    volatile struct SimbricksProtoPcieH2DReadcomp *rc;

    /* allocate completion */
    h2d = simbricks_comm_h2d_alloc(simbricks, ts);
    rc = &h2d->readcomp;

    assert(read->len <= simbricks->h2d_elen - sizeof (*rc));

    /* perform dma read */
    pci_dma_read(&simbricks->pdev, read->offset, (void *) rc->data, read->len);

    /* return completion */
    rc->req_id = read->req_id;
    smp_wmb();
    rc->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_READCOMP |
        SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;
}

static void simbricks_comm_d2h_dma_write(
        SimbricksPciState *simbricks,
        int64_t ts,
        volatile struct SimbricksProtoPcieD2HWrite *write)
{
    volatile union SimbricksProtoPcieH2D *h2d;
    volatile struct SimbricksProtoPcieH2DWritecomp *wc;

    /* allocate completion */
    h2d = simbricks_comm_h2d_alloc(simbricks, ts);
    wc = &h2d->writecomp;

    /* perform dma write */
    pci_dma_write(&simbricks->pdev, write->offset, (void *) write->data,
            write->len);

    /* return completion */
    wc->req_id = write->req_id;
    smp_wmb();
    wc->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITECOMP |
        SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;
}

static void simbricks_comm_d2h_interrupt(SimbricksPciState *simbricks,
        volatile struct SimbricksProtoPcieD2HInterrupt *interrupt)
{
    if (interrupt->inttype == SIMBRICKS_PROTO_PCIE_INT_MSI) {
        if (interrupt->vector >= 32) {
            warn_report("simbricks_comm_d2h_interrupt: invalid MSI vector (%u)",
                    interrupt->vector);
            return;
        }
        msi_notify(&simbricks->pdev, interrupt->vector);
    } else if (interrupt->inttype == SIMBRICKS_PROTO_PCIE_INT_MSIX) {
        if (interrupt->vector >= simbricks->dev_intro.pci_msix_nvecs) {
            warn_report("simbricks_comm_d2h_interrupt: invalid MSI-X vector "
                        "(%u)", interrupt->vector);
            return;
        }
        msix_notify(&simbricks->pdev, interrupt->vector);
    } else if (interrupt->inttype == SIMBRICKS_PROTO_PCIE_INT_LEGACY_HI) {
        pci_irq_assert(&simbricks->pdev);
    } else if (interrupt->inttype == SIMBRICKS_PROTO_PCIE_INT_LEGACY_LO) {
        pci_irq_deassert(&simbricks->pdev);
    } else {
        warn_report("simbricks_comm_d2h_interrupt: not yet implented int type "
                    "(%u) TODO", interrupt->inttype);
    }
}

static void simbricks_comm_d2h_rcomp(SimbricksPciState *simbricks,
                                     uint64_t cur_ts,
                                     uint64_t req_id,
                                     const void *data)
{
    SimbricksPciRequest *req = simbricks->reqs + req_id;
    CPUState *cpu;

    assert(req_id <= simbricks->reqs_len);

    if (!req->processing) {
        panic("simbricks_comm_d2h_rcomp: no request currently processing");
    }

    /* copy read value from message */
    req->value = 0;
    memcpy(&req->value, data, req->size);

    req->processing = false;

    if (simbricks->sync) {
        cpu = req->cpu;

#ifdef DEBUG_PRINTS
        warn_report("simbricks_comm_d2h_rcomp: kicking cpu %lu ts=%lu",
                req_id, cur_ts);
#endif

        cpu->stopped = 0;
        //qemu_cpu_kick(cpu);
    } else {
        qemu_cond_broadcast(&req->cond);
    }
}

/* process and complete message */
static void simbricks_comm_d2h_process(
        SimbricksPciState *simbricks,
        int64_t ts,
        volatile union SimbricksProtoPcieD2H *msg)
{
    uint8_t type;

    type = msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_D2H_MSG_MASK;
#ifdef DEBUG_PRINTS
    warn_report("simbricks_comm_d2h_process: ts=%ld type=%u", ts, type);
#endif

    switch (type) {
        case SIMBRICKS_PROTO_PCIE_D2H_MSG_SYNC:
            /* nop */
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_READ:
            simbricks_comm_d2h_dma_read(simbricks, ts, &msg->read);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITE:
            simbricks_comm_d2h_dma_write(simbricks, ts, &msg->write);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_INTERRUPT:
            simbricks_comm_d2h_interrupt(simbricks, &msg->interrupt);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP:
            simbricks_comm_d2h_rcomp(simbricks, ts, msg->readcomp.req_id,
                    (void *) msg->readcomp.data);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP:
            /* we treat writes as posted, so nothing we need to do here */
            break;

        default:
            panic("simbricks_comm_poll_d2h: unhandled type");
    }

    smp_wmb();

    /* mark message as done */
    msg->dummy.own_type =
        (msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_D2H_MSG_MASK) |
        SIMBRICKS_PROTO_PCIE_D2H_OWN_DEV;
}

/* peek at head of d2h queue */
static int simbricks_comm_d2h_peek(
        SimbricksPciState *simbricks,
        int64_t ts,
        int64_t *next_ts,
        volatile union SimbricksProtoPcieD2H **pmsg)
{
    uint8_t *pos;
    int64_t msg_ts;
    volatile union SimbricksProtoPcieD2H *msg;

    pos = simbricks->d2h_base + (simbricks->d2h_elen * simbricks->d2h_pos);
    msg = (volatile union SimbricksProtoPcieD2H *) pos;

    /* check if this message is ready for us */
    if ((msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_D2H_OWN_MASK) !=
        SIMBRICKS_PROTO_PCIE_D2H_OWN_HOST)
    {
        return -1;
    }

    smp_rmb();

    msg_ts = ts_from_proto(simbricks, msg->dummy.timestamp);

    *pmsg = msg;
    *next_ts = msg_ts;
    if (simbricks->sync && msg_ts > ts) {
        /* still in the future */
        return 1;
    }

    return 0;
}

/* move on to next d2h queue position */
static void simbricks_comm_d2h_next(SimbricksPciState *simbricks)
{
    /* advance position to next entry */
    simbricks->d2h_pos = (simbricks->d2h_pos + 1) % simbricks->d2h_nentries;
}

static void simbricks_trigger_sync(SimbricksPciState *simbricks,
                                   int64_t cur_ts)
{
    volatile union SimbricksProtoPcieH2D *msg;
    volatile struct SimbricksProtoPcieH2DSync *sy;

    msg = simbricks_comm_h2d_alloc(simbricks, cur_ts);
    sy = &msg->sync;

    smp_wmb(); /* barrier to make sure earlier fields are written before
                  handing over ownership */

    sy->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_SYNC |
                   SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;

    /* simbricks_comm_h2d_alloc has already re-scheduled the timer */
}

static void simbricks_timer_dummy(void *data)
{
}

static void simbricks_timer_poll(void *data)
{
    SimbricksPciState *simbricks = data;
    volatile union SimbricksProtoPcieD2H *msg;
    volatile union SimbricksProtoPcieD2H *next_msg;
    int64_t cur_ts, next_ts;
    int ret;

    cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
#ifdef DEBUG_PRINTS
    if (cur_ts > simbricks->poll_ts + 1 || cur_ts < simbricks->poll_ts)
        warn_report("simbricks_timer_poll: expected_ts=%lu cur_ts=%lu",
                    simbricks->poll_ts, cur_ts);
#endif

    if (cur_ts > simbricks->sync_ts + 1) {
#ifdef DEBUG_PRINTS
        warn_report("simbricks_timer_poll: triggering sync: ts=%ld sync_ts=%ld",
                    cur_ts, timer_expire_time_ns(simbricks->timer_sync));
#endif
        simbricks_trigger_sync(simbricks, cur_ts);
    }

#ifdef DEBUG_PRINTS
    warn_report("simbricks_timer_poll: ts=%ld sync_ts=%ld", cur_ts,
                timer_expire_time_ns(simbricks->timer_sync));
#endif

    /* poll until we have a message */
    do {
        ret = simbricks_comm_d2h_peek(simbricks, cur_ts, &next_ts, &msg);
    } while (ret < 0);

    if (ret == 0) {
        /* message is ready to be processed */
        simbricks_comm_d2h_next(simbricks);

        /* now poll until we have next message so we know the timestamp */
        while (simbricks_comm_d2h_peek(simbricks, cur_ts, &next_ts, &next_msg)
               < 0);
    } else {
        next_msg = msg;
    }

    /* set timer for next message */
    /* we need to do this before actually processing the message, in order to
     * have a timer set to prevent the clock from running away from us. We set a
     * dummy timer with the current ts to prevent the clock from jumping */
    timer_mod_ns(simbricks->timer_dummy, cur_ts);
    timer_mod_ns(simbricks->timer_poll, next_ts);
    if (simbricks->sync_ts_bumped) {
        timer_mod_ns(simbricks->timer_sync, simbricks->sync_ts);
        simbricks->sync_ts_bumped = false;
    }
    simbricks->poll_ts = next_ts;

    /* run */
    if (ret == 0) {
        simbricks_comm_d2h_process(simbricks, cur_ts, msg);
    }

#ifdef DEBUG_PRINTS
    int64_t now_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    if (cur_ts != now_ts)
        warn_report("simbricks_timer_poll: time advanced from %lu to %lu",
                    cur_ts, now_ts);

    warn_report("simbricks_timer_poll: done, next=%ld", next_ts);
#endif
}

static void simbricks_timer_sync(void *data)
{
    SimbricksPciState *simbricks = data;
    int64_t cur_ts;

    cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);

#ifdef DEBUG_PRINTS
    if (cur_ts > simbricks->sync_ts + 1)
        warn_report("simbricks_timer_sync: expected_ts=%lu cur_ts=%lu",
                    simbricks->sync_ts, cur_ts);

    warn_report("simbricks_timer_sync: ts=%ld", cur_ts);
#endif

    simbricks_trigger_sync(simbricks, cur_ts);

#ifdef DEBUG_PRINTS
    int64_t now_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    if (cur_ts != now_ts)
        warn_report("simbricks_timer_poll: time advanced from %lu to %lu",
                    cur_ts, now_ts);
#endif

    if (simbricks->sync_mode == SIMBRICKS_PROTO_SYNC_BARRIER) {
        simbricks->sync_ts = cur_ts + simbricks->sync_period;
        simbricks->sync_ts_bumped = true;
    }
    assert(simbricks->sync_ts_bumped);
    timer_mod_ns(simbricks->timer_sync, simbricks->sync_ts);
    simbricks->sync_ts_bumped = false;
}

static void *simbricks_poll_thread(void *opaque)
{
    SimbricksPciState *simbricks = opaque;
    volatile union SimbricksProtoPcieD2H *msg;
    int64_t next_ts;
    int ret;

    assert(!simbricks->sync);

    while (!simbricks->stopping) {
        ret = simbricks_comm_d2h_peek(simbricks, 0, &next_ts, &msg);
        if (ret)
            continue;

        simbricks_comm_d2h_next(simbricks);

        /* actually process the operation. this needs to be done with the I/O
         * lock held. */
        qemu_mutex_lock_iothread();
        simbricks_comm_d2h_process(simbricks, 0, msg);
        qemu_mutex_unlock_iothread();
    }

    return NULL;
}

/******************************************************************************/
/* MMIO interface */

/* submit a bar read or write to the worker thread and wait for it to
 * complete */
static void simbricks_mmio_rw(SimbricksPciState *simbricks,
                              uint8_t bar,
                              hwaddr addr,
                              unsigned size,
                              uint64_t *val,
                              bool is_write)
{
    CPUState *cpu = current_cpu;
    SimbricksPciRequest *req;
    volatile union SimbricksProtoPcieH2D *msg;
    volatile struct SimbricksProtoPcieH2DRead *read;
    volatile struct SimbricksProtoPcieH2DWrite *write;
    int64_t cur_ts;

    assert(simbricks->reqs_len > cpu->cpu_index);
    req = simbricks->reqs + cpu->cpu_index;
    assert(req->cpu == cpu);

    cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);

    if (req->requested) {
        /* a request from this CPU has been started */
        /* note that address might not match if an interrupt occurs, which in
         * turn triggers another read. */

        if (req->processing) {
            /* request in progress, we have to wait */
            cpu->stopped = 1;
            cpu_loop_exit(cpu);
        } else if (req->addr == addr && req->bar == bar && req->size == size) {
            /* request finished */
#ifdef DEBUG_PRINTS
            warn_report("simbricks_mmio_rw: done (%lu) a=%lx s=%x val=%lx",
                        cur_ts, addr, size, req->value);
#endif
            *val = req->value;
            req->requested = false;
            return;
        } else {
            /* request is done processing, but for a different address */
            req->requested = false;
        }
    }

    assert(!req->processing);


    /* allocate host-to-device queue entry */
    msg = simbricks_comm_h2d_alloc(simbricks, cur_ts);

    /* prepare operation */
    if (is_write) {
        write = &msg->write;

        write->req_id = cpu->cpu_index;
        write->offset = addr;
        write->len = size;
        write->bar = bar;

        assert(size <= simbricks->h2d_elen - sizeof (*write));
        /* FIXME: this probably only works for LE */
        memcpy((void *) write->data, val, size);

        smp_wmb(); /* barrier to make sure earlier fields are written before
                      handing over ownership */

        write->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE |
            SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;

#ifdef DEBUG_PRINTS
        warn_report("simbricks_mmio_rw: finished write (%lu) addr=%lx size=%x "
                    "val=%lx", cur_ts, addr, size, *val);
#endif

        /* we treat writes as posted and don't wait for completion */
        return;
    } else {
        read = &msg->read;

        read->req_id = req - simbricks->reqs;
        read->offset = addr;
        read->len = size;
        read->bar = bar;

        smp_wmb(); /* barrier to make sure earlier fields are written before
                      handing over ownership */

        read->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_READ |
            SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;

        /* start processing request */
        req->processing = true;
        req->requested = true;
        req->size = size;

        req->addr = addr;
        req->bar = bar;
        req->size = size;

#ifdef DEBUG_PRINTS
        warn_report("simbricks_mmio_rw: starting wait for read (%lu) addr=%lx "
                    "size=%x", cur_ts, addr, size);
#endif

        if (simbricks->sync) {
            cpu->stopped = 1;
            cpu_loop_exit(cpu);
        } else {
            while (req->processing)
                qemu_cond_wait_iothread(&req->cond);

            *val = req->value;
            req->requested = false;
        }
    }
}

static uint64_t simbricks_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    SimbricksPciBarInfo *bar = opaque;
    SimbricksPciState *simbricks = bar->simbricks;
    uint64_t ret = 0;

    if (bar->is_dummy)
        return 0;

    simbricks_mmio_rw(simbricks, bar->index, addr, size, &ret, false);

    return ret;
}

static void simbricks_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    SimbricksPciBarInfo *bar = opaque;
    SimbricksPciState *simbricks = bar->simbricks;

    if (bar->is_dummy)
        return;

    simbricks_mmio_rw(simbricks, bar->index, addr, size, &val, true);
}

static const MemoryRegionOps simbricks_mmio_ops = {
    .read = simbricks_mmio_read,
    .write = simbricks_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 64,
    .impl.max_access_size = 64,
    .valid.unaligned = true,
    .impl.unaligned = true,
};


static void simbricks_config_write(PCIDevice *dev,
                                   uint32_t address,
                                   uint32_t val,
                                   int len)
{
    SimbricksPciState *simbricks = SIMBRICKS_PCI(dev);
    volatile union SimbricksProtoPcieH2D *msg;
    volatile struct SimbricksProtoPcieH2DDevctrl *devctrl;
    bool intx_before, intx_after;
    bool msi_before, msi_after;
    bool msix_before, msix_after;

    intx_before = !pci_irq_disabled(dev);
    msi_before = msi_enabled(dev);
    msix_before = msix_enabled(dev);

    pci_default_write_config(dev, address, val, len);

    intx_after = !pci_irq_disabled(dev);
    msi_after = msi_enabled(dev);
    msix_after = msix_enabled(dev);

    if (intx_before != intx_after || msi_before != msi_after ||
            msix_before != msix_after)
    {
        msg = simbricks_comm_h2d_alloc(simbricks,
                                       qemu_clock_get_ns(SIMBRICKS_CLOCK));
        devctrl = &msg->devctrl;

        devctrl->flags = 0;
        if (intx_after)
            devctrl->flags |= SIMBRICKS_PROTO_PCIE_CTRL_INTX_EN;
        if (msi_after)
            devctrl->flags |= SIMBRICKS_PROTO_PCIE_CTRL_MSI_EN;
        if (msix_after)
            devctrl->flags |= SIMBRICKS_PROTO_PCIE_CTRL_MSIX_EN;

        smp_wmb(); /* barrier to make sure earlier fields are written before
                      handing over ownership */

        devctrl->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_DEVCTRL |
            SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;
    }
}


/******************************************************************************/
/* Initialization */

static int simbricks_connect(SimbricksPciState *simbricks, Error **errp)
{
    struct SimbricksProtoPcieDevIntro *d_i = &simbricks->dev_intro;
    struct SimbricksProtoPcieHostIntro host_intro;
    struct stat statbuf;
    int ret, off, len;
    CPUState *cpu;
    void *p;
    uint8_t *pci_conf = simbricks->pdev.config;

    if (!qemu_chr_fe_backend_connected(&simbricks->sim_chr)) {
        error_setg(errp, "no simulator chardev specified");
        return 0;
    }

    /* send host intro */
    memset(&host_intro, 0, sizeof(host_intro));

    if (simbricks->sync)
        host_intro.flags = SIMBRICKS_PROTO_PCIE_FLAGS_HI_SYNC;

    len = sizeof(host_intro);
    off = 0;
    while (off < len) {
        ret = qemu_chr_fe_write_all(&simbricks->sim_chr,
                ((uint8_t *) &host_intro) + off, len - off);
        if (ret <= 0) {
            if (ret == -EINTR) {
                continue;
            }
            error_setg(errp, "simbricks_connect: sending host intro failed");
            return 0;
        }

        off += ret;
    }


    /* receive device intro */
    len = sizeof(*d_i);
    off = 0;
    while (off < len) {
        ret = qemu_chr_fe_read_all(&simbricks->sim_chr,
                ((uint8_t *) d_i) + off, len - off);
        if (ret <= 0) {
            if (ret == -EINTR) {
                continue;
            }
            error_setg(errp, "simbricks_connect: receiving dev intro failed");
            return 0;
        }
        off += ret;

    }

    if (simbricks->sync) {
        if (!(d_i->flags & SIMBRICKS_PROTO_PCIE_FLAGS_DI_SYNC)) {
            error_setg(errp, "simbricks_connect: sync not reciprocated");
            return 0;
        }
    }

    /* get shared memory fd */
    ret = qemu_chr_fe_get_msgfd(&simbricks->sim_chr);
    if (ret < 0) {
        error_setg(errp, "simbricks_connect: receiving shm fd failed");
        return 0;
    }

    /* receive and validate lengths and offsets */
    if (fstat(ret, &statbuf) != 0) {
        error_setg_errno(errp, errno, "simbricks_connect: fstat failed");
        close(ret);
        return 0;
    }
    simbricks->shm_len = statbuf.st_size;

    /* TODO: validate queue offsets and lengths */

    /* mmap shared memory fd */
    p = mmap(NULL, simbricks->shm_len, PROT_READ | PROT_WRITE, MAP_SHARED, ret,
             0);
    close(ret);
    if (p == MAP_FAILED) {
        error_setg_errno(errp, errno, "simbricks_connect: mmap failed");
        return 0;
    }
    simbricks->shm_base = p;

    /* setup queues */
    simbricks->d2h_base = (uint8_t *) simbricks->shm_base + d_i->d2h_offset;
    simbricks->d2h_elen = d_i->d2h_elen;
    simbricks->d2h_nentries = d_i->d2h_nentries;
    simbricks->d2h_pos = 0;

    simbricks->h2d_base = (uint8_t *) simbricks->shm_base + d_i->h2d_offset;
    simbricks->h2d_elen = d_i->h2d_elen;
    simbricks->h2d_nentries = d_i->h2d_nentries;
    simbricks->h2d_pos = 0;

    simbricks->reqs_len = 0;
    CPU_FOREACH(cpu) {
        simbricks->reqs_len++;
    }
    simbricks->reqs = calloc(simbricks->reqs_len, sizeof(*simbricks->reqs));
    CPU_FOREACH(cpu) {
        simbricks->reqs[cpu->cpu_index].cpu = cpu;
        qemu_cond_init(&simbricks->reqs[cpu->cpu_index].cond);
    }

    if (simbricks->sync) {
        simbricks->timer_dummy =
            timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_dummy, simbricks);

        simbricks->ts_base = qemu_clock_get_ns(SIMBRICKS_CLOCK);
        simbricks->timer_sync =
            timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_sync, simbricks);
        simbricks->sync_ts = simbricks->ts_base;
        timer_mod_ns(simbricks->timer_sync, simbricks->ts_base);
        simbricks->poll_ts = simbricks->ts_base + 1;
        simbricks->timer_poll =
            timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_poll, simbricks);
        timer_mod_ns(simbricks->timer_poll, simbricks->ts_base + 1);
    } else {
        qemu_thread_create(&simbricks->thread, "simbricks-poll",
                simbricks_poll_thread, simbricks, QEMU_THREAD_JOINABLE);
    }

    /* set vendor, device, revision, and class id */
    pci_config_set_vendor_id(pci_conf, d_i->pci_vendor_id);
    pci_config_set_device_id(pci_conf, d_i->pci_device_id);
    pci_config_set_revision(pci_conf, d_i->pci_revision);
    pci_config_set_class(pci_conf,
            ((uint16_t) d_i->pci_class << 8) | d_i->pci_subclass);

    return 1;
}

static void pci_simbricks_realize(PCIDevice *pdev, Error **errp)
{
    SimbricksPciState *simbricks = SIMBRICKS_PCI(pdev);
    size_t i;
    uint64_t len, flags;
    uint8_t attr;
    const char *label;
    uint8_t *pci_conf = pdev->config;

    if (!simbricks_connect(simbricks, errp)) {
      return;
    }

    pci_config_set_interrupt_pin(pci_conf, 1);
    if (simbricks->dev_intro.pci_msi_nvecs > 0) {
        if (msi_init(pdev, 0, simbricks->dev_intro.pci_msi_nvecs, true, false,
                    errp))
        {
            return;
        }
    }

    for (i = 0; i < 6; i++) {
        len = simbricks->dev_intro.bars[i].len;
        flags = simbricks->dev_intro.bars[i].flags;

        /* skip empty bars */
        if (len == 0) {
            continue;
        }

        simbricks->bar_info[i].simbricks = simbricks;
        simbricks->bar_info[i].index = i;

        if (!(flags & SIMBRICKS_PROTO_PCIE_BAR_IO)) {
            /* memory bar */
            simbricks->bar_info[i].is_io = false;

            attr = PCI_BASE_ADDRESS_SPACE_MEMORY;
            if ((flags & SIMBRICKS_PROTO_PCIE_BAR_64)) {
                attr |= PCI_BASE_ADDRESS_MEM_TYPE_64;
            } else {
                attr |= PCI_BASE_ADDRESS_MEM_TYPE_32;
            }
            if ((flags & SIMBRICKS_PROTO_PCIE_BAR_PF)) {
                attr |= PCI_BASE_ADDRESS_MEM_PREFETCH;
            }
            label = "simbricks-bar-mmio";
        } else {
            /* I/O port bar */
            simbricks->bar_info[i].is_io = true;

            label = "simbricks-bar-ioport";
            attr = PCI_BASE_ADDRESS_SPACE_IO;
        }

        if ((flags & SIMBRICKS_PROTO_PCIE_BAR_DUMMY))
            simbricks->bar_info[i].is_dummy = true;

        memory_region_init_io(&simbricks->mmio_bars[i], OBJECT(simbricks),
                &simbricks_mmio_ops, &simbricks->bar_info[i], label, len);
        pci_register_bar(pdev, i, attr, &simbricks->mmio_bars[i]);
    }

    if (simbricks->dev_intro.pci_msix_nvecs > 0) {
        /* TODO: MSI-X cap offset is hardcoded */
        if (msix_init(pdev, simbricks->dev_intro.pci_msix_nvecs,
                &simbricks->mmio_bars[simbricks->dev_intro.pci_msix_table_bar],
                simbricks->dev_intro.pci_msix_table_bar,
                simbricks->dev_intro.pci_msix_table_offset,
                &simbricks->mmio_bars[simbricks->dev_intro.pci_msix_pba_bar],
                simbricks->dev_intro.pci_msix_pba_bar,
                simbricks->dev_intro.pci_msix_pba_offset,
                simbricks->dev_intro.psi_msix_cap_offset, errp))
        {
            return;
        }

        for (i = 0; i < simbricks->dev_intro.pci_msix_nvecs; i++) {
            if (msix_vector_use(pdev, i)) {
                error_setg(errp, "simbricks_connect: msix_vector_use failed");
                return;
            }
        }
    }
}

static void pci_simbricks_uninit(PCIDevice *pdev)
{
    SimbricksPciState *simbricks = SIMBRICKS_PCI(pdev);
    CPUState *cpu;

    if (!simbricks->sync) {
        simbricks->stopping = true;
        qemu_thread_join(&simbricks->thread);
    }

    CPU_FOREACH(cpu) {
        qemu_cond_destroy(&simbricks->reqs[cpu->cpu_index].cond);
    }
    free(simbricks->reqs);

    if (simbricks->sync) {
        timer_del(simbricks->timer_dummy);
        timer_free(simbricks->timer_dummy);
        timer_del(simbricks->timer_sync);
        timer_free(simbricks->timer_sync);
        timer_del(simbricks->timer_poll);
        timer_free(simbricks->timer_poll);
    }

    if (simbricks->dev_intro.pci_msi_nvecs > 0)
        msi_uninit(pdev);
}

static void simbricks_pci_instance_init(Object *obj)
{
}

static Property simbricks_pci_dev_properties[] = {
  DEFINE_PROP_CHR("chardev", SimbricksPciState, sim_chr),
  DEFINE_PROP_BOOL("sync", SimbricksPciState, sync, false),
  DEFINE_PROP_INT32("sync-mode", SimbricksPciState, sync_mode,
      SIMBRICKS_PROTO_SYNC_SIMBRICKS),
  DEFINE_PROP_UINT64("pci-latency", SimbricksPciState, pci_latency, 500),
  DEFINE_PROP_UINT64("sync-period", SimbricksPciState, sync_period, 500),
  DEFINE_PROP_END_OF_LIST(),
};

static void simbricks_pci_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    device_class_set_props(dc, simbricks_pci_dev_properties);

    k->realize = pci_simbricks_realize;
    k->exit = pci_simbricks_uninit;
    k->config_write = simbricks_config_write;

    /* TODO: how can we parametrize these? */
    k->vendor_id = 0x10EE; /* xilinx */
    k->device_id = 0x1234;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Co-Simulation PCI adapter";
}

static void pci_simbricks_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo simbricks_info = {
        .name          = TYPE_PCI_SIMBRICKS_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(SimbricksPciState),
        .instance_init = simbricks_pci_instance_init,
        .class_init    = simbricks_pci_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&simbricks_info);
}
type_init(pci_simbricks_register_types)
