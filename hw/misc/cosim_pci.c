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

#include <simbricks/proto/pcie.h>

//#define DEBUG_PRINTS

#define COSIM_CLOCK QEMU_CLOCK_VIRTUAL

#define TYPE_PCI_COSIM_DEVICE "cosim-pci"
#define COSIM_PCI(obj)        OBJECT_CHECK(CosimPciState, obj, TYPE_PCI_COSIM_DEVICE)

typedef struct CosimPciBarInfo {
    struct CosimPciState *cosim;
    uint8_t index;
    bool is_io;
    bool is_dummy;
} CosimPciBarInfo;

typedef struct CosimPciRequest {
    CPUState *cpu;          /* CPU associated with this request */
    QemuCond cond;
    uint64_t addr;
    uint8_t bar;
    uint64_t value;         /* value read/to be written */
    unsigned size;
    bool processing;
    bool requested;
} CosimPciRequest;

#define SYNC_MODES   0
#define SYNC_BARRIER 1

typedef struct CosimPciState {
    PCIDevice pdev;

    QemuThread thread;
    volatile bool stopping;

    CharBackend sim_chr;

    MemoryRegion mmio_bars[6];
    CosimPciBarInfo bar_info[6];

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
    CosimPciRequest *reqs;

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
} CosimPciState;

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

static inline uint64_t ts_to_proto(CosimPciState *cosim, int64_t qemu_ts)
{
    return (qemu_ts - cosim->ts_base) * 1000;
}

static inline int64_t ts_from_proto(CosimPciState *cosim, uint64_t proto_ts)
{
    return (proto_ts / 1000) + cosim->ts_base;
}

/******************************************************************************/
/* Worker thread */

static volatile union SimbricksProtoPcieH2D *cosim_comm_h2d_alloc(
        CosimPciState *cosim, int64_t ts)
{
    uint8_t *pos;
    volatile union SimbricksProtoPcieH2D *msg;

    pos = cosim->h2d_base + (cosim->h2d_elen * cosim->h2d_pos);
    msg = (volatile union SimbricksProtoPcieH2D *) pos;

    while ((msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_H2D_OWN_MASK) !=
            SIMBRICKS_PROTO_PCIE_H2D_OWN_HOST)
    {
#ifdef DEBUG_PRINTS
        warn_report("cosim_comm_h2d_alloc: ran into non-owned entry in h2d queue");
#endif
    }

    /* tag message with timestamp */
    msg->dummy.timestamp = ts_to_proto(cosim, ts + cosim->pci_latency);

    /* re-arm sync timer */
    if (cosim->sync_mode == SYNC_MODES) {
        cosim->sync_ts = ts + cosim->sync_period;
        cosim->sync_ts_bumped = true;
    }

#ifdef DEBUG_PRINTS
    warn_report("cosim_comm_h2d_alloc: ts=%lu msg_ts=%lu next=%ld", ts,
            msg->dummy.timestamp, ts + cosim->sync_period);
#endif

    /* advance position to next entry */
    cosim->h2d_pos = (cosim->h2d_pos + 1) % cosim->h2d_nentries;

    return msg;
}

static void cosim_comm_d2h_dma_read(CosimPciState *cosim, int64_t ts,
        volatile struct SimbricksProtoPcieD2HRead *read)
{
    volatile union SimbricksProtoPcieH2D *h2d;
    volatile struct SimbricksProtoPcieH2DReadcomp *rc;

    /* allocate completion */
    h2d = cosim_comm_h2d_alloc(cosim, ts);
    rc = &h2d->readcomp;

    assert(read->len <= cosim->h2d_elen - sizeof (*rc));

    /* perform dma read */
    pci_dma_read(&cosim->pdev, read->offset, (void *) rc->data, read->len);

    /* return completion */
    rc->req_id = read->req_id;
    smp_wmb();
    rc->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_READCOMP |
        SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;
}

static void cosim_comm_d2h_dma_write(CosimPciState *cosim, int64_t ts,
        volatile struct SimbricksProtoPcieD2HWrite *write)
{
    volatile union SimbricksProtoPcieH2D *h2d;
    volatile struct SimbricksProtoPcieH2DWritecomp *wc;

    /* allocate completion */
    h2d = cosim_comm_h2d_alloc(cosim, ts);
    wc = &h2d->writecomp;

    /* perform dma write */
    pci_dma_write(&cosim->pdev, write->offset, (void *) write->data,
            write->len);

    /* return completion */
    wc->req_id = write->req_id;
    smp_wmb();
    wc->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITECOMP |
        SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;
}

static void cosim_comm_d2h_interrupt(CosimPciState *cosim,
        volatile struct SimbricksProtoPcieD2HInterrupt *interrupt)
{
    if (interrupt->inttype == SIMBRICKS_PROTO_PCIE_INT_MSI) {
        if (interrupt->vector >= 32) {
            warn_report("cosim_comm_d2h_interrupt: invalid MSI vector (%u)",
                    interrupt->vector);
            return;
        }
        msi_notify(&cosim->pdev, interrupt->vector);
    } else if (interrupt->inttype == SIMBRICKS_PROTO_PCIE_INT_MSIX) {
        if (interrupt->vector >= cosim->dev_intro.pci_msix_nvecs) {
            warn_report("cosim_comm_d2h_interrupt: invalid MSI-X vector (%u)",
                    interrupt->vector);
            return;
        }
        msix_notify(&cosim->pdev, interrupt->vector);
    } else {
        warn_report("cosim_comm_d2h_interrupt: not yet implented int type (%u)"
                " TODO", interrupt->inttype);
    }
}

static void cosim_comm_d2h_rcomp(CosimPciState *cosim, uint64_t cur_ts,
        uint64_t req_id, const void *data)
{
    CosimPciRequest *req = cosim->reqs + req_id;
    CPUState *cpu;

    assert(req_id <= cosim->reqs_len);

    if (!req->processing) {
        panic("cosim_comm_d2h_rcomp: no request currently processing");
    }

    /* copy read value from message */
    req->value = 0;
    memcpy(&req->value, data, req->size);

    req->processing = false;

    if (cosim->sync) {
        cpu = req->cpu;

#ifdef DEBUG_PRINTS
        warn_report("cosim_comm_d2h_rcomp: kicking cpu %lu ts=%lu",
                req_id, cur_ts);
#endif

        cpu->stopped = 0;
        //qemu_cpu_kick(cpu);
    } else {
        qemu_cond_broadcast(&req->cond);
    }
}

/* process and complete message */
static void cosim_comm_d2h_process(CosimPciState *cosim, int64_t ts,
        volatile union SimbricksProtoPcieD2H *msg)
{
    uint8_t type;

    type = msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_D2H_MSG_MASK;
#ifdef DEBUG_PRINTS
    warn_report("cosim_comm_d2h_process: ts=%ld type=%u", ts, type);
#endif

    switch (type) {
        case SIMBRICKS_PROTO_PCIE_D2H_MSG_SYNC:
            /* nop */
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_READ:
            cosim_comm_d2h_dma_read(cosim, ts, &msg->read);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITE:
            cosim_comm_d2h_dma_write(cosim, ts, &msg->write);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_INTERRUPT:
            cosim_comm_d2h_interrupt(cosim, &msg->interrupt);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP:
            cosim_comm_d2h_rcomp(cosim, ts, msg->readcomp.req_id,
                    (void *) msg->readcomp.data);
            break;

        case SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP:
            /* we treat writes as posted, so nothing we need to do here */
            break;

        default:
            panic("cosim_comm_poll_d2h: unhandled type");
    }

    smp_wmb();

    /* mark message as done */
    msg->dummy.own_type =
        (msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_D2H_MSG_MASK) |
        SIMBRICKS_PROTO_PCIE_D2H_OWN_DEV;
}

/* peek at head of d2h queue */
static int cosim_comm_d2h_peek(CosimPciState *cosim, int64_t ts,
        int64_t *next_ts, volatile union SimbricksProtoPcieD2H **pmsg)
{
    uint8_t *pos;
    int64_t msg_ts;
    volatile union SimbricksProtoPcieD2H *msg;

    pos = cosim->d2h_base + (cosim->d2h_elen * cosim->d2h_pos);
    msg = (volatile union SimbricksProtoPcieD2H *) pos;

    /* check if this message is ready for us */
    if ((msg->dummy.own_type & SIMBRICKS_PROTO_PCIE_D2H_OWN_MASK) !=
        SIMBRICKS_PROTO_PCIE_D2H_OWN_HOST)
    {
        return -1;
    }

    smp_rmb();

    msg_ts = ts_from_proto(cosim, msg->dummy.timestamp);

    *pmsg = msg;
    *next_ts = msg_ts;
    if (cosim->sync && msg_ts > ts) {
        /* still in the future */
        return 1;
    }

    return 0;
}

/* move on to next d2h queue position */
static void cosim_comm_d2h_next(CosimPciState *cosim)
{
    /* advance position to next entry */
    cosim->d2h_pos = (cosim->d2h_pos + 1) % cosim->d2h_nentries;
}

static void cosim_trigger_sync(CosimPciState *cosim, int64_t cur_ts)
{
    volatile union SimbricksProtoPcieH2D *msg;
    volatile struct SimbricksProtoPcieH2DSync *sy;

    msg = cosim_comm_h2d_alloc(cosim, cur_ts);
    sy = &msg->sync;

    smp_wmb(); /* barrier to make sure earlier fields are written before
                  handing over ownership */

    sy->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_SYNC | SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;

    /* cosim_comm_h2d_alloc has already re-scheduled the timer */
}

static void cosim_timer_dummy(void *data)
{
}

static void cosim_timer_poll(void *data)
{
    CosimPciState *cosim = data;
    volatile union SimbricksProtoPcieD2H *msg;
    volatile union SimbricksProtoPcieD2H *next_msg;
    int64_t cur_ts, next_ts;
    int ret;

    cur_ts = qemu_clock_get_ns(COSIM_CLOCK);
#ifdef DEBUG_PRINTS
    if (cur_ts > cosim->poll_ts + 1 || cur_ts < cosim->poll_ts)
        warn_report("cosim_timer_poll: expected_ts=%lu cur_ts=%lu", cosim->poll_ts, cur_ts);
#endif

    if (cur_ts > cosim->sync_ts + 1) {
#ifdef DEBUG_PRINTS
        warn_report("cosim_timer_poll: triggering sync: ts=%ld sync_ts=%ld", cur_ts, timer_expire_time_ns(cosim->timer_sync));
#endif
        cosim_trigger_sync(cosim, cur_ts);
    }

#ifdef DEBUG_PRINTS
    warn_report("cosim_timer_poll: ts=%ld sync_ts=%ld", cur_ts, timer_expire_time_ns(cosim->timer_sync));
#endif

    /* poll until we have a message */
    do {
        ret = cosim_comm_d2h_peek(cosim, cur_ts, &next_ts, &msg);
    } while (ret < 0);

    if (ret == 0) {
        /* message is ready to be processed */
        cosim_comm_d2h_next(cosim);

        /* now poll until we have next message so we know the timestamp */
        while (cosim_comm_d2h_peek(cosim, cur_ts, &next_ts, &next_msg) < 0);
    } else {
        next_msg = msg;
    }

    /* set timer for next message */
    /* we need to do this before actually processing the message, in order to
     * have a timer set to prevent the clock from running away from us. We set a
     * dummy timer with the current ts to prevent the clock from jumping */
    timer_mod_ns(cosim->timer_dummy, cur_ts);
    timer_mod_ns(cosim->timer_poll, next_ts);
    if (cosim->sync_ts_bumped) {
        timer_mod_ns(cosim->timer_sync, cosim->sync_ts);
        cosim->sync_ts_bumped = false;
    }
    cosim->poll_ts = next_ts;

    /* run */
    if (ret == 0) {
        cosim_comm_d2h_process(cosim, cur_ts, msg);
    }

#ifdef DEBUG_PRINTS
    int64_t now_ts = qemu_clock_get_ns(COSIM_CLOCK);
    if (cur_ts != now_ts)
        warn_report("cosim_timer_poll: time advanced from %lu to %lu", cur_ts, now_ts);

    warn_report("cosim_timer_poll: done, next=%ld", next_ts);
#endif
}

static void cosim_timer_sync(void *data)
{
    CosimPciState *cosim = data;
    int64_t cur_ts;

    cur_ts = qemu_clock_get_ns(COSIM_CLOCK);

#ifdef DEBUG_PRINTS
    if (cur_ts > cosim->sync_ts + 1)
        warn_report("cosim_timer_sync: expected_ts=%lu cur_ts=%lu", cosim->sync_ts, cur_ts);

    warn_report("cosim_timer_sync: ts=%ld", cur_ts);
#endif

    cosim_trigger_sync(cosim, cur_ts);

#ifdef DEBUG_PRINTS
    int64_t now_ts = qemu_clock_get_ns(COSIM_CLOCK);
    if (cur_ts != now_ts)
        warn_report("cosim_timer_poll: time advanced from %lu to %lu", cur_ts, now_ts);
#endif

    if (cosim->sync_mode == SYNC_BARRIER) {
        cosim->sync_ts = cur_ts + cosim->sync_period;
        cosim->sync_ts_bumped = true;
    }
    assert(cosim->sync_ts_bumped);
    timer_mod_ns(cosim->timer_sync, cosim->sync_ts);
    cosim->sync_ts_bumped = false;
}

static void *cosim_poll_thread(void *opaque)
{
    CosimPciState *cosim = opaque;
    volatile union SimbricksProtoPcieD2H *msg;
    int64_t next_ts;
    int ret;

    assert(!cosim->sync);

    while (!cosim->stopping) {
        ret = cosim_comm_d2h_peek(cosim, 0, &next_ts, &msg);
        if (ret)
            continue;

        cosim_comm_d2h_next(cosim);

        /* actually process the operation. this needs to be done with the I/O
         * lock held. */
        qemu_mutex_lock_iothread();
        cosim_comm_d2h_process(cosim, 0, msg);
        qemu_mutex_unlock_iothread();
    }

    return NULL;
}

/******************************************************************************/
/* MMIO interface */

/* submit a bar read or write to the worker thread and wait for it to
 * complete */
static void cosim_mmio_rw(CosimPciState *cosim, uint8_t bar, hwaddr addr,
        unsigned size, uint64_t *val, bool is_write)
{
    CPUState *cpu = current_cpu;
    CosimPciRequest *req;
    volatile union SimbricksProtoPcieH2D *msg;
    volatile struct SimbricksProtoPcieH2DRead *read;
    volatile struct SimbricksProtoPcieH2DWrite *write;
    int64_t cur_ts;

    assert(cosim->reqs_len > cpu->cpu_index);
    req = cosim->reqs + cpu->cpu_index;
    assert(req->cpu == cpu);

    cur_ts = qemu_clock_get_ns(COSIM_CLOCK);

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
            warn_report("cosim_mmio_rw: done (%lu) a=%lx s=%x val=%lx", cur_ts, addr, size, req->value);
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
    msg = cosim_comm_h2d_alloc(cosim, cur_ts);

    /* prepare operation */
    if (is_write) {
        write = &msg->write;

        write->req_id = cpu->cpu_index;
        write->offset = addr;
        write->len = size;
        write->bar = bar;

        assert(size <= cosim->h2d_elen - sizeof (*write));
        /* FIXME: this probably only works for LE */
        memcpy((void *) write->data, val, size);

        smp_wmb(); /* barrier to make sure earlier fields are written before
                      handing over ownership */

        write->own_type = SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE |
            SIMBRICKS_PROTO_PCIE_H2D_OWN_DEV;

#ifdef DEBUG_PRINTS
        warn_report("cosim_mmio_rw: finished write (%lu) addr=%lx size=%x val=%lx",
                cur_ts, addr, size, *val);
#endif

        /* we treat writes as posted and don't wait for completion */
        return;
    } else {
        read = &msg->read;

        read->req_id = req - cosim->reqs;
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
        warn_report("cosim_mmio_rw: starting wait for read (%lu) addr=%lx size=%x", cur_ts, addr, size);
#endif

        if (cosim->sync) {
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

static uint64_t cosim_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CosimPciBarInfo *bar = opaque;
    CosimPciState *cosim = bar->cosim;
    uint64_t ret = 0;

    if (bar->is_dummy)
        return 0;

    cosim_mmio_rw(cosim, bar->index, addr, size, &ret, false);

    return ret;
}

static void cosim_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    CosimPciBarInfo *bar = opaque;
    CosimPciState *cosim = bar->cosim;

    if (bar->is_dummy)
        return;

    cosim_mmio_rw(cosim, bar->index, addr, size, &val, true);
}

static const MemoryRegionOps cosim_mmio_ops = {
    .read = cosim_mmio_read,
    .write = cosim_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 64,
    .impl.max_access_size = 64,
    .valid.unaligned = true,
    .impl.unaligned = true,
};


static void cosim_config_write(PCIDevice *dev, uint32_t address, uint32_t val,
        int len)
{
    CosimPciState *cosim = COSIM_PCI(dev);
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
        msg = cosim_comm_h2d_alloc(cosim, qemu_clock_get_ns(COSIM_CLOCK));
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

static int cosim_connect(CosimPciState *cosim, Error **errp)
{
    struct SimbricksProtoPcieDevIntro *d_i = &cosim->dev_intro;
    struct SimbricksProtoPcieHostIntro host_intro;
    struct stat statbuf;
    int ret, off, len;
    CPUState *cpu;
    void *p;
    uint8_t *pci_conf = cosim->pdev.config;

    if (!qemu_chr_fe_backend_connected(&cosim->sim_chr)) {
        error_setg(errp, "no simulator chardev specified");
        return 0;
    }

    /* send host intro */
    memset(&host_intro, 0, sizeof(host_intro));

    if (cosim->sync)
        host_intro.flags = SIMBRICKS_PROTO_PCIE_FLAGS_HI_SYNC;

    len = sizeof(host_intro);
    off = 0;
    while (off < len) {
        ret = qemu_chr_fe_write_all(&cosim->sim_chr,
                ((uint8_t *) &host_intro) + off, len - off);
        if (ret <= 0) {
            if (ret == -EINTR) {
                continue;
            }
            error_setg(errp, "cosim_connect: sending host intro failed");
            return 0;
        }

        off += ret;
    }


    /* receive device intro */
    len = sizeof(*d_i);
    off = 0;
    while (off < len) {
        ret = qemu_chr_fe_read_all(&cosim->sim_chr,
                ((uint8_t *) d_i) + off, len - off);
        if (ret <= 0) {
            if (ret == -EINTR) {
                continue;
            }
            error_setg(errp, "cosim_connect: receiving dev intro failed");
            return 0;
        }
        off += ret;

    }

    if (cosim->sync) {
        if (!(d_i->flags & SIMBRICKS_PROTO_PCIE_FLAGS_DI_SYNC)) {
            error_setg(errp, "cosim_connect: sync not reciprocated");
            return 0;
        }
    }

    /* get shared memory fd */
    ret = qemu_chr_fe_get_msgfd(&cosim->sim_chr);
    if (ret < 0) {
        error_setg(errp, "cosim_connect: receiving shm fd failed");
        return 0;
    }

    /* receive and validate lengths and offsets */
    if (fstat(ret, &statbuf) != 0) {
        error_setg_errno(errp, errno, "cosim_connect: fstat failed");
        close(ret);
        return 0;
    }
    cosim->shm_len = statbuf.st_size;

    /* TODO: validate queue offsets and lengths */

    /* mmap shared memory fd */
    p = mmap(NULL, cosim->shm_len, PROT_READ | PROT_WRITE, MAP_SHARED, ret, 0);
    close(ret);
    if (p == MAP_FAILED) {
        error_setg_errno(errp, errno, "cosim_connect: mmap failed");
        return 0;
    }
    cosim->shm_base = p;

    /* setup queues */
    cosim->d2h_base = (uint8_t *) cosim->shm_base + d_i->d2h_offset;
    cosim->d2h_elen = d_i->d2h_elen;
    cosim->d2h_nentries = d_i->d2h_nentries;
    cosim->d2h_pos = 0;

    cosim->h2d_base = (uint8_t *) cosim->shm_base + d_i->h2d_offset;
    cosim->h2d_elen = d_i->h2d_elen;
    cosim->h2d_nentries = d_i->h2d_nentries;
    cosim->h2d_pos = 0;

    cosim->reqs_len = 0;
    CPU_FOREACH(cpu) {
        cosim->reqs_len++;
    }
    cosim->reqs = calloc(cosim->reqs_len, sizeof(*cosim->reqs));
    CPU_FOREACH(cpu) {
        cosim->reqs[cpu->cpu_index].cpu = cpu;
        qemu_cond_init(&cosim->reqs[cpu->cpu_index].cond);
    }

    if (cosim->sync) {
        cosim->timer_dummy = timer_new_ns(COSIM_CLOCK, cosim_timer_dummy, cosim);

        cosim->ts_base = qemu_clock_get_ns(COSIM_CLOCK);
        cosim->timer_sync = timer_new_ns(COSIM_CLOCK, cosim_timer_sync, cosim);
        cosim->sync_ts = cosim->ts_base;
        timer_mod_ns(cosim->timer_sync, cosim->ts_base);
        cosim->poll_ts = cosim->ts_base + 1;
        cosim->timer_poll = timer_new_ns(COSIM_CLOCK, cosim_timer_poll, cosim);
        timer_mod_ns(cosim->timer_poll, cosim->ts_base + 1);
    } else {
        qemu_thread_create(&cosim->thread, "cosim-poll", cosim_poll_thread,
                cosim, QEMU_THREAD_JOINABLE);
    }

    /* set vendor, device, revision, and class id */
    pci_config_set_vendor_id(pci_conf, d_i->pci_vendor_id);
    pci_config_set_device_id(pci_conf, d_i->pci_device_id);
    pci_config_set_revision(pci_conf, d_i->pci_revision);
    pci_config_set_class(pci_conf,
            ((uint16_t) d_i->pci_class << 8) | d_i->pci_subclass);

    return 1;
}

static void pci_cosim_realize(PCIDevice *pdev, Error **errp)
{
    CosimPciState *cosim = COSIM_PCI(pdev);
    size_t i;
    uint64_t len, flags;
    uint8_t attr;
    const char *label;
    uint8_t *pci_conf = pdev->config;

    if (!cosim_connect(cosim, errp)) {
      return;
    }

    pci_config_set_interrupt_pin(pci_conf, 1);
    if (cosim->dev_intro.pci_msi_nvecs > 0) {
        if (msi_init(pdev, 0, cosim->dev_intro.pci_msi_nvecs, true, false,
                    errp))
        {
            return;
        }
    }

    for (i = 0; i < 6; i++) {
        len = cosim->dev_intro.bars[i].len;
        flags = cosim->dev_intro.bars[i].flags;

        /* skip empty bars */
        if (len == 0) {
            continue;
        }

        cosim->bar_info[i].cosim = cosim;
        cosim->bar_info[i].index = i;

        if (!(flags & SIMBRICKS_PROTO_PCIE_BAR_IO)) {
            /* memory bar */
            cosim->bar_info[i].is_io = false;

            attr = PCI_BASE_ADDRESS_SPACE_MEMORY;
            if ((flags & SIMBRICKS_PROTO_PCIE_BAR_64)) {
                attr |= PCI_BASE_ADDRESS_MEM_TYPE_64;
            } else {
                attr |= PCI_BASE_ADDRESS_MEM_TYPE_32;
            }
            if ((flags & SIMBRICKS_PROTO_PCIE_BAR_PF)) {
                attr |= PCI_BASE_ADDRESS_MEM_PREFETCH;
            }
            label = "cosim-bar-mmio";
        } else {
            /* I/O port bar */
            cosim->bar_info[i].is_io = true;

            label = "cosim-bar-ioport";
            attr = PCI_BASE_ADDRESS_SPACE_IO;
        }

        if ((flags & SIMBRICKS_PROTO_PCIE_BAR_DUMMY))
            cosim->bar_info[i].is_dummy = true;

        memory_region_init_io(&cosim->mmio_bars[i], OBJECT(cosim),
                &cosim_mmio_ops, &cosim->bar_info[i], label, len);
        pci_register_bar(pdev, i, attr, &cosim->mmio_bars[i]);
    }

    if (cosim->dev_intro.pci_msix_nvecs > 0) {
        /* TODO: MSI-X cap offset is hardcoded */
        if (msix_init(pdev, cosim->dev_intro.pci_msix_nvecs,
                &cosim->mmio_bars[cosim->dev_intro.pci_msix_table_bar],
                cosim->dev_intro.pci_msix_table_bar,
                cosim->dev_intro.pci_msix_table_offset,
                &cosim->mmio_bars[cosim->dev_intro.pci_msix_pba_bar],
                cosim->dev_intro.pci_msix_pba_bar,
                cosim->dev_intro.pci_msix_pba_offset,
                cosim->dev_intro.psi_msix_cap_offset, errp))
        {
            return;
        }

        for (i = 0; i < cosim->dev_intro.pci_msix_nvecs; i++) {
            if (msix_vector_use(pdev, i)) {
                error_setg(errp, "cosim_connect: msix_vector_use failed");
                return;
            }
        }
    }
}

static void pci_cosim_uninit(PCIDevice *pdev)
{
    CosimPciState *cosim = COSIM_PCI(pdev);
    CPUState *cpu;

    if (!cosim->sync) {
        cosim->stopping = true;
        qemu_thread_join(&cosim->thread);
    }

    CPU_FOREACH(cpu) {
        qemu_cond_destroy(&cosim->reqs[cpu->cpu_index].cond);
    }
    free(cosim->reqs);

    if (cosim->sync) {
        timer_del(cosim->timer_dummy);
        timer_free(cosim->timer_dummy);
        timer_del(cosim->timer_sync);
        timer_free(cosim->timer_sync);
        timer_del(cosim->timer_poll);
        timer_free(cosim->timer_poll);
    }

    if (cosim->dev_intro.pci_msi_nvecs > 0)
        msi_uninit(pdev);
}

static void cosim_pci_instance_init(Object *obj)
{
}

static Property cosim_pci_dev_properties[] = {
  DEFINE_PROP_CHR("chardev", CosimPciState, sim_chr),
  DEFINE_PROP_BOOL("sync", CosimPciState, sync, false),
  DEFINE_PROP_INT32("sync-mode", CosimPciState, sync_mode, SYNC_MODES),
  DEFINE_PROP_UINT64("pci-latency", CosimPciState, pci_latency, 500),
  DEFINE_PROP_UINT64("sync-period", CosimPciState, sync_period, 500),
  DEFINE_PROP_END_OF_LIST(),
};

static void cosim_pci_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    device_class_set_props(dc, cosim_pci_dev_properties);

    k->realize = pci_cosim_realize;
    k->exit = pci_cosim_uninit;
    k->config_write = cosim_config_write;

    /* TODO: how can we parametrize these? */
    k->vendor_id = 0x10EE; /* xilinx */
    k->device_id = 0x1234;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Co-Simulation PCI adapter";
}

static void pci_cosim_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo cosim_info = {
        .name          = TYPE_PCI_COSIM_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(CosimPciState),
        .instance_init = cosim_pci_instance_init,
        .class_init    = cosim_pci_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&cosim_info);
}
type_init(pci_cosim_register_types)
