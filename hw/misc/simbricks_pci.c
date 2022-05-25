/*
 * Co-simulation PCI device
 *
 * Copyright (c) 2020-2022 Max Planck Institute for Software Systems
 * Copyright (c) 2020-2022 National University of Singapore
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

#include <simbricks/pcie/if.h>

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

typedef struct SimbricksPciState {
    PCIDevice pdev;

    QemuThread thread;
    volatile bool stopping;

    /* config parameters */
    char *socket_path;      /* path to ux socket to connect to */
    uint64_t pci_latency;
    uint64_t sync_period;

    MemoryRegion mmio_bars[6];
    SimbricksPciBarInfo bar_info[6];

    struct SimbricksPcieIf pcieif;
    struct SimbricksProtoPcieDevIntro dev_intro;

    bool sync_ts_bumped;

    /* communication bewteen main io thread and worker thread
     * (protected by thr_mutex). */
    size_t reqs_len;
    SimbricksPciRequest *reqs;

    /* timers for synchronization etc. */
    bool sync;
    int sync_mode;
    int64_t ts_base;
    QEMUTimer *timer_dummy;
    QEMUTimer *timer_sync;
    QEMUTimer *timer_poll;
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

static inline volatile union SimbricksProtoPcieH2D *simbricks_comm_h2d_alloc(
        SimbricksPciState *simbricks,
        uint64_t ts)
{
    volatile union SimbricksProtoPcieH2D *msg;
    while (!(msg = SimbricksPcieIfH2DOutAlloc(&simbricks->pcieif,
                                              ts_to_proto(simbricks, ts))));

    // whenever we send a message, we need to reschedule our sync timer
    simbricks->sync_ts_bumped = true;
    return msg;
}

/******************************************************************************/
/* Worker thread */

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

    assert(read->len <=
        SimbricksPcieIfH2DOutMsgLen(&simbricks->pcieif) - sizeof (*rc));

    /* perform dma read */
    pci_dma_read(&simbricks->pdev, read->offset, (void *) rc->data, read->len);

    /* return completion */
    rc->req_id = read->req_id;
    SimbricksPcieIfH2DOutSend(&simbricks->pcieif, h2d,
        SIMBRICKS_PROTO_PCIE_H2D_MSG_READCOMP);
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
    SimbricksPcieIfH2DOutSend(&simbricks->pcieif, h2d,
        SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITECOMP);
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

    type = SimbricksPcieIfD2HInType(&simbricks->pcieif, msg);
#ifdef DEBUG_PRINTS
    warn_report("simbricks_comm_d2h_process: ts=%ld type=%u", ts, type);
#endif

    switch (type) {
        case SIMBRICKS_PROTO_MSG_TYPE_SYNC:
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

    SimbricksPcieIfD2HInDone(&simbricks->pcieif, msg);
}

static void simbricks_timer_dummy(void *data)
{
}

static void simbricks_timer_poll(void *data)
{
    SimbricksPciState *simbricks = data;
    volatile union SimbricksProtoPcieD2H *msg;
    volatile union SimbricksProtoPcieD2H *next_msg;
    int64_t cur_ts, next_ts, proto_ts;

    cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    proto_ts = ts_to_proto(simbricks, cur_ts + 1); // + 1 to avoid getting stuck
                                                   // on off by ones due to of
                                                   // rounding
#ifdef DEBUG_PRINTS
    uint64_t poll_ts = SimbricksPcieIfD2HInTimestamp(&simbricks->pcieif);
    if (proto_ts > poll_ts + 1 || proto_ts < poll_ts)
        warn_report("simbricks_timer_poll: expected_ts=%lu cur_ts=%lu",
                    poll_ts, cur_ts);
    warn_report("simbricks_timer_poll: ts=%ld sync_ts=%ld", cur_ts,
                timer_expire_time_ns(simbricks->timer_sync));
#endif

    /* poll until we have a message (should not usually spin) */
    do {
        msg = SimbricksPcieIfD2HInPoll(&simbricks->pcieif, proto_ts);
    } while (msg == NULL);

    /* wait for next message so we know its timestamp and when to schedule the
     * timer. */
    do {
        next_msg = SimbricksPcieIfD2HInPeek(&simbricks->pcieif, proto_ts);
        next_ts = SimbricksPcieIfD2HInTimestamp(&simbricks->pcieif);
    } while (!next_msg && next_ts <= proto_ts);

    /* set timer for next message */
    /* we need to do this before actually processing the message, in order to
     * have a timer set to prevent the clock from running away from us. We set a
     * dummy timer with the current ts to prevent the clock from jumping */
    timer_mod_ns(simbricks->timer_dummy, cur_ts);
    timer_mod_ns(simbricks->timer_poll, ts_from_proto(simbricks, next_ts));
    if (simbricks->sync_ts_bumped) {
        timer_mod_ns(simbricks->timer_sync,
            ts_from_proto(simbricks,
                SimbricksBaseIfOutNextSync(&simbricks->pcieif.base)));
        simbricks->sync_ts_bumped = false;
    }

    /* now process the message */
    simbricks_comm_d2h_process(simbricks, cur_ts, msg);

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
    uint64_t proto_ts;

    cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    proto_ts = ts_to_proto(simbricks, cur_ts);

#ifdef DEBUG_PRINTS
    uint64_t sync_ts = SimbricksPcieIfH2DOutNextSync(&simbricks->pcieif);
    if (proto_ts > sync_ts + 1)
        warn_report("simbricks_timer_sync: expected_ts=%lu cur_ts=%lu",
                    sync_ts, proto_ts);

    warn_report("simbricks_timer_sync: ts=%lu pts=%lu npts=%lu", cur_ts,
            proto_ts, sync_ts);
#endif

    while (SimbricksPcieIfH2DOutSync(&simbricks->pcieif, proto_ts));

#ifdef DEBUG_PRINTS
    int64_t now_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    if (cur_ts != now_ts)
        warn_report("simbricks_timer_poll: time advanced from %lu to %lu",
                    cur_ts, now_ts);
#endif
    uint64_t next_sync_pts = SimbricksPcieIfH2DOutNextSync(&simbricks->pcieif);
    uint64_t next_sync_ts = ts_from_proto(simbricks, next_sync_pts);
#ifdef DEBUG_PRINTS
    warn_report("simbricks_timer_sync: next pts=%lu ts=%lu", next_sync_pts,
        next_sync_ts);
#endif
    timer_mod_ns(simbricks->timer_sync, next_sync_ts);
}

static void *simbricks_poll_thread(void *opaque)
{
    SimbricksPciState *simbricks = opaque;
    volatile union SimbricksProtoPcieD2H *msg;

    assert(!simbricks->sync);

    while (!simbricks->stopping) {
        msg = SimbricksPcieIfD2HInPoll(&simbricks->pcieif, 0);
        if (!msg)
            continue;

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

        assert(size <=
            SimbricksPcieIfH2DOutMsgLen(&simbricks->pcieif) - sizeof (*write));
        /* FIXME: this probably only works for LE */
        memcpy((void *) write->data, val, size);

        SimbricksPcieIfH2DOutSend(&simbricks->pcieif, msg,
            SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE);

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

        SimbricksPcieIfH2DOutSend(&simbricks->pcieif, msg,
            SIMBRICKS_PROTO_PCIE_H2D_MSG_READ);

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

        SimbricksPcieIfH2DOutSend(&simbricks->pcieif, msg,
            SIMBRICKS_PROTO_PCIE_H2D_MSG_DEVCTRL);
    }
}


/******************************************************************************/
/* Initialization */

static int simbricks_connect(SimbricksPciState *simbricks, Error **errp)
{
    struct SimbricksProtoPcieDevIntro *d_i = &simbricks->dev_intro;
    struct SimbricksProtoPcieHostIntro host_intro;
    struct SimbricksBaseIfParams params;
    size_t len;
    CPUState *cpu;
    uint8_t *pci_conf = simbricks->pdev.config;
    uint64_t first_sync_ts = 0, first_msg_ts = 0;
    volatile union SimbricksProtoPcieD2H *msg;
    struct SimbricksBaseIf *base_if = &simbricks->pcieif.base;

    if (!simbricks->socket_path) {
        error_setg(errp, "socket path not set but required");
        return 0;
    }

    SimbricksPcieIfDefaultParams(&params);
    params.link_latency = simbricks->pci_latency * 1000;
    params.sync_interval = simbricks->sync_period * 1000;
    params.blocking_conn = true;
    params.sock_path = simbricks->socket_path;
    params.sync_mode = (simbricks->sync ? kSimbricksBaseIfSyncRequired :
        kSimbricksBaseIfSyncDisabled);

    if (SimbricksBaseIfInit(base_if, &params)) {
        error_setg(errp, "SimbricksBaseIfInit failed");
        return 0;
    }

    if (SimbricksBaseIfConnect(base_if)) {
        error_setg(errp, "SimbricksBaseIfConnect failed");
        return 0;
    }

    if (SimbricksBaseIfConnected(base_if)) {
        error_setg(errp, "SimbricksBaseIfConnected indicates unconnected");
        return 0;
    }

    /* prepare & send host intro */
    memset(&host_intro, 0, sizeof(host_intro));
    if (SimbricksBaseIfIntroSend(base_if, &host_intro,
                                 sizeof(host_intro))) {
        error_setg(errp, "SimbricksBaseIfIntroSend failed");
        return 0;
    }

    /* receive device intro */
    len = sizeof(*d_i);
    if (SimbricksBaseIfIntroRecv(base_if, d_i, &len)) {
        error_setg(errp, "SimbricksBaseIfIntroRecv failed");
        return 0;
    }
    if (len != sizeof(*d_i)) {
        error_setg(errp, "rx dev intro: length is not as expected");
        return 0;
    }

    if (simbricks->sync) {
        /* send a first sync */
        if (SimbricksPcieIfH2DOutSync(&simbricks->pcieif, 0)) {
            error_setg(errp, "sending initial sync failed");
            return 0;
        }
        first_sync_ts = SimbricksPcieIfH2DOutNextSync(&simbricks->pcieif);

        /* wait for first message so we know its timestamp */
        do {
            msg = SimbricksPcieIfD2HInPeek(&simbricks->pcieif, 0);
            first_msg_ts = SimbricksPcieIfD2HInTimestamp(&simbricks->pcieif);
        } while (!msg && !first_msg_ts);
    }
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
        timer_mod_ns(simbricks->timer_sync,
            ts_from_proto(simbricks, first_sync_ts));
        simbricks->timer_poll =
            timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_poll, simbricks);
        timer_mod_ns(simbricks->timer_poll,
            ts_from_proto(simbricks, first_msg_ts));
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
    pci_config_set_prog_interface(pci_conf, d_i->pci_progif);

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

    SimbricksBaseIfClose(&simbricks->pcieif.base);
}

static void simbricks_pci_instance_init(Object *obj)
{
}

static Property simbricks_pci_dev_properties[] = {
  DEFINE_PROP_STRING("socket", SimbricksPciState, socket_path),
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
