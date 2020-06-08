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
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "chardev/char-fe.h"
#include "qapi/error.h"
#include "qapi/visitor.h"

#include <cosim_pcie_proto.h>

#define TYPE_PCI_COSIM_DEVICE "cosim-pci"
#define COSIM_PCI(obj)        OBJECT_CHECK(CosimPciState, obj, TYPE_PCI_COSIM_DEVICE)

typedef struct CosimPciBarInfo {
    struct CosimPciState *cosim;
    uint8_t index;
    bool is_io;
} CosimPciBarInfo;



typedef struct CosimPciState {
    PCIDevice pdev;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    CharBackend sim_chr;

    MemoryRegion mmio_bars[6];
    CosimPciBarInfo bar_info[6];

    struct cosim_pcie_proto_dev_intro dev_intro;

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
    uint64_t h2d_nextid;

    /* communication bewteen main io thread and worker thread
     * (protected by thr_mutex). */
    struct CosimPciRequest {
        uint64_t addr;          /* address for request */
        uint64_t value;         /* value read/to be written */
        unsigned size;          /* size for request */
        uint8_t bar;            /* bar index */
        bool write;             /* write if true, read otherwise */
        bool processing;        /* request is being processed by device */
        volatile bool requested;/* request waiting to be started */
    } req;
} CosimPciState;

static void panic(const char *msg, ...) __attribute__((noreturn));

static void panic(const char *msg, ...)
{
    va_list ap;

    va_start(ap, msg);
    error_vreport(msg, ap);
    va_end(ap);

    abort();
}

#if 0
static void edu_raise_irq(CosimPciState *cosim, uint32_t val)
{
    cosim->irq_status |= val;
    if (cosim->irq_status) {
        if (edu_msi_enabled(cosim)) {
            msi_notify(&cosim->pdev, 0);
        } else {
            pci_set_irq(&cosim->pdev, 1);
        }
    }
}

static void edu_lower_irq(CosimPciState *cosim, uint32_t val)
{
    cosim->irq_status &= ~val;

    if (!cosim->irq_status && !edu_msi_enabled(cosim)) {
        pci_set_irq(&cosim->pdev, 0);
    }
}
#endif

/******************************************************************************/
/* Worker thread */

static volatile union cosim_pcie_proto_h2d *cosim_comm_h2d_alloc(
        CosimPciState *cosim)
{
    uint8_t *pos;
    volatile union cosim_pcie_proto_h2d *msg;

    pos = cosim->h2d_base + (cosim->h2d_elen * cosim->h2d_pos);
    msg = (volatile union cosim_pcie_proto_h2d *) pos;

    if ((msg->dummy.own_type & COSIM_PCIE_PROTO_H2D_OWN_MASK) !=
            COSIM_PCIE_PROTO_H2D_OWN_HOST)
    {
        /* this should never happen as we synchronously submit requests
         * one-by-one. */
        panic("cosim_comm_h2d_alloc: ran into non-owned entry in h2d queue");
    }

    /* advance position to next entry */
    cosim->h2d_pos = (cosim->h2d_pos + 1) % cosim->h2d_nentries;

    return msg;
}

static void cosim_comm_d2h_dma_read(CosimPciState *cosim,
        volatile struct cosim_pcie_proto_d2h_read *read)
{
    volatile union cosim_pcie_proto_h2d *h2d;
    volatile struct cosim_pcie_proto_h2d_readcomp *rc;

    /* allocate completion */
    h2d = cosim_comm_h2d_alloc(cosim);
    rc = &h2d->readcomp;

    assert(read->len <= cosim->h2d_elen - sizeof (*rc));

    /* perform dma read */
    //qemu_mutex_lock_iothread();
    pci_dma_read(&cosim->pdev, read->offset, (void *) rc->data, read->len);
    //qemu_mutex_unlock_iothread();

    /* return completion */
    rc->req_id = read->req_id;
    smp_wmb();
    rc->own_type = COSIM_PCIE_PROTO_H2D_MSG_READCOMP |
        COSIM_PCIE_PROTO_H2D_OWN_DEV;
}

static void cosim_comm_d2h_dma_write(CosimPciState *cosim,
        volatile struct cosim_pcie_proto_d2h_write *write)
{
    volatile union cosim_pcie_proto_h2d *h2d;
    volatile struct cosim_pcie_proto_h2d_writecomp *wc;

    /* allocate completion */
    h2d = cosim_comm_h2d_alloc(cosim);
    wc = &h2d->writecomp;

    /* perform dma write */
    //qemu_mutex_lock_iothread();
    pci_dma_write(&cosim->pdev, write->offset, (void *) write->data,
            write->len);
    //qemu_mutex_unlock_iothread();

    /* return completion */
    wc->req_id = write->req_id;
    smp_wmb();
    wc->own_type = COSIM_PCIE_PROTO_H2D_MSG_WRITECOMP |
        COSIM_PCIE_PROTO_H2D_OWN_DEV;
}

static void cosim_comm_d2h_interrupt(CosimPciState *cosim,
        volatile struct cosim_pcie_proto_d2h_interrupt *interrupt)
{
    warn_report("cosim_comm_d2h_interrupt: TODO");
}

static void cosim_comm_d2h_rwcomp(CosimPciState *cosim, uint64_t req_id,
        const void *data)
{
    qemu_mutex_lock(&cosim->thr_mutex);
    if (!cosim->req.processing) {
        panic("cosim_comm_d2h_rwcomp: no request currently processing");
    }

    if (cosim->h2d_nextid != req_id) {
        panic("cosim_comm_d2h_rwcomp: unexpected request completion "
                "(exp=%lu, got=%lu)\n", cosim->h2d_nextid, req_id);
    }

    if ((data == NULL) != cosim->req.write) {
        panic("cosim_comm_d2h_rwcomp: completion type does not match request "
                "type");
    }

    /* copy read value from message */
    if (data) {
        cosim->req.value = 0;
        memcpy(&cosim->req.value, data, cosim->req.size);
    }

    cosim->req.processing = false;

    /* signal waiting main thread */
    qemu_cond_signal(&cosim->thr_cond);
    qemu_mutex_unlock(&cosim->thr_mutex);
}

/* poll device-to-host queue */
static void cosim_comm_poll_d2h(CosimPciState *cosim)
{
    uint8_t *pos;
    uint8_t type;
    volatile union cosim_pcie_proto_d2h *msg;

    while (1) {
        pos = cosim->d2h_base + (cosim->d2h_elen * cosim->d2h_pos);
        msg = (volatile union cosim_pcie_proto_d2h *) pos;

        /* check if this message is ready for us */
        if ((msg->dummy.own_type & COSIM_PCIE_PROTO_D2H_OWN_MASK) !=
            COSIM_PCIE_PROTO_D2H_OWN_HOST)
        {
            break;
        }

        smp_rmb();

        type = msg->dummy.own_type & COSIM_PCIE_PROTO_D2H_MSG_MASK;
        switch (type) {
            case COSIM_PCIE_PROTO_D2H_MSG_SYNC:
                /* nop */
                break;

            case COSIM_PCIE_PROTO_D2H_MSG_READ:
                cosim_comm_d2h_dma_read(cosim, &msg->read);
                break;

            case COSIM_PCIE_PROTO_D2H_MSG_WRITE:
                cosim_comm_d2h_dma_write(cosim, &msg->write);
                break;

            case COSIM_PCIE_PROTO_D2H_MSG_INTERRUPT:
                cosim_comm_d2h_interrupt(cosim, &msg->interrupt);
                break;

            case COSIM_PCIE_PROTO_D2H_MSG_READCOMP:
                cosim_comm_d2h_rwcomp(cosim, msg->readcomp.req_id,
                        (void *) msg->readcomp.data);
                break;

            case COSIM_PCIE_PROTO_D2H_MSG_WRITECOMP:
                cosim_comm_d2h_rwcomp(cosim, msg->writecomp.req_id, NULL);
                break;

            default:
                panic("cosim_comm_poll_d2h: unhandled type");
        }

        smp_wmb();

        /* mark message as done */
        msg->dummy.own_type = (msg->dummy.own_type & COSIM_PCIE_PROTO_D2H_MSG_MASK) |
            COSIM_PCIE_PROTO_D2H_OWN_DEV;

        /* advance position to next entry */
        cosim->d2h_pos = (cosim->d2h_pos + 1) % cosim->d2h_nentries;
    }
}

/* handle requests from main thread and submit to host-to-device queue */
static void cosim_comm_request(CosimPciState *cosim)
{
    volatile union cosim_pcie_proto_h2d *msg;
    volatile struct cosim_pcie_proto_h2d_read *read;
    volatile struct cosim_pcie_proto_h2d_write *write;

    /* nothing new */
    if (!cosim->req.requested) {
        return;
    }

    qemu_mutex_lock(&cosim->thr_mutex);
    if (!cosim->req.requested) {
        warn_report("cosim_comm_request: saw requested flag, but gone after "
                "lock, this is probably a bug");
        goto out;
    }

    if (cosim->req.processing) {
        /* this should never happen as we synchronously submit requests. */
        panic("cosim_comm_request: received request while processing another");
    }

    /* allocate host-to-device queue entry */
    msg = cosim_comm_h2d_alloc(cosim);

    /* prepare operation */
    if (cosim->req.write) {
        write = &msg->write;

        write->req_id = ++cosim->h2d_nextid;
        write->offset = cosim->req.addr;
        write->len = cosim->req.size;
        write->bar = cosim->req.bar;

        assert(cosim->req.size <= cosim->h2d_elen - sizeof (*write));
        /* FIXME: this probably only works for LE */
        memcpy((void *) write->data, &cosim->req.value, cosim->req.size);

        smp_wmb(); /* barrier to make sure earlier fields are written before
                      handing over ownership */

        write->own_type = COSIM_PCIE_PROTO_H2D_MSG_WRITE | COSIM_PCIE_PROTO_H2D_OWN_DEV;
    } else {
        read = &msg->read;

        read->req_id = ++cosim->h2d_nextid;
        read->offset = cosim->req.addr;
        read->len = cosim->req.size;
        read->bar = cosim->req.bar;

        smp_wmb(); /* barrier to make sure earlier fields are written before
                      handing over ownership */

        read->own_type = COSIM_PCIE_PROTO_H2D_MSG_READ | COSIM_PCIE_PROTO_H2D_OWN_DEV;
    }

    /* start processing request */
    cosim->req.processing = true;
    cosim->req.requested = false;

out:
    qemu_mutex_unlock(&cosim->thr_mutex);
}

static void *cosim_comm_thread(void *opaque)
{
    CosimPciState *cosim = opaque;

    while (!cosim->stopping) {
        cosim_comm_poll_d2h(cosim);
        cosim_comm_request(cosim);
#if 0
        uint32_t val, ret = 1;

        qemu_mutex_lock(&cosim->thr_mutex);
        while ((atomic_read(&cosim->status) & EDU_STATUS_COMPUTING) == 0 &&
                        !cosim->stopping) {
            qemu_cond_wait(&cosim->thr_cond, &cosim->thr_mutex);
        }

        if (cosim->stopping) {
            qemu_mutex_unlock(&cosim->thr_mutex);
            break;
        }

        val = cosim->fact;
        qemu_mutex_unlock(&cosim->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&cosim->thr_mutex);
        cosim->fact = ret;
        qemu_mutex_unlock(&cosim->thr_mutex);
        atomic_and(&cosim->status, ~EDU_STATUS_COMPUTING);

        if (atomic_read(&cosim->status) & EDU_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            edu_raise_irq(cosim, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
#endif
    }

    return NULL;
}

/******************************************************************************/
/* MMIO interface */

/* submit a bar read or write to the worker thread and wait for it to
 * complete */
static void cosim_mmio_rw(CosimPciState *cosim, uint8_t bar, hwaddr addr,
        unsigned size, uint64_t *val, bool write)
{
    qemu_mutex_lock(&cosim->thr_mutex);

    /* we only ever submit one at a time, so there should not be any ongoing
     * requests */
    assert(!cosim->req.requested && !cosim->req.processing);

    cosim->req.addr = addr;
    cosim->req.size = size;
    cosim->req.bar = bar;
    cosim->req.write = write;

    if (write) {
        cosim->req.value = *val;
    }
    cosim->req.requested = true;

    /* wait for operation to finish */
    while (cosim->req.requested || cosim->req.processing) {
        qemu_cond_wait(&cosim->thr_cond, &cosim->thr_mutex);
    }

    if (!write) {
        *val = cosim->req.value;
    }
    qemu_mutex_unlock(&cosim->thr_mutex);
}

static uint64_t cosim_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CosimPciBarInfo *bar = opaque;
    CosimPciState *cosim = bar->cosim;
    uint64_t ret = 0;

    cosim_mmio_rw(cosim, bar->index, addr, size, &ret, false);

    return ret;
}

static void cosim_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    CosimPciBarInfo *bar = opaque;
    CosimPciState *cosim = bar->cosim;

    cosim_mmio_rw(cosim, bar->index, addr, size, &val, true);
}

static const MemoryRegionOps cosim_mmio_ops = {
    .read = cosim_mmio_read,
    .write = cosim_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};



/******************************************************************************/
/* Initialization */

static int cosim_connect(CosimPciState *cosim, Error **errp)
{
    struct cosim_pcie_proto_dev_intro *d_i = &cosim->dev_intro;
    struct cosim_pcie_proto_host_intro host_intro;
    struct stat statbuf;
    int ret, off, len;
    void *p;
    uint8_t *pci_conf = cosim->pdev.config;

    if (!qemu_chr_fe_backend_connected(&cosim->sim_chr)) {
        error_setg(errp, "no simulator chardev specified");
        return 0;
    }

    /* send host intro */
    memset(&host_intro, 0, sizeof(host_intro));

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
    cosim->h2d_nextid = 0;

    cosim->req.processing = false;
    cosim->req.requested = false;

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

        if (!(flags & COSIM_PCIE_PROTO_BAR_IO)) {
            /* memory bar */
            cosim->bar_info[i].is_io = false;

            attr = PCI_BASE_ADDRESS_SPACE_MEMORY;
            if ((flags & COSIM_PCIE_PROTO_BAR_64)) {
                attr |= PCI_BASE_ADDRESS_MEM_TYPE_64;
            } else {
                attr |= PCI_BASE_ADDRESS_MEM_TYPE_32;
            }
            if ((flags & COSIM_PCIE_PROTO_BAR_PF)) {
                attr |= PCI_BASE_ADDRESS_MEM_PREFETCH;
            }
            label = "cosim-bar-mmio";
        } else {
            /* I/O port bar */
            cosim->bar_info[i].is_io = true;

            label = "cosim-bar-ioport";
            attr = PCI_BASE_ADDRESS_SPACE_IO;
        }

        memory_region_init_io(&cosim->mmio_bars[i], OBJECT(cosim),
                &cosim_mmio_ops, &cosim->bar_info[i], label, len);
        pci_register_bar(pdev, i, attr, &cosim->mmio_bars[i]);
    }

    //timer_init_ms(&cosim->dma_timer, QEMU_CLOCK_VIRTUAL, edu_dma_timer, cosim);

    qemu_mutex_init(&cosim->thr_mutex);
    qemu_cond_init(&cosim->thr_cond);
    qemu_thread_create(&cosim->thread, "cosim", cosim_comm_thread,
                       cosim, QEMU_THREAD_JOINABLE);
}

static void pci_cosim_uninit(PCIDevice *pdev)
{
    CosimPciState *cosim = COSIM_PCI(pdev);

    qemu_mutex_lock(&cosim->thr_mutex);
    cosim->stopping = true;
    qemu_mutex_unlock(&cosim->thr_mutex);
    qemu_thread_join(&cosim->thread);

    qemu_cond_destroy(&cosim->thr_cond);
    qemu_mutex_destroy(&cosim->thr_mutex);

    if (cosim->dev_intro.pci_msi_nvecs > 0)
        msi_uninit(pdev);
}

static void cosim_pci_instance_init(Object *obj)
{
    /*CosimPciState *cosim = COSIM_PCI(obj);*/
}

static Property cosim_pci_dev_properties[] = {
  DEFINE_PROP_CHR("chardev", CosimPciState, sim_chr),
  DEFINE_PROP_END_OF_LIST(),
};

static void cosim_pci_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    device_class_set_props(dc, cosim_pci_dev_properties);

    k->realize = pci_cosim_realize;
    k->exit = pci_cosim_uninit;

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
