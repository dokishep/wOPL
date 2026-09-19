#include "irx_imports.h"
#include "../include/iidx_diag.h"

#define MODNAME "iidx_diag"
IRX_ID(MODNAME, 1, 1);

#define REQ_USB_OUT (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define USB_REQ_SET_IDLE 0x0A

static int diag_probe(int devId);
static int diag_connect(int devId);
static int diag_disconnect(int devId);
static void diag_config_set(int result, int count, void *arg);
static void diag_data_cb(int resultCode, int bytes, void *arg);
static void diag_submit_transfer(void);
static void rpc_thread(void *data);

static UsbDriver diag_driver = {
    NULL, NULL, "iidx_diag", diag_probe, diag_connect, diag_disconnect
};

static iidx_diag_data_t diag_info;
static int controlEndp = -1;
static int interruptEndp = -1;
static int transfer_active = 0;
static u8 usb_buf[DIAG_PACKET_MAX + 32] __attribute__((aligned(64)));

static SifRpcDataQueue_t rpc_que __attribute__((aligned(16)));
static SifRpcServerData_t rpc_svr __attribute__((aligned(16)));
static u8 rpc_buf[sizeof(iidx_diag_data_t) + 128] __attribute__((aligned(64)));

static int diag_sema = -1;
static int active_intf_num = 0;

/* Store raw endpoint descriptors so we can re-open on EP switch */
static UsbEndpointDescriptor saved_endpoints[DIAG_MAX_ENDPOINTS];

static void add_log_entry(const char *msg)
{
    strncpy(diag_info.recent_changes[diag_info.log_head], msg, 47);
    diag_info.recent_changes[diag_info.log_head][47] = '\0';
    diag_info.log_head = (diag_info.log_head + 1) % DIAG_LOG_ENTRIES;
}

static int diag_probe(int devId)
{
    UsbDeviceDescriptor *device;
    char msg[48];

    diag_info.change_count++;

    device = (UsbDeviceDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) {
        sprintf(msg, "P#%d: devId=%d desc=NULL", (int)diag_info.change_count, devId);
        add_log_entry(msg);
        return 1;
    }

    sprintf(msg, "P#%d: devId=%d %04X:%04X c=%02X",
            (int)diag_info.change_count, devId,
            device->idVendor, device->idProduct, device->bDeviceClass);
    add_log_entry(msg);

    return 1;
}

static int diag_connect(int devId)
{
    UsbDeviceDescriptor *device;
    UsbConfigDescriptor *config;
    const u8 *p, *end;
    int best_ep_idx = -1;
    char msg[48];

    printf(MODNAME ": connect devId=%d\n", devId);

    PollSema(diag_sema);

    diag_info.devId = devId;
    diag_info.connected = 0;
    diag_info.configured = 0;
    diag_info.num_endpoints = 0;
    diag_info.active_ep_idx = -1;
    diag_info.total_packets = 0;
    diag_info.last_result = 0;
    diag_info.last_bytes = 0;
    memset(diag_info.current_packet, 0, sizeof(diag_info.current_packet));
    memset(diag_info.prev_packet, 0, sizeof(diag_info.prev_packet));
    memset(diag_info.diff_mask, 0, sizeof(diag_info.diff_mask));
    controlEndp = -1;
    interruptEndp = -1;
    transfer_active = 0;

    device = (UsbDeviceDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) {
        sprintf(msg, "C: devId=%d desc=NULL!", devId);
        add_log_entry(msg);
        SignalSema(diag_sema);
        return 1;
    }

    diag_info.connected = 1;
    diag_info.idVendor = device->idVendor;
    diag_info.idProduct = device->idProduct;
    diag_info.bcdDevice = device->bcdDevice;
    diag_info.bDeviceClass = device->bDeviceClass;
    diag_info.bDeviceSubClass = device->bDeviceSubClass;
    diag_info.bDeviceProtocol = device->bDeviceProtocol;
    diag_info.bMaxPacketSize0 = device->bMaxPacketSize0;
    diag_info.bNumConfigurations = device->bNumConfigurations;

    sprintf(msg, "C: %04X:%04X c=%02X cfg=%d",
            device->idVendor, device->idProduct,
            device->bDeviceClass, device->bNumConfigurations);
    add_log_entry(msg);

    controlEndp = sceUsbdOpenPipe(devId, NULL);

    config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, device, USB_DT_CONFIG);
    if (config == NULL)
        config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_CONFIG);

    active_intf_num = 0;
    int best_ep_score = 0;
    int cur_intf_num = 0;
    int cur_intf_class = 0;

    UsbEndpointDescriptor *best_ep_desc = NULL;

    if (config != NULL && config->wTotalLength >= sizeof(UsbConfigDescriptor)) {
        diag_info.bNumInterfaces = config->bNumInterfaces;
        p = (const u8 *)config;
        end = p + config->wTotalLength;

        while (p + 2 <= end) {
            u8 len = p[0];
            u8 type = p[1];
            if (len < 2 || p + len > end)
                break;

            if (type == USB_DT_INTERFACE && len >= sizeof(UsbInterfaceDescriptor)) {
                UsbInterfaceDescriptor *intf = (UsbInterfaceDescriptor *)p;
                diag_info.bInterfaceNumber = intf->bInterfaceNumber;
                diag_info.bInterfaceClass = intf->bInterfaceClass;
                diag_info.bInterfaceSubClass = intf->bInterfaceSubClass;
                diag_info.bInterfaceProtocol = intf->bInterfaceProtocol;
                cur_intf_num = intf->bInterfaceNumber;
                cur_intf_class = intf->bInterfaceClass;
            } else if (type == USB_DT_ENDPOINT && len >= sizeof(UsbEndpointDescriptor)) {
                if (diag_info.num_endpoints < DIAG_MAX_ENDPOINTS) {
                    UsbEndpointDescriptor *ep = (UsbEndpointDescriptor *)p;
                    int idx = diag_info.num_endpoints;
                    u16 pkt = (ep->wMaxPacketSizeHB << 8) | ep->wMaxPacketSizeLB;

                    diag_info.endpoints[idx].bEndpointAddress = ep->bEndpointAddress;
                    diag_info.endpoints[idx].bmAttributes = ep->bmAttributes;
                    diag_info.endpoints[idx].wMaxPacketSize = pkt;
                    diag_info.endpoints[idx].bInterval = ep->bInterval;

                    memcpy(&saved_endpoints[idx], ep, sizeof(UsbEndpointDescriptor));
                    diag_info.num_endpoints++;

                    /* Look for Interrupt IN endpoint, prioritizing HID interface (class 0x03) */
                    if ((ep->bmAttributes & 0x03) == USB_ENDPOINT_XFER_INT &&
                        (ep->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
                        int score = (cur_intf_class == 0x03) ? 2 : 1;
                        if (score > best_ep_score) {
                            best_ep_idx = idx;
                            best_ep_score = score;
                            active_intf_num = cur_intf_num;
                            best_ep_desc = ep;
                        }
                    }
                }
            }
            p += len;
        }
    }

    /* Fallback: If no interrupt IN endpoint found, check for bulk IN endpoint */
    if (best_ep_idx < 0) {
        int i;
        for (i = 0; i < diag_info.num_endpoints; i++) {
            if ((diag_info.endpoints[i].bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
                best_ep_idx = i;
                break;
            }
        }
    }

    if (best_ep_idx >= 0) {
        diag_info.active_ep_idx = best_ep_idx;
        diag_info.active_ep_addr = diag_info.endpoints[best_ep_idx].bEndpointAddress;
        diag_info.active_ep_size = diag_info.endpoints[best_ep_idx].wMaxPacketSize;
        if (diag_info.active_ep_size == 0 || diag_info.active_ep_size > DIAG_PACKET_MAX)
            diag_info.active_ep_size = DIAG_PACKET_MAX;
    } else {
        diag_info.active_ep_idx = -1;
    }

    add_log_entry("Device attached");

    /* Set device configuration. */
    if (config != NULL) {
        sceUsbdSetConfiguration(controlEndp, config->bConfigurationValue, diag_config_set, (void *)(long)devId);
    } else {
        sceUsbdSetConfiguration(controlEndp, 1, diag_config_set, (void *)(long)devId);
    }

    SignalSema(diag_sema);
    return 0;
}

static void diag_config_set(int result, int count, void *arg)
{
    int devId = (int)(long)arg;

    PollSema(diag_sema);

    printf(MODNAME ": config_set result=%d\n", result);

    if (result == USB_RC_OK) {
        diag_info.configured = 1;
        add_log_entry("Config set OK");

        /* Open active endpoint pipe AFTER configuration is active! */
        if (diag_info.active_ep_idx >= 0 && interruptEndp < 0) {
            interruptEndp = sceUsbdOpenPipe(devId, &saved_endpoints[diag_info.active_ep_idx]);
            printf(MODNAME ": opened ep pipe id=%d addr=%02X\n",
                   interruptEndp, diag_info.active_ep_addr);
        }

        if (interruptEndp >= 0) {
            /* Send SET_IDLE 0 to active interface to ensure continuous reporting */
            sceUsbdControlTransfer(controlEndp, REQ_USB_OUT, USB_REQ_SET_IDLE, 0, active_intf_num, 0, NULL, NULL, NULL);

            /* Start transfer loop */
            diag_submit_transfer();
        } else {
            add_log_entry("Failed to open EP pipe!");
        }
    } else {
        char msg[48];
        sprintf(msg, "Config set FAIL (rc=%d)", result);
        add_log_entry(msg);
    }
    }

    SignalSema(diag_sema);
}

static void diag_submit_transfer(void)
{
    int ret;

    if (!diag_info.connected || interruptEndp < 0 || transfer_active)
        return;

    transfer_active = 1;
    ret = sceUsbdInterruptTransfer(interruptEndp,
                                   usb_buf,
                                   diag_info.active_ep_size ? diag_info.active_ep_size : DIAG_PACKET_MAX,
                                   diag_data_cb,
                                   NULL);
    if (ret != USB_RC_OK) {
        transfer_active = 0;
        diag_info.last_result = ret;
    }
}

static void diag_data_cb(int resultCode, int bytes, void *arg)
{
    int i;
    (void)arg;

    transfer_active = 0;
    diag_info.last_result = resultCode;
    diag_info.last_bytes = bytes;

    if (resultCode == USB_RC_OK && bytes > 0) {
        int changed = 0;
        if (bytes > DIAG_PACKET_MAX)
            bytes = DIAG_PACKET_MAX;

        diag_info.total_packets++;

        if (diag_info.total_packets == 1) {
            /* First packet: initialize baseline */
            memcpy(diag_info.current_packet, usb_buf, bytes);
            memcpy(diag_info.prev_packet, usb_buf, bytes);
            memset(diag_info.diff_mask, 0, sizeof(diag_info.diff_mask));
            add_log_entry("First packet received!");
        } else {
            for (i = 0; i < bytes; i++) {
                if (usb_buf[i] != diag_info.current_packet[i]) {
                    char msg[48];
                    diag_info.diff_mask[i] = 1;
                    changed = 1;

                    sprintf(msg, "[#%d] B%02d: %02X->%02X",
                            (int)(diag_info.change_count + 1), i,
                            diag_info.current_packet[i], usb_buf[i]);
                    add_log_entry(msg);
                } else {
                    diag_info.diff_mask[i] = 0;
                }
            }

            if (changed) {
                diag_info.change_count++;
                memcpy(diag_info.prev_packet, diag_info.current_packet, bytes);
                memcpy(diag_info.current_packet, usb_buf, bytes);
            }
        }
    }

    /* Re-submit transfer */
    if (diag_info.connected && interruptEndp >= 0) {
        diag_submit_transfer();
    }
}

static int diag_disconnect(int devId)
{
    printf(MODNAME ": disconnect devId=%d\n", devId);

    PollSema(diag_sema);

    if (interruptEndp >= 0) {
        sceUsbdClosePipe(interruptEndp);
        interruptEndp = -1;
    }
    if (controlEndp >= 0) {
        sceUsbdClosePipe(controlEndp);
        controlEndp = -1;
    }

    transfer_active = 0;
    diag_info.connected = 0;
    diag_info.configured = 0;
    add_log_entry("Device disconnected");

    SignalSema(diag_sema);
    return 0;
}

extern void *QueryLibraryEntryTable(iop_library_t *lib);

typedef int (*sceUsbdGetDiagLog_t)(char *dst, int max_len);
static sceUsbdGetDiagLog_t p_sceUsbdGetDiagLog = NULL;

static void init_usbd_diag_hook(void)
{
    iop_library_t lib;
    struct irx_export_table *table;

    if (p_sceUsbdGetDiagLog != NULL)
        return;

    memset(&lib, 0, sizeof(iop_library_t));
    strncpy(lib.name, "usbd", 8);

    table = (struct irx_export_table *)QueryLibraryEntryTable(&lib);
    if (table != NULL) {
        p_sceUsbdGetDiagLog = (sceUsbdGetDiagLog_t)table->fptrs[17];
        add_log_entry("USBD hook: OK");
    }
}

static u32 last_port_status[2] = {0xFFFFFFFF, 0xFFFFFFFF};

static void diag_check_ohci(void)
{
    volatile u32 *ohci_base = (volatile u32 *)0xBF801600;
    diag_info.ohci_control    = ohci_base[1];  /* 0xBF801604: HcControl */
    diag_info.ohci_cmd_status = ohci_base[2];  /* 0xBF801608: HcCommandStatus */
    diag_info.ohci_int_status = ohci_base[3];  /* 0xBF80160C: HcInterruptStatus */
    diag_info.ohci_rh_status  = ohci_base[20]; /* 0xBF801650: HcRhStatus */
    diag_info.ohci_port_status[0] = ohci_base[21]; /* 0xBF801654: HcRhPortStatus[0] (Port 1) */
    diag_info.ohci_port_status[1] = ohci_base[22]; /* 0xBF801658: HcRhPortStatus[1] (Port 2) */

    for (int p = 0; p < 2; p++) {
        u32 cur = diag_info.ohci_port_status[p];
        if (cur != last_port_status[p]) {
            char msg[48];
            if (last_port_status[p] == 0xFFFFFFFF) {
                sprintf(msg, "P%d init: %08X [C=%d E=%d]",
                        p + 1, (unsigned int)cur, (int)(cur & 1), (int)((cur >> 1) & 1));
                add_log_entry(msg);
            } else if ((cur & 3) != (last_port_status[p] & 3)) {
                /* Only log if Connection (bit 0) or Enable (bit 1) changed! */
                sprintf(msg, "P%d chg: %08X [C=%d E=%d]",
                        p + 1, (unsigned int)cur,
                        (int)(cur & 1),
                        (int)((cur >> 1) & 1));
                add_log_entry(msg);
            }
            last_port_status[p] = cur;
        }
    }

    if (!p_sceUsbdGetDiagLog)
        init_usbd_diag_hook();

    if (p_sceUsbdGetDiagLog) {
        char usbd_msg[48];
        while (p_sceUsbdGetDiagLog(usbd_msg, sizeof(usbd_msg))) {
            add_log_entry(usbd_msg);
        }
    }
}

static void *rpc_sf(int cmd, void *data, int size)
{
    switch (cmd) {
        case IIDX_DIAG_CMD_GET_DATA:
            PollSema(diag_sema);
            diag_check_ohci();
            memcpy(data, &diag_info, sizeof(iidx_diag_data_t));
            SignalSema(diag_sema);
            break;

        case IIDX_DIAG_CMD_SELECT_EP: {
            int ep_idx = *(int *)data;
            PollSema(diag_sema);
            if (ep_idx >= 0 && ep_idx < diag_info.num_endpoints && diag_info.connected) {
                if (interruptEndp >= 0) {
                    sceUsbdClosePipe(interruptEndp);
                    interruptEndp = -1;
                }
                transfer_active = 0;
                diag_info.active_ep_idx = ep_idx;
                diag_info.active_ep_addr = diag_info.endpoints[ep_idx].bEndpointAddress;
                diag_info.active_ep_size = diag_info.endpoints[ep_idx].wMaxPacketSize;
                if (diag_info.active_ep_size == 0 || diag_info.active_ep_size > DIAG_PACKET_MAX)
                    diag_info.active_ep_size = DIAG_PACKET_MAX;

                interruptEndp = sceUsbdOpenPipe(diag_info.devId, &saved_endpoints[ep_idx]);
                diag_submit_transfer();
                add_log_entry("Switched endpoint");
            }
            SignalSema(diag_sema);
            break;
        }

        case IIDX_DIAG_CMD_RESET:
            PollSema(diag_sema);
            if (interruptEndp >= 0) {
                sceUsbdClosePipe(interruptEndp);
                interruptEndp = -1;
            }
            transfer_active = 0;
            if (diag_info.connected && diag_info.active_ep_idx >= 0) {
                interruptEndp = sceUsbdOpenPipe(diag_info.devId, &saved_endpoints[diag_info.active_ep_idx]);
                diag_submit_transfer();
                add_log_entry("Reset & re-opened EP");
            }
            SignalSema(diag_sema);
            break;

        case IIDX_DIAG_CMD_FORCE_RESET_PORT: {
            int port_idx = *(int *)data;
            PollSema(diag_sema);
            if (port_idx == 0 || port_idx == 1) {
                volatile u32 *port_reg = (volatile u32 *)(0xBF801654 + port_idx * 4);
                *port_reg = (1 << 4); /* PORT_RESET */
                char msg[48];
                sprintf(msg, "Force reset Port %d sent", port_idx + 1);
                add_log_entry(msg);
            }
            SignalSema(diag_sema);
            break;
        }

        default:
            break;
    }

    return data;
}

static void rpc_thread(void *data)
{
    (void)data;
    SifInitRpc(0);
    SifSetRpcQueue(&rpc_que, GetThreadId());
    SifRegisterRpc(&rpc_svr, IIDX_DIAG_RPC_ID, rpc_sf, rpc_buf, NULL, NULL, &rpc_que);
    SifRpcLoop(&rpc_que);
}

int _start(int argc, char *argv[])
{
    iop_thread_t th;
    int thid;
    int ret;
    (void)argc;
    (void)argv;

    printf(MODNAME ": starting diagnostic driver\n");

    diag_sema = CreateMutex(IOP_MUTEX_UNLOCKED);
    if (diag_sema < 0) {
        printf(MODNAME ": failed to create mutex\n");
        return MODULE_NO_RESIDENT_END;
    }

    memset(&diag_info, 0, sizeof(diag_info));
    init_usbd_diag_hook();
    add_log_entry("Diagnostic driver started");

    /* Start RPC thread FIRST so EE can always connect and receive diagnostic data */
    th.attr = TH_C;
    th.thread = rpc_thread;
    th.priority = 40;
    th.stacksize = 0x1000;
    th.option = 0;

    thid = CreateThread(&th);
    if (thid > 0) {
        StartThread(thid, NULL);
    } else {
        printf(MODNAME ": failed to create RPC thread\n");
        return MODULE_NO_RESIDENT_END;
    }

    ret = sceUsbdRegisterLdd(&diag_driver);
    if (ret != USB_RC_OK) {
        printf(MODNAME ": failed to register USBD driver (ret=%d)\n", ret);
        add_log_entry("Register USBD driver FAILED");
    } else {
        add_log_entry("USBD driver registered OK");
    }

    return MODULE_RESIDENT_END;
}
