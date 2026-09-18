#include "types.h"
#include "loadcore.h"
#include "stdio.h"
#include "sifrpc.h"
#include "sysclib.h"
#include "usbd.h"
#include "usbd_macro.h"
#include "thbase.h"
#include "thsemap.h"
#include "iidxhid.h"
#include "sys_utils.h"
#include "padmacro.h"
#include "pademu.h"
#include "ds34common.h"

#define MODNAME "IIDXHID"

#ifdef DEBUG
#define DPRINTF(format, args...) \
    printf(MODNAME ": " format, ##args)
#else
#define DPRINTF(args...)
#endif

#define REQ_USB_OUT (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define REQ_USB_IN  (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE)

static iidx_device iidx_pad[IIDX_MAX_PADS];
static struct pad_funcs padf[IIDX_MAX_PADS];
static int usb_resultCode;

static int iidxhid_probe(int devId);
static int iidxhid_connect(int devId);
static int iidxhid_disconnect(int devId);
static void iidx_release(int pad);
static void iidx_config_set(int result, int count, void *arg);
static void usb_data_cb(int resultCode, int bytes, void *arg);
static void usb_cmd_cb(int resultCode, int bytes, void *arg);

static int iidxhid_get_model(struct pad_funcs *pf, int port);
static int iidxhid_get_data(struct pad_funcs *pf, u8 *dst, int size, int port);
static void iidxhid_set_rumble(struct pad_funcs *pf, u8 lrum, u8 rrum);
static void iidxhid_set_mode(struct pad_funcs *pf, int mode, int lock);

static UsbDriver iidxhid_driver = {NULL, NULL, "iidxhid", iidxhid_probe, iidxhid_connect, iidxhid_disconnect};

static unsigned int timeout(void *arg)
{
    int sema = (int)arg;
    iSignalSema(sema);
    return 0;
}

static void TransferWait(int sema)
{
    iop_sys_clock_t cmd_timeout;

    cmd_timeout.lo = 200000;
    cmd_timeout.hi = 0;

    if (SetAlarm(&cmd_timeout, timeout, (void *)sema) == 0) {
        WaitSema(sema);
        CancelAlarm(timeout, NULL);
    }
}


static void iidx_process_turntable(iidx_device *pad, u16 x_raw, u8 *up_out, u8 *down_out)
{
    u16 low_press = pad->config.low_press;
    u16 low_release = pad->config.low_release;
    u16 high_press = pad->config.high_press;
    u16 high_release = pad->config.high_release;

    /* Process LOW side (UP by default) */
    if (!pad->tt_up) {
        if (x_raw < low_press) {
            pad->tt_up = 1;
            pad->tt_down = 0; /* Guarantee mutual exclusion */
        }
    } else {
        if (x_raw > low_release) {
            pad->tt_up = 0;
        }
    }

    /* Process HIGH side (DOWN by default) */
    if (!pad->tt_down) {
        if (x_raw > high_press) {
            pad->tt_down = 1;
            pad->tt_up = 0; /* Guarantee mutual exclusion */
        }
    } else {
        if (x_raw < high_release) {
            pad->tt_down = 0;
        }
    }

    /* Inversion option */
    if (pad->config.invert_turntable) {
        *up_out = pad->tt_down;
        *down_out = pad->tt_up;
    } else {
        *up_out = pad->tt_up;
        *down_out = pad->tt_down;
    }
}

static void iidx_readReport(u8 *buf, iidx_device *pad)
{
    u16 x_raw = 0x7FFF;
    u32 hid_buttons = 0;
    u8 up = 0, down = 0;

    /*
     * Non-blocking dynamic layout auto-detection based on turntable resting center value (~32767).
     * Once detected, layout_detected is locked to 1.
     */
    if (!pad->layout_detected) {
        u16 c0 = (u16)(buf[0] | (buf[1] << 8));
        u16 c1 = (u16)(buf[1] | (buf[2] << 8));
        u16 c4 = (u16)(buf[4] | (buf[5] << 8));
        u16 c5 = (u16)(buf[5] | (buf[6] << 8));

        if (c0 >= 24000 && c0 <= 42000) {
            pad->x_byte_offset = 0;
            pad->btn_byte_offset = 12;
            pad->layout_detected = 1;
            DPRINTF("Auto-detected layout 1: X at offset 0, Buttons at offset 12\n");
        } else if (c1 >= 24000 && c1 <= 42000) {
            pad->x_byte_offset = 1;
            pad->btn_byte_offset = 13;
            pad->layout_detected = 1;
            DPRINTF("Auto-detected layout 2: X at offset 1, Buttons at offset 13\n");
        } else if (c4 >= 24000 && c4 <= 42000) {
            pad->x_byte_offset = 4;
            pad->btn_byte_offset = 0;
            pad->layout_detected = 1;
            DPRINTF("Auto-detected layout 3: X at offset 4, Buttons at offset 0\n");
        } else if (c5 >= 24000 && c5 <= 42000) {
            pad->x_byte_offset = 5;
            pad->btn_byte_offset = 1;
            pad->layout_detected = 1;
            DPRINTF("Auto-detected layout 4: X at offset 5, Buttons at offset 1\n");
        }
    }

    if (pad->layout_detected) {
        int xo = pad->x_byte_offset;
        int bo = pad->btn_byte_offset;
        x_raw = (u16)(buf[xo] | (buf[xo + 1] << 8));
        hid_buttons = (u32)(buf[bo] | (buf[bo + 1] << 8) | (buf[bo + 2] << 16) | (buf[bo + 3] << 24));
    } else {
        /* Fallback if controller booted while turntable was actively held away from center */
        if (buf[0] != 0 && buf[0] < 8) {
            x_raw = (u16)(buf[1] | (buf[2] << 8));
            hid_buttons = (u32)(buf[13] | (buf[14] << 8) | (buf[15] << 16) | (buf[16] << 24));
        } else {
            x_raw = (u16)(buf[0] | (buf[1] << 8));
            hid_buttons = (u32)(buf[12] | (buf[13] << 8) | (buf[14] << 16) | (buf[15] << 24));
        }
    }

    pad->last_x = x_raw;
    iidx_process_turntable(pad, x_raw, &up, &down);

    /* Rate-limited debug: only log on state transitions */
    if (hid_buttons != pad->last_raw_buttons || up != pad->last_debug_up || down != pad->last_debug_down) {
        DPRINTF("IIDX Input: rawX=%u UP=%d DOWN=%d btns=0x%08X [K1:%d K2:%d K3:%d K4:%d K5:%d K6:%d K7:%d SEL:%d ST:%d]\n",
                x_raw, up, down, (unsigned int)hid_buttons,
                (hid_buttons >> 0) & 1,
                (hid_buttons >> 1) & 1,
                (hid_buttons >> 2) & 1,
                (hid_buttons >> 3) & 1,
                (hid_buttons >> 4) & 1,
                (hid_buttons >> 5) & 1,
                (hid_buttons >> 6) & 1,
                (hid_buttons >> 7) & 1,
                (hid_buttons >> 9) & 1);
        pad->last_raw_buttons = hid_buttons;
        pad->last_debug_up = up;
        pad->last_debug_down = down;
    }

    /* Start with all buttons released (PS2 SIO2 is active-low: 1 = released, 0 = pressed) */
    uint16_t buttons_state = 0xFFFF;

    /*
     * Exact Required Mapping:
     * Physical IIDX control | HID input | PS2 input
     * IIDX key 1            | Button 1  | SQUARE
     * IIDX key 2            | Button 2  | L1
     * IIDX key 3            | Button 3  | CROSS
     * IIDX key 4            | Button 4  | R1
     * IIDX key 5            | Button 5  | CIRCLE
     * IIDX key 6            | Button 6  | R2
     * IIDX key 7            | Button 7  | LEFT
     * SELECT                | Button 8  | SELECT
     * START                 | Button 10 | START
     * TT one direction      | X low     | UP
     * TT opposite direction | X high    | DOWN
     */
    if (hid_buttons & (1 << 0))
        buttons_state &= ~(1 << DS2BtnBit_Square);

    if (hid_buttons & (1 << 1))
        buttons_state &= ~(1 << DS2BtnBit_L1);

    if (hid_buttons & (1 << 2))
        buttons_state &= ~(1 << DS2BtnBit_Cross);

    if (hid_buttons & (1 << 3))
        buttons_state &= ~(1 << DS2BtnBit_R1);

    if (hid_buttons & (1 << 4))
        buttons_state &= ~(1 << DS2BtnBit_Circle);

    if (hid_buttons & (1 << 5))
        buttons_state &= ~(1 << DS2BtnBit_R2);

    if (hid_buttons & (1 << 6))
        buttons_state &= ~(1 << DS2BtnBit_Left);

    if (hid_buttons & (1 << 7))
        buttons_state &= ~(1 << DS2BtnBit_Select);

    if (hid_buttons & (1 << 9))
        buttons_state &= ~(1 << DS2BtnBit_Start);

    if (up)
        buttons_state &= ~(1 << DS2BtnBit_Up);

    if (down)
        buttons_state &= ~(1 << DS2BtnBit_Down);

    pad->ds2.nButtonState = buttons_state;

    /* Neutral analog sticks */
    pad->ds2.RightStickX = 128;
    pad->ds2.RightStickY = 128;
    pad->ds2.LeftStickX = 128;
    pad->ds2.LeftStickY = 128;

    /* Pressure values (255 when pressed, 0 when released) */
    pad->ds2.PressureSquare   = (hid_buttons & (1 << 0)) ? 255 : 0;
    pad->ds2.PressureL1       = (hid_buttons & (1 << 1)) ? 255 : 0;
    pad->ds2.PressureCross    = (hid_buttons & (1 << 2)) ? 255 : 0;
    pad->ds2.PressureR1       = (hid_buttons & (1 << 3)) ? 255 : 0;
    pad->ds2.PressureCircle   = (hid_buttons & (1 << 4)) ? 255 : 0;
    pad->ds2.PressureR2       = (hid_buttons & (1 << 5)) ? 255 : 0;
    pad->ds2.PressureLeft     = (hid_buttons & (1 << 6)) ? 255 : 0;
    pad->ds2.PressureUp       = up ? 255 : 0;
    pad->ds2.PressureDown     = down ? 255 : 0;
    pad->ds2.PressureRight    = 0;
    pad->ds2.PressureTriangle = 0;
    pad->ds2.PressureL2       = 0;
}

static int iidxhid_probe(int devId)
{
    UsbDeviceDescriptor *device = NULL;
    UsbConfigDescriptor *config = NULL;
    UsbInterfaceDescriptor *interface = NULL;
    UsbEndpointDescriptor *endpoint = NULL;
    int epCount;

    device = (UsbDeviceDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) {
        return 0;
    }

    /* Exclude Sony devices (handled by ds34usb / ds34bt) */
    if (device->idVendor == SONY_VID || device->idVendor == DS34_VID) {
        return 0;
    }

    config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, device, USB_DT_CONFIG);
    if (config == NULL) {
        return 0;
    }

    interface = (UsbInterfaceDescriptor *)((char *)config + config->bLength);
    if (interface == NULL) {
        return 0;
    }

    /* Must be USB HID class (0x03) */
    if (interface->bInterfaceClass != 0x03) {
        return 0;
    }

    /* Reject keyboards (protocol 1) and mice (protocol 2) */
    if (interface->bInterfaceSubClass == 1 &&
        (interface->bInterfaceProtocol == 1 || interface->bInterfaceProtocol == 2)) {
        return 0;
    }

    /* Find Interrupt IN endpoint */
    endpoint = (UsbEndpointDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_ENDPOINT);
    epCount = interface->bNumEndpoints;
    while (endpoint != NULL && epCount > 0) {
        if (endpoint->bmAttributes == USB_ENDPOINT_XFER_INT &&
            (endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
            DPRINTF("IIDX HID probe matched! devId=%d VID=%04X PID=%04X epAddr=%02X\n",
                    devId, device->idVendor, device->idProduct, endpoint->bEndpointAddress);
            return 1;
        }
        endpoint = (UsbEndpointDescriptor *)((char *)endpoint + endpoint->bLength);
        epCount--;
    }

    return 0;
}

static int iidxhid_connect(int devId)
{
    int pad;
    UsbDeviceDescriptor *device;
    UsbConfigDescriptor *config;
    UsbInterfaceDescriptor *interface;
    UsbEndpointDescriptor *endpoint;
    int epCount;

    DPRINTF("connect: devId=%i\n", devId);

    for (pad = 0; pad < IIDX_MAX_PADS; pad++) {
        if (iidx_pad[pad].devId == -1 && iidx_pad[pad].enabled)
            break;
    }

    if (pad >= IIDX_MAX_PADS) {
        DPRINTF("connect: no available pad slot!\n");
        return 1;
    }

    PollSema(iidx_pad[pad].sema);

    iidx_pad[pad].devId = devId;
    iidx_pad[pad].status = IIDXHID_STATE_AUTHORIZED;
    iidx_pad[pad].controlEndp = UsbOpenEndpoint(devId, NULL);

    device = (UsbDeviceDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, device, USB_DT_CONFIG);
    interface = (UsbInterfaceDescriptor *)((char *)config + config->bLength);
    iidx_pad[pad].interfaceNumber = interface->bInterfaceNumber;

    endpoint = (UsbEndpointDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_ENDPOINT);
    epCount = interface->bNumEndpoints;

    do {
        if (endpoint->bmAttributes == USB_ENDPOINT_XFER_INT) {
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN && iidx_pad[pad].interruptEndp < 0) {
                iidx_pad[pad].interruptEndp = sceUsbdOpenPipe(devId, endpoint);
                DPRINTF("Registered interrupt IN endpoint id=%d addr=%02X\n",
                        iidx_pad[pad].interruptEndp, endpoint->bEndpointAddress);
            }
        }
        endpoint = (UsbEndpointDescriptor *)((char *)endpoint + endpoint->bLength);
    } while (--epCount > 0);

    if (iidx_pad[pad].interruptEndp < 0) {
        DPRINTF("connect: failed to open interrupt endpoint!\n");
        iidx_release(pad);
        return 1;
    }

    iidx_pad[pad].status |= IIDXHID_STATE_CONNECTED;
    sceUsbdSetConfiguration(iidx_pad[pad].controlEndp, config->bConfigurationValue, iidx_config_set, (void *)pad);
    SignalSema(iidx_pad[pad].sema);

    return 0;
}

static void iidx_config_set(int result, int count, void *arg)
{
    int pad = (int)(long)arg;

    PollSema(iidx_pad[pad].sema);

    iidx_pad[pad].status |= IIDXHID_STATE_CONFIGURED;

    /* Initialize controller state to all buttons released */
    iidx_pad[pad].ds2.nButtonState = 0xFFFF;
    iidx_pad[pad].ds2.RightStickX = 128;
    iidx_pad[pad].ds2.RightStickY = 128;
    iidx_pad[pad].ds2.LeftStickX = 128;
    iidx_pad[pad].ds2.LeftStickY = 128;

    iidx_pad[pad].status |= IIDXHID_STATE_RUNNING;
    SignalSema(iidx_pad[pad].sema);

    pademu_connect(&padf[pad]);
}

static int iidxhid_disconnect(int devId)
{
    int pad;

    DPRINTF("disconnect: devId=%i\n", devId);

    for (pad = 0; pad < IIDX_MAX_PADS; pad++) {
        if (iidx_pad[pad].devId == devId)
            break;
    }

    if (pad < IIDX_MAX_PADS) {
        iidx_release(pad);
        pademu_disconnect(&padf[pad]);
    }

    return 0;
}

static void iidx_release(int pad)
{
    PollSema(iidx_pad[pad].sema);

    if (iidx_pad[pad].interruptEndp >= 0)
        sceUsbdClosePipe(iidx_pad[pad].interruptEndp);

    iidx_pad[pad].controlEndp = -1;
    iidx_pad[pad].interruptEndp = -1;
    iidx_pad[pad].devId = -1;
    iidx_pad[pad].status = IIDXHID_STATE_DISCONNECTED;
    iidx_pad[pad].layout_detected = 0;
    iidx_pad[pad].tt_up = 0;
    iidx_pad[pad].tt_down = 0;

    SignalSema(iidx_pad[pad].sema);
}

static void usb_data_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)(long)arg;
    usb_resultCode = resultCode;
    SignalSema(iidx_pad[pad].sema);
}

static void usb_cmd_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)(long)arg;
    SignalSema(iidx_pad[pad].cmd_sema);
}

static int iidxhid_get_data(struct pad_funcs *pf, u8 *dst, int size, int port)
{
    iidx_device *pad = pf->priv;
    int ret = 0;

    if (!(pad->status & IIDXHID_STATE_RUNNING) || pad->interruptEndp < 0) {
        memcpy(dst, pad->data, size);
        return pad->analog_btn & 1;
    }

    WaitSema(pad->sema);
    PollSema(pad->sema);

    ret = sceUsbdInterruptTransfer(pad->interruptEndp, pad->usb_buf, sizeof(pad->usb_buf), usb_data_cb, (void *)(long)pad->pad_idx);

    if (ret == USB_RC_OK) {
        TransferWait(pad->sema);
        if (!usb_resultCode)
            iidx_readReport(pad->usb_buf, pad);

        usb_resultCode = 1;
    }

    memcpy(dst, pad->data, size);
    ret = pad->analog_btn & 1;

    SignalSema(pad->sema);
    return ret;
}

static void iidxhid_set_rumble(struct pad_funcs *pf, u8 lrum, u8 rrum)
{
    (void)pf;
    (void)lrum;
    (void)rrum;
}

static void iidxhid_set_mode(struct pad_funcs *pf, int mode, int lock)
{
    iidx_device *pad = pf->priv;
    WaitSema(pad->sema);
    if (lock == 3)
        pad->analog_btn = 3;
    else
        pad->analog_btn = mode;
    SignalSema(pad->sema);
}

static int iidxhid_get_model(struct pad_funcs *pf, int port)
{
    (void)pf;
    (void)port;
    return MODEL_PS2;
}

int iidxhid_init(u8 pad_enable, u8 pad_options)
{
    int pad;
    iop_sema_t sema;

    sema.attr = 1;
    sema.initial = 1;
    sema.max = 1;

    for (pad = 0; pad < IIDX_MAX_PADS; pad++) {
        iidx_pad[pad].pad_idx = pad;
        iidx_pad[pad].sema = CreateSema(&sema);
        iidx_pad[pad].cmd_sema = CreateSema(&sema);
        iidx_pad[pad].devId = -1;
        iidx_pad[pad].status = IIDXHID_STATE_DISCONNECTED;
        iidx_pad[pad].controlEndp = -1;
        iidx_pad[pad].interruptEndp = -1;
        iidx_pad[pad].analog_btn = 0;
        iidx_pad[pad].enabled = (pad_enable >> pad) & 1;

        /* Default thresholds and hysteresis */
        iidx_pad[pad].config.low_press = IIDX_TT_LOW_PRESS_DEFAULT;
        iidx_pad[pad].config.low_release = IIDX_TT_LOW_RELEASE_DEFAULT;
        iidx_pad[pad].config.high_press = IIDX_TT_HIGH_PRESS_DEFAULT;
        iidx_pad[pad].config.high_release = IIDX_TT_HIGH_RELEASE_DEFAULT;
        iidx_pad[pad].config.invert_turntable = 0;
        iidx_pad[pad].config.vid_filter = 0;
        iidx_pad[pad].config.pid_filter = 0;

        iidx_pad[pad].layout_detected = 0;
        iidx_pad[pad].x_byte_offset = 0;
        iidx_pad[pad].btn_byte_offset = 12;
        iidx_pad[pad].tt_up = 0;
        iidx_pad[pad].tt_down = 0;
        iidx_pad[pad].last_x = 0x7FFF;
        iidx_pad[pad].last_raw_buttons = 0;
        iidx_pad[pad].last_debug_up = 0;
        iidx_pad[pad].last_debug_down = 0;

        /* Initialize DS2 state to all released */
        iidx_pad[pad].ds2.nButtonState = 0xFFFF;
        iidx_pad[pad].ds2.RightStickX = 128;
        iidx_pad[pad].ds2.RightStickY = 128;
        iidx_pad[pad].ds2.LeftStickX = 128;
        iidx_pad[pad].ds2.LeftStickY = 128;

        padf[pad].priv = &iidx_pad[pad];
        padf[pad].get_data = iidxhid_get_data;
        padf[pad].set_rumble = iidxhid_set_rumble;
        padf[pad].set_mode = iidxhid_set_mode;
        padf[pad].get_model = iidxhid_get_model;
    }

    sceUsbdRegisterLdd(&iidxhid_driver);
    DPRINTF("IIDX HID driver registered\n");
    return 0;
}

void iidxhid_reset(void)
{
    int pad;
    for (pad = 0; pad < IIDX_MAX_PADS; pad++) {
        iidx_release(pad);
    }
}
