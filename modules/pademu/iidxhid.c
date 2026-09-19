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

static int iidxhid_probe(int devId);
static int iidxhid_connect(int devId);
static int iidxhid_disconnect(int devId);
static void iidx_release(int pad);
static void iidx_config_set(int result, int count, void *arg);
static void usb_data_cb(int resultCode, int bytes, void *arg);

static int iidxhid_get_model(struct pad_funcs *pf, int port);
static int iidxhid_get_data(struct pad_funcs *pf, u8 *dst, int size, int port);
static void iidxhid_set_rumble(struct pad_funcs *pf, u8 lrum, u8 rrum);
static void iidxhid_set_mode(struct pad_funcs *pf, int mode, int lock);

static UsbDriver iidxhid_driver = {NULL, NULL, "iidxhid", iidxhid_probe, iidxhid_connect, iidxhid_disconnect};


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

static void iidx_detect_layout(iidx_device *pad, const u8 *buf, int len)
{
    /*
     * 1. YuanCon miniDX / Standard Gamepad with Report ID 0x06:
     *    buf[0] = 0x06 (Report ID)
     *    buf[1..4] = 32 Buttons (Buttons 1..32)
     *    buf[5..6] = X-axis (Turntable, signed 16-bit: -32768..+32767)
     *    buf[15] = Hat switch
     */
    if (len >= 7 && buf[0] == 0x06) {
        pad->x_byte_offset = 5;
        pad->btn_byte_offset = 1;
        pad->turntable_is_16bit = 1;
        pad->is_signed_turntable = 1;
        pad->has_hat = (len >= 16) ? 1 : 0;
        pad->hat_byte_offset = 15;
        pad->is_yuancon_report6 = 1;
        pad->layout_detected = 1;
        DPRINTF("Detected Layout: YuanCon miniDX (Report 0x06, Btns@1, TT@5 signed 16-bit)\n");
        return;
    }

    /*
     * 2. DInput / GP2040-CE / Generic Gamepad (8-bit axes):
     *    buf[0] = X axis (~128)
     *    buf[1] = Y axis (~128)
     *    Both axes resting around 128 (64..192)
     */
    if (len >= 6 && buf[0] >= 64 && buf[0] <= 192 && buf[1] >= 64 && buf[1] <= 192) {
        pad->x_byte_offset = 0;
        pad->turntable_is_16bit = 0;
        pad->has_hat = 1;
        pad->hat_byte_offset = 4;
        pad->btn_byte_offset = (len >= 7) ? 5 : 4;
        pad->layout_detected = 1;
        DPRINTF("Detected Layout: DInput/GP2040-CE (Btns@%d, TT@0 8-bit)\n", pad->btn_byte_offset);
        return;
    }

    /*
     * 3. YuanCon / Phoenixwan w/ Report ID:
     *    buf[0] = Report ID (1..7)
     *    buf[1..2] = buttons (< 64 at rest)
     *    buf[3..4] = TT 16-bit (~32767)
     */
    if (len >= 5 && buf[0] > 0 && buf[0] < 8) {
        u16 tt3 = (u16)(buf[3] | (buf[4] << 8));
        if (tt3 >= 16000 && tt3 <= 48000) {
            pad->x_byte_offset = 3;
            pad->btn_byte_offset = 1;
            pad->turntable_is_16bit = 1;
            pad->has_hat = 0;
            pad->layout_detected = 1;
            DPRINTF("Detected Layout: YuanCon/ReportID (Btns@1, TT@3 16-bit)\n");
            return;
        }
    }

    /*
     * 3. DJ DAO / Phoenixwan (Arcin):
     *    buf[0..1] = buttons (< 64 at rest)
     *    buf[2..3] = TT 16-bit (~32767)
     */
    if (len >= 4) {
        u16 tt2 = (u16)(buf[2] | (buf[3] << 8));
        if (tt2 >= 16000 && tt2 <= 48000) {
            pad->x_byte_offset = 2;
            pad->btn_byte_offset = 0;
            pad->turntable_is_16bit = 1;
            pad->has_hat = 0;
            pad->layout_detected = 1;
            DPRINTF("Detected Layout: DAO/Phoenixwan (Btns@0, TT@2 16-bit)\n");
            return;
        }
    }

    /*
     * 4. Konami Official Entry Model (16-64 bytes)
     */
    if (len >= 14) {
        u16 tt0 = (u16)(buf[0] | (buf[1] << 8));
        u16 tt1 = (u16)(buf[1] | (buf[2] << 8));
        if (tt0 >= 16000 && tt0 <= 48000) {
            pad->x_byte_offset = 0;
            pad->btn_byte_offset = 12;
            pad->turntable_is_16bit = 1;
            pad->has_hat = 0;
            pad->layout_detected = 1;
            DPRINTF("Detected Layout: Konami Entry Model (Btns@12, TT@0 16-bit)\n");
            return;
        } else if (tt1 >= 16000 && tt1 <= 48000) {
            pad->x_byte_offset = 1;
            pad->btn_byte_offset = 13;
            pad->turntable_is_16bit = 1;
            pad->has_hat = 0;
            pad->layout_detected = 1;
            DPRINTF("Detected Layout: Konami Entry Model w/ ReportID (Btns@13, TT@1 16-bit)\n");
            return;
        }
    }

    /*
     * 5. Default Fallback: Btns@0, TT@2 (standard DAO/Phoenixwan)
     */
    if (buf[0] > 0 && buf[0] < 8) {
        pad->x_byte_offset = 3;
        pad->btn_byte_offset = 1;
    } else {
        pad->x_byte_offset = 2;
        pad->btn_byte_offset = 0;
    }
    pad->turntable_is_16bit = 1;
    pad->has_hat = 0;
}

static void iidx_readReport(u8 *buf, int len, iidx_device *pad)
{
    u16 x_raw = 0x7FFF;
    u32 hid_buttons = 0;
    u8 up = 0, down = 0;

    if (!pad->layout_detected) {
        iidx_detect_layout(pad, buf, len);
    }

    /* Read buttons from detected offset */
    if (pad->btn_byte_offset + 1 < len) {
        hid_buttons = (u32)(buf[pad->btn_byte_offset] | (buf[pad->btn_byte_offset + 1] << 8));
        if (pad->btn_byte_offset + 3 < len) {
            hid_buttons |= (u32)((buf[pad->btn_byte_offset + 2] << 16) | (buf[pad->btn_byte_offset + 3] << 24));
        }
    }

    /*
     * Dynamic button fail-safe:
     * If hid_buttons is 0 (no buttons registered at current btn_byte_offset),
     * check if any candidate offset in buf[] has buttons pressed!
     * Candidate offsets: 0, 1, 2, 4, 5, 12, 13
     */
    if (hid_buttons == 0) {
        static const u8 candidates[] = {0, 1, 2, 4, 5, 12, 13};
        int c;
        for (c = 0; c < (int)(sizeof(candidates)/sizeof(candidates[0])); c++) {
            int off = candidates[c];
            if (off + 1 < len && off != pad->x_byte_offset && (!pad->has_hat || off != pad->hat_byte_offset)) {
                u16 val = (u16)(buf[off] | (buf[off + 1] << 8));
                /* Exclude analog resting values (e.g. 0x8080 or ~32768) */
                if (val != 0 && (val & 0xFF) != 0x80 && (val & 0xFF) != 0x7F) {
                    hid_buttons = (u32)val;
                    pad->btn_byte_offset = off;
                    DPRINTF("Dynamic button fail-safe: locked to offset %d\n", off);
                    break;
                }
            }
        }
    }

    /* Read turntable axis */
    if (pad->turntable_is_16bit) {
        if (pad->x_byte_offset + 1 < len) {
            if (pad->is_signed_turntable) {
                int16_t s_x = (int16_t)(buf[pad->x_byte_offset] | (buf[pad->x_byte_offset + 1] << 8));
                x_raw = (u16)(s_x + 32768);
            } else {
                x_raw = (u16)(buf[pad->x_byte_offset] | (buf[pad->x_byte_offset + 1] << 8));
            }
        }
    } else {
        if (pad->x_byte_offset < len) {
            x_raw = (u16)buf[pad->x_byte_offset] * 257;
        }
    }

    pad->last_x = x_raw;
    iidx_process_turntable(pad, x_raw, &up, &down);

    /* Also check D-pad hat switch if controller has one */
    if (pad->has_hat && pad->hat_byte_offset < len) {
        u8 hat = buf[pad->hat_byte_offset] & 0x0F;
        if (hat == 0 || hat == 1 || hat == 7) up = 1;
        if (hat == 3 || hat == 4 || hat == 5) down = 1;
    }

    /* Also check dedicated Scratch UP/DOWN buttons (buttons 11 & 12 / bits 10 & 11) */
    if (hid_buttons & (1 << 10)) up = 1;
    if (hid_buttons & (1 << 11)) down = 1;

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
                ((hid_buttons >> 8) | (hid_buttons >> 9)) & 1);
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
     * START                 | Button 9/10| START
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

    /*
     * Start & Select:
     * YuanCon miniDX (Report 0x06):
     *   START is bit 7 (8th bit of B1 / 0x80)
     *   SELECT is bit 9 (2nd bit of B2 / 0x02)
     * Other controllers (e.g. Phoenixwan / Arcin / Konami Entry):
     *   SELECT is bit 7 (0x80)
     *   START is bit 8/9 (0x01/0x02 of byte 2)
     */
    if (pad->is_yuancon_report6) {
        if (hid_buttons & (1 << 7))
            buttons_state &= ~(1 << DS2BtnBit_Start);
        if ((hid_buttons & (1 << 8)) || (hid_buttons & (1 << 9)))
            buttons_state &= ~(1 << DS2BtnBit_Select);
    } else {
        if (hid_buttons & (1 << 7))
            buttons_state &= ~(1 << DS2BtnBit_Select);
        if ((hid_buttons & (1 << 8)) || (hid_buttons & (1 << 9)))
            buttons_state &= ~(1 << DS2BtnBit_Start);
    }

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

#ifndef USB_CLASS_HID
#define USB_CLASS_HID 0x03
#endif

#ifndef USB_DT_INTERFACE
#define USB_DT_INTERFACE 0x04
#endif

#ifndef USB_DT_ENDPOINT
#define USB_DT_ENDPOINT 0x05
#endif

static int iidxhid_probe(int devId)
{
    UsbDeviceDescriptor *device = NULL;

    DPRINTF("probe: devId=%i\n", devId);

    device = (UsbDeviceDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) {
        return 0;
    }

    /* Do not claim USB Mass Storage devices (handled by usbmass_bd) */
    if (device->bDeviceClass == 0x08) {
        return 0;
    }

    /* Do not claim USB Hubs */
    if (device->bDeviceClass == 0x09) {
        return 0;
    }

    /* Explicitly claim YuanCon miniDX: VID 0x1ccf, PID 0x8048 */
    if (device->idVendor == 0x1ccf && device->idProduct == 0x8048) {
        DPRINTF("probe: matched YuanCon miniDX (%04X:%04X)\n", device->idVendor, device->idProduct);
        return 1;
    }

    /* Check if configuration descriptor has a Mass Storage interface (0x08) */
    UsbConfigDescriptor *config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, device, USB_DT_CONFIG);
    if (!config)
        config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_CONFIG);

    if (config != NULL && config->wTotalLength >= sizeof(UsbConfigDescriptor)) {
        const u8 *p = (const u8 *)config;
        const u8 *end = p + config->wTotalLength;
        while (p + 2 <= end) {
            u8 len = p[0];
            u8 type = p[1];
            if (len < 2 || p + len > end)
                break;
            if (type == USB_DT_INTERFACE && len >= sizeof(UsbInterfaceDescriptor)) {
                UsbInterfaceDescriptor *intf = (UsbInterfaceDescriptor *)p;
                if (intf->bInterfaceClass == 0x08) {
                    return 0; /* Mass storage interface - do not claim */
                }
            }
            p += len;
        }
    }

    /* Only exclude the specific DualShock/Guitar controllers handled by ds34usb */
    if ((device->idVendor == SONY_VID || device->idVendor == DS34_VID) &&
        (device->idProduct == DS3_PID || device->idProduct == DS4_PID ||
         device->idProduct == DS4_PID_SLIM || device->idProduct == DS5_PID ||
         device->idProduct == GUITAR_HERO_PS3_PID || device->idProduct == ROCK_BAND_PS3_PID)) {
        return 0;
    }

    /* Claim all other USB devices (arcade controllers, generic HID, Konami, DIY, etc.) */
    return 1;
}

static int iidxhid_connect(int devId)
{
    int pad;
    UsbDeviceDescriptor *device;
    UsbConfigDescriptor *config;
    UsbEndpointDescriptor *endpoint;

    DPRINTF("connect: devId=%i\n", devId);

    for (pad = 0; pad < IIDX_MAX_PADS; pad++) {
        if (iidx_pad[pad].devId == -1)
            break;
    }

    if (pad >= IIDX_MAX_PADS) {
        DPRINTF("connect: no available pad slot!\n");
        return 1;
    }

    PollSema(iidx_pad[pad].sema);

    iidx_pad[pad].devId = devId;
    iidx_pad[pad].status = IIDXHID_STATE_AUTHORIZED;
    iidx_pad[pad].controlEndp = sceUsbdOpenPipe(devId, NULL);

    device = (UsbDeviceDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) {
        iidx_release(pad);
        return 1;
    }

    config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, device, USB_DT_CONFIG);
    if (config == NULL) {
        config = (UsbConfigDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_CONFIG);
    }

    UsbEndpointDescriptor *best_ep = NULL;

    /* Scan configuration descriptor buffer for Interrupt IN endpoint (prioritizing HID interface) */
    if (config != NULL && config->wTotalLength >= sizeof(UsbConfigDescriptor)) {
        const u8 *p = (const u8 *)config;
        const u8 *end = p + config->wTotalLength;
        int cur_intf_num = 0;
        int cur_intf_class = 0;
        int best_score = 0;

        while (p + 2 <= end) {
            u8 len = p[0];
            u8 type = p[1];
            if (len < 2 || p + len > end)
                break;

            if (type == USB_DT_INTERFACE && len >= sizeof(UsbInterfaceDescriptor)) {
                UsbInterfaceDescriptor *intf = (UsbInterfaceDescriptor *)p;
                cur_intf_num = intf->bInterfaceNumber;
                cur_intf_class = intf->bInterfaceClass;
            } else if (type == USB_DT_ENDPOINT && len >= sizeof(UsbEndpointDescriptor)) {
                UsbEndpointDescriptor *ep = (UsbEndpointDescriptor *)p;
                if ((ep->bmAttributes & 0x03) == USB_ENDPOINT_XFER_INT &&
                    (ep->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
                    int score = (cur_intf_class == USB_CLASS_HID) ? 2 : 1;
                    if (score > best_score) {
                        best_score = score;
                        best_ep = ep;
                        u16 pkt = (ep->wMaxPacketSizeHB << 8) | ep->wMaxPacketSizeLB;
                        if (pkt == 0 || pkt > 64)
                            pkt = 64;
                        iidx_pad[pad].packet_size = pkt;
                        iidx_pad[pad].interfaceNumber = cur_intf_num;
                        if (score == 2) {
                            /* Found HID interface endpoint - optimal! */
                            break;
                        }
                    }
                }
            }

            p += len;
        }

        if (best_ep != NULL) {
            memcpy(&iidx_pad[pad].saved_ep, best_ep, sizeof(UsbEndpointDescriptor));
            iidx_pad[pad].ep_found = 1;
            DPRINTF("Found interrupt IN endpoint addr=%02X pktSize=%u intf=%d\n",
                    best_ep->bEndpointAddress,
                    iidx_pad[pad].packet_size, iidx_pad[pad].interfaceNumber);
        }
    }

    /* Fallback: Scan using FreeUsbd static descriptor scanner if not found in config buffer */
    if (!iidx_pad[pad].ep_found) {
        endpoint = (UsbEndpointDescriptor *)sceUsbdScanStaticDescriptor(devId, NULL, USB_DT_ENDPOINT);
        while (endpoint != NULL) {
            if ((endpoint->bmAttributes & 0x03) == USB_ENDPOINT_XFER_INT &&
                (endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
                u16 pkt = (endpoint->wMaxPacketSizeHB << 8) | endpoint->wMaxPacketSizeLB;
                if (pkt == 0 || pkt > 64)
                    pkt = 64;
                iidx_pad[pad].packet_size = pkt;
                memcpy(&iidx_pad[pad].saved_ep, endpoint, sizeof(UsbEndpointDescriptor));
                iidx_pad[pad].ep_found = 1;
                break;
            }
            endpoint = (UsbEndpointDescriptor *)((char *)endpoint + endpoint->bLength);
            if (endpoint->bLength < 2)
                break;
        }
    }

    if (!iidx_pad[pad].ep_found) {
        DPRINTF("connect: failed to find interrupt endpoint!\n");
        iidx_release(pad);
        return 1;
    }

    iidx_pad[pad].status |= IIDXHID_STATE_CONNECTED;

    /* Initialize controller state immediately so pad is fully valid when SIO2 queries */
    iidx_pad[pad].ds2.nButtonState = 0xFFFF;
    iidx_pad[pad].ds2.RightStickX = 128;
    iidx_pad[pad].ds2.RightStickY = 128;
    iidx_pad[pad].ds2.LeftStickX = 128;
    iidx_pad[pad].ds2.LeftStickY = 128;

    /* Connect pad to PADEMU immediately */
    pademu_connect(&padf[pad]);

    if (config != NULL) {
        sceUsbdSetConfiguration(iidx_pad[pad].controlEndp, config->bConfigurationValue, iidx_config_set, (void *)(long)pad);
    } else {
        sceUsbdSetConfiguration(iidx_pad[pad].controlEndp, 1, iidx_config_set, (void *)(long)pad);
    }
    SignalSema(iidx_pad[pad].sema);

    return 0;
}

static void iidx_config_set(int result, int count, void *arg)
{
    int pad = (int)(long)arg;

    PollSema(iidx_pad[pad].sema);

    if (result == USB_RC_OK) {
        iidx_pad[pad].status |= (IIDXHID_STATE_CONFIGURED | IIDXHID_STATE_RUNNING);

        /* Open interrupt endpoint AFTER configuration is active */
        if (iidx_pad[pad].ep_found && iidx_pad[pad].interruptEndp < 0) {
            iidx_pad[pad].interruptEndp = sceUsbdOpenPipe(iidx_pad[pad].devId, &iidx_pad[pad].saved_ep);
            DPRINTF("Opened interrupt IN pipe id=%d addr=%02X\n",
                    iidx_pad[pad].interruptEndp, iidx_pad[pad].saved_ep.bEndpointAddress);
        }

        /* Send HID SET_IDLE (0 = report on change / continuous) to target interface */
        sceUsbdControlTransfer(iidx_pad[pad].controlEndp, REQ_USB_OUT, 0x0A /* SET_IDLE */, 0, iidx_pad[pad].interfaceNumber, 0, NULL, NULL, NULL);

        /* Ensure pad is connected to PADEMU */
        pademu_connect(&padf[pad]);
    }

    SignalSema(iidx_pad[pad].sema);
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
    }

    return 0;
}

static void iidx_release(int pad)
{
    PollSema(iidx_pad[pad].sema);

    iidx_pad[pad].status = IIDXHID_STATE_DISCONNECTED;
    iidx_pad[pad].transfer_active = 0;

    if (iidx_pad[pad].interruptEndp >= 0) {
        sceUsbdClosePipe(iidx_pad[pad].interruptEndp);
        iidx_pad[pad].interruptEndp = -1;
    }

    iidx_pad[pad].controlEndp = -1;
    iidx_pad[pad].devId = -1;
    iidx_pad[pad].layout_detected = 0;
    iidx_pad[pad].ep_found = 0;
    iidx_pad[pad].tt_up = 0;
    iidx_pad[pad].tt_down = 0;
    iidx_pad[pad].is_signed_turntable = 0;
    iidx_pad[pad].is_yuancon_report6 = 0;

    SignalSema(iidx_pad[pad].sema);

    pademu_disconnect(&padf[pad]);
}

static unsigned int iidx_timeout(void *arg)
{
    int sema = (int)(long)arg;
    iSignalSema(sema);
    return 0;
}

static void iidx_transfer_wait(int sema)
{
    iop_sys_clock_t cmd_timeout;

    cmd_timeout.lo = 200000;
    cmd_timeout.hi = 0;

    if (SetAlarm(&cmd_timeout, iidx_timeout, (void *)(long)sema) == 0) {
        WaitSema(sema);
        CancelAlarm(iidx_timeout, NULL);
    }
}

static int usb_resultCode = 0;
static int usb_resultBytes = 0;

static void usb_data_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)(long)arg;

    usb_resultCode = resultCode;
    usb_resultBytes = bytes;

    SignalSema(iidx_pad[pad].sema);
}

static int iidxhid_get_data(struct pad_funcs *pf, u8 *dst, int size, int port)
{
    iidx_device *pad = pf->priv;
    int ret = 0;

    WaitSema(pad->sema);
    PollSema(pad->sema);

    if (pad->interruptEndp >= 0) {
        usb_resultCode = -1;
        usb_resultBytes = 0;
        ret = sceUsbdInterruptTransfer(pad->interruptEndp,
                                       pad->usb_buf,
                                       pad->packet_size ? pad->packet_size : 64,
                                       usb_data_cb,
                                       (void *)(long)pad->pad_idx);
        if (ret == USB_RC_OK) {
            iidx_transfer_wait(pad->sema);
            if (usb_resultCode == USB_RC_OK && usb_resultBytes > 0) {
                iidx_readReport(pad->usb_buf, usb_resultBytes, pad);
            }
            usb_resultCode = -1;
        }
    }

    if (size > 18)
        size = 18;
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
        iidx_pad[pad].x_byte_offset = 2;
        iidx_pad[pad].btn_byte_offset = 0;
        iidx_pad[pad].hat_byte_offset = 4;
        iidx_pad[pad].turntable_is_16bit = 1;
        iidx_pad[pad].has_hat = 0;
        iidx_pad[pad].packet_size = 64;
        iidx_pad[pad].transfer_active = 0;
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
