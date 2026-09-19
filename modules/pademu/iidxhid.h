#ifndef _IIDXHID_H_
#define _IIDXHID_H_

#include "irx.h"
#include "types.h"
#include "ds34common.h"

enum eIIDXHIDStatus {
    IIDXHID_STATE_DISCONNECTED = 0x00,
    IIDXHID_STATE_AUTHORIZED   = 0x01,
    IIDXHID_STATE_CONFIGURED   = 0x02,
    IIDXHID_STATE_CONNECTED    = 0x04,
    IIDXHID_STATE_RUNNING      = 0x08,
};

/* Turntable thresholds and hysteresis defaults */
#define IIDX_TT_LOW_PRESS_DEFAULT    16000
#define IIDX_TT_LOW_RELEASE_DEFAULT  20000
#define IIDX_TT_HIGH_PRESS_DEFAULT   49000
#define IIDX_TT_HIGH_RELEASE_DEFAULT 45000

#define IIDX_MAX_PADS 2

typedef struct _iidx_config
{
    u16 low_press;
    u16 low_release;
    u16 high_press;
    u16 high_release;
    u8 invert_turntable;
    u16 vid_filter; /* 0 = accept generic HID */
    u16 pid_filter; /* 0 = accept generic HID */
} iidx_config_t;

typedef struct _iidx_device
{
    int pad_idx;
    int devId;
    int sema;
    int cmd_sema;
    int controlEndp;
    int interruptEndp;
    int interfaceNumber;
    u8 enabled;
    u8 status;
    u8 analog_btn;

    /* Turntable state */
    u8 tt_up;
    u8 tt_down;
    u16 last_x;

    /* Configuration */
    iidx_config_t config;

    /* Auto-detected report offsets */
    u8 layout_detected;
    u8 x_byte_offset;
    u8 btn_byte_offset;
    u8 hat_byte_offset;
    u8 turntable_is_16bit;
    u8 has_hat;
    u16 packet_size;
    volatile u8 transfer_active;
    UsbEndpointDescriptor saved_ep;
    u8 ep_found;

    /* Debug tracking (for rate-limiting log output to state changes) */
    u32 last_raw_buttons;
    u8 last_debug_up;
    u8 last_debug_down;

    /* Controller state matching PADEMU expectation */
    union
    {
        struct ds2report ds2;
        u8 data[18];
    };

    u8 usb_buf[MAX_BUFFER_SIZE + 32] __attribute__((aligned(4)));
} iidx_device;

int iidxhid_init(u8 pads, u8 options);
void iidxhid_reset(void);

#endif /* _IIDXHID_H_ */
