#ifndef _IIDX_DIAG_H_
#define _IIDX_DIAG_H_

#if defined(_EE) || defined(__EE__)
#include <tamtypes.h>
#else
#include <types.h>
#endif

#define IIDX_DIAG_RPC_ID 0x49494458 /* "IIDX" */

#define IIDX_DIAG_CMD_GET_DATA  1
#define IIDX_DIAG_CMD_SELECT_EP 2
#define IIDX_DIAG_CMD_RESET     3
#define IIDX_DIAG_CMD_FORCE_RESET_PORT 4

#define DIAG_MAX_ENDPOINTS 8
#define DIAG_PACKET_MAX    64
#define DIAG_LOG_ENTRIES   12

typedef struct {
    u8 bEndpointAddress;
    u8 bmAttributes;
    u16 wMaxPacketSize;
    u8 bInterval;
} diag_ep_info_t;

typedef struct {
    /* Connection and configuration state */
    int connected;        /* 1 = device connected */
    int configured;       /* 1 = config set and ready */
    int devId;

    /* Hardware OHCI Root Hub Telemetry */
    u32 ohci_port_status[2];
    u32 ohci_control;
    u32 ohci_cmd_status;
    u32 ohci_int_status;
    u32 ohci_rh_status;

    /* Device Descriptor */
    u16 idVendor;
    u16 idProduct;
    u16 bcdDevice;
    u8 bDeviceClass;
    u8 bDeviceSubClass;
    u8 bDeviceProtocol;
    u8 bMaxPacketSize0;
    u8 bNumConfigurations;

    /* Active Configuration & Interface */
    u8 bNumInterfaces;
    u8 bInterfaceNumber;
    u8 bInterfaceClass;
    u8 bInterfaceSubClass;
    u8 bInterfaceProtocol;

    /* Discovered Endpoints */
    u8 num_endpoints;
    diag_ep_info_t endpoints[DIAG_MAX_ENDPOINTS];

    /* Active Polling Endpoint */
    int active_ep_idx;
    u8 active_ep_addr;
    u16 active_ep_size;

    /* Transfer Telemetry */
    u32 total_packets;
    int last_result;      /* USB_RC_xxx */
    int last_bytes;       /* bytes received in last packet */

    /* Packet Data */
    u8 current_packet[DIAG_PACKET_MAX];
    u8 prev_packet[DIAG_PACKET_MAX];
    u8 diff_mask[DIAG_PACKET_MAX]; /* 1 where current != prev */

    /* Activity tracking */
    u32 change_count;
    char recent_changes[DIAG_LOG_ENTRIES][48];
    int log_head;
} iidx_diag_data_t;

#endif /* _IIDX_DIAG_H_ */
