#include <tamtypes.h>
#include <sifrpc.h>
#include <sifcmd.h>
#include <loadfile.h>
#include <iopcontrol.h>
#include <kernel.h>
#include <debug.h>
#include <libpad.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sbv_patches.h>
#include <iopheap.h>

#include "../include/iidx_diag.h"

extern unsigned char usbd_mini_irx[];
extern unsigned int size_usbd_mini_irx;

extern unsigned char iidx_diag_irx[];
extern unsigned int size_iidx_diag_irx;

static SifRpcClientData_t diag_client;
static u8 rpc_buf[sizeof(iidx_diag_data_t) + 128] __attribute__((aligned(64)));

static char padBuf[256] __attribute__((aligned(64)));
static char padBuf1[256] __attribute__((aligned(64)));

static const char *usb_rc_str(int rc)
{
    switch (rc) {
        case 0:  return "OK";
        case 1:  return "CRC";
        case 2:  return "BITSTUFF";
        case 3:  return "TOGGLE";
        case 4:  return "STALL";
        case 5:  return "NORESPONSE";
        case 6:  return "BADPID";
        case 7:  return "UNEXPECTEDPID";
        case 8:  return "DATAOVERRUN";
        case 9:  return "DATAUNDERRUN";
        case 10: return "BUFFERUNDERRUN";
        case 11: return "BUFFEROVERRUN";
        case 12: return "NOTACCESSED";
        default: return "UNKNOWN";
    }
}

static void print_bits(u8 val)
{
    int i;
    for (i = 7; i >= 0; i--) {
        scr_printf("%d", (val >> i) & 1);
    }
}

static void save_log(iidx_diag_data_t *diag, const char *path)
{
    int fd = open(path, O_CREAT | O_WRONLY | O_TRUNC);
    if (fd >= 0) {
        char buf[512];
        int i, len;

        len = snprintf(buf, sizeof(buf),
            "=== IIDX DIAGNOSTIC LOG ===\n"
            "OHCI Port 1: 0x%08X  Port 2: 0x%08X\n"
            "OHCI Ctrl: 0x%08X  Intr: 0x%08X  Cmd: 0x%08X  RH: 0x%08X\n"
            "VID: 0x%04X  PID: 0x%04X  BCD: 0x%04X\n"
            "Class: 0x%02X  SubClass: 0x%02X  Proto: 0x%02X\n"
            "Active EP: 0x%02X  MaxPkt: %d\n"
            "Total Packets: %u  Changes: %u  Last RC: %d (%s)\n\n"
            "Current Packet (%d bytes):\n",
            (unsigned int)diag->ohci_port_status[0], (unsigned int)diag->ohci_port_status[1],
            (unsigned int)diag->ohci_control, (unsigned int)diag->ohci_int_status,
            (unsigned int)diag->ohci_cmd_status, (unsigned int)diag->ohci_rh_status,
            diag->idVendor, diag->idProduct, diag->bcdDevice,
            diag->bDeviceClass, diag->bDeviceSubClass, diag->bDeviceProtocol,
            diag->active_ep_addr, diag->active_ep_size,
            (unsigned int)diag->total_packets, (unsigned int)diag->change_count,
            diag->last_result, usb_rc_str(diag->last_result),
            diag->last_bytes);
        write(fd, buf, len);

        /* Hex dump */
        for (i = 0; i < diag->last_bytes && i < DIAG_PACKET_MAX; i++) {
            len = snprintf(buf, sizeof(buf), "%02X ", diag->current_packet[i]);
            write(fd, buf, len);
        }
        write(fd, "\n\nRecent Changes:\n", 18);
        for (i = 0; i < DIAG_LOG_ENTRIES; i++) {
            if (diag->recent_changes[i][0] != '\0') {
                len = snprintf(buf, sizeof(buf), "  %s\n", diag->recent_changes[i]);
                write(fd, buf, len);
            }
        }
        close(fd);
    }
}

int main(int argc, char *argv[])
{
    int ret;
    u32 prev_pkt_count = 0;
    u32 pkt_rate = 0;
    u32 frame_count = 0;
    char status_msg[64] = "Press START on Controller to save log";
    struct padButtonStatus buttons;
    u32 paddata = 0, old_pad = 0, new_pad = 0;

    (void)argc;
    (void)argv;

    /* Initialize Screen */
    init_scr();

    scr_printf("================================================================\n");
    scr_printf("       PS2 USB IIDX CONTROLLER HARDWARE DIAGNOSTIC v1.1\n");
    scr_printf("================================================================\n\n");

    /* Reset IOP */
    scr_printf("Resetting IOP...\n");
    SifInitRpc(0);
    while (!SifIopReset("", 0)) ;
    while (!SifIopSync()) ;
    SifInitRpc(0);
    SifLoadFileInit();
    SifInitIopHeap();

    /* Apply SBV patches so SifExecModuleBuffer works */
    sbv_patch_enable_lmb();
    sbv_patch_disable_prefix_check();

    /* Load SIO2MAN and PADMAN from ROM so controller port 2 works */
    SifLoadModule("rom0:SIO2MAN", 0, 0);
    SifLoadModule("rom0:PADMAN", 0, 0);

    /* Load USBD */
    scr_printf("Loading USBD driver... ");
    ret = -999;
    SifExecModuleBuffer(usbd_mini_irx, size_usbd_mini_irx, 0, NULL, &ret);
    scr_printf("id=%d\n", ret);

    /* Load IIDX Diag IOP driver */
    scr_printf("Loading IIDX Diag IOP driver... ");
    ret = -999;
    SifExecModuleBuffer(iidx_diag_irx, size_iidx_diag_irx, 0, NULL, &ret);
    scr_printf("id=%d\n", ret);

    /* Bind to RPC */
    scr_printf("Connecting to diagnostic RPC server...\n");
    do {
        if (SifBindRpc(&diag_client, IIDX_DIAG_RPC_ID, 0) < 0) {
            scr_printf("RPC bind error, retrying...\n");
            sleep(1);
        }
        nopdelay();
    } while (!diag_client.server);

    /* Init DualShock pads (try Port 0 and Port 1) */
    padInit(0);
    padPortOpen(0, 0, padBuf);
    padPortOpen(1, 0, padBuf1);

    scr_printf("Ready! Starting diagnostic dashboard...\n");
    sleep(1);

    int log_frozen = 0;

    while (1) {
        static iidx_diag_data_t diag;

        /* Fetch data from IOP (skip if log is frozen) */
        if (!log_frozen) {
            void *uncached_rpc_buf = (void *)((u32)rpc_buf | 0x20000000);
            memset(uncached_rpc_buf, 0, sizeof(iidx_diag_data_t));
            SyncDCache((void *)rpc_buf, (void *)(rpc_buf + sizeof(rpc_buf)));
            SifCallRpc(&diag_client, IIDX_DIAG_CMD_GET_DATA, 0, NULL, 0, uncached_rpc_buf, sizeof(iidx_diag_data_t), NULL, NULL);
            memcpy(&diag, uncached_rpc_buf, sizeof(iidx_diag_data_t));
        }

        /* Read DualShock input for interactive controls */
        paddata = 0;
        if (padGetState(1, 0) == PAD_STATE_STABLE) {
            if (padRead(1, 0, &buttons) != 0) {
                paddata = 0xffff ^ buttons.btns;
            }
        } else if (padGetState(0, 0) == PAD_STATE_STABLE) {
            if (padRead(0, 0, &buttons) != 0) {
                paddata = 0xffff ^ buttons.btns;
            }
        }
        new_pad = paddata & ~old_pad;
        old_pad = paddata;

        /* Check button presses */
        if (new_pad & PAD_CROSS) {
            log_frozen = !log_frozen;
            snprintf(status_msg, sizeof(status_msg), log_frozen ? "LOG PAUSED - Press [X] to resume" : "LOG RESUMED");
        }
        if (new_pad & PAD_START) {
            save_log(&diag, "mc0:/iidx_diag.txt");
            save_log(&diag, "mass:/iidx_diag.txt");
            snprintf(status_msg, sizeof(status_msg), "LOG SAVED to mc0:/iidx_diag.txt!");
        }
        if ((new_pad & PAD_R1) && diag.num_endpoints > 0) {
            int next_ep = (diag.active_ep_idx + 1) % diag.num_endpoints;
            SifCallRpc(&diag_client, IIDX_DIAG_CMD_SELECT_EP, 0, &next_ep, sizeof(int), NULL, 0, NULL, NULL);
            snprintf(status_msg, sizeof(status_msg), "Switched to EP index %d", next_ep);
        }
        if ((new_pad & PAD_L1) && diag.num_endpoints > 0) {
            int prev_ep = (diag.active_ep_idx - 1 + diag.num_endpoints) % diag.num_endpoints;
            SifCallRpc(&diag_client, IIDX_DIAG_CMD_SELECT_EP, 0, &prev_ep, sizeof(int), NULL, 0, NULL, NULL);
            snprintf(status_msg, sizeof(status_msg), "Switched to EP index %d", prev_ep);
        }
        if (new_pad & PAD_TRIANGLE) {
            SifCallRpc(&diag_client, IIDX_DIAG_CMD_RESET, 0, NULL, 0, NULL, 0, NULL, NULL);
            snprintf(status_msg, sizeof(status_msg), "Device reset requested");
        }
        if (new_pad & PAD_SQUARE) {
            int p = 0;
            SifCallRpc(&diag_client, IIDX_DIAG_CMD_FORCE_RESET_PORT, 0, &p, sizeof(int), NULL, 0, NULL, NULL);
            snprintf(status_msg, sizeof(status_msg), "Force reset Port 1 requested");
        }
        if (new_pad & PAD_CIRCLE) {
            int p = 1;
            SifCallRpc(&diag_client, IIDX_DIAG_CMD_FORCE_RESET_PORT, 0, &p, sizeof(int), NULL, 0, NULL, NULL);
            snprintf(status_msg, sizeof(status_msg), "Force reset Port 2 requested");
        }

        /* Calculate packet rate once per second */
        frame_count++;
        if (frame_count >= 60) {
            pkt_rate = (diag.total_packets >= prev_pkt_count) ? (diag.total_packets - prev_pkt_count) : 0;
            prev_pkt_count = diag.total_packets;
            frame_count = 0;
        }

        /* Render Dashboard */
        scr_setXY(0, 0);

        scr_printf("=== PS2 USB IIDX CONTROLLER HARDWARE DIAGNOSTIC v1.1 ===\n");

        /* Hardware Root Hub Status (Always visible) */
        {
            int p;
            for (p = 0; p < 2; p++) {
                u32 st = diag.ohci_port_status[p];
                scr_printf("Port %d: 0x%08X [Conn:%-3s En:%-3s Reset:%-3s OC:%-4s Spd:%-4s]\n",
                           p + 1, (unsigned int)st,
                           (st & 1) ? "YES" : "NO",
                           (st & 2) ? "YES" : "NO",
                           (st & 0x10) ? "YES" : "NO",
                           (st & 8) ? "FAIL" : "OK",
                           (st & 0x200) ? "LOW" : "FULL");
            }
            scr_printf("HC Ctrl:0x%08X Intr:0x%08X Cmd:0x%08X RH:0x%08X\n",
                       (unsigned int)diag.ohci_control,
                       (unsigned int)diag.ohci_int_status,
                       (unsigned int)diag.ohci_cmd_status,
                       (unsigned int)diag.ohci_rh_status);
        }
        scr_printf("----------------------------------------------------------------\n");

        if (!diag.connected) {
            int i;
            scr_printf("STATUS: [NO DEVICE CONNECTED]\n");
            scr_printf("--- IOP USB EVENT LOG (%d events) ---\n", (int)diag.change_count);
            for (i = 0; i < 8; i++) {
                int idx = (diag.log_head - 1 - i + DIAG_LOG_ENTRIES) % DIAG_LOG_ENTRIES;
                diag.recent_changes[idx][47] = '\0';
                if (diag.recent_changes[idx][0] != '\0')
                    scr_printf(" > %-60s\n", diag.recent_changes[idx]);
                else
                    scr_printf("                                                                \n");
            }
        } else {
            int i;
            scr_printf("STATUS: CONNECTED  |  Configured: %s  |  DevID: %d\n",
                       diag.configured ? "YES" : "NO ", diag.devId);
            scr_printf("Vendor ID : 0x%04X  |  Product ID : 0x%04X  |  BCD: 0x%04X\n",
                       diag.idVendor, diag.idProduct, diag.bcdDevice);
            scr_printf("Class: 0x%02X  Sub: 0x%02X  Proto: 0x%02X  |  Intf: #%d (Class: 0x%02X)\n",
                       diag.bDeviceClass, diag.bDeviceSubClass, diag.bDeviceProtocol,
                       diag.bInterfaceNumber, diag.bInterfaceClass);

            scr_printf("EP 0x%02X Pkt:%2d | Pkts:%-6u Rate:%3u/s Chgs:%-5u RC:%d (%s)\n",
                       diag.active_ep_addr, diag.active_ep_size,
                       (unsigned int)diag.total_packets,
                       (unsigned int)pkt_rate,
                       (unsigned int)diag.change_count,
                       diag.last_result,
                       usb_rc_str(diag.last_result));

            scr_printf("00: ");
            for (i = 0; i < 16; i++) {
                if (i < diag.last_bytes)
                    scr_printf("%02X ", diag.current_packet[i]);
                else
                    scr_printf(".. ");
            }
            scr_printf("\nDF: ");
            for (i = 0; i < 16; i++) {
                if (i < diag.last_bytes && diag.diff_mask[i])
                    scr_printf("^^ ");
                else
                    scr_printf("   ");
            }
            scr_printf("\n");

            scr_printf("B0: "); print_bits(diag.current_packet[0]);
            scr_printf("  B1: "); print_bits(diag.current_packet[1]);
            scr_printf("\nB2: "); print_bits(diag.current_packet[2]);
            scr_printf("  B3: "); print_bits(diag.current_packet[3]);
            scr_printf("\n");
        }

        scr_printf("----------------------------------------------------------------\n");
        scr_printf("[START] Save | [X] Pause | [[]]/(O) Reset P1/P2 | [/\\ ] Reset\n");
        scr_printf("================================================================\n");

        /* ~60 FPS delay */
        nopdelay();
    }

    return 0;
}
