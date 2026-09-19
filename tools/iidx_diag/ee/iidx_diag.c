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
            "VID: 0x%04X  PID: 0x%04X  BCD: 0x%04X\n"
            "Class: 0x%02X  SubClass: 0x%02X  Proto: 0x%02X\n"
            "Active EP: 0x%02X  MaxPkt: %d\n"
            "Total Packets: %u  Changes: %u  Last RC: %d (%s)\n\n"
            "Current Packet (%d bytes):\n",
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
    scr_printf("Initializing IIDX Diagnostic Tool...\n");

    /* Reset IOP */
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

    while (1) {
        iidx_diag_data_t diag;

        /* Fetch data from IOP */
        memset(rpc_buf, 0, sizeof(rpc_buf));
        SifCallRpc(&diag_client, IIDX_DIAG_CMD_GET_DATA, 0, NULL, 0, rpc_buf, sizeof(iidx_diag_data_t), NULL, NULL);
        memcpy(&diag, rpc_buf, sizeof(iidx_diag_data_t));

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

        /* Calculate packet rate once per second */
        frame_count++;
        if (frame_count >= 60) {
            pkt_rate = (diag.total_packets >= prev_pkt_count) ? (diag.total_packets - prev_pkt_count) : 0;
            prev_pkt_count = diag.total_packets;
            frame_count = 0;
        }

        /* Render Dashboard */
        scr_setXY(0, 0);

        scr_printf("================================================================\n");
        scr_printf("           PS2 USB CONTROLLER RAW INPUT DIAGNOSTIC v1.0\n");
        scr_printf("================================================================\n");

        if (!diag.connected) {
            int i;
            scr_printf("STATUS: [NO DEVICE CONNECTED]\n\n");
            scr_printf("  Please connect your IIDX USB Controller to either USB port.\n");
            scr_printf("  If already plugged in, try unplugging and replugging.\n\n");
            scr_printf("--- IOP USB EVENT LOG (%d events) ---\n", (int)diag.change_count);
            for (i = 0; i < DIAG_LOG_ENTRIES; i++) {
                int idx = (diag.log_head - 1 - i + DIAG_LOG_ENTRIES) % DIAG_LOG_ENTRIES;
                if (diag.recent_changes[idx][0] != '\0')
                    scr_printf(" > %-60s\n", diag.recent_changes[idx]);
                else
                    scr_printf("                                                                \n");
            }
            for (i = 0; i < 6; i++) {
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

            scr_printf("\n--- ENDPOINTS (%d found) ---\n", diag.num_endpoints);
            for (i = 0; i < diag.num_endpoints && i < 4; i++) {
                char is_in = (diag.endpoints[i].bEndpointAddress & 0x80) ? 'I' : 'O';
                char is_active = (i == diag.active_ep_idx) ? '*' : ' ';
                const char *type_str = "CTRL";
                switch (diag.endpoints[i].bmAttributes & 3) {
                    case 1: type_str = "ISOC"; break;
                    case 2: type_str = "BULK"; break;
                    case 3: type_str = "INT "; break;
                }
                scr_printf(" [%c] EP 0x%02X (%cN) Type:%s Pkt:%2d Int:%2dms%s\n",
                           is_active,
                           diag.endpoints[i].bEndpointAddress,
                           is_in,
                           type_str,
                           diag.endpoints[i].wMaxPacketSize,
                           diag.endpoints[i].bInterval,
                           (i == diag.active_ep_idx) ? " <-- ACTIVE" : "           ");
            }
            for (; i < 4; i++) {
                scr_printf("                                                                \n");
            }

            scr_printf("\n--- TELEMETRY ---\n");
            scr_printf("Packets: %-8u  Rate: %3u/s  Changes: %-6u  RC: %d (%s) \n",
                       (unsigned int)diag.total_packets,
                       (unsigned int)pkt_rate,
                       (unsigned int)diag.change_count,
                       diag.last_result,
                       usb_rc_str(diag.last_result));
            scr_printf("Last Packet Size: %2d bytes\n", diag.last_bytes);

            scr_printf("\n--- RAW PACKET (HEX) ---\n");
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

            scr_printf("\n--- BITFIELD (First 4 Bytes) ---\n");
            scr_printf("B0: "); print_bits(diag.current_packet[0]);
            scr_printf("  B1: "); print_bits(diag.current_packet[1]);
            scr_printf("\nB2: "); print_bits(diag.current_packet[2]);
            scr_printf("  B3: "); print_bits(diag.current_packet[3]);
            scr_printf("\n");

            scr_printf("\n--- RECENT ACTIVITY ---\n");
            for (i = 0; i < 3; i++) {
                int idx = (diag.log_head - 1 - i + DIAG_LOG_ENTRIES) % DIAG_LOG_ENTRIES;
                if (diag.recent_changes[idx][0] != '\0')
                    scr_printf(" > %-40s\n", diag.recent_changes[idx]);
                else
                    scr_printf("                                                 \n");
            }
        }

        scr_printf("\n================================================================\n");
        scr_printf("%-64s\n", status_msg);
        scr_printf("[START] Save mc0:/mass: | [L1/R1] Switch EP | [/\\ ] Reset\n");
        scr_printf("================================================================\n");

        /* ~60 FPS delay */
        nopdelay();
    }

    return 0;
}
