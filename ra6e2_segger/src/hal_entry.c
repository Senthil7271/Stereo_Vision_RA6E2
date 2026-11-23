/***********************************************************************************************************************
 * File Name    : hal_entry.c
 * Description  : Entry function demonstrating SEGGER RTT data transfer from RA6E2 to PC.
 **********************************************************************************************************************/

#include "hal_data.h"
#include "segger_transfer.h"
#include "image_data.h"

static void print_menu(void)
{
    segger_printf("\r\n=========================================================\r\n");
    segger_printf("       RA6E2 SEGGER RTT Data Transfer Ready\r\n");
    segger_printf("=========================================================\r\n");
    segger_printf(" Commands (Type in RTT Viewer 'Input' bar and hit Enter):\r\n");
    segger_printf("   [1] Send Image 0 (4096 bytes) as Framed Binary Packet\r\n");
    segger_printf("   [2] Send Image 1 (4096 bytes) as Framed Binary Packet\r\n");
    segger_printf("   [h] Display Image 0 as Hex Dump (Text in RTT Viewer)\r\n");
    segger_printf("   [c] Validate & Check Stored Image CRCs\r\n");
    segger_printf("   [t] Send a Sample Telemetry Data Packet\r\n");
    segger_printf("   [?] Show this menu again\r\n");
    segger_printf("=========================================================\r\n\r\n");
}

/*******************************************************************************************************************//**
 * hal_entry() is called by main()
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* Initialize SEGGER RTT transfer system */
    segger_transfer_init();

    /* Software delay to allow debugger to attach if launching fresh */
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);

    segger_printf("\r\n\r\n[SYSTEM] Booting RA6E2 SEGGER Transfer Project...\r\n");
    segger_printf("[SYSTEM] MCU: R7FA6E2BB (Arm Cortex-M33, 200MHz)\r\n");
    segger_printf("[SYSTEM] SEGGER RTT Control Block @ %p\r\n", &_SEGGER_RTT);

    print_menu();

    uint32_t heartbeat_tick = 0;

    while (1)
    {
        /* Check if host PC sent a command key via RTT Viewer */
        if (segger_has_key())
        {
            int key = segger_get_key();

            switch (key)
            {
                case '1':
                {
                    segger_printf("\r\n--> Sending Image 0 (Left Frame: 64x64, 4096 bytes)...\r\n");
                    uint32_t calc_crc = segger_calc_crc32(g_image_0, IMAGE_SIZE_BYTES);
                    segger_printf("    Expected CRC: 0x%08X | Computed CRC: 0x%08X\r\n",
                                  IMAGE_0_CRC32, calc_crc);

                    /* Send framed binary packet (ID: 0) */
                    segger_send_framed_packet(0, g_image_0, IMAGE_SIZE_BYTES);
                    segger_printf("--> Image 0 Transmit Complete! (4096 bytes sent over RTT)\r\n\r\n");
                    break;
                }

                case '2':
                {
                    segger_printf("\r\n--> Sending Image 1 (Right Frame: 64x64, 4096 bytes)...\r\n");
                    uint32_t calc_crc = segger_calc_crc32(g_image_1, IMAGE_SIZE_BYTES);
                    segger_printf("    Expected CRC: 0x%08X | Computed CRC: 0x%08X\r\n",
                                  IMAGE_1_CRC32, calc_crc);

                    /* Send framed binary packet (ID: 1) */
                    segger_send_framed_packet(1, g_image_1, IMAGE_SIZE_BYTES);
                    segger_printf("--> Image 1 Transmit Complete! (4096 bytes sent over RTT)\r\n\r\n");
                    break;
                }

                case 'h':
                case 'H':
                {
                    segger_printf("\r\n--> Printing first 256 bytes of Image 0 in Hex:\r\n");
                    segger_send_hex_dump(g_image_0, 256, 16);
                    break;
                }

                case 'c':
                case 'C':
                {
                    uint32_t crc0 = segger_calc_crc32(g_image_0, IMAGE_SIZE_BYTES);
                    uint32_t crc1 = segger_calc_crc32(g_image_1, IMAGE_SIZE_BYTES);
                    segger_printf("\r\n--- Flash Data Integrity Check ---\r\n");
                    segger_printf("Image 0: CRC=0x%08X [%s]\r\n", crc0, (crc0 == IMAGE_0_CRC32) ? "PASS" : "FAIL");
                    segger_printf("Image 1: CRC=0x%08X [%s]\r\n", crc1, (crc1 == IMAGE_1_CRC32) ? "PASS" : "FAIL");
                    segger_printf("----------------------------------\r\n\r\n");
                    break;
                }

                case 't':
                case 'T':
                {
                    static uint32_t telemetry_counter = 0;
                    struct {
                        uint32_t counter;
                        uint32_t timestamp_ms;
                        uint32_t dummy_sensor_val;
                    } __attribute__((packed)) telem_data;

                    telem_data.counter = ++telemetry_counter;
                    telem_data.timestamp_ms = heartbeat_tick * 100;
                    telem_data.dummy_sensor_val = 12345;

                    segger_printf("\r\n--> Sending Telemetry Packet #%u...\r\n", telem_data.counter);
                    segger_send_framed_packet(0xFF, (const uint8_t *)&telem_data, sizeof(telem_data));
                    segger_printf("--> Telemetry sent successfully!\r\n\r\n");
                    break;
                }

                case '?':
                {
                    print_menu();
                    break;
                }

                case '\r':
                case '\n':
                case ' ':
                    /* Ignore whitespace */
                    break;

                default:
                {
                    segger_printf("[!] Unknown command: '%c' (ASCII %d). Type '?' for menu.\r\n", (char)key, key);
                    break;
                }
            }
        }

        /* Periodic heartbeat tick every 100ms */
        R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
        heartbeat_tick++;

        /* Send status ping every 5 seconds (50 ticks) */
        if ((heartbeat_tick % 50) == 0)
        {
            segger_printf("[STATUS] Heartbeat: %u sec | Listening for commands ('1', '2', 'h', 'c', 't', '?')...\r\n",
                          (heartbeat_tick / 10));
        }
    }
}
