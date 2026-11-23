/*******************************************************************************************************************
 * @file    segger_transfer.c
 * @brief   SEGGER RTT Data Transfer Implementation for Renesas RA6E2.
 *******************************************************************************************************************/

#include "segger_transfer.h"
#include <stdarg.h>
#include <stdio.h>

/* IEEE 802.3 CRC32 calculation (polynomial 0xEDB88320) */
uint32_t segger_calc_crc32(const uint8_t *p_data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= p_data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 1U)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc = (crc >> 1);
            }
        }
    }
    return ~crc;
}

void segger_transfer_init(void)
{
    /* Initialize SEGGER RTT control block and buffers */
    SEGGER_RTT_Init();

    /* Configure Up-Buffer 0: "Terminal", non-blocking skip by default to prevent freeze if no debugger */
    SEGGER_RTT_ConfigUpBuffer(SEGGER_TERMINAL_CHANNEL, "Terminal", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

int segger_printf(const char *sFormat, ...)
{
    int ret;
    va_list param_list;
    va_start(param_list, sFormat);
    ret = SEGGER_RTT_vprintf(SEGGER_TERMINAL_CHANNEL, sFormat, &param_list);
    va_end(param_list);
    return ret;
}

uint32_t segger_send_raw(const uint8_t *p_data, uint32_t length)
{
    if ((NULL == p_data) || (0 == length))
    {
        return 0;
    }

    /* Temporarily switch to blocking mode to ensure all bytes are transferred without FIFO overflow drop */
    SEGGER_RTT_SetFlagsUpBuffer(SEGGER_TERMINAL_CHANNEL, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    uint32_t bytes_sent = 0;
    const uint32_t chunk_size = 256;

    while (bytes_sent < length)
    {
        uint32_t to_write = length - bytes_sent;
        if (to_write > chunk_size)
        {
            to_write = chunk_size;
        }

        unsigned written = SEGGER_RTT_Write(SEGGER_TERMINAL_CHANNEL, &p_data[bytes_sent], to_write);
        bytes_sent += written;
    }

    /* Restore to non-blocking mode */
    SEGGER_RTT_SetFlagsUpBuffer(SEGGER_TERMINAL_CHANNEL, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    return bytes_sent;
}

void segger_send_hex_dump(const uint8_t *p_data, uint32_t length, uint32_t bytes_per_row)
{
    if ((NULL == p_data) || (0 == length))
    {
        return;
    }

    if (bytes_per_row == 0 || bytes_per_row > 32)
    {
        bytes_per_row = 16;
    }

    segger_printf("\r\n--- HEX DUMP (%u bytes) ---\r\n", length);

    for (uint32_t i = 0; i < length; i += bytes_per_row)
    {
        segger_printf("%04X: ", i);

        uint32_t count = length - i;
        if (count > bytes_per_row)
        {
            count = bytes_per_row;
        }

        for (uint32_t j = 0; j < count; j++)
        {
            segger_printf("%02X ", p_data[i + j]);
        }
        segger_printf("\r\n");
    }

    segger_printf("--- END HEX DUMP ---\r\n\r\n");
}

void segger_send_framed_packet(uint8_t packet_id, const uint8_t *p_data, uint32_t length)
{
    /* Packet Structure:
     * [0..3]: Magic 0xAA, 0x55, 0xDE, 0xAD
     * [4]:    Packet ID (e.g. 0x00: Image 0, 0x01: Image 1, 0x02: Telemetry)
     * [5..8]: Payload length (uint32_t, little-endian)
     * [9..N]: Payload bytes
     * [N+1..N+4]: CRC32 (uint32_t, little-endian)
     */
    uint8_t header[9];
    header[0] = SEGGER_FRAME_MAGIC_0;
    header[1] = SEGGER_FRAME_MAGIC_1;
    header[2] = SEGGER_FRAME_MAGIC_2;
    header[3] = SEGGER_FRAME_MAGIC_3;
    header[4] = packet_id;
    header[5] = (uint8_t)(length & 0xFFU);
    header[6] = (uint8_t)((length >> 8) & 0xFFU);
    header[7] = (uint8_t)((length >> 16) & 0xFFU);
    header[8] = (uint8_t)((length >> 24) & 0xFFU);

    uint32_t crc = segger_calc_crc32(p_data, length);
    uint8_t footer[4];
    footer[0] = (uint8_t)(crc & 0xFFU);
    footer[1] = (uint8_t)((crc >> 8) & 0xFFU);
    footer[2] = (uint8_t)((crc >> 16) & 0xFFU);
    footer[3] = (uint8_t)((crc >> 24) & 0xFFU);

    /* Check if host is draining buffer; timeout if host is absent or disconnected */
    uint32_t timeout = 50000;
    while ((SEGGER_RTT_GetAvailWriteSpace(SEGGER_TERMINAL_CHANNEL) < sizeof(header)) && (--timeout > 0));
    if (timeout == 0) return; /* Host not reading, abort safely */

    SEGGER_RTT_Write(SEGGER_TERMINAL_CHANNEL, header, sizeof(header));
    if (length > 0 && p_data != NULL)
    {
        uint32_t sent = 0;
        const uint32_t chunk_size = 256;
        while (sent < length)
        {
            uint32_t to_write = length - sent;
            if (to_write > chunk_size)
            {
                to_write = chunk_size;
            }

            timeout = 100000;
            while ((SEGGER_RTT_GetAvailWriteSpace(SEGGER_TERMINAL_CHANNEL) < to_write) && (--timeout > 0));
            if (timeout == 0) return; /* Stalled, abort safely */

            SEGGER_RTT_Write(SEGGER_TERMINAL_CHANNEL, &p_data[sent], to_write);
            sent += to_write;
        }
    }

    timeout = 50000;
    while ((SEGGER_RTT_GetAvailWriteSpace(SEGGER_TERMINAL_CHANNEL) < sizeof(footer)) && (--timeout > 0));
    if (timeout > 0)
    {
        SEGGER_RTT_Write(SEGGER_TERMINAL_CHANNEL, footer, sizeof(footer));
    }
}

bool segger_has_key(void)
{
    return (SEGGER_RTT_HasKey() > 0);
}

int segger_get_key(void)
{
    return SEGGER_RTT_GetKey();
}
