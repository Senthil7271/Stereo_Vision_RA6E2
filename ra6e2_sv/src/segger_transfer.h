/*******************************************************************************************************************
 * @file    segger_transfer.h
 * @brief   SEGGER RTT Data Transfer Interface for Renesas RA6E2.
 *          Provides formatted text, raw binary streaming, and framed image transfer over J-Link RTT.
 *******************************************************************************************************************/

#ifndef SEGGER_TRANSFER_H_
#define SEGGER_TRANSFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "SEGGER_RTT/SEGGER_RTT.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Packet framing magic header for binary streaming */
#define SEGGER_FRAME_MAGIC_0        (0xAAU)
#define SEGGER_FRAME_MAGIC_1        (0x55U)
#define SEGGER_FRAME_MAGIC_2        (0xDEU)
#define SEGGER_FRAME_MAGIC_3        (0xADU)

/* Channel definitions */
#define SEGGER_TERMINAL_CHANNEL     (0U)

/**
 * @brief  Initialize SEGGER RTT channels and buffers.
 */
void segger_transfer_init(void);

/**
 * @brief  Print formatted string to PC over RTT Channel 0.
 */
int segger_printf(const char *sFormat, ...);

/**
 * @brief  Send raw binary byte stream over RTT Channel 0.
 *         Handles chunking to prevent buffer overflow.
 *
 * @param  p_data  Pointer to source data buffer
 * @param  length  Number of bytes to transfer
 * @return Total bytes transmitted
 */
uint32_t segger_send_raw(const uint8_t *p_data, uint32_t length);

/**
 * @brief  Send data formatted as ASCII Hex with row offsets (viewable directly in RTT Viewer).
 *
 * @param  p_data      Pointer to source data buffer
 * @param  length      Number of bytes
 * @param  bytes_per_row Number of bytes per line (e.g. 16 or 32)
 */
void segger_send_hex_dump(const uint8_t *p_data, uint32_t length, uint32_t bytes_per_row);

/**
 * @brief  Send framed binary packet with Header + Length + Payload + CRC32.
 *         Ideal for automated reception by PC scripts.
 *
 * @param  packet_id   Identifier / type of packet (e.g. 0 for Image 0, 1 for Image 1)
 * @param  p_data      Pointer to payload
 * @param  length      Length of payload in bytes
 */
void segger_send_framed_packet(uint8_t packet_id, const uint8_t *p_data, uint32_t length);

/**
 * @brief  Check if host PC has sent a key / character.
 * @return true if character is available, false otherwise.
 */
bool segger_has_key(void);

/**
 * @brief  Read one character sent from host PC.
 * @return Character code (>= 0) or -1 if no character is available.
 */
int segger_get_key(void);

/**
 * @brief  Compute 32-bit CRC (standard IEEE 802.3 polynomial 0xEDB88320)
 */
uint32_t segger_calc_crc32(const uint8_t *p_data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* SEGGER_TRANSFER_H_ */
