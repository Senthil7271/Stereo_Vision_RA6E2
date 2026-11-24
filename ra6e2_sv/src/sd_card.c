/*******************************************************************************************************************
 * @file    sd_card.c
 * @brief   Micro SD / SDHC SPI Protocol Driver Implementation for Renesas FPB-RA6E2.
 *******************************************************************************************************************/

#include "sd_card.h"
#include "sd_spi.h"

/* Standard SD Card SPI Commands */
#define CMD0_GO_IDLE_STATE           (0x40 | 0)
#define CMD8_SEND_IF_COND            (0x40 | 8)
#define CMD16_SET_BLOCKLEN           (0x40 | 16)
#define CMD17_READ_SINGLE_BLOCK      (0x40 | 17)
#define CMD55_APP_CMD                (0x40 | 55)
#define CMD58_READ_OCR               (0x40 | 58)
#define ACMD41_SD_SEND_OP_COND       (0x40 | 41)

#define SD_TOKEN_START_BLOCK         (0xFE)
#define SD_CMD_TIMEOUT               (2000U)
#define SD_READ_TIMEOUT              (50000U)

sd_card_info_t g_sd_card_info = { 0 };

/* Sends a 6-byte SD SPI command frame */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t res = 0xFF;

    /* Deselect then select to sync */
    SD_CS_HIGH();
    sd_spi_tx_rx_byte(0xFF);
    SD_CS_LOW();
    sd_spi_tx_rx_byte(0xFF);

    /* Send 6-byte command frame */
    sd_spi_tx_rx_byte(cmd);
    sd_spi_tx_rx_byte((uint8_t)(arg >> 24));
    sd_spi_tx_rx_byte((uint8_t)(arg >> 16));
    sd_spi_tx_rx_byte((uint8_t)(arg >> 8));
    sd_spi_tx_rx_byte((uint8_t)(arg));
    sd_spi_tx_rx_byte(crc);

    /* Wait for R1 response (MSB must be 0) */
    for (uint32_t i = 0; i < SD_CMD_TIMEOUT; i++)
    {
        res = sd_spi_tx_rx_byte(0xFF);
        if ((res & 0x80) == 0)
        {
            break;
        }
    }

    return res;
}

sd_err_t sd_card_init(void)
{
    uint8_t res;
    uint32_t timeout;

    g_sd_card_info.initialized = false;
    g_sd_card_info.is_sdhc = false;

    /* 1. Initialize low-level SPI pins */
    sd_spi_init();
    sd_spi_set_high_speed(false); /* Slow clock (<400 kHz) for init */

    /* 2. Send 80+ clock cycles with CS High to power on the card */
    sd_spi_send_clocks(100);

    /* 3. Send CMD0 to enter SPI mode */
    timeout = 100;
    do
    {
        res = sd_send_cmd(CMD0_GO_IDLE_STATE, 0, 0x95);
        timeout--;
    } while ((res != 0x01) && (timeout > 0));

    if (res != 0x01)
    {
        SD_CS_HIGH();
        sd_spi_tx_rx_byte(0xFF);
        return SD_ERR_IDLE_STATE;
    }

    /* 4. Send CMD8 to check voltage (2.7 - 3.6V pattern: 0x1AA) */
    res = sd_send_cmd(CMD8_SEND_IF_COND, 0x000001AA, 0x87);
    if (res == 0x01)
    {
        /* Read 4-byte R7 payload */
        uint8_t r7[4];
        for (int i = 0; i < 4; i++)
        {
            r7[i] = sd_spi_tx_rx_byte(0xFF);
        }

        if (r7[2] != 0x01 || r7[3] != 0xAA)
        {
            SD_CS_HIGH();
            return SD_ERR_VOLTAGE_REJECT;
        }
    }

    /* 5. Send ACMD41 in a loop until card leaves idle state (res == 0x00) */
    timeout = 2000;
    do
    {
        sd_send_cmd(CMD55_APP_CMD, 0, 0x65);
        /* Argument 0x40000000 sets HCS bit (High Capacity Support for 32GB) */
        res = sd_send_cmd(ACMD41_SD_SEND_OP_COND, 0x40000000, 0x77);
        timeout--;
    } while ((res != 0x00) && (timeout > 0));

    if (res != 0x00)
    {
        SD_CS_HIGH();
        return SD_ERR_ACMD41_TIMEOUT;
    }

    /* 6. Read OCR (CMD58) to check Card Capacity Status (CCS) */
    res = sd_send_cmd(CMD58_READ_OCR, 0, 0xFD);
    if (res == 0x00)
    {
        uint8_t ocr[4];
        for (int i = 0; i < 4; i++)
        {
            ocr[i] = sd_spi_tx_rx_byte(0xFF);
        }

        /* Bit 30 of OCR is CCS (1 = SDHC/SDXC block-addressed; 0 = Byte-addressed) */
        if (ocr[0] & 0x40)
        {
            g_sd_card_info.is_sdhc = true;
        }
    }

    /* Set block size to 512 bytes for SD v1 / byte cards */
    if (!g_sd_card_info.is_sdhc)
    {
        sd_send_cmd(CMD16_SET_BLOCKLEN, 512, 0xFF);
    }

    /* 7. Deselect card and switch to high speed */
    SD_CS_HIGH();
    sd_spi_tx_rx_byte(0xFF);

    sd_spi_set_high_speed(true); /* Switch to high speed (>12 MHz) */
    g_sd_card_info.initialized = true;

    return SD_SUCCESS;
}

sd_err_t sd_read_sector(uint32_t sector_lba, uint8_t *p_buffer)
{
    if (!g_sd_card_info.initialized)
    {
        return SD_ERR_NOT_INITIALIZED;
    }

    /* For standard SD (non-SDHC), address is byte-offset; for SDHC (32GB), it is sector LBA */
    uint32_t addr = g_sd_card_info.is_sdhc ? sector_lba : (sector_lba * 512U);

    uint8_t res = sd_send_cmd(CMD17_READ_SINGLE_BLOCK, addr, 0xFF);
    if (res != 0x00)
    {
        SD_CS_HIGH();
        sd_spi_tx_rx_byte(0xFF);
        return SD_ERR_TIMEOUT;
    }

    /* Wait for data start token (0xFE) */
    uint8_t token = 0xFF;
    for (uint32_t i = 0; i < SD_READ_TIMEOUT; i++)
    {
        token = sd_spi_tx_rx_byte(0xFF);
        if (token == SD_TOKEN_START_BLOCK)
        {
            break;
        }
    }

    if (token != SD_TOKEN_START_BLOCK)
    {
        SD_CS_HIGH();
        sd_spi_tx_rx_byte(0xFF);
        return SD_ERR_READ_TOKEN;
    }

    /* Read exactly 512 data bytes */
    sd_spi_read_buffer(p_buffer, 512);

    /* Discard 2 CRC bytes */
    sd_spi_tx_rx_byte(0xFF);
    sd_spi_tx_rx_byte(0xFF);

    /* Deselect card and send flush clocks */
    SD_CS_HIGH();
    sd_spi_tx_rx_byte(0xFF);

    return SD_SUCCESS;
}

sd_err_t sd_read_sectors(uint32_t start_lba, uint32_t count, uint8_t *p_buffer)
{
    sd_err_t err;
    for (uint32_t s = 0; s < count; s++)
    {
        err = sd_read_sector(start_lba + s, &p_buffer[s * 512U]);
        if (err != SD_SUCCESS)
        {
            return err;
        }
    }
    return SD_SUCCESS;
}
