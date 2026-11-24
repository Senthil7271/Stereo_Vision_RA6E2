/*******************************************************************************************************************
 * @file    sd_card.h
 * @brief   Micro SD / SDHC SPI Protocol Driver for Renesas BGK-RA6E2.
 *          Supports standard 32GB FAT32 SDHC cards with 512-byte sector block addressing.
 *******************************************************************************************************************/

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SD_SUCCESS             = 0,
    SD_ERR_TIMEOUT         = 1,
    SD_ERR_IDLE_STATE      = 2,
    SD_ERR_VOLTAGE_REJECT  = 3,
    SD_ERR_ACMD41_TIMEOUT  = 4,
    SD_ERR_OCR_READ        = 5,
    SD_ERR_READ_TOKEN      = 6,
    SD_ERR_NOT_INITIALIZED = 7
} sd_err_t;

typedef struct
{
    bool     initialized;
    bool     is_sdhc;          /* True for 32GB SDHC (Block addressed) */
    uint32_t total_sectors;    /* Capacity in 512-byte blocks */
} sd_card_info_t;

extern sd_card_info_t    g_sd_card_info;
extern volatile uint8_t  g_sd_last_cmd0_res;
extern volatile sd_err_t g_sd_init_error;

/* Public API */
sd_err_t    sd_card_init(void);
sd_err_t    sd_read_sector(uint32_t sector_lba, uint8_t *p_buffer);
sd_err_t    sd_read_sectors(uint32_t start_lba, uint32_t count, uint8_t *p_buffer);
const char* sd_get_error_str(sd_err_t err);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
