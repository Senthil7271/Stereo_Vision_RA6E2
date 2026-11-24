/*******************************************************************************************************************
 * @file    simple_fat32.h
 * @brief   Lightweight Read-Only FAT32 Filesystem Parser for Renesas BGK-RA6E2.
 *******************************************************************************************************************/

#ifndef SIMPLE_FAT32_H
#define SIMPLE_FAT32_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FAT32_OK               = 0,
    FAT32_ERR_SD_FAIL      = 1,
    FAT32_ERR_NO_MBR       = 2,
    FAT32_ERR_NO_FAT32     = 3,
    FAT32_ERR_DIR_NOT_FOUND= 4,
    FAT32_ERR_FILE_NOT_FOUND=5,
    FAT32_ERR_READ_FAIL    = 6
} fat32_err_t;

typedef struct
{
    uint32_t first_cluster;
    uint32_t current_cluster;
    uint32_t current_cluster_sector;
    uint32_t file_size;
    uint32_t bytes_read;
} fat32_file_t;

typedef struct
{
    uint32_t partition_lba;
    uint32_t fat_lba;
    uint32_t cluster_begin_lba;
    uint32_t sectors_per_fat;
    uint32_t root_dir_cluster;
    uint8_t  sectors_per_cluster;
    bool     mounted;
} fat32_volume_t;

extern fat32_volume_t       g_fat32_vol;
extern volatile fat32_err_t g_fat32_last_err;

/* Public API */
fat32_err_t fat32_mount(void);
fat32_err_t fat32_open(const char *dir_name, const char *file_name, fat32_file_t *p_file);
fat32_err_t fat32_read(fat32_file_t *p_file, uint8_t *p_dest, uint32_t bytes_to_read, uint32_t *p_actual_read);
const char* fat32_get_error_str(fat32_err_t err);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_FAT32_H */
