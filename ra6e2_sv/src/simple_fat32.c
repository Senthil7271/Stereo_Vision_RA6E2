/*******************************************************************************************************************
 * @file    simple_fat32.c
 * @brief   Lightweight Read-Only FAT32 Filesystem Implementation for Renesas FPB-RA6E2.
 *******************************************************************************************************************/

#include "simple_fat32.h"
#include "sd_card.h"
#include <string.h>
#include <ctype.h>

fat32_volume_t g_fat32_vol = { 0 };

/* Single shared 512-byte scratch buffer for metadata */
static uint8_t s_sector_buf[512];

/* Helper to convert standard filename like "L0001.BIN" to FAT 8.3 format "L0001   BIN" */
static void format_8_3(const char *in_name, char *out_83)
{
    memset(out_83, ' ', 11);
    out_83[11] = '\0';

    int name_idx = 0;
    while (*in_name && *in_name != '.' && name_idx < 8)
    {
        out_83[name_idx++] = (char)toupper((unsigned char)*in_name++);
    }

    while (*in_name && *in_name != '.')
    {
        in_name++;
    }

    if (*in_name == '.')
    {
        in_name++;
        int ext_idx = 8;
        while (*in_name && ext_idx < 11)
        {
            out_83[ext_idx++] = (char)toupper((unsigned char)*in_name++);
        }
    }
}

static uint32_t cluster_to_lba(uint32_t cluster)
{
    return g_fat32_vol.cluster_begin_lba + ((cluster - 2U) * g_fat32_vol.sectors_per_cluster);
}

/* Reads the next cluster from the FAT table */
static uint32_t get_next_cluster(uint32_t current_cluster)
{
    uint32_t fat_sector_lba = g_fat32_vol.fat_lba + ((current_cluster * 4U) / 512U);
    uint32_t fat_offset = (current_cluster * 4U) % 512U;

    if (sd_read_sector(fat_sector_lba, s_sector_buf) != SD_SUCCESS)
    {
        return 0x0FFFFFFF; /* End of chain on error */
    }

    uint32_t next_cluster = (uint32_t)(s_sector_buf[fat_offset] |
                                      (s_sector_buf[fat_offset + 1] << 8) |
                                      (s_sector_buf[fat_offset + 2] << 16) |
                                      (s_sector_buf[fat_offset + 3] << 24));

    return next_cluster & 0x0FFFFFFF;
}

fat32_err_t fat32_mount(void)
{
    g_fat32_vol.mounted = false;

    /* 1. Ensure SD card is initialized */
    if (sd_card_init() != SD_SUCCESS)
    {
        return FAT32_ERR_SD_FAIL;
    }

    /* 2. Read Sector 0 (MBR or VBR) */
    if (sd_read_sector(0, s_sector_buf) != SD_SUCCESS)
    {
        return FAT32_ERR_READ_FAIL;
    }

    if (s_sector_buf[510] != 0x55 || s_sector_buf[511] != 0xAA)
    {
        return FAT32_ERR_NO_MBR;
    }

    /* Check if Sector 0 is MBR (Partition Entry at 0x1BE) */
    uint32_t partition_lba = 0;
    uint8_t part_type = s_sector_buf[0x1BE + 4];
    if (part_type == 0x0B || part_type == 0x0C) /* FAT32 Partition */
    {
        partition_lba = (uint32_t)(s_sector_buf[0x1BE + 8] |
                                  (s_sector_buf[0x1BE + 9] << 8) |
                                  (s_sector_buf[0x1BE + 10] << 16) |
                                  (s_sector_buf[0x1BE + 11] << 24));

        /* Read Volume Boot Record (VBR) */
        if (sd_read_sector(partition_lba, s_sector_buf) != SD_SUCCESS)
        {
            return FAT32_ERR_READ_FAIL;
        }
    }
    else if (s_sector_buf[0] != 0xEB && s_sector_buf[0] != 0xE9)
    {
        /* Try partition 1 anyway if non-standard ID */
        partition_lba = (uint32_t)(s_sector_buf[0x1BE + 8] |
                                  (s_sector_buf[0x1BE + 9] << 8) |
                                  (s_sector_buf[0x1BE + 10] << 16) |
                                  (s_sector_buf[0x1BE + 11] << 24));
        if (partition_lba != 0)
        {
            sd_read_sector(partition_lba, s_sector_buf);
        }
    }

    /* 3. Parse BPB parameters */
    uint16_t bytes_per_sector = (uint16_t)(s_sector_buf[0x0B] | (s_sector_buf[0x0C] << 8));
    if (bytes_per_sector != 512)
    {
        return FAT32_ERR_NO_FAT32;
    }

    g_fat32_vol.sectors_per_cluster = s_sector_buf[0x0D];
    uint16_t reserved_sectors = (uint16_t)(s_sector_buf[0x0E] | (s_sector_buf[0x0F] << 8));
    uint8_t num_fats = s_sector_buf[0x10];

    g_fat32_vol.sectors_per_fat = (uint32_t)(s_sector_buf[0x24] |
                                           (s_sector_buf[0x25] << 8) |
                                           (s_sector_buf[0x26] << 16) |
                                           (s_sector_buf[0x27] << 24));

    g_fat32_vol.root_dir_cluster = (uint32_t)(s_sector_buf[0x2C] |
                                            (s_sector_buf[0x2D] << 8) |
                                            (s_sector_buf[0x2E] << 16) |
                                            (s_sector_buf[0x2F] << 24));

    g_fat32_vol.partition_lba = partition_lba;
    g_fat32_vol.fat_lba = partition_lba + reserved_sectors;
    g_fat32_vol.cluster_begin_lba = g_fat32_vol.fat_lba + (num_fats * g_fat32_vol.sectors_per_fat);
    g_fat32_vol.mounted = true;

    return FAT32_OK;
}

/* Searches a directory cluster chain for a matching 8.3 entry */
static fat32_err_t find_entry_in_cluster(uint32_t cluster, const char *name_83, bool is_dir,
                                         uint32_t *p_first_cluster, uint32_t *p_size)
{
    while (cluster < 0x0FFFFFF8 && cluster >= 2)
    {
        uint32_t cluster_lba = cluster_to_lba(cluster);

        for (uint8_t s = 0; s < g_fat32_vol.sectors_per_cluster; s++)
        {
            if (sd_read_sector(cluster_lba + s, s_sector_buf) != SD_SUCCESS)
            {
                return FAT32_ERR_READ_FAIL;
            }

            /* Each sector has 16 directory entries (32 bytes each) */
            for (int e = 0; e < 16; e++)
            {
                uint8_t *entry = &s_sector_buf[e * 32];

                if (entry[0] == 0x00)
                {
                    return is_dir ? FAT32_ERR_DIR_NOT_FOUND : FAT32_ERR_FILE_NOT_FOUND;
                }
                if (entry[0] == 0xE5)
                {
                    continue; /* Deleted entry */
                }
                if (entry[11] == 0x0F)
                {
                    continue; /* Long File Name entry */
                }

                /* Check attribute and name */
                bool entry_is_dir = (entry[11] & 0x10) ? true : false;
                if (entry_is_dir == is_dir)
                {
                    if (memcmp(entry, name_83, 11) == 0)
                    {
                        uint16_t high = (uint16_t)(entry[20] | (entry[21] << 8));
                        uint16_t low  = (uint16_t)(entry[26] | (entry[27] << 8));
                        *p_first_cluster = ((uint32_t)high << 16) | low;

                        if (p_size)
                        {
                            *p_size = (uint32_t)(entry[28] | (entry[29] << 8) |
                                                (entry[30] << 16) | (entry[31] << 24));
                        }
                        return FAT32_OK;
                    }
                }
            }
        }

        cluster = get_next_cluster(cluster);
    }

    return is_dir ? FAT32_ERR_DIR_NOT_FOUND : FAT32_ERR_FILE_NOT_FOUND;
}

fat32_err_t fat32_open(const char *dir_name, const char *file_name, fat32_file_t *p_file)
{
    if (!g_fat32_vol.mounted)
    {
        return FAT32_ERR_SD_FAIL;
    }

    uint32_t search_cluster = g_fat32_vol.root_dir_cluster;

    /* 1. If directory is specified, search for directory cluster first */
    if (dir_name && strlen(dir_name) > 0)
    {
        char dir_83[12];
        format_8_3(dir_name, dir_83);

        uint32_t dir_cluster = 0;
        fat32_err_t err = find_entry_in_cluster(search_cluster, dir_83, true, &dir_cluster, NULL);
        if (err != FAT32_OK)
        {
            return err;
        }
        search_cluster = dir_cluster;
    }

    /* 2. Search for the file in the resolved directory */
    char file_83[12];
    format_8_3(file_name, file_83);

    uint32_t file_cluster = 0;
    uint32_t file_size = 0;
    fat32_err_t err = find_entry_in_cluster(search_cluster, file_83, false, &file_cluster, &file_size);
    if (err != FAT32_OK)
    {
        return err;
    }

    p_file->first_cluster = file_cluster;
    p_file->current_cluster = file_cluster;
    p_file->current_cluster_sector = 0;
    p_file->file_size = file_size;
    p_file->bytes_read = 0;

    return FAT32_OK;
}

/* Fast block read directly into user buffer (e.g. 16,384 bytes for 128x128 image) */
fat32_err_t fat32_read(fat32_file_t *p_file, uint8_t *p_dest, uint32_t bytes_to_read, uint32_t *p_actual_read)
{
    if (!g_fat32_vol.mounted || !p_file)
    {
        return FAT32_ERR_SD_FAIL;
    }

    if (p_file->bytes_read + bytes_to_read > p_file->file_size)
    {
        bytes_to_read = p_file->file_size - p_file->bytes_read;
    }

    uint32_t bytes_remaining = bytes_to_read;
    uint32_t dest_offset = 0;

    while (bytes_remaining > 0 && p_file->current_cluster < 0x0FFFFFF8)
    {
        uint32_t cluster_lba = cluster_to_lba(p_file->current_cluster);
        uint32_t sector_lba = cluster_lba + p_file->current_cluster_sector;

        if (bytes_remaining >= 512)
        {
            /* Direct transfer straight into destination buffer */
            if (sd_read_sector(sector_lba, &p_dest[dest_offset]) != SD_SUCCESS)
            {
                return FAT32_ERR_READ_FAIL;
            }
            dest_offset += 512;
            bytes_remaining -= 512;
            p_file->bytes_read += 512;
        }
        else
        {
            /* Partial sector read */
            if (sd_read_sector(sector_lba, s_sector_buf) != SD_SUCCESS)
            {
                return FAT32_ERR_READ_FAIL;
            }
            memcpy(&p_dest[dest_offset], s_sector_buf, bytes_remaining);
            dest_offset += bytes_remaining;
            p_file->bytes_read += bytes_remaining;
            bytes_remaining = 0;
        }

        /* Advance to next sector / cluster */
        p_file->current_cluster_sector++;
        if (p_file->current_cluster_sector >= g_fat32_vol.sectors_per_cluster)
        {
            p_file->current_cluster_sector = 0;
            p_file->current_cluster = get_next_cluster(p_file->current_cluster);
        }
    }

    if (p_actual_read)
    {
        *p_actual_read = dest_offset;
    }

    return FAT32_OK;
}
