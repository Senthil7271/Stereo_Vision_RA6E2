/*******************************************************************************************************************
 * @file    sd_spi.c
 * @brief   High-Speed SPI Hardware/GPIO Interface for Micro SD Card Adapter on Renesas BGK-RA6E2.
 *          Pins configured on Arduino Interface:
 *            - D10 (Pin 25): P301 (CS Output)
 *            - D11 (Pin 26): P109 (MOSI Output)
 *            - D12 (Pin 27): P110 (MISO Input with Pull-Up)
 *            - D13 (Pin 28): P111 (SCK Output)
 *******************************************************************************************************************/

#include "sd_spi.h"

static bool g_high_speed_mode = false;

/* Short delay for slow speed (SD initialization ~300-400 kHz) */
static inline void sd_spi_delay(void)
{
    if (!g_high_speed_mode)
    {
        /* ~150-200 cycles delay @ 200 MHz = ~1 microsecond */
        for (volatile int i = 0; i < 40; i++)
        {
            __NOP();
        }
    }
}

void sd_spi_init(void)
{
    /* Enable write access to PFS registers */
    R_BSP_PinAccessEnable();

    /* D10: P301 (Pin 25) CS Output, High by default */
    R_PFS->PORT[3].PIN[1].PmnPFS = 0x00000005U; /* PDR=1, PODR=1, PMR=0 (GPIO Out High) */

    /* D13: P111 (Pin 28) SCK Output, Low by default */
    R_PFS->PORT[1].PIN[11].PmnPFS = 0x00000004U; /* PDR=1, PODR=0, PMR=0 (GPIO Out Low) */

    /* D11: P109 (Pin 26) MOSI Output, High by default */
    R_PFS->PORT[1].PIN[9].PmnPFS = 0x00000005U; /* PDR=1, PODR=1, PMR=0 (GPIO Out High) */

    /* D12: P110 (Pin 27) MISO Input with Pull-up enabled */
    R_PFS->PORT[1].PIN[10].PmnPFS = 0x00000010U; /* PDR=0, PCR=1 (Pull-up), PMR=0 (GPIO In) */

    /* Lock PFS registers */
    R_BSP_PinAccessDisable();

    /* Set default states */
    SD_CS_HIGH();
    SD_SCK_LOW();
    SD_MOSI_HIGH();

    g_high_speed_mode = false;
}

void sd_spi_set_high_speed(bool enable_high_speed)
{
    g_high_speed_mode = enable_high_speed;
}

/* Transfers one byte over SPI Mode 0 (MSB first, CPOL=0, CPHA=0) */
uint8_t sd_spi_tx_rx_byte(uint8_t byte)
{
    uint8_t rx = 0;

    for (int i = 7; i >= 0; i--)
    {
        /* Set MOSI bit (P109) */
        if (byte & (1U << i))
        {
            SD_MOSI_HIGH();
        }
        else
        {
            SD_MOSI_LOW();
        }

        sd_spi_delay();

        /* Clock High (P111): SD card samples MOSI, asserts MISO */
        SD_SCK_HIGH();

        sd_spi_delay();

        /* Sample MISO bit (P110) */
        if (SD_MISO_READ())
        {
            rx |= (uint8_t)(1U << i);
        }

        /* Clock Low (P111) */
        SD_SCK_LOW();
    }

    /* Idle MOSI high */
    SD_MOSI_HIGH();

    return rx;
}

/* High-speed block receive optimized for 512-byte sector transfers */
void sd_spi_read_buffer(uint8_t *p_dest, uint32_t length)
{
    SD_MOSI_HIGH();

    for (uint32_t b = 0; b < length; b++)
    {
        uint8_t rx = 0;

        for (int i = 7; i >= 0; i--)
        {
            SD_SCK_HIGH();
            if (SD_MISO_READ())
            {
                rx |= (uint8_t)(1U << i);
            }
            SD_SCK_LOW();
        }

        p_dest[b] = rx;
    }
}

/* Sends clock cycles with CS High to power-up / flush SD card bus */
void sd_spi_send_clocks(uint32_t count)
{
    SD_CS_HIGH();
    SD_MOSI_HIGH();

    for (uint32_t i = 0; i < count; i++)
    {
        SD_SCK_HIGH();
        sd_spi_delay();
        SD_SCK_LOW();
        sd_spi_delay();
    }
}
