/*******************************************************************************************************************
 * @file    sd_spi.h
 * @brief   High-Speed SPI Hardware/GPIO Interface for Micro SD Card Adapter on Renesas FPB-RA6E2.
 *          Target MCU: Renesas RA6E2 (Arm Cortex-M33 @ 200 MHz)
 *
 * Pin Mapping on Header J1:
 *   - CS   (Chip Select) : P205 (GPIO Output, Active Low)
 *   - SCK  (Clock)       : P204 (Output, SPI Mode 0: CPOL=0, CPHA=0)
 *   - MOSI (Master Out)  : P203 (Output)
 *   - MISO (Master In)   : P202 (Input with internal pull-up)
 *******************************************************************************************************************/

#ifndef SD_SPI_H
#define SD_SPI_H

#include "bsp_api.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pin definitions on Port 2 */
#define SD_CS_PIN      BSP_IO_PORT_02_PIN_05
#define SD_SCK_PIN     BSP_IO_PORT_02_PIN_04
#define SD_MOSI_PIN    BSP_IO_PORT_02_PIN_03
#define SD_MISO_PIN    BSP_IO_PORT_02_PIN_02

#define SD_CS_BIT      (5U)
#define SD_SCK_BIT     (4U)
#define SD_MOSI_BIT    (3U)
#define SD_MISO_BIT    (2U)

/* Direct register-level single-cycle pin manipulation on Port 2 */
#define SD_CS_HIGH()   do { R_PORT2->PCNTR3 = (1U << SD_CS_BIT); } while(0)
#define SD_CS_LOW()    do { R_PORT2->PCNTR3 = (1U << (16 + SD_CS_BIT)); } while(0)

#define SD_SCK_HIGH()  do { R_PORT2->PCNTR3 = (1U << SD_SCK_BIT); } while(0)
#define SD_SCK_LOW()   do { R_PORT2->PCNTR3 = (1U << (16 + SD_SCK_BIT)); } while(0)

#define SD_MOSI_HIGH() do { R_PORT2->PCNTR3 = (1U << SD_MOSI_BIT); } while(0)
#define SD_MOSI_LOW()  do { R_PORT2->PCNTR3 = (1U << (16 + SD_MOSI_BIT)); } while(0)

#define SD_MISO_READ() ((uint8_t)((R_PORT2->PCNTR2 >> SD_MISO_BIT) & 1U))

/* Public SPI functions */
void     sd_spi_init(void);
void     sd_spi_set_high_speed(bool enable_high_speed);
uint8_t  sd_spi_tx_rx_byte(uint8_t byte);
void     sd_spi_read_buffer(uint8_t *p_dest, uint32_t length);
void     sd_spi_send_clocks(uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* SD_SPI_H */
