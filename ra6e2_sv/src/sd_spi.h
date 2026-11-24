/*******************************************************************************************************************
 * @file    sd_spi.h
 * @brief   High-Speed SPI Hardware/GPIO Interface for Micro SD Card Adapter on Renesas BGK-RA6E2.
 *          Target MCU: Renesas RA6E2 (Arm Cortex-M33 @ 200 MHz) - BGK-RA6E2 MCU Board
 *
 * Dedicated Arduino Interface Pins (Header Pins 25 to 29):
 *   - CS   (D10, Pin 25) : P301 (Port 3, Pin 1 - GPIO Output, Active Low)
 *   - MOSI (D11, Pin 26) : P109 (Port 1, Pin 9 - GPIO Output)
 *   - MISO (D12, Pin 27) : P110 (Port 1, Pin 10 - GPIO Input with Internal Pull-Up)
 *   - SCK  (D13, Pin 28) : P111 (Port 1, Pin 11 - GPIO Output, SPI Mode 0: CPOL=0, CPHA=0)
 *   - GND  (Pin 29)      : Ground
 *   - VCC  (Pin 10 / 11) : +5V / +3.3V Power Supply
 *******************************************************************************************************************/

#ifndef SD_SPI_H
#define SD_SPI_H

#include "bsp_api.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pin definitions for BGK-RA6E2 Arduino Interface */
#define SD_CS_PIN      BSP_IO_PORT_03_PIN_01   /* D10 (Pin 25) */
#define SD_MOSI_PIN    BSP_IO_PORT_01_PIN_09   /* D11 (Pin 26) */
#define SD_MISO_PIN    BSP_IO_PORT_01_PIN_10   /* D12 (Pin 27) */
#define SD_SCK_PIN     BSP_IO_PORT_01_PIN_11   /* D13 (Pin 28) */

#define SD_CS_BIT      (1U)    /* P301 on Port 3 */
#define SD_MOSI_BIT    (9U)    /* P109 on Port 1 */
#define SD_MISO_BIT    (10U)   /* P110 on Port 1 */
#define SD_SCK_BIT     (11U)   /* P111 on Port 1 */

/* Direct register-level single-cycle pin manipulation on Port 1 & Port 3 */
#define SD_CS_HIGH()   do { R_PORT3->PCNTR3 = (1U << SD_CS_BIT); } while(0)
#define SD_CS_LOW()    do { R_PORT3->PCNTR3 = (1U << (16 + SD_CS_BIT)); } while(0)

#define SD_SCK_HIGH()  do { R_PORT1->PCNTR3 = (1U << SD_SCK_BIT); } while(0)
#define SD_SCK_LOW()   do { R_PORT1->PCNTR3 = (1U << (16 + SD_SCK_BIT)); } while(0)

#define SD_MOSI_HIGH() do { R_PORT1->PCNTR3 = (1U << SD_MOSI_BIT); } while(0)
#define SD_MOSI_LOW()  do { R_PORT1->PCNTR3 = (1U << (16 + SD_MOSI_BIT)); } while(0)

#define SD_MISO_READ() ((uint8_t)((R_PORT1->PCNTR2 >> SD_MISO_BIT) & 1U))

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
