/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MICROPY_DEBUG_PRINTERS
#define MICROPY_DEBUG_PRINTERS      (1)
#endif

#define MICROPY_HW_BOARD_NAME       "alpha:ONE by systematica"
#define MICROPY_HW_MCU_NAME         "STM32F722IEK"

#define MICROPY_PY_PYB_LEGACY       (1)
#define MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE (0)
#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (1)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_TIMER     (1)
#define MICROPY_HW_ENABLE_SERVO     (1)
#define MICROPY_HW_ENABLE_DAC       (1)
#define MICROPY_HW_ENABLE_USB       (1)
#define MICROPY_HW_ENABLE_SDCARD    (1)
#define MICROPY_HW_ENABLE_MMCARD    (1)

#define MICROPY_BOARD_EARLY_INIT    board_early_init
#define MICROPY_BOARD_ENTER_STOP    board_sleep(1);
#define MICROPY_BOARD_LEAVE_STOP    board_sleep(0);
#define MICROPY_BOARD_ENTER_STANDBY board_sleep(1);
void board_early_init(void);
void board_sleep(int value);

// HSE is 25MHz, run SYS at 120MHz
#define MICROPY_HW_CLK_PLLM         (20)
#define MICROPY_HW_CLK_PLLN         (192)
#define MICROPY_HW_CLK_PLLP         (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ         (5)
#define MICROPY_HW_FLASH_LATENCY    (FLASH_LATENCY_3)

// There is an external 32kHz oscillator
#define RTC_ASYNCH_PREDIV           (3)
#define RTC_SYNCH_PREDIV            (0x1fff)
#define MICROPY_HW_RTC_USE_BYPASS   (1)
#define MICROPY_HW_RTC_USE_US       (1)
#define MICROPY_HW_RTC_USE_CALOUT   (1)

// SPI flash #1, for R/W storage -> mounted as FAT
#define MICROPY_HW_SOFTQSPI_SCK_LOW(self) (GPIOE->BSRR = (0x10000 << 11))
#define MICROPY_HW_SOFTQSPI_SCK_HIGH(self) (GPIOE->BSRR = (1 << 11))
#define MICROPY_HW_SOFTQSPI_NIBBLE_READ(self) ((GPIOE->IDR >> 7) & 0xf)
#define MICROPY_HW_SPIFLASH_SIZE_BITS (64 * 1024 * 1024)
#define MICROPY_HW_SPIFLASH_CS      (pyb_pin_QSPI1_CS)
#define MICROPY_HW_SPIFLASH_SCK     (pyb_pin_QSPI1_CLK)
#define MICROPY_HW_SPIFLASH_IO0     (pyb_pin_QSPI1_D0)
#define MICROPY_HW_SPIFLASH_IO1     (pyb_pin_QSPI1_D1)
#define MICROPY_HW_SPIFLASH_IO2     (pyb_pin_QSPI1_D2)
#define MICROPY_HW_SPIFLASH_IO3     (pyb_pin_QSPI1_D3)

// SPI flash #1, block device config
extern const struct _mp_spiflash_config_t spiflash_config;
extern struct _spi_bdev_t spi_bdev;
#if !BUILDING_MBOOT
#define MICROPY_HW_SPIFLASH_ENABLE_CACHE (1)
#endif
#define MICROPY_HW_BDEV_IOCTL(op, arg) ( \
    (op) == BDEV_IOCTL_NUM_BLOCKS ? (MICROPY_HW_SPIFLASH_SIZE_BITS / 8 / FLASH_BLOCK_SIZE) : \
    (op) == BDEV_IOCTL_INIT ? spi_bdev_ioctl(&spi_bdev, (op), (uint32_t)&spiflash_config) : \
    spi_bdev_ioctl(&spi_bdev, (op), (arg)) \
)
#define MICROPY_HW_BDEV_READBLOCKS(dest, bl, n) spi_bdev_readblocks(&spi_bdev, (dest), (bl), (n))
#define MICROPY_HW_BDEV_WRITEBLOCKS(src, bl, n) spi_bdev_writeblocks(&spi_bdev, (src), (bl), (n))
#define MICROPY_HW_BDEV_SPIFLASH_EXTENDED (&spi_bdev) // for extended block protocol

// SPI flash #2, to be memory mapped
#define MICROPY_HW_QSPIFLASH_SIZE_BITS_LOG2 (26)
#define MICROPY_HW_QSPIFLASH_CS     (pyb_pin_QSPI2_CS)
#define MICROPY_HW_QSPIFLASH_SCK    (pyb_pin_QSPI2_CLK)
#define MICROPY_HW_QSPIFLASH_IO0    (pyb_pin_QSPI2_D0)
#define MICROPY_HW_QSPIFLASH_IO1    (pyb_pin_QSPI2_D1)
#define MICROPY_HW_QSPIFLASH_IO2    (pyb_pin_QSPI2_D2)
#define MICROPY_HW_QSPIFLASH_IO3    (pyb_pin_QSPI2_D3)

// SPI flash #2, block device config
extern const struct _mp_spiflash_config_t spiflash2_config;
extern struct _spi_bdev_t spi_bdev2;


// UART config
#define MICROPY_HW_UART1_NAME       "RS485_1"
#define MICROPY_HW_UART1_TX         (pin_A9)
#define MICROPY_HW_UART1_RX         (pin_A10)
#define MICROPY_HW_UART1_RTS        (pin_A12)
#define MICROPY_HW_UART1_CTS        (pin_A11)

#define MICROPY_HW_UART3_NAME       "RS232_485"
#define MICROPY_HW_UART3_TX         (pin_D8)
#define MICROPY_HW_UART3_RX         (pin_D9)

#define MICROPY_HW_UART4_TX         (pin_H13)
#define MICROPY_HW_UART4_RX         (pin_H14)

#define MICROPY_HW_UART7_TX         (pin_F7)
#define MICROPY_HW_UART7_RX         (pin_F6)
#define MICROPY_HW_UART7_RTS        (pin_F8)
#define MICROPY_HW_UART7_CTS        (pin_F9)

#define MICROPY_HW_UART8_NAME       "RS485_2"
#define MICROPY_HW_UART8_TX         (pin_E1)
#define MICROPY_HW_UART8_RX         (pin_E0)
#define MICROPY_HW_UART8_RTS        (pin_D15)
#define MICROPY_HW_UART8_CTS        (pin_D14)


// I2C busses
#define MICROPY_HW_I2C2_SCL         (pin_B10)
#define MICROPY_HW_I2C2_SDA         (pin_B11)

#define MICROPY_HW_I2C3_SCL         (pin_H7)
#define MICROPY_HW_I2C3_SDA         (pin_H8)


// SPI busses

#define MICROPY_HW_SPI1_NAME        "status_LEDs"
#define MICROPY_HW_SPI1_SCK         (pin_B3)
//#define MICROPY_HW_SPI1_MISO        (pin_B4)
#define MICROPY_HW_SPI1_MOSI        (pin_B5)

#define MICROPY_HW_SPI2_NAME        "W5500"
#define MICROPY_HW_SPI2_NSS         (pin_H15)
#define MICROPY_HW_SPI2_SCK         (pin_I1)
#define MICROPY_HW_SPI2_MISO        (pin_I2)
#define MICROPY_HW_SPI2_MOSI        (pin_I3)

#define MICROPY_HW_SPI4_NSS         (pin_E4)
#define MICROPY_HW_SPI4_SCK         (pin_E12)
#define MICROPY_HW_SPI4_MISO        (pin_E5)
#define MICROPY_HW_SPI4_MOSI        (pin_E6)

#define MICROPY_HW_SPI5_NSS         (pin_F6)
#define MICROPY_HW_SPI5_SCK         (pin_F7)
#define MICROPY_HW_SPI5_MISO        (pin_F8)
#define MICROPY_HW_SPI5_MOSI        (pin_F9)


// CAN busses
#define MICROPY_HW_CAN1_NAME        "CAN"
#define MICROPY_HW_CAN1_TX          (pin_D1)
#define MICROPY_HW_CAN1_RX          (pin_D0)


// USRSW is not pulled, and pressing the button makes the input go low.
#define MICROPY_HW_USRSW_PIN        (pin_A13)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)


// LEDs
#define MICROPY_HW_LED_INVERTED     (1) // LEDs are on when pin is driven low
#define MICROPY_HW_LED1             (pin_I11)   //red
#define MICROPY_HW_LED2             (pin_I10)   //green
#define MICROPY_HW_LED3             (pin_I9)    //blue
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_low(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_high(pin))


// SD card
#define MICROPY_HW_SDMMC2_CK                (pyb_pin_SD_CK)
#define MICROPY_HW_SDMMC2_CMD               (pyb_pin_SD_CMD)
#define MICROPY_HW_SDMMC2_D0                (pyb_pin_SD_D0)
#define MICROPY_HW_SDMMC2_D1                (pyb_pin_SD_D1)
#define MICROPY_HW_SDMMC2_D2                (pyb_pin_SD_D2)
#define MICROPY_HW_SDMMC2_D3                (pyb_pin_SD_D3)
#define MICROPY_HW_SDCARD_MOUNT_AT_BOOT     (0)


// USB config
#define MICROPY_HW_USB_FS           (0)     // pins A12 A11 are used for RS485_1_DE/nRE
#define MICROPY_HW_USB_HS           (1)     // HS_USB is connected to the microUSB connector
#define MICROPY_HW_USB_HS_IN_FS     (1)
#define MICROPY_HW_USB_MAIN_DEV     (USB_PHY_HS_ID)


/******************************************************************************/
// Bootloader configuration

#define MBOOT_USB_AUTODETECT_PORT   (0)
#define MBOOT_FSLOAD                (1)
#define MBOOT_VFS_FAT               (1)

// use I2C3 on GPIOs connector
#define MBOOT_I2C_PERIPH_ID         1
#define MBOOT_I2C_SCL               (pin_H7)
#define MBOOT_I2C_SDA               (pin_H8)
#define MBOOT_I2C_ALTFUNC           (4)

#define MBOOT_SPIFLASH_ADDR         (0x80000000)
#define MBOOT_SPIFLASH_BYTE_SIZE    (64 * 128 * 1024)
#define MBOOT_SPIFLASH_LAYOUT       "/0x80000000/64*128Kg"
#define MBOOT_SPIFLASH_ERASE_BLOCKS_PER_PAGE (128 / 4)
#define MBOOT_SPIFLASH_SPIFLASH     (&spi_bdev.spiflash)
#define MBOOT_SPIFLASH_CONFIG       (&spiflash_config)

#define MBOOT_SPIFLASH2_ADDR        (0x90000000)
#define MBOOT_SPIFLASH2_BYTE_SIZE   (64 * 128 * 1024)
#define MBOOT_SPIFLASH2_LAYOUT      "/0x90000000/64*128Kg"
#define MBOOT_SPIFLASH2_ERASE_BLOCKS_PER_PAGE (128 / 4)
#define MBOOT_SPIFLASH2_SPIFLASH    (&spi_bdev2.spiflash)
#define MBOOT_SPIFLASH2_CONFIG      (&spiflash2_config)

#define MBOOT_BOARD_EARLY_INIT mboot_board_early_init
void mboot_board_early_init(void);

