/*
 * M2FC ChibiOS Board definition file.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifier.
 */
#define BOARD_M2FC
#define BOARD_NAME                  "SPALAX"


/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header.
 */
#define STM32F40_41xxx

/*
 * Disable MCU's VBUS sensing. (Otherwise VBUS would have to be wired to PA9.)
 */
#define BOARD_OTG_NOVBUSSENS

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0
#define GPIOA_MPU_IRQ               1
#define GPIOA_USART2_TX             2
#define GPIOA_USART2_RX             3
#define GPIOA_MPU_NSS               4
#define GPIOA_IMU_SCK               5
#define GPIOA_IMU_MISO              6
#define GPIOA_IMU_MOSI              7
#define GPIOA_PIN8                  8
#define GPIOA_USART1_TX             9
#define GPIOA_USART1_RX             10
#define GPIOA_OTG_FS_DM             11
#define GPIOA_OTG_DS_DP             12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_ESPI_NSS              15

#define GPIOB_ADIS_NRESET           0
#define GPIOB_PIN1                  1
#define GPIOB_STAT_SENSORS          2
#define GPIOB_ESPI_SCK              3
#define GPIOB_ESPI_MISO             4
#define GPIOB_ESPI_MOSI             5
#define GPIOB_PIN6                  6
#define GPIOB_PIN7                  7
#define GPIOB_PIN8                  8
#define GPIOB_PIN9                  9
#define GPIOB_PIN10                 10
#define GPIOB_PIN11                 11
#define GPIOB_ALT_NSS               12
#define GPIOB_ALT_SCK               13
#define GPIOB_ALT_MISO              14
#define GPIOB_ALT_MOSI              15

#define GPIOC_PIN0                  0
#define GPIOC_PIN1                  1
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_ADIS_NSS                4
#define GPIOC_ADIS_IRQ              5
#define GPIOC_PIN6                  6
#define GPIOC_PIN7                  7
#define GPIOC_SD_DAT0               8
#define GPIOC_SD_DAT1               9
#define GPIOC_SD_DAT2               10
#define GPIOC_SD_DAT3               11
#define GPIOC_SD_CLK                12
#define GPIOC_PIN13                 13
#define GPIOC_PIN14                 14
#define GPIOC_PIN15                 15

#define GPIOD_CAN1_RX               0
#define GPIOD_CAN1_TX               1
#define GPIOD_SD_CMD                2
#define GPIOD_SD_CD                 3
#define GPIOD_PIN4                  4
#define GPIOD_PIN5                  5
#define GPIOD_PIN6                  6
#define GPIOD_PIN7                  7
#define GPIOD_PIN8                  8
#define GPIOD_PIN9                  9
#define GPIOD_PIN10                 10
#define GPIOD_PIN11                 11
#define GPIOD_PIN12                 12
#define GPIOD_PIN13                 13
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_ESPI_DRDY             0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_STAT_BUZZER           3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_STAT_NSENSORS         7
#define GPIOE_STAT_IMU              8
#define GPIOE_STAT_NIMU             9
#define GPIOE_PIN10                 10
#define GPIOE_PIN11                 11
#define GPIOE_PIN12                 12
#define GPIOE_PIN13                 13
#define GPIOE_PIN14                 14
#define GPIOE_PIN15                 15

#define GPIOF_PIN0                  0
#define GPIOF_PIN1                  1
#define GPIOF_PIN2                  2
#define GPIOF_PIN3                  3
#define GPIOF_PIN4                  4
#define GPIOF_PIN5                  5
#define GPIOF_PIN6                  6
#define GPIOF_PIN7                  7
#define GPIOF_PIN8                  8
#define GPIOF_PIN9                  9
#define GPIOF_PIN10                 10
#define GPIOF_PIN11                 11
#define GPIOF_PIN12                 12
#define GPIOF_PIN13                 13
#define GPIOF_PIN14                 14
#define GPIOF_PIN15                 15

#define GPIOG_PIN0                  0
#define GPIOG_PIN1                  1
#define GPIOG_PIN2                  2
#define GPIOG_PIN3                  3
#define GPIOG_PIN4                  4
#define GPIOG_PIN5                  5
#define GPIOG_PIN6                  6
#define GPIOG_PIN7                  7
#define GPIOG_PIN8                  8
#define GPIOG_PIN9                  9
#define GPIOG_PIN10                 10
#define GPIOG_PIN11                 11
#define GPIOG_PIN12                 12
#define GPIOG_PIN13                 13
#define GPIOG_PIN14                 14
#define GPIOG_PIN15                 15

#define GPIOH_PIN0                  0
#define GPIOH_PIN1                  1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_PIN8                  8
#define GPIOH_PIN9                  9
#define GPIOH_PIN10                 10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_PIN15                 15

#define GPIOI_PIN0                  0
#define GPIOI_PIN1                  1
#define GPIOI_PIN2                  2
#define GPIOI_PIN3                  3
#define GPIOI_PIN4                  4
#define GPIOI_PIN5                  5
#define GPIOI_PIN6                  6
#define GPIOI_PIN7                  7
#define GPIOI_PIN8                  8
#define GPIOI_PIN9                  9
#define GPIOI_PIN10                 10
#define GPIOI_PIN11                 11
#define GPIOI_PIN12                 12
#define GPIOI_PIN13                 13
#define GPIOI_PIN14                 14
#define GPIOI_PIN15                 15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0)        |\
                                     PIN_MODE_INPUT(GPIOA_MPU_IRQ)        |\
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_RX) | \
                                     PIN_MODE_OUTPUT(GPIOA_MPU_NSS) | \
                                     PIN_MODE_ALTERNATE(GPIOA_IMU_SCK) | \
                                     PIN_MODE_ALTERNATE(GPIOA_IMU_MISO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_IMU_MOSI) | \
                                     PIN_MODE_INPUT(GPIOA_PIN8) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_DS_DP) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_OUTPUT(GPIOA_ESPI_NSS))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_MPU_IRQ)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART2_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART2_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MPU_NSS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_IMU_SCK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_IMU_MISO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_IMU_MOSI) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_DS_DP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ESPI_NSS))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_2M(GPIOA_PIN0)          |\
                                     PIN_OSPEED_2M(GPIOA_MPU_IRQ)          |\
                                     PIN_OSPEED_2M(GPIOA_USART2_TX) | \
                                     PIN_OSPEED_2M(GPIOA_USART2_RX) | \
                                     PIN_OSPEED_2M(GPIOA_MPU_NSS) | \
                                     PIN_OSPEED_100M(GPIOA_IMU_SCK) | \
                                     PIN_OSPEED_100M(GPIOA_IMU_MISO) | \
                                     PIN_OSPEED_100M(GPIOA_IMU_MOSI) | \
                                     PIN_OSPEED_2M(GPIOA_PIN8) | \
                                     PIN_OSPEED_100M(GPIOA_USART1_TX) | \
                                     PIN_OSPEED_100M(GPIOA_USART1_RX) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_DS_DP) | \
                                     PIN_OSPEED_2M(GPIOA_SWDIO) | \
                                     PIN_OSPEED_2M(GPIOA_SWCLK) | \
                                     PIN_OSPEED_2M(GPIOA_ESPI_NSS))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0)     |\
                                     PIN_PUPDR_FLOATING(GPIOA_MPU_IRQ)     |\
                                     PIN_PUPDR_FLOATING(GPIOA_USART2_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USART2_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_MPU_NSS) | \
                                     PIN_PUPDR_FLOATING(GPIOA_IMU_SCK) | \
                                     PIN_PUPDR_FLOATING(GPIOA_IMU_MISO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_IMU_MOSI) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USART1_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USART1_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_DS_DP) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ESPI_NSS))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_PIN0)           |\
                                     PIN_ODR_LOW(GPIOA_MPU_IRQ)           |\
                                     PIN_ODR_HIGH(GPIOA_USART2_TX) | \
                                     PIN_ODR_HIGH(GPIOA_USART2_RX) | \
                                     PIN_ODR_LOW(GPIOA_MPU_NSS) | \
                                     PIN_ODR_LOW(GPIOA_IMU_SCK) | \
                                     PIN_ODR_LOW(GPIOA_IMU_MISO) | \
                                     PIN_ODR_LOW(GPIOA_IMU_MOSI) | \
                                     PIN_ODR_LOW(GPIOA_PIN8) | \
                                     PIN_ODR_HIGH(GPIOA_USART1_TX) | \
                                     PIN_ODR_HIGH(GPIOA_USART1_RX) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_DS_DP) | \
                                     PIN_ODR_LOW(GPIOA_SWDIO) | \
                                     PIN_ODR_LOW(GPIOA_SWCLK) | \
                                     PIN_ODR_LOW(GPIOA_ESPI_NSS))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0)         |\
                                     PIN_AFIO_AF(GPIOA_MPU_IRQ, 0)         |\
                                     PIN_AFIO_AF(GPIOA_USART2_TX, 7) | \
                                     PIN_AFIO_AF(GPIOA_USART2_RX, 7) | \
                                     PIN_AFIO_AF(GPIOA_MPU_NSS, 0) | \
                                     PIN_AFIO_AF(GPIOA_IMU_SCK, 5) | \
                                     PIN_AFIO_AF(GPIOA_IMU_MISO, 5) | \
                                     PIN_AFIO_AF(GPIOA_IMU_MOSI, 5))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0)  |\
                                     PIN_AFIO_AF(GPIOA_USART1_TX, 7)  |\
                                     PIN_AFIO_AF(GPIOA_USART1_RX, 7) |\
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |\
                                     PIN_AFIO_AF(GPIOA_OTG_DS_DP, 10) |\
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |\
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |\
                                     PIN_AFIO_AF(GPIOA_ESPI_NSS, 0))

#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_ADIS_NRESET)             |\
                                     PIN_MODE_INPUT(GPIOB_PIN1)             |\
                                     PIN_MODE_OUTPUT(GPIOB_STAT_SENSORS)             |\
                                     PIN_MODE_ALTERNATE(GPIOB_ESPI_SCK)     |\
                                     PIN_MODE_ALTERNATE(GPIOB_ESPI_MISO)    |\
                                     PIN_MODE_ALTERNATE(GPIOB_ESPI_MOSI)   |\
                                     PIN_MODE_INPUT(GPIOB_PIN6)    |\
                                     PIN_MODE_INPUT(GPIOB_PIN7)     |\
                                     PIN_MODE_INPUT(GPIOB_PIN8)    |\
                                     PIN_MODE_INPUT(GPIOB_PIN9)    |\
                                     PIN_MODE_INPUT(GPIOB_PIN10)     |\
                                     PIN_MODE_INPUT(GPIOB_PIN11)     |\
                                     PIN_MODE_OUTPUT(GPIOB_ALT_NSS)     |\
                                     PIN_MODE_ALTERNATE(GPIOB_ALT_SCK) |\
                                     PIN_MODE_ALTERNATE(GPIOB_ALT_MISO)|\
                                     PIN_MODE_ALTERNATE(GPIOB_ALT_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_ADIS_NRESET)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_STAT_SENSORS)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ESPI_SCK)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ESPI_MISO)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ESPI_MOSI)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8)   |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9)   |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ALT_NSS)  |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ALT_SCK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ALT_MISO)|\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ALT_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_2M(GPIOB_ADIS_NRESET)              |\
                                     PIN_OSPEED_2M(GPIOB_PIN1)              |\
                                     PIN_OSPEED_2M(GPIOB_STAT_SENSORS)              |\
                                     PIN_OSPEED_100M(GPIOB_ESPI_SCK)         |\
                                     PIN_OSPEED_100M(GPIOB_ESPI_MISO)        |\
                                     PIN_OSPEED_100M(GPIOB_ESPI_MOSI)        |\
                                     PIN_OSPEED_2M(GPIOB_PIN6)         |\
                                     PIN_OSPEED_2M(GPIOB_PIN7)          |\
                                     PIN_OSPEED_2M(GPIOB_PIN8)         |\
                                     PIN_OSPEED_2M(GPIOB_PIN9)         |\
                                     PIN_OSPEED_2M(GPIOB_PIN10)          |\
                                     PIN_OSPEED_2M(GPIOB_PIN11)          |\
                                     PIN_OSPEED_2M(GPIOB_ALT_NSS)      |\
                                     PIN_OSPEED_100M(GPIOB_ALT_SCK)     |\
                                     PIN_OSPEED_100M(GPIOB_ALT_MISO)    |\
                                     PIN_OSPEED_100M(GPIOB_ALT_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_ADIS_NRESET)           |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1)           |\
                                     PIN_PUPDR_FLOATING(GPIOB_STAT_SENSORS)           |\
                                     PIN_PUPDR_FLOATING(GPIOB_ESPI_SCK)     |\
                                     PIN_PUPDR_FLOATING(GPIOB_ESPI_MISO)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_ESPI_MOSI)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN7)     |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10)     |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN11)     |\
                                     PIN_PUPDR_FLOATING(GPIOB_ALT_NSS)  |\
                                     PIN_PUPDR_FLOATING(GPIOB_ALT_SCK) |\
                                     PIN_PUPDR_FLOATING(GPIOB_ALT_MISO)|\
                                     PIN_PUPDR_FLOATING(GPIOB_ALT_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_ADIS_NRESET)               |\
                                     PIN_ODR_LOW(GPIOB_PIN1)               |\
                                     PIN_ODR_LOW(GPIOB_STAT_SENSORS)               |\
                                     PIN_ODR_HIGH(GPIOB_ESPI_SCK)           |\
                                     PIN_ODR_HIGH(GPIOB_ESPI_MISO)          |\
                                     PIN_ODR_HIGH(GPIOB_ESPI_MOSI)          |\
                                     PIN_ODR_LOW(GPIOB_PIN6)          |\
                                     PIN_ODR_LOW(GPIOB_PIN7)           |\
                                     PIN_ODR_LOW(GPIOB_PIN8)          |\
                                     PIN_ODR_LOW(GPIOB_PIN9)          |\
                                     PIN_ODR_LOW(GPIOB_PIN10)           |\
                                     PIN_ODR_LOW(GPIOB_PIN11)           |\
                                     PIN_ODR_LOW(GPIOB_ALT_NSS)        |\
                                     PIN_ODR_HIGH(GPIOB_ALT_SCK)       |\
                                     PIN_ODR_HIGH(GPIOB_ALT_MISO)      |\
                                     PIN_ODR_HIGH(GPIOB_ALT_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_ADIS_NRESET, 0)             |\
                                     PIN_AFIO_AF(GPIOB_PIN1, 0)             |\
                                     PIN_AFIO_AF(GPIOB_STAT_SENSORS, 0)             |\
                                     PIN_AFIO_AF(GPIOB_ESPI_SCK, 6)         |\
                                     PIN_AFIO_AF(GPIOB_ESPI_MISO, 6)        |\
                                     PIN_AFIO_AF(GPIOB_ESPI_MOSI, 6)        |\
                                     PIN_AFIO_AF(GPIOB_PIN6, 0)        |\
                                     PIN_AFIO_AF(GPIOB_PIN7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0)        |\
                                     PIN_AFIO_AF(GPIOB_PIN9, 0)        |\
                                     PIN_AFIO_AF(GPIOB_PIN10, 0)         |\
                                     PIN_AFIO_AF(GPIOB_PIN11, 0)         |\
                                     PIN_AFIO_AF(GPIOB_ALT_NSS, 0)      |\
                                     PIN_AFIO_AF(GPIOB_ALT_SCK, 5)     |\
                                     PIN_AFIO_AF(GPIOB_ALT_MISO, 5)    |\
                                     PIN_AFIO_AF(GPIOB_ALT_MOSI, 5))

#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0)             |\
                                     PIN_MODE_INPUT(GPIOC_PIN1)        |\
                                     PIN_MODE_INPUT(GPIOC_PIN2)        |\
                                     PIN_MODE_INPUT(GPIOC_PIN3)        |\
                                     PIN_MODE_OUTPUT(GPIOC_ADIS_NSS)    |\
                                     PIN_MODE_INPUT(GPIOC_ADIS_IRQ)    |\
                                     PIN_MODE_INPUT(GPIOC_PIN6)           |\
                                     PIN_MODE_INPUT(GPIOC_PIN7)           |\
                                     PIN_MODE_ALTERNATE(GPIOC_SD_DAT0)      |\
                                     PIN_MODE_ALTERNATE(GPIOC_SD_DAT1)      |\
                                     PIN_MODE_ALTERNATE(GPIOC_SD_DAT2)      |\
                                     PIN_MODE_ALTERNATE(GPIOC_SD_DAT3)      |\
                                     PIN_MODE_ALTERNATE(GPIOC_SD_CLK)       |\
                                     PIN_MODE_INPUT(GPIOC_PIN13)            |\
                                     PIN_MODE_INPUT(GPIOC_PIN14)            |\
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADIS_NSS)|\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADIS_IRQ)|\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_DAT0)      |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_DAT1)      |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_DAT2)      |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_DAT3)      |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_CLK)       |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_2M(GPIOC_PIN0)              |\
                                     PIN_OSPEED_2M(GPIOC_PIN1)          |\
                                     PIN_OSPEED_2M(GPIOC_PIN2)          |\
                                     PIN_OSPEED_2M(GPIOC_PIN3)          |\
                                     PIN_OSPEED_2M(GPIOC_ADIS_NSS)     |\
                                     PIN_OSPEED_2M(GPIOC_ADIS_IRQ)     |\
                                     PIN_OSPEED_2M(GPIOC_PIN6)             |\
                                     PIN_OSPEED_2M(GPIOC_PIN7)             |\
                                     PIN_OSPEED_100M(GPIOC_SD_DAT0)         |\
                                     PIN_OSPEED_100M(GPIOC_SD_DAT1)         |\
                                     PIN_OSPEED_100M(GPIOC_SD_DAT2)         |\
                                     PIN_OSPEED_100M(GPIOC_SD_DAT3)         |\
                                     PIN_OSPEED_100M(GPIOC_SD_CLK)          |\
                                     PIN_OSPEED_2M(GPIOC_PIN13)             |\
                                     PIN_OSPEED_2M(GPIOC_PIN14)             |\
                                     PIN_OSPEED_2M(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0)           |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1)     |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2)     |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3)     |\
                                     PIN_PUPDR_FLOATING(GPIOC_ADIS_NSS)|\
                                     PIN_PUPDR_FLOATING(GPIOC_ADIS_IRQ)|\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6)        |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN7)        |\
                                     PIN_PUPDR_FLOATING(GPIOC_SD_DAT0)      |\
                                     PIN_PUPDR_FLOATING(GPIOC_SD_DAT1)      |\
                                     PIN_PUPDR_FLOATING(GPIOC_SD_DAT2)      |\
                                     PIN_PUPDR_FLOATING(GPIOC_SD_DAT3)      |\
                                     PIN_PUPDR_FLOATING(GPIOC_SD_CLK)       |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN13)          |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14)          |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_PIN0)               |\
                                     PIN_ODR_LOW(GPIOC_PIN1)           |\
                                     PIN_ODR_LOW(GPIOC_PIN2)           |\
                                     PIN_ODR_LOW(GPIOC_PIN3)           |\
                                     PIN_ODR_LOW(GPIOC_ADIS_NSS)      |\
                                     PIN_ODR_LOW(GPIOC_ADIS_IRQ)      |\
                                     PIN_ODR_LOW(GPIOC_PIN6)               |\
                                     PIN_ODR_LOW(GPIOC_PIN7)               |\
                                     PIN_ODR_HIGH(GPIOC_SD_DAT0)            |\
                                     PIN_ODR_HIGH(GPIOC_SD_DAT1)            |\
                                     PIN_ODR_HIGH(GPIOC_SD_DAT2)            |\
                                     PIN_ODR_HIGH(GPIOC_SD_DAT3)            |\
                                     PIN_ODR_HIGH(GPIOC_SD_CLK)             |\
                                     PIN_ODR_LOW(GPIOC_PIN13)              |\
                                     PIN_ODR_LOW(GPIOC_PIN14)              |\
                                     PIN_ODR_LOW(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0)             |\
                                     PIN_AFIO_AF(GPIOC_PIN1, 0)         |\
                                     PIN_AFIO_AF(GPIOC_PIN2, 0)         |\
                                     PIN_AFIO_AF(GPIOC_PIN3, 0)         |\
                                     PIN_AFIO_AF(GPIOC_ADIS_NSS, 0)    |\
                                     PIN_AFIO_AF(GPIOC_ADIS_IRQ, 0)    |\
                                     PIN_AFIO_AF(GPIOC_PIN6, 0)            |\
                                     PIN_AFIO_AF(GPIOC_PIN7, 0))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SD_DAT0, 12)         |\
                                     PIN_AFIO_AF(GPIOC_SD_DAT1, 12)         |\
                                     PIN_AFIO_AF(GPIOC_SD_DAT2, 12)         |\
                                     PIN_AFIO_AF(GPIOC_SD_DAT3, 12)         |\
                                     PIN_AFIO_AF(GPIOC_SD_CLK, 12)          |\
                                     PIN_AFIO_AF(GPIOC_PIN13, 0)            |\
                                     PIN_AFIO_AF(GPIOC_PIN14, 0)            |\
                                     PIN_AFIO_AF(GPIOC_PIN15, 0))


#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_CAN1_RX)             |\
                                     PIN_MODE_ALTERNATE(GPIOD_CAN1_TX)            |\
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CMD)       |\
                                     PIN_MODE_INPUT(GPIOD_SD_CD)             |\
                                     PIN_MODE_INPUT(GPIOD_PIN4)             |\
                                     PIN_MODE_INPUT(GPIOD_PIN5)    |\
                                     PIN_MODE_INPUT(GPIOD_PIN6)     |\
                                     PIN_MODE_INPUT(GPIOD_PIN7)         |\
                                     PIN_MODE_INPUT(GPIOD_PIN8)    |\
                                     PIN_MODE_INPUT(GPIOD_PIN9)    |\
                                     PIN_MODE_INPUT(GPIOD_PIN10)      |\
                                     PIN_MODE_INPUT(GPIOD_PIN11)            |\
                                     PIN_MODE_INPUT(GPIOD_PIN12)            |\
                                     PIN_MODE_INPUT(GPIOD_PIN13)              |\
                                     PIN_MODE_INPUT(GPIOD_PIN14)            |\
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_CAN1_RX)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_CAN1_TX)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CMD)       |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CD)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7)      |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8)|\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9)|\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10)  |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13)          |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_CAN1_RX)              |\
                                     PIN_OSPEED_100M(GPIOD_CAN1_TX)             |\
                                     PIN_OSPEED_100M(GPIOD_SD_CMD)          |\
                                     PIN_OSPEED_2M(GPIOD_SD_CD)              |\
                                     PIN_OSPEED_2M(GPIOD_PIN4)              |\
                                     PIN_OSPEED_2M(GPIOD_PIN5)         |\
                                     PIN_OSPEED_2M(GPIOD_PIN6)          |\
                                     PIN_OSPEED_2M(GPIOD_PIN7)          |\
                                     PIN_OSPEED_2M(GPIOD_PIN8)     |\
                                     PIN_OSPEED_2M(GPIOD_PIN9)     |\
                                     PIN_OSPEED_2M(GPIOD_PIN10)      |\
                                     PIN_OSPEED_2M(GPIOD_PIN11)             |\
                                     PIN_OSPEED_2M(GPIOD_PIN12)             |\
                                     PIN_OSPEED_2M(GPIOD_PIN13)               |\
                                     PIN_OSPEED_2M(GPIOD_PIN14)             |\
                                     PIN_OSPEED_2M(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_CAN1_RX)           |\
                                     PIN_PUPDR_FLOATING(GPIOD_CAN1_TX)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CMD)       |\
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CD)           |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4)           |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5)    |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6)     |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7)      |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8)|\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9)|\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10)  |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11)          |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN12)          |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN13)            |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14)          |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_CAN1_RX)               |\
                                     PIN_ODR_HIGH(GPIOD_CAN1_TX)              |\
                                     PIN_ODR_HIGH(GPIOD_SD_CMD)             |\
                                     PIN_ODR_LOW(GPIOD_SD_CD)               |\
                                     PIN_ODR_LOW(GPIOD_PIN4)               |\
                                     PIN_ODR_LOW(GPIOD_PIN5)          |\
                                     PIN_ODR_LOW(GPIOD_PIN6)           |\
                                     PIN_ODR_LOW(GPIOD_PIN7)            |\
                                     PIN_ODR_LOW(GPIOD_PIN8)      |\
                                     PIN_ODR_LOW(GPIOD_PIN9)      |\
                                     PIN_ODR_LOW(GPIOD_PIN10)        |\
                                     PIN_ODR_LOW(GPIOD_PIN11)              |\
                                     PIN_ODR_LOW(GPIOD_PIN12)              |\
                                     PIN_ODR_LOW(GPIOD_PIN13)                |\
                                     PIN_ODR_LOW(GPIOD_PIN14)              |\
                                     PIN_ODR_LOW(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_CAN1_RX, 9)             |\
                                     PIN_AFIO_AF(GPIOD_CAN1_TX, 9)            |\
                                     PIN_AFIO_AF(GPIOD_SD_CMD, 12)          |\
                                     PIN_AFIO_AF(GPIOD_SD_CD, 0)             |\
                                     PIN_AFIO_AF(GPIOD_PIN4, 0)             |\
                                     PIN_AFIO_AF(GPIOD_PIN5, 0)        |\
                                     PIN_AFIO_AF(GPIOD_PIN6, 0)         |\
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0)    |\
                                     PIN_AFIO_AF(GPIOD_PIN9, 0)    |\
                                     PIN_AFIO_AF(GPIOD_PIN10, 0)      |\
                                     PIN_AFIO_AF(GPIOD_PIN11, 0)            |\
                                     PIN_AFIO_AF(GPIOD_PIN12, 0)            |\
                                     PIN_AFIO_AF(GPIOD_PIN13, 0)              |\
                                     PIN_AFIO_AF(GPIOD_PIN14, 0)            |\
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_ESPI_DRDY)       |\
                                     PIN_MODE_INPUT(GPIOE_PIN1)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN2)             |\
                                     PIN_MODE_OUTPUT(GPIOE_STAT_BUZZER)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN4)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN5)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN6)             |\
                                     PIN_MODE_OUTPUT(GPIOE_STAT_NSENSORS)         |\
                                     PIN_MODE_OUTPUT(GPIOE_STAT_IMU)         |\
                                     PIN_MODE_OUTPUT(GPIOE_STAT_NIMU)         |\
                                     PIN_MODE_INPUT(GPIOE_PIN10)        |\
                                     PIN_MODE_INPUT(GPIOE_PIN11)        |\
                                     PIN_MODE_INPUT(GPIOE_PIN12)        |\
                                     PIN_MODE_INPUT(GPIOE_PIN13)        |\
                                     PIN_MODE_INPUT(GPIOE_PIN14)        |\
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_ESPI_DRDY)   |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_STAT_BUZZER)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_STAT_NSENSORS)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_STAT_IMU)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_STAT_NIMU)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_2M(GPIOE_ESPI_DRDY)        |\
                                     PIN_OSPEED_2M(GPIOE_PIN1)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN2)              |\
                                     PIN_OSPEED_2M(GPIOE_STAT_BUZZER)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN4)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN5)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN6)              |\
                                     PIN_OSPEED_2M(GPIOE_STAT_NSENSORS)          |\
                                     PIN_OSPEED_2M(GPIOE_STAT_IMU)          |\
                                     PIN_OSPEED_2M(GPIOE_STAT_NIMU)          |\
                                     PIN_OSPEED_2M(GPIOE_PIN10)          |\
                                     PIN_OSPEED_2M(GPIOE_PIN11)          |\
                                     PIN_OSPEED_2M(GPIOE_PIN12)          |\
                                     PIN_OSPEED_2M(GPIOE_PIN13)         |\
                                     PIN_OSPEED_2M(GPIOE_PIN14)         |\
                                     PIN_OSPEED_2M(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_ESPI_DRDY)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_STAT_BUZZER)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_STAT_NSENSORS)       |\
                                     PIN_PUPDR_FLOATING(GPIOE_STAT_IMU)       |\
                                     PIN_PUPDR_FLOATING(GPIOE_STAT_NIMU)       |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN11)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN13)      |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN14)      |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_ESPI_DRDY)         |\
                                     PIN_ODR_LOW(GPIOE_PIN1)               |\
                                     PIN_ODR_LOW(GPIOE_PIN2)               |\
                                     PIN_ODR_LOW(GPIOE_STAT_BUZZER)               |\
                                     PIN_ODR_LOW(GPIOE_PIN4)               |\
                                     PIN_ODR_LOW(GPIOE_PIN5)               |\
                                     PIN_ODR_LOW(GPIOE_PIN6)               |\
                                     PIN_ODR_LOW(GPIOE_STAT_NSENSORS)            |\
                                     PIN_ODR_LOW(GPIOE_STAT_IMU)            |\
                                     PIN_ODR_LOW(GPIOE_STAT_NIMU)           |\
                                     PIN_ODR_LOW(GPIOE_PIN10)            |\
                                     PIN_ODR_LOW(GPIOE_PIN11)            |\
                                     PIN_ODR_LOW(GPIOE_PIN12)            |\
                                     PIN_ODR_LOW(GPIOE_PIN13)          |\
                                     PIN_ODR_LOW(GPIOE_PIN14)          |\
                                     PIN_ODR_LOW(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_ESPI_DRDY, 0)       |\
                                     PIN_AFIO_AF(GPIOE_PIN1, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN2, 0)             |\
                                     PIN_AFIO_AF(GPIOE_STAT_BUZZER, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN4, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN5, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN6, 0)             |\
                                     PIN_AFIO_AF(GPIOE_STAT_NSENSORS, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_STAT_IMU, 0)         |\
                                     PIN_AFIO_AF(GPIOE_STAT_NIMU, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PIN10, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PIN11, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PIN12, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PIN13, 0)        |\
                                     PIN_AFIO_AF(GPIOE_PIN14, 0)        |\
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN1)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN2)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN3)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN4)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN5)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN6)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN7)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN8)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN9)             |\
                                     PIN_MODE_INPUT(GPIOF_PIN10)            |\
                                     PIN_MODE_INPUT(GPIOF_PIN11)            |\
                                     PIN_MODE_INPUT(GPIOF_PIN12)            |\
                                     PIN_MODE_INPUT(GPIOF_PIN13)            |\
                                     PIN_MODE_INPUT(GPIOF_PIN14)            |\
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_2M(GPIOF_PIN0) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN1) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN2) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN3) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN4) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN5) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN6) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN7) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN8) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN9) |          \
                                     PIN_OSPEED_2M(GPIOF_PIN10) |         \
                                     PIN_OSPEED_2M(GPIOF_PIN11) |         \
                                     PIN_OSPEED_2M(GPIOF_PIN12) |         \
                                     PIN_OSPEED_2M(GPIOF_PIN13) |         \
                                     PIN_OSPEED_2M(GPIOF_PIN14) |         \
                                     PIN_OSPEED_2M(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_PIN0) |             \
                                     PIN_ODR_LOW(GPIOF_PIN1) |             \
                                     PIN_ODR_LOW(GPIOF_PIN2) |             \
                                     PIN_ODR_LOW(GPIOF_PIN3) |             \
                                     PIN_ODR_LOW(GPIOF_PIN4) |             \
                                     PIN_ODR_LOW(GPIOF_PIN5) |             \
                                     PIN_ODR_LOW(GPIOF_PIN6) |             \
                                     PIN_ODR_LOW(GPIOF_PIN7) |             \
                                     PIN_ODR_LOW(GPIOF_PIN8) |             \
                                     PIN_ODR_LOW(GPIOF_PIN9) |             \
                                     PIN_ODR_LOW(GPIOF_PIN10) |            \
                                     PIN_ODR_LOW(GPIOF_PIN11) |            \
                                     PIN_ODR_LOW(GPIOF_PIN12) |            \
                                     PIN_ODR_LOW(GPIOF_PIN13) |            \
                                     PIN_ODR_LOW(GPIOF_PIN14) |            \
                                     PIN_ODR_LOW(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))

#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_2M(GPIOG_PIN0) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN1) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN2) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN3) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN4) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN5) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN6) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN7) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN8) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN9) |          \
                                     PIN_OSPEED_2M(GPIOG_PIN10) |         \
                                     PIN_OSPEED_2M(GPIOG_PIN11) |         \
                                     PIN_OSPEED_2M(GPIOG_PIN12) |         \
                                     PIN_OSPEED_2M(GPIOG_PIN13) |         \
                                     PIN_OSPEED_2M(GPIOG_PIN14) |         \
                                     PIN_OSPEED_2M(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_LOW(GPIOG_PIN0) |             \
                                     PIN_ODR_LOW(GPIOG_PIN1) |             \
                                     PIN_ODR_LOW(GPIOG_PIN2) |             \
                                     PIN_ODR_LOW(GPIOG_PIN3) |             \
                                     PIN_ODR_LOW(GPIOG_PIN4) |             \
                                     PIN_ODR_LOW(GPIOG_PIN5) |             \
                                     PIN_ODR_LOW(GPIOG_PIN6) |             \
                                     PIN_ODR_LOW(GPIOG_PIN7) |             \
                                     PIN_ODR_LOW(GPIOG_PIN8) |             \
                                     PIN_ODR_LOW(GPIOG_PIN9) |             \
                                     PIN_ODR_LOW(GPIOG_PIN10) |            \
                                     PIN_ODR_LOW(GPIOG_PIN11) |            \
                                     PIN_ODR_LOW(GPIOG_PIN12) |            \
                                     PIN_ODR_LOW(GPIOG_PIN13) |            \
                                     PIN_ODR_LOW(GPIOG_PIN14) |            \
                                     PIN_ODR_LOW(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0))

#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_PIN0) |         \
                                     PIN_MODE_INPUT(GPIOH_PIN1) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_PIN0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_2M(GPIOH_PIN0) |        \
                                     PIN_OSPEED_2M(GPIOH_PIN1) |       \
                                     PIN_OSPEED_2M(GPIOH_PIN2) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN3) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN4) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN5) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN6) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN7) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN8) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN9) |          \
                                     PIN_OSPEED_2M(GPIOH_PIN10) |         \
                                     PIN_OSPEED_2M(GPIOH_PIN11) |         \
                                     PIN_OSPEED_2M(GPIOH_PIN12) |         \
                                     PIN_OSPEED_2M(GPIOH_PIN13) |         \
                                     PIN_OSPEED_2M(GPIOH_PIN14) |         \
                                     PIN_OSPEED_2M(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_PIN0) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN1) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_LOW(GPIOH_PIN0) |           \
                                     PIN_ODR_LOW(GPIOH_PIN1) |          \
                                     PIN_ODR_LOW(GPIOH_PIN2) |             \
                                     PIN_ODR_LOW(GPIOH_PIN3) |             \
                                     PIN_ODR_LOW(GPIOH_PIN4) |             \
                                     PIN_ODR_LOW(GPIOH_PIN5) |             \
                                     PIN_ODR_LOW(GPIOH_PIN6) |             \
                                     PIN_ODR_LOW(GPIOH_PIN7) |             \
                                     PIN_ODR_LOW(GPIOH_PIN8) |             \
                                     PIN_ODR_LOW(GPIOH_PIN9) |             \
                                     PIN_ODR_LOW(GPIOH_PIN10) |            \
                                     PIN_ODR_LOW(GPIOH_PIN11) |            \
                                     PIN_ODR_LOW(GPIOH_PIN12) |            \
                                     PIN_ODR_LOW(GPIOH_PIN13) |            \
                                     PIN_ODR_LOW(GPIOH_PIN14) |            \
                                     PIN_ODR_LOW(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_PIN0, 0) |         \
                                     PIN_AFIO_AF(GPIOH_PIN1, 0) |        \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0))

#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_2M(GPIOI_PIN0) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN1) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN2) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN3) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN4) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN5) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN6) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN7) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN8) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN9) |          \
                                     PIN_OSPEED_2M(GPIOI_PIN10) |         \
                                     PIN_OSPEED_2M(GPIOI_PIN11) |         \
                                     PIN_OSPEED_2M(GPIOI_PIN12) |         \
                                     PIN_OSPEED_2M(GPIOI_PIN13) |         \
                                     PIN_OSPEED_2M(GPIOI_PIN14) |         \
                                     PIN_OSPEED_2M(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_LOW(GPIOI_PIN0) |             \
                                     PIN_ODR_LOW(GPIOI_PIN1) |             \
                                     PIN_ODR_LOW(GPIOI_PIN2) |             \
                                     PIN_ODR_LOW(GPIOI_PIN3) |             \
                                     PIN_ODR_LOW(GPIOI_PIN4) |             \
                                     PIN_ODR_LOW(GPIOI_PIN5) |             \
                                     PIN_ODR_LOW(GPIOI_PIN6) |             \
                                     PIN_ODR_LOW(GPIOI_PIN7) |             \
                                     PIN_ODR_LOW(GPIOI_PIN8) |             \
                                     PIN_ODR_LOW(GPIOI_PIN9) |             \
                                     PIN_ODR_LOW(GPIOI_PIN10) |            \
                                     PIN_ODR_LOW(GPIOI_PIN11) |            \
                                     PIN_ODR_LOW(GPIOI_PIN12) |            \
                                     PIN_ODR_LOW(GPIOI_PIN13) |            \
                                     PIN_ODR_LOW(GPIOI_PIN14) |            \
                                     PIN_ODR_LOW(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
