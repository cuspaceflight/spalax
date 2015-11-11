/*
 * M2FC ChibiOS Board definition file.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifier.
 */
#define BOARD_M2FC
#define BOARD_NAME                  "BADGER3"


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
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0
#define GPIOA_BUZZER                1
#define GPIOA_EXT_TX                2
#define GPIOA_EXT_RX                3
#define GPIOA_PIN4                  4
#define GPIOA_ACCEL_SCK             5
#define GPIOA_ACCEL_MISO            6
#define GPIOA_ACCEL_MOSI            7
#define GPIOA_PIN8                  8
#define GPIOA_GPS_RX                  9
#define GPIOA_GPS_TX                 10
#define GPIOA_GPS_RESET                 11
#define GPIOA_PIN12                 12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_PIN15                 15

#define GPIOB_PIN0                  0
#define GPIOB_PIN1                  1
#define GPIOB_PIN2                  2
#define GPIOB_RADIO_SCK              3
#define GPIOB_RADIO_MISO             4
#define GPIOB_RADIO_MOSI             5
#define GPIOB_PIN6                  6
#define GPIOB_PIN7                  7
#define GPIOB_GYRO_SCL              8
#define GPIOB_GYRO_SDA              9
#define GPIOB_MAGNO_SCL             10
#define GPIOB_MAGNO_SDA             11
#define GPIOB_BARO_CS               12
#define GPIOB_BARO_SCK              13
#define GPIOB_BARO_MISO             14
#define GPIOB_BARO_MOSI             15

#define GPIOC_PIN0                  0
#define GPIOC_PIN1                  1
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_ACCEL_CS			    4
#define GPIOC_ACCEL_INT             5
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

#define GPIOD_PIN0                  0
#define GPIOD_SD_CD                 1
#define GPIOD_SD_CMD                2
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_EXT_TX                5
#define GPIOD_PIN6                  6
#define GPIOD_RADIO_CS              7
#define GPIOD_PIN8                  8
#define GPIOD_PIN9                  9
#define GPIOD_PYRO_GRN              10
#define GPIOD_PYRO_RED              11
#define GPIOD_RADIO_GRN             12
#define GPIOD_RADIO_RED             13
#define GPIOD_IMU_GRN               14
#define GPIOD_IMU_RED               15

#define GPIOE_PIN0                  0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_PY1_CHK               7
#define GPIOE_PY1_TRG               8
#define GPIOE_PY2_TRG               9
#define GPIOE_PY2_CHK               10
#define GPIOE_PY3_CHK               11
#define GPIOE_PY3_TRG               12
#define GPIOE_PY4_TRG               13
#define GPIOE_PY4_CHK               14
#define GPIOE_MAGNO_DRDY            15

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
                                     PIN_MODE_OUTPUT(GPIOA_BUZZER)        |\
									 PIN_MODE_ALTERNATE(GPIOA_EXT_TX) | \
									 PIN_MODE_ALTERNATE(GPIOA_EXT_RX) | \
									 PIN_MODE_INPUT(GPIOA_PIN4) | \
									 PIN_MODE_ALTERNATE(GPIOA_ACCEL_SCK) | \
									 PIN_MODE_ALTERNATE(GPIOA_ACCEL_MISO) | \
									 PIN_MODE_ALTERNATE(GPIOA_ACCEL_MOSI) | \
									 PIN_MODE_INPUT(GPIOA_PIN8) | \
									 PIN_MODE_ALTERNATE(GPIOA_GPS_RX) | \
									 PIN_MODE_ALTERNATE(GPIOA_GPS_TX) | \
									 PIN_MODE_OUTPUT(GPIOA_GPS_RESET) | \
									 PIN_MODE_INPUT(GPIOA_PIN12) | \
									 PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
									 PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BUZZER)     |\
									 PIN_OTYPE_PUSHPULL(GPIOA_EXT_TX) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_EXT_RX) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_PIN4) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_ACCEL_SCK) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_ACCEL_MISO) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_ACCEL_MOSI) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_PIN8) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_GPS_RX) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_GPS_TX) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_GPS_RESET) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_PIN12) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) | \
									 PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_2M(GPIOA_PIN0)          |\
                                     PIN_OSPEED_2M(GPIOA_BUZZER)          |\
									 PIN_OSPEED_2M(GPIOA_EXT_TX) | \
									 PIN_OSPEED_2M(GPIOA_EXT_RX) | \
									 PIN_OSPEED_2M(GPIOA_PIN4) | \
									 PIN_OSPEED_100M(GPIOA_ACCEL_SCK) | \
									 PIN_OSPEED_100M(GPIOA_ACCEL_MISO) | \
									 PIN_OSPEED_100M(GPIOA_ACCEL_MOSI) | \
									 PIN_OSPEED_2M(GPIOA_PIN8) | \
									 PIN_OSPEED_100M(GPIOA_GPS_RX) | \
									 PIN_OSPEED_100M(GPIOA_GPS_TX) | \
									 PIN_OSPEED_2M(GPIOA_GPS_RESET) | \
									 PIN_OSPEED_2M(GPIOA_PIN12) | \
									 PIN_OSPEED_2M(GPIOA_SWDIO) | \
									 PIN_OSPEED_2M(GPIOA_SWCLK) | \
									 PIN_OSPEED_2M(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0)     |\
                                     PIN_PUPDR_FLOATING(GPIOA_BUZZER)     |\
									 PIN_PUPDR_FLOATING(GPIOA_EXT_TX) | \
									 PIN_PUPDR_FLOATING(GPIOA_EXT_RX) | \
									 PIN_PUPDR_FLOATING(GPIOA_PIN4) | \
									 PIN_PUPDR_FLOATING(GPIOA_ACCEL_SCK) | \
									 PIN_PUPDR_FLOATING(GPIOA_ACCEL_MISO) | \
									 PIN_PUPDR_FLOATING(GPIOA_ACCEL_MOSI) | \
									 PIN_PUPDR_FLOATING(GPIOA_PIN8) | \
									 PIN_PUPDR_FLOATING(GPIOA_GPS_RX) | \
									 PIN_PUPDR_FLOATING(GPIOA_GPS_TX) | \
									 PIN_PUPDR_FLOATING(GPIOA_GPS_RESET) | \
									 PIN_PUPDR_FLOATING(GPIOA_PIN12) | \
									 PIN_PUPDR_FLOATING(GPIOA_SWDIO) | \
									 PIN_PUPDR_FLOATING(GPIOA_SWCLK) | \
									 PIN_PUPDR_FLOATING(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_PIN0)           |\
                                     PIN_ODR_LOW(GPIOA_BUZZER)           |\
									 PIN_ODR_HIGH(GPIOA_EXT_TX) | \
									 PIN_ODR_HIGH(GPIOA_EXT_RX) | \
									 PIN_ODR_LOW(GPIOA_PIN4) | \
									 PIN_ODR_LOW(GPIOA_ACCEL_SCK) | \
									 PIN_ODR_LOW(GPIOA_ACCEL_MISO) | \
									 PIN_ODR_LOW(GPIOA_ACCEL_MOSI) | \
									 PIN_ODR_LOW(GPIOA_PIN8) | \
									 PIN_ODR_HIGH(GPIOA_GPS_RX) | \
									 PIN_ODR_HIGH(GPIOA_GPS_TX) | \
									 PIN_ODR_HIGH(GPIOA_GPS_RESET) | \
									 PIN_ODR_LOW(GPIOA_PIN12) | \
									 PIN_ODR_LOW(GPIOA_SWDIO) | \
									 PIN_ODR_LOW(GPIOA_SWCLK) | \
									 PIN_ODR_LOW(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0)         |\
                                     PIN_AFIO_AF(GPIOA_BUZZER, 0)         |\
									 PIN_AFIO_AF(GPIOA_EXT_TX, 7) | \
									 PIN_AFIO_AF(GPIOA_EXT_RX, 7) | \
									 PIN_AFIO_AF(GPIOA_PIN4, 0) | \
									 PIN_AFIO_AF(GPIOA_ACCEL_SCK, 5) | \
									 PIN_AFIO_AF(GPIOA_ACCEL_MISO, 5) | \
									 PIN_AFIO_AF(GPIOA_ACCEL_MOSI, 5))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0)  |\
                                     PIN_AFIO_AF(GPIOA_GPS_RX, 7)  |\
									 PIN_AFIO_AF(GPIOA_GPS_TX, 7) |\
									 PIN_AFIO_AF(GPIOA_GPS_RESET, 0) |\
									 PIN_AFIO_AF(GPIOA_PIN12, 0) |\
									 PIN_AFIO_AF(GPIOA_SWDIO, 0) |\
									 PIN_AFIO_AF(GPIOA_SWCLK, 0) |\
									 PIN_AFIO_AF(GPIOA_PIN15, 0))

#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0)             |\
                                     PIN_MODE_INPUT(GPIOB_PIN1)             |\
                                     PIN_MODE_INPUT(GPIOB_PIN2)             |\
                                     PIN_MODE_ALTERNATE(GPIOB_RADIO_SCK)     |\
                                     PIN_MODE_ALTERNATE(GPIOB_RADIO_MISO)    |\
                                     PIN_MODE_ALTERNATE(GPIOB_RADIO_MOSI)   |\
                                     PIN_MODE_INPUT(GPIOB_PIN6)    |\
                                     PIN_MODE_INPUT(GPIOB_PIN7)     |\
                                     PIN_MODE_ALTERNATE(GPIOB_GYRO_SCL)    |\
                                     PIN_MODE_ALTERNATE(GPIOB_GYRO_SDA)    |\
                                     PIN_MODE_ALTERNATE(GPIOB_MAGNO_SCL)     |\
                                     PIN_MODE_ALTERNATE(GPIOB_MAGNO_SDA)     |\
                                     PIN_MODE_OUTPUT(GPIOB_BARO_CS)     |\
                                     PIN_MODE_ALTERNATE(GPIOB_BARO_SCK) |\
                                     PIN_MODE_ALTERNATE(GPIOB_BARO_MISO)|\
                                     PIN_MODE_ALTERNATE(GPIOB_BARO_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_RADIO_SCK)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_RADIO_MISO)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_RADIO_MOSI)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7)     |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_GYRO_SCL)   |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_GYRO_SDA)   |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_MAGNO_SCL)    |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_MAGNO_SDA)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BARO_CS)  |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BARO_SCK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BARO_MISO)|\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BARO_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_2M(GPIOB_PIN0)              |\
                                     PIN_OSPEED_2M(GPIOB_PIN1)              |\
                                     PIN_OSPEED_2M(GPIOB_PIN2)              |\
                                     PIN_OSPEED_100M(GPIOB_RADIO_SCK)         |\
                                     PIN_OSPEED_100M(GPIOB_RADIO_MISO)        |\
                                     PIN_OSPEED_100M(GPIOB_RADIO_MOSI)        |\
                                     PIN_OSPEED_2M(GPIOB_PIN6)         |\
                                     PIN_OSPEED_2M(GPIOB_PIN7)          |\
                                     PIN_OSPEED_100M(GPIOB_GYRO_SCL)         |\
                                     PIN_OSPEED_100M(GPIOB_GYRO_SDA)         |\
                                     PIN_OSPEED_100M(GPIOB_MAGNO_SCL)          |\
                                     PIN_OSPEED_100M(GPIOB_MAGNO_SDA)          |\
                                     PIN_OSPEED_2M(GPIOB_BARO_CS)      |\
                                     PIN_OSPEED_100M(GPIOB_BARO_SCK)     |\
                                     PIN_OSPEED_100M(GPIOB_BARO_MISO)    |\
                                     PIN_OSPEED_100M(GPIOB_BARO_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PIN0)           |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1)           |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN2)           |\
                                     PIN_PUPDR_FLOATING(GPIOB_RADIO_SCK)     |\
                                     PIN_PUPDR_FLOATING(GPIOB_RADIO_MISO)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_RADIO_MOSI)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN7)     |\
                                     PIN_PUPDR_FLOATING(GPIOB_GYRO_SCL)    |\
                                     PIN_PUPDR_FLOATING(GPIOB_GYRO_SDA)    |\
                                     PIN_PUPDR_PULLUP(GPIOB_MAGNO_SCL)     |\
                                     PIN_PUPDR_PULLUP(GPIOB_MAGNO_SDA)     |\
                                     PIN_PUPDR_FLOATING(GPIOB_BARO_CS)  |\
                                     PIN_PUPDR_FLOATING(GPIOB_BARO_SCK) |\
                                     PIN_PUPDR_FLOATING(GPIOB_BARO_MISO)|\
                                     PIN_PUPDR_FLOATING(GPIOB_BARO_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_PIN0)               |\
                                     PIN_ODR_LOW(GPIOB_PIN1)               |\
                                     PIN_ODR_LOW(GPIOB_PIN2)               |\
                                     PIN_ODR_LOW(GPIOB_RADIO_SCK)           |\
                                     PIN_ODR_LOW(GPIOB_RADIO_MISO)          |\
                                     PIN_ODR_LOW(GPIOB_RADIO_MOSI)          |\
                                     PIN_ODR_LOW(GPIOB_PIN6)          |\
                                     PIN_ODR_LOW(GPIOB_PIN7)           |\
                                     PIN_ODR_LOW(GPIOB_GYRO_SCL)          |\
                                     PIN_ODR_LOW(GPIOB_GYRO_SDA)          |\
                                     PIN_ODR_HIGH(GPIOB_MAGNO_SCL)           |\
                                     PIN_ODR_HIGH(GPIOB_MAGNO_SDA)           |\
                                     PIN_ODR_HIGH(GPIOB_BARO_CS)        |\
                                     PIN_ODR_LOW(GPIOB_BARO_SCK)       |\
                                     PIN_ODR_LOW(GPIOB_BARO_MISO)      |\
                                     PIN_ODR_LOW(GPIOB_BARO_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0)             |\
                                     PIN_AFIO_AF(GPIOB_PIN1, 0)             |\
                                     PIN_AFIO_AF(GPIOB_PIN2, 0)             |\
                                     PIN_AFIO_AF(GPIOB_RADIO_SCK, 6)         |\
                                     PIN_AFIO_AF(GPIOB_RADIO_MISO, 6)        |\
                                     PIN_AFIO_AF(GPIOB_RADIO_MOSI, 6)        |\
                                     PIN_AFIO_AF(GPIOB_PIN6, 0)        |\
                                     PIN_AFIO_AF(GPIOB_PIN7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_GYRO_SCL, 4)        |\
                                     PIN_AFIO_AF(GPIOB_GYRO_SDA, 4)        |\
                                     PIN_AFIO_AF(GPIOB_MAGNO_SCL, 4)         |\
                                     PIN_AFIO_AF(GPIOB_MAGNO_SDA, 4)         |\
                                     PIN_AFIO_AF(GPIOB_BARO_CS, 0)      |\
                                     PIN_AFIO_AF(GPIOB_BARO_SCK, 5)     |\
                                     PIN_AFIO_AF(GPIOB_BARO_MISO, 5)    |\
                                     PIN_AFIO_AF(GPIOB_BARO_MOSI, 5))

#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0)             |\
                                     PIN_MODE_INPUT(GPIOC_PIN1)        |\
                                     PIN_MODE_INPUT(GPIOC_PIN2)        |\
                                     PIN_MODE_INPUT(GPIOC_PIN3)        |\
                                     PIN_MODE_OUTPUT(GPIOC_ACCEL_CS)    |\
                                     PIN_MODE_INPUT(GPIOC_ACCEL_INT)    |\
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
                                     PIN_OTYPE_PUSHPULL(GPIOC_ACCEL_CS)|\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ACCEL_INT)|\
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
                                     PIN_OSPEED_2M(GPIOC_ACCEL_CS)     |\
                                     PIN_OSPEED_2M(GPIOC_ACCEL_INT)     |\
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
                                     PIN_PUPDR_FLOATING(GPIOC_ACCEL_CS)|\
                                     PIN_PUPDR_PULLUP(GPIOC_ACCEL_INT)|\
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
                                     PIN_ODR_LOW(GPIOC_ACCEL_CS)      |\
                                     PIN_ODR_LOW(GPIOC_ACCEL_INT)      |\
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
                                     PIN_AFIO_AF(GPIOC_ACCEL_CS, 0)    |\
                                     PIN_AFIO_AF(GPIOC_ACCEL_INT, 0)    |\
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


#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0)             |\
                                     PIN_MODE_INPUT(GPIOD_SD_CD)            |\
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CMD)       |\
                                     PIN_MODE_INPUT(GPIOD_PIN3)             |\
                                     PIN_MODE_INPUT(GPIOD_PIN4)             |\
                                     PIN_MODE_ALTERNATE(GPIOD_EXT_TX)    |\
                                     PIN_MODE_INPUT(GPIOD_PIN6)     |\
                                     PIN_MODE_OUTPUT(GPIOD_RADIO_CS)         |\
                                     PIN_MODE_INPUT(GPIOD_PIN8)    |\
                                     PIN_MODE_INPUT(GPIOD_PIN9)    |\
                                     PIN_MODE_OUTPUT(GPIOD_PYRO_GRN)      |\
                                     PIN_MODE_OUTPUT(GPIOD_PYRO_RED)            |\
                                     PIN_MODE_OUTPUT(GPIOD_RADIO_GRN)            |\
                                     PIN_MODE_OUTPUT(GPIOD_RADIO_RED)              |\
                                     PIN_MODE_OUTPUT(GPIOD_IMU_GRN)            |\
                                     PIN_MODE_OUTPUT(GPIOD_IMU_RED))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CD)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CMD)       |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_EXT_TX)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_RADIO_CS)      |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8)|\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9)|\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PYRO_GRN)  |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PYRO_RED)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_RADIO_GRN)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_RADIO_RED)          |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_IMU_GRN)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_IMU_RED))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_2M(GPIOD_PIN0)              |\
                                     PIN_OSPEED_2M(GPIOD_SD_CD)             |\
                                     PIN_OSPEED_100M(GPIOD_SD_CMD)          |\
                                     PIN_OSPEED_2M(GPIOD_PIN3)              |\
                                     PIN_OSPEED_2M(GPIOD_PIN4)              |\
                                     PIN_OSPEED_100M(GPIOD_EXT_TX)         |\
                                     PIN_OSPEED_2M(GPIOD_PIN6)          |\
                                     PIN_OSPEED_100M(GPIOD_RADIO_CS)          |\
                                     PIN_OSPEED_2M(GPIOD_PIN8)     |\
                                     PIN_OSPEED_2M(GPIOD_PIN9)     |\
                                     PIN_OSPEED_2M(GPIOD_PYRO_GRN)      |\
                                     PIN_OSPEED_2M(GPIOD_PYRO_RED)             |\
                                     PIN_OSPEED_2M(GPIOD_RADIO_GRN)             |\
                                     PIN_OSPEED_2M(GPIOD_RADIO_RED)               |\
                                     PIN_OSPEED_2M(GPIOD_IMU_GRN)             |\
                                     PIN_OSPEED_2M(GPIOD_IMU_RED))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_PIN0)           |\
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CD)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CMD)       |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3)           |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4)           |\
                                     PIN_PUPDR_FLOATING(GPIOD_EXT_TX)    |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6)     |\
                                     PIN_PUPDR_FLOATING(GPIOD_RADIO_CS)      |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8)|\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9)|\
                                     PIN_PUPDR_FLOATING(GPIOD_PYRO_GRN)  |\
                                     PIN_PUPDR_FLOATING(GPIOD_PYRO_RED)          |\
                                     PIN_PUPDR_FLOATING(GPIOD_RADIO_GRN)          |\
                                     PIN_PUPDR_FLOATING(GPIOD_RADIO_RED)            |\
                                     PIN_PUPDR_FLOATING(GPIOD_IMU_GRN)          |\
                                     PIN_PUPDR_FLOATING(GPIOD_IMU_RED))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_PIN0)               |\
                                     PIN_ODR_HIGH(GPIOD_SD_CD)              |\
                                     PIN_ODR_HIGH(GPIOD_SD_CMD)             |\
                                     PIN_ODR_LOW(GPIOD_PIN3)               |\
                                     PIN_ODR_LOW(GPIOD_PIN4)               |\
                                     PIN_ODR_HIGH(GPIOD_EXT_TX)          |\
                                     PIN_ODR_LOW(GPIOD_PIN6)           |\
                                     PIN_ODR_HIGH(GPIOD_RADIO_CS)            |\
                                     PIN_ODR_LOW(GPIOD_PIN8)      |\
                                     PIN_ODR_LOW(GPIOD_PIN9)      |\
                                     PIN_ODR_LOW(GPIOD_PYRO_GRN)        |\
                                     PIN_ODR_LOW(GPIOD_PYRO_RED)              |\
                                     PIN_ODR_LOW(GPIOD_RADIO_GRN)              |\
                                     PIN_ODR_LOW(GPIOD_RADIO_RED)                |\
                                     PIN_ODR_LOW(GPIOD_IMU_GRN)              |\
                                     PIN_ODR_LOW(GPIOD_IMU_RED))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0)             |\
                                     PIN_AFIO_AF(GPIOD_SD_CD, 0)            |\
                                     PIN_AFIO_AF(GPIOD_SD_CMD, 12)          |\
                                     PIN_AFIO_AF(GPIOD_PIN3, 0)             |\
                                     PIN_AFIO_AF(GPIOD_PIN4, 0)             |\
                                     PIN_AFIO_AF(GPIOD_EXT_TX, 7)        |\
                                     PIN_AFIO_AF(GPIOD_PIN6, 0)         |\
                                     PIN_AFIO_AF(GPIOD_RADIO_CS, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0)    |\
                                     PIN_AFIO_AF(GPIOD_PIN9, 0)    |\
                                     PIN_AFIO_AF(GPIOD_PYRO_GRN, 0)      |\
                                     PIN_AFIO_AF(GPIOD_PYRO_RED, 0)            |\
                                     PIN_AFIO_AF(GPIOD_RADIO_GRN, 0)            |\
                                     PIN_AFIO_AF(GPIOD_RADIO_RED, 0)              |\
                                     PIN_AFIO_AF(GPIOD_IMU_GRN, 0)            |\
                                     PIN_AFIO_AF(GPIOD_IMU_RED, 0))

#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0)       |\
                                     PIN_MODE_INPUT(GPIOE_PIN1)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN2)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN3)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN4)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN5)             |\
                                     PIN_MODE_INPUT(GPIOE_PIN6)             |\
                                     PIN_MODE_INPUT(GPIOE_PY1_CHK)         |\
                                     PIN_MODE_OUTPUT(GPIOE_PY1_TRG)         |\
                                     PIN_MODE_OUTPUT(GPIOE_PY2_TRG)         |\
                                     PIN_MODE_INPUT(GPIOE_PY2_CHK)        |\
                                     PIN_MODE_INPUT(GPIOE_PY3_CHK)        |\
                                     PIN_MODE_OUTPUT(GPIOE_PY3_TRG)        |\
                                     PIN_MODE_OUTPUT(GPIOE_PY4_TRG)        |\
                                     PIN_MODE_INPUT(GPIOE_PY4_CHK)        |\
                                     PIN_MODE_INPUT(GPIOE_MAGNO_DRDY))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0)   |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6)         |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY1_CHK)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY1_TRG)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY2_TRG)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY2_CHK)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY3_CHK)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY3_TRG)     |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY4_TRG)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PY4_CHK)    |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_MAGNO_DRDY))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_2M(GPIOE_PIN0)        |\
                                     PIN_OSPEED_2M(GPIOE_PIN1)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN2)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN3)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN4)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN5)              |\
                                     PIN_OSPEED_2M(GPIOE_PIN6)              |\
                                     PIN_OSPEED_2M(GPIOE_PY1_CHK)          |\
                                     PIN_OSPEED_2M(GPIOE_PY1_TRG)          |\
                                     PIN_OSPEED_2M(GPIOE_PY2_TRG)          |\
                                     PIN_OSPEED_2M(GPIOE_PY2_CHK)          |\
                                     PIN_OSPEED_2M(GPIOE_PY3_CHK)          |\
                                     PIN_OSPEED_2M(GPIOE_PY3_TRG)          |\
                                     PIN_OSPEED_2M(GPIOE_PY4_TRG)         |\
                                     PIN_OSPEED_2M(GPIOE_PY4_CHK)         |\
                                     PIN_OSPEED_2M(GPIOE_MAGNO_DRDY))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6)           |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY1_CHK)       |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY1_TRG)       |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY2_TRG)       |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY2_CHK)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY3_CHK)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY3_TRG)     |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY4_TRG)      |\
                                     PIN_PUPDR_FLOATING(GPIOE_PY4_CHK)      |\
                                     PIN_PUPDR_PULLUP(GPIOE_MAGNO_DRDY))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_PIN0)         |\
                                     PIN_ODR_LOW(GPIOE_PIN1)               |\
                                     PIN_ODR_LOW(GPIOE_PIN2)               |\
                                     PIN_ODR_LOW(GPIOE_PIN3)               |\
                                     PIN_ODR_LOW(GPIOE_PIN4)               |\
                                     PIN_ODR_LOW(GPIOE_PIN5)               |\
                                     PIN_ODR_LOW(GPIOE_PIN6)               |\
                                     PIN_ODR_LOW(GPIOE_PY1_CHK)            |\
                                     PIN_ODR_LOW(GPIOE_PY1_TRG)            |\
                                     PIN_ODR_LOW(GPIOE_PY2_TRG)           |\
                                     PIN_ODR_LOW(GPIOE_PY2_CHK)            |\
                                     PIN_ODR_LOW(GPIOE_PY3_CHK)            |\
                                     PIN_ODR_LOW(GPIOE_PY3_TRG)            |\
                                     PIN_ODR_LOW(GPIOE_PY4_TRG)          |\
                                     PIN_ODR_LOW(GPIOE_PY4_CHK)          |\
                                     PIN_ODR_LOW(GPIOE_MAGNO_DRDY))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0)       |\
                                     PIN_AFIO_AF(GPIOE_PIN1, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN2, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN3, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN4, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN5, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PIN6, 0)             |\
                                     PIN_AFIO_AF(GPIOE_PY1_CHK, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PY1_TRG, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PY2_TRG, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PY2_CHK, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PY3_CHK, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PY3_TRG, 0)         |\
                                     PIN_AFIO_AF(GPIOE_PY4_TRG, 0)        |\
                                     PIN_AFIO_AF(GPIOE_PY4_CHK, 0)        |\
                                     PIN_AFIO_AF(GPIOE_MAGNO_DRDY, 0))

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
