#include <stdlib.h>
#include "ch.h"
#include "chprintf.h"
#include "mpu9250.h"
#include "badthinghandler.h"

#define MPU9250_SPID         SPID1
#define MPU9250_SPI_CS_PORT  GPIOA
#define MPU9250_SPI_CS_PIN   GPIOA_MPU_NSS

static BinarySemaphore mpu9250_semaphore;

static uint8_t mpu9250_read_u8(uint8_t addr) {
    // Set the read bit
    addr |= (1 << 7);

    uint8_t read_val;

    spiSelect(&MPU9250_SPID);
    spiSend(&MPU9250_SPID, 1, (void*)&addr);
    spiReceive(&MPU9250_SPID, 1, (void*)&read_val);
    spiUnselect(&MPU9250_SPID);

    return read_val;
}

static void mpu9250_read_multiple(uint8_t addr, uint8_t* buf, int num) {
    // Set the read bit
    addr |= (1 << 7);

    spiSelect(&MPU9250_SPID);
    spiSend(&MPU9250_SPID, 1, (void*)&addr);
    spiReceive(&MPU9250_SPID, num, (void*)buf);
    spiUnselect(&MPU9250_SPID);
}

static void mpu9250_write_u8(uint8_t addr, uint8_t val) {
    // Clear the read bit
    addr &= ~(1 << 7);

    uint8_t tx_buff[2];
    tx_buff[0] = addr;
    tx_buff[1] = val;

    spiSelect(&MPU9250_SPID);
    spiSend(&MPU9250_SPID, 2, (void*)&tx_buff);
    spiUnselect(&MPU9250_SPID);
}

static void mpu9250_burst_read(uint16_t data_out[10]) {
    // Grab all the sensor data in the minimal number of steps
    // It also merges high and low bytes together for convenience
    // The array contains the following in order
    // ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT, TEMP_OUT, GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT
    // MAGNO_XOUT, MAGNO_YOUT, MAGNO_ZOUT


    // We first read accel and gyro data
    uint8_t rx_buff[14];

    mpu9250_read_multiple(0x3B, rx_buff, 14);

    int j = 0;
    for (int i = 0; i < 7; i++) {
        // Merge high byte (which comes first) with low byte (which comes second)
        data_out[i] = (rx_buff[j] << 7) | rx_buff[j + 1];
        j += 2;
    }

    // We next get the magno data
    mpu9250_read_multiple(0x03, rx_buff, 6);

    j = 0;
    for (int i = 7; i < 10; i++) {
        // Merge high byte (which comes second) with low byte (which comes first)
        data_out[i] = (rx_buff[j+1] << 7) | rx_buff[j];
        j += 2;
    }

}

static bool mpu9250_id_check(void) {
    // Read WHOAMI Register
    uint8_t whoami = mpu9250_read_u8(0x75);
    return whoami == 0x71;
}

static void mpu9250_init(void) {
    // Set User Control Register - Disable FIFO, Disable I2C
    mpu9250_write_u8(0x6A, 0x10);

    // TODO: Initialize sensor - setting control registers to appropriate values


    // TODO: Perform self test
}

void mpu9250_wakeup(EXTDriver *extp, expchannel_t channel) {
    (void)extp;
    (void)channel;

    chSysLockFromIsr();
    chBSemSignalI(&mpu9250_semaphore);
    chSysUnlockFromIsr();
}

msg_t mpu9250_thread(void *arg) {
    (void)arg;
    const SPIConfig spi_cfg = {
        NULL,
        MPU9250_SPI_CS_PORT,
        MPU9250_SPI_CS_PIN,
        // CPOL, CPHA, MSB First, 8-bit frame
        // Clock rate should be <= 1 MHz for burst mode
        // I believe this sets it to 168000000 / 4 / 64 ~= 1MHz
        // TODO: Verify this
        SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA
    };

    chBSemInit(&mpu9250_semaphore, true);

    chRegSetThreadName("MPU9250");

    spiStart(&MPU9250_SPID, &spi_cfg);

    // Wait for startup
    while (!mpu9250_id_check()) {
        bthandler_set_error(ERROR_MPU9250, true);
        chThdSleepMilliseconds(50);
    }
    bthandler_set_error(ERROR_MPU9250, false);

    mpu9250_init();

    uint16_t raw_data[10];
    while(TRUE) {
        chSysLock();
        chBSemWaitTimeoutS(&mpu9250_semaphore, 100);
        chSysUnlock();

        mpu9250_burst_read(raw_data);

        // TODO Convert raw_data to meaningful numbers
    }
}
