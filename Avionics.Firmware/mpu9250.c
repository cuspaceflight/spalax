#include <stdlib.h>
#include <math.h> // for self-test

#include "ch.h"
#include "chprintf.h"
#include "mpu9250.h"
#include "badthinghandler.h"
#include "compilermacros.h"
#include "mpu9250-reg.h"

#define MPU9250_SPID         SPID1
#define MPU9250_SPI_CS_PORT  GPIOA
#define MPU9250_SPI_CS_PIN   GPIOA_MPU_NSS

static BinarySemaphore mpu9250_semaphore;

//// LOW-LEVEL COMMUNICATION ////

// mpu9250_read_u8 reads a single byte value from an 8-bit MPU9250 register
static uint8_t mpu9250_read_u8(uint8_t addr);

// mpu9250_read_multiple reads a multi-byte register from the MPU9250 into a
// memory buffer. The buffer pointed to by buf should have at least num bytes
// available.
static void mpu9250_read_multiple(uint8_t addr, uint8_t* buf, int num);

// mpu9250_write_u8 writes a single byte into an 8-bit MPU9250 register.
static void mpu9250_write_u8(uint8_t addr, uint8_t val);

//// HIGH-LEVEL OPERATIONS ////

// mpu9250_id_check performs a simple sanity check on MPU9250 communication by
// checking that the WHO_AM_I register of the MPU9250 has an expected value.
static bool mpu9250_id_check(void);

// mpu9250_self_test_gyro performs a self-test on the gyro, compares the result
// to the factory stored values and returns true iff the self-test succeeds.
static bool mpu9250_self_test_gyro(void);

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
    // Read WHO_AM_I Register
    uint8_t whoami = mpu9250_read_u8(MPU9250_REG_WHO_AM_I);
    return whoami == MPU9250_WHO_AM_I_RESET_VALUE;
}

// Take the mean of the three measurement values represented by the six
// registers starting at base_reg. Write 16-bit results to the buffer pointed to
// by out. The behaviour is to sample 256 values at 1KHz and return the mean.
static void mpu9250_read_mean_triplet(uint8_t base_reg, int16_t *out) {
    uint8_t sensor_values[6];
    int32_t sensor_accumulator[3] = {0, 0, 0};

    // Accumulate 256 readings
    for(int i=0; i<256; ++i) {
        // Read gyro values
        mpu9250_read_multiple(base_reg, sensor_values, 6);

        sensor_accumulator[0] += (int32_t)bytes_to_uint16(
            sensor_values[0], sensor_values[1]);
        sensor_accumulator[1] += (int32_t)bytes_to_uint16(
            sensor_values[2], sensor_values[3]);
        sensor_accumulator[2] += (int32_t)bytes_to_uint16(
            sensor_values[4], sensor_values[5]);

        chThdSleepMilliseconds(1);
    }

    // Shift right to take mean and record
    for(int i=0; i<3; ++i) {
        sensor_accumulator[i] >>= 8;
        out[i] = sensor_accumulator[i];
    }
}

// mpu9250_self_test_to_factory_trim takes self-test register values and
// transforms them to the corresponding factory trim values which should match
// the sensor readings. (It is assumed that the full-scale select is 00 ==
// +/-250 deg/sec.) It appears non-trivial to find the application note which
// describes the self-test procedure. This formula was lifted from [1].
//
// [1] https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino
static inline int16_t mpu9250_self_test_to_factory_trim(uint8_t st_val) {
    return (int16_t)(2620.f * powf(1.01f, ((float)st_val) - 1.f));
}

static bool mpu9250_self_test_gyro(void) {
    uint8_t factory_st_output[3], old_gyro_config;
    int16_t base_values[3], self_test_values[3];

    // Read old gyro config
    old_gyro_config = mpu9250_read_u8(MPU9250_REG_GYRO_CONFIG);

    // Gyro config => range of +/- 250 degrees/sec
    mpu9250_write_u8(MPU9250_REG_GYRO_CONFIG, 0x00);

    // Delay for a short while to let device stabilise
    chThdSleepMilliseconds(50);

    // Record mean of incoming values
    mpu9250_read_mean_triplet(MPU9250_REG_GYRO_XOUT_H, base_values);

    // Set gyro config to self test on all axes with gyro range of +/- 250
    // degrees/s
    mpu9250_write_u8(MPU9250_REG_GYRO_CONFIG, 0xe0);

    // Delay for a short while to let device stabilise
    chThdSleepMilliseconds(50);

    // Record mean of incoming values
    mpu9250_read_mean_triplet(MPU9250_REG_GYRO_XOUT_H, self_test_values);

    // Restore old gyro config
    mpu9250_write_u8(MPU9250_REG_GYRO_CONFIG, old_gyro_config);

    // Subtract base readings from self_test_values.
    for(int i=0; i<3; ++i) {
        self_test_values[i] -= base_values[i];
    }

    // Read factory self-test output
    mpu9250_read_multiple(MPU9250_REG_SELF_TEST_X_GYRO, factory_st_output, 3);

    // Convert this to expected values for self-test output and check absolute
    // difference from measured. Fail if the difference is greater than 5% of
    // the expected value.
    for(int i=0; i<3; ++i) {
        int16_t expected_val = mpu9250_self_test_to_factory_trim(
            factory_st_output[i]);
        int32_t delta = (int32_t)self_test_values[i] - (int32_t)expected_val;
        if(abs(delta) > abs(expected_val)/20) {
            // We fail :(
            return false;
        }
    }

    // Self test passed!
    return true;
}

static void mpu9250_init(void) {
    // Register/value pairs to reset/initialise MPU9250
    uint8_t init_sequence[][2] = {
        { MPU9250_REG_PWR_MGMT_1, 0x80 }, // Reset
        { MPU9250_REG_PWR_MGMT_1, 0x00 }, // Select internal 20MHz source
        { MPU9250_REG_PWR_MGMT_2, 0x00 }, // Enable gyro & accel
        { MPU9250_REG_USER_CTRL, 0x10 }, // SPI only, disable FIFO
    };

    // Perform initial reset
    for(size_t i=0; i < sizeof(init_sequence)/sizeof(init_sequence[0]); ++i) {
        mpu9250_write_u8(init_sequence[i][0], init_sequence[i][1]);
    }
}

void mpu9250_wakeup(EXTDriver *extp, expchannel_t channel) {
    (void)extp;
    (void)channel;

    chSysLockFromIsr();
    chBSemSignalI(&mpu9250_semaphore);
    chSysUnlockFromIsr();
}

msg_t mpu9250_thread(COMPILER_UNUSED_ARG(void *arg)) {
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

    // Perform a self-test of the MPU9250
    if(!mpu9250_self_test_gyro()) {
        bthandler_set_error(ERROR_MPU9250, true);
    }

    uint16_t raw_data[10];
    while(TRUE) {
        chSysLock();
        chBSemWaitTimeoutS(&mpu9250_semaphore, 100);
        chSysUnlock();

        mpu9250_burst_read(raw_data);

        // TODO Convert raw_data to meaningful numbers
    }
}
