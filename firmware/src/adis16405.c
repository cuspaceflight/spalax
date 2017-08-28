
#include <stdlib.h>
#include "ch.h"
#include "adis16405.h"
#include "adis16405-reg.h"
#include "calibration/adis16405_calibration.h"
#include "badthinghandler.h"
#include "spalaxconf.h"
#include "messaging.h"
#include "platform.h"

#if BUILD_ADIS

static binary_semaphore_t adis16405_semaphore;
static const uint32_t adis16405_send_over_usb_count = 0; // Will send 1 in every 100 samples
static const uint32_t adis16405_send_over_can_count = 0;
static volatile bool adis16405_initialized = false;

static const SPIConfig spi_cfg = {
    NULL,
    ADIS16405_SPI_CS_PORT,
    ADIS16405_SPI_CS_PIN,
    // CPOL, CPHA, MSB First, 16-bit frame
    // Clock rate should be <= 1 MHz for burst mode
    // I believe this sets it to 168000000 / 4 / 64 ~= 1MHz
    // TODO: Verify this
    SPI_CR1_BR_2 | SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF
};

static uint16_t internal_adis16405_read_u16(uint8_t addr_in) {
    uint16_t addr_out = (uint16_t)(addr_in & 0x7F) << 8;
    uint16_t data_rx;

    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)&addr_out);
    spiReceive(&ADIS16405_SPID, 1, (void*)&data_rx);
    spiUnselect(&ADIS16405_SPID);
    return data_rx;
}

static uint16_t adis16405_read_u16(uint8_t addr_in) {
    // All transfers are 16 bits
    // Addresses are 7 bits
    // We therefore address using the following system for reading

    // MSB --- LSB
    // 0 write bit, 7 bit address, 8 bits of 0
    // For example, 0x0A00 would read address 0x0A

    // Clear write bit
    // Shift into position


    spiAcquireBus(&ADIS16405_SPID);
    spiStart(&ADIS16405_SPID, &spi_cfg);

    uint16_t data_rx = internal_adis16405_read_u16(addr_in);

    spiReleaseBus(&ADIS16405_SPID);

    return data_rx;
}

static void adis16405_read_multiple(int base_addr, int num, uint16_t* rx_buff) {
    spiAcquireBus(&ADIS16405_SPID);
    spiStart(&ADIS16405_SPID, &spi_cfg);
    for (int i = 0; i < num; i++)
        rx_buff[i] = internal_adis16405_read_u16(base_addr + i * 2);

    spiReleaseBus(&ADIS16405_SPID);
}

static void adis16405_write_u8(uint8_t addr, uint8_t val) {
    uint16_t txbuf[1] = {
        ((((uint16_t)addr | 0x80) << 8) | val)
    };

    spiAcquireBus(&ADIS16405_SPID);
    spiStart(&ADIS16405_SPID, &spi_cfg);

    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)txbuf);
    spiUnselect(&ADIS16405_SPID);

    spiReleaseBus(&ADIS16405_SPID);
}

static void adis16405_write_u16(uint8_t addr, uint16_t val) {
    // All transfers are 16 bits
    // Addresses are 7 bits
    // We therefore address using the following system

    // MSB --- LSB
    // 1 Write bit, 7 bit address, 8 bits of data
    // For example, 0xA11F loads 0x1F into location 0x21

    uint16_t txbuf[2] = {
        ((((uint16_t)(addr+1) | 0x80) << 8) | ((val >> 8) & 0xFF)),
        ((((uint16_t)addr | 0x80) << 8) | (val & 0xFF))
    };

    spiAcquireBus(&ADIS16405_SPID);
    spiStart(&ADIS16405_SPID, &spi_cfg);

    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)txbuf);
    spiUnselect(&ADIS16405_SPID);
    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)&txbuf[1]);
    spiUnselect(&ADIS16405_SPID);

    spiReleaseBus(&ADIS16405_SPID);
}

static int16_t sign_extend(uint16_t val, int bits) {
	if((val&(1<<(bits-1))) != 0){
		val = val - (1<<bits);
	}
	return val;
}



static bool adis16405_burst_read(int16_t rx_buff[10]) {
    // Burst mode data collection is a more efficient way of reading the sensor data
    // We write the value 0x3E00
    // Then for the next 12 clock periods the sensor module will write back the sensor data
    // The order is:
    //     SUPPLY_OUT, XGYRO_OUT, YGYRO_OUT, ZGYRO_OUT, XACCL_OUT, YACCL_OUT, ZACCL_OUT
    //     XMAGN_OUT, YMAGN_OUT, ZMAGN_OUT, TEMP_OUT, AUX_ADC

    // All bar the final 2 have 14 bits of data - the final 2 are only 12 bits
    // The MSB (bit 15) is the ND flag - it is 1 if this data hasn't been read before
    // Bit 14 is the EA flag - it is 1 if there is an error flag in the DIAG_STAT register

    // The burst read doesn't seem to work so resorted to trying to get full duplex
    // read to work. This also didn't work, so eventually just resorted to reading
    // Each register manually

    adis16405_read_multiple(0x02, 10, (uint16_t*)rx_buff);
    for (int i = 0; i < 10; i++) {
        if((rx_buff[i] & 0x4000) == 1)
        {
          uint16_t errors = adis16405_read_u16(0x3C);
          COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
          component_state_update(avionics_component_adis16405, state_error, errors);
          return false;
        }
    }
    rx_buff[0] = sign_extend((uint16_t) (rx_buff[0] & 0x3fff), 14);
    rx_buff[1] = sign_extend((uint16_t) (rx_buff[1] & 0x3fff), 14);
    rx_buff[2] = sign_extend((uint16_t) (rx_buff[2] & 0x3fff), 14);
    rx_buff[3] = sign_extend((uint16_t) (rx_buff[3] & 0x3fff), 14);
    rx_buff[4] = sign_extend((uint16_t) (rx_buff[4] & 0x3fff), 14);
    rx_buff[5] = sign_extend((uint16_t) (rx_buff[5] & 0x3fff), 14);
    rx_buff[6] = sign_extend((uint16_t) (rx_buff[6] & 0x3fff), 14);
    rx_buff[7] = sign_extend((uint16_t) (rx_buff[7] & 0x3fff), 14);
    rx_buff[8] = sign_extend((uint16_t) (rx_buff[8] & 0x3fff), 14);
    rx_buff[9] = sign_extend((uint16_t) (rx_buff[9] & 0x3fff), 14);
    //rx_buff[10] = sign_extend(rx_buff[10] & 0x3fff, 12);
    //rx_buff[11] = rx_buff[11] & 0x0fff;

    return true;
}

static void adis16405_init() {

    // Reset calibration to factory defaults
    adis16405_write_u16(ADIS16405_REG_GLOB_CMD, 0x0001);
    chThdSleepMilliseconds(100);
    // Setting the data rate to the default of 819.2 samples per second 0x0001
    adis16405_write_u16(ADIS16405_REG_SMPL_PRD,0x0001);
    chThdSleepMilliseconds(50);
    // Setting sleep mode 0x0000 turn sleep mode off
    adis16405_write_u16(ADIS16405_REG_SLP_CNT,0x0000);
    chThdSleepMilliseconds(50);
    // Setting the data rate to 300 degrees per sec
    // And the number of filter taps to 4 resulting in a
    // filter bandwidth of ~205Hz
    adis16405_write_u16(ADIS16405_REG_SENS_AVG, 0x0402);
    chThdSleepMilliseconds(50);
    // Reset GPIO controls
    adis16405_write_u16(ADIS16405_REG_GPIO_CTRL, 0x0000);
    chThdSleepMilliseconds(50);
    // Enable data ready interrupt
    adis16405_write_u8(ADIS16405_REG_MSC_CTRL, 0x86);
    chThdSleepMilliseconds(50);

    // Disable Alarms
    adis16405_write_u16(ADIS16405_REG_ALM_CTRL, 0x0000);
    chThdSleepMilliseconds(50);

    // Write to flash
    // adis16405_write_u16(ADIS16405_REG_GLOB_CMD, 0x0008);
    // chThdSleepMilliseconds(100);

    uint16_t errors = adis16405_read_u16(0x3C);
    if (errors != 0) {
        COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
        component_state_update(avionics_component_adis16405, state_error, errors);
        component_state_update(avionics_component_adis16405, state_error, errors >> 8);
    }

    adis16405_initialized = true;
    memory_barrier_release();
}

static bool adis16405_id_check(void) {
    uint16_t id = adis16405_read_u16(0x56);
    return id == 0x4015;
}

static bool adis16405_self_test(void) {
    // Self tests - using the internal testing routine - do all self test
    adis16405_write_u8(ADIS16405_REG_MSC_CTRL+1,0x04);

    // Self test - Checks if bit has been cleared
    while ((adis16405_read_u16(ADIS16405_REG_MSC_CTRL) & 0x0400) != 0) {
        chThdSleepMilliseconds(500);
    }

    uint16_t errors = adis16405_read_u16(0x3C);

    // Self test - Checks if error found when test run
    return errors == 0;
}

//static void adis16405_gyroscope_precision_null_calibration() {
//    adis16405_write_u8(ADIS16405_REG_GLOB_CMD, 0x10);
//
//    while ((adis16405_read_u16(ADIS16405_REG_GLOB_CMD) & 0x10) != 0) {
//        chThdSleepMilliseconds(500);
//    }
//}

void adis16405_wakeup(EXTDriver *extp, expchannel_t channel) {
    (void)extp;
    (void)channel;

    memory_barrier_acquire();
    if (adis16405_initialized == false)
        return;

    chSysLockFromISR();
    chBSemSignalI(&adis16405_semaphore);
    chSysUnlockFromISR();
}

MESSAGING_PRODUCER(messaging_producer_data, ts_adis16405_data, sizeof(adis16405_data_t), 100)

void adis16405_thread(void *arg) {
    (void)arg;

    chBSemObjectInit(&adis16405_semaphore, true);

    chRegSetThreadName("ADIS16405");

    COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_initializing);

    chThdSleepMilliseconds(220);

    palClearPad(ADIS16405_NRST_PORT, ADIS16405_NRST_PIN);

    chThdSleepMilliseconds(200);

    palSetPad(ADIS16405_NRST_PORT, ADIS16405_NRST_PIN);

    chThdSleepMilliseconds(200);

    // Wait for startup
    while (!adis16405_id_check()) {
        chThdSleepMilliseconds(100);
    }

    // Log that we have passed id check
    COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_initializing);

    uint16_t errors = adis16405_read_u16(0x3C);
    if (errors != 0) {
        COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
        component_state_update(avionics_component_adis16405, state_error, errors);
        return;
    }

    // Log that we have passed the error check
    COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_initializing);

    while (!adis16405_self_test()) {
        chThdSleepMilliseconds(100);
    }


    errors = adis16405_read_u16(0x3C);
    if (errors != 0) {
        COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
        component_state_update(avionics_component_adis16405, state_error, errors);
        return;
    }

    COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_initializing);

    //adis16405_gyroscope_precision_null_calibration();

    adis16405_init();

    messaging_producer_init(&messaging_producer_data);

    COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_ok);

    adis16405_data_t data;

    uint32_t send_over_usb_count = adis16405_send_over_usb_count;
    uint32_t send_over_can_count = adis16405_send_over_can_count;
    while(TRUE) {
        //chSysLock();
        //chBSemWaitS(&adis16405_semaphore);
        //chSysUnlock();

        chThdSleepMicroseconds(410);

        if (!adis16405_burst_read((int16_t*)&data))
            continue;

        uint16_t value = adis16405_read_u16(ADIS16405_REG_MSC_CTRL);
        if (value != 0x0086) {
            COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
        }

        message_metadata_t flags = 0;

        if (send_over_usb_count == adis16405_send_over_usb_count)
            send_over_usb_count = 0;
        else {
            flags |= message_flags_dont_send_over_usb;
            send_over_usb_count++;
        }

        if (send_over_can_count == adis16405_send_over_can_count) {
            send_over_can_count = 0;
            flags |= message_flags_send_over_can;
        } else {
            send_over_can_count++;
        }

        messaging_producer_send(&messaging_producer_data, flags, (const uint8_t*)&data);

    }
}

#endif