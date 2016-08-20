
#include <stdlib.h>
#include "ch.h"
#include "adis16405.h"
#include "adis16405-reg.h"
#include "badthinghandler.h"
#include "spalaxconf.h"
#include "math_utils.h"

static binary_semaphore_t adis16405_semaphore;


static uint16_t adis16405_read_u16(uint8_t addr_in) {
    // All transfers are 16 bits
    // Addresses are 7 bits
    // We therefore address using the following system for reading

    // MSB --- LSB
    // 0 write bit, 7 bit address, 8 bits of 0
    // For example, 0x0A00 would read address 0x0A

    // Clear write bit
    // Shift into position
    uint16_t addr_out = (uint16_t)(addr_in & 0x7F) << 8;
    uint16_t data_rx;

    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)&addr_out);
    spiReceive(&ADIS16405_SPID, 1, (void*)&data_rx);
    spiUnselect(&ADIS16405_SPID);

    return data_rx;
}

static void adis16405_read_multiple(int base_addr, int num, uint16_t* rx_buff, uint16_t* tx_buff) {
    for (int i = 0; i < num; i++) {
        tx_buff[i] =  (uint16_t)((base_addr + 2*i) & 0x7F) << 8;
    }
    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)&tx_buff[0]);
    spiExchange(&ADIS16405_SPID, num - 1, (void*)&tx_buff[1], (void*)rx_buff);
    spiReceive(&ADIS16405_SPID, 1, (void*)&tx_buff[num-1]);
    spiUnselect(&ADIS16405_SPID);
}

static void adis16405_write_u16(uint8_t addr, uint16_t val) {
    // All transfers are 16 bits
    // Addresses are 7 bits
    // We therefore address using the following system

    // MSB --- LSB
    // 1 Write bit, 7 bit address, 8 bits of data
    // For example, 0xA11F loads 0x1F into location 0x21

    uint16_t txbuf[2] = {
        ((((uint16_t)addr | 0x80) << 8) | (val & 0xFF)),
		((((uint16_t)(addr+1) | 0x80) << 8) | ((val >> 8) & 0xFF))
    };

    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)txbuf);
    spiUnselect(&ADIS16405_SPID);
    spiSelect(&ADIS16405_SPID);
    spiSend(&ADIS16405_SPID, 1, (void*)&txbuf[1]);
    spiUnselect(&ADIS16405_SPID);
}

static int16_t sign_extend(uint16_t val, int bits) {
	if((val&(1<<(bits-1))) != 0){
		val = val - (1<<bits);
	}
	return val;
}



static bool adis16405_burst_read(int16_t data_out[12]) {
    // Burst mode data collection is a more efficient way of reading the sensor data
    // We write the value 0x3E00
    // Then for the next 12 clock periods the sensor module will write back the sensor data
    // The order is:
    //     SUPPLY_OUT, XGYRO_OUT, YGYRO_OUT, ZGYRO_OUT, XACCL_OUT, YACCL_OUT, ZACCL_OUT
    //     XMAGN_OUT, YMAGN_OUT, ZMAGN_OUT, TEMP_OUT, AUX_ADC

    // All bar the final 2 have 14 bits of data - the final 2 are only 12 bits
    // The MSB (bit 15) is the ND flag - it is 1 if this data hasn't been read before
    // Bit 14 is the EA flag - it is 1 if there is an error flag in the DIAG_STAT register

    // The burst read doesn't seem to work so resorted to full duplex read instead

    uint16_t tx_buff[12];

    // Temporary
    for (int i = 0; i < 12; i++) {
        tx_buff[i] = 0xBEEF;
    }

    adis16405_read_multiple(0x02, 12, (uint16_t*)data_out, (uint16_t*)tx_buff);
    for (int i = 0; i < 12; i++) {
        data_out[i] = adis16405_read_u16(0x02 + 2*i);
        if((data_out[i] & 0x4000) == 1)
        {
          uint16_t errors = adis16405_read_u16(0x3C);
          COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
          component_state_update(avionics_component_adis16405, state_error, errors);
          return false;
        }
        // if((data_out[i] & 0x8000) == 0)
        // {
        //    // Data has already been read
        //    return false;
        // }
    }
    data_out[0] = sign_extend(data_out[0] & 0x3fff, 14);
    data_out[1] = sign_extend(data_out[1] & 0x3fff, 14);
    data_out[2] = sign_extend(data_out[2] & 0x3fff, 14);
    data_out[3] = sign_extend(data_out[3] & 0x3fff, 14);
    data_out[4] = sign_extend(data_out[4] & 0x3fff, 14);
    data_out[5] = sign_extend(data_out[5] & 0x3fff, 14);
    data_out[6] = sign_extend(data_out[6] & 0x3fff, 14);
    data_out[7] = sign_extend(data_out[7] & 0x3fff, 14);
    data_out[8] = sign_extend(data_out[8] & 0x3fff, 14);
    data_out[9] = sign_extend(data_out[9] & 0x3fff, 14);
    data_out[10] = sign_extend(data_out[10] & 0x3fff, 12);
    data_out[11] = data_out[11] & 0x0fff;

    return true;
}

static void adis16405_init(void) {

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
    adis16405_write_u16(ADIS16405_REG_SENS_AVG, 0x0402);
    chThdSleepMilliseconds(50);
    // Reset GPIO controls
    adis16405_write_u16(ADIS16405_REG_GPIO_CTRL, 0x0000);
    chThdSleepMilliseconds(50);
    // Enable data ready interrupt
    adis16405_write_u16(ADIS16405_REG_MSC_CTRL, 0x0006);
    chThdSleepMilliseconds(50);
    // Disable Alarms
    adis16405_write_u16(ADIS16405_REG_ALM_CTRL, 0x0000);
    chThdSleepMilliseconds(50);

    uint16_t errors = adis16405_read_u16(0x3C);
    if (errors != 0) {
        COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
        component_state_update(avionics_component_adis16405, state_error, errors);
    }

}

static bool adis16405_id_check(void) {
    uint16_t id = adis16405_read_u16(0x56);
    return id == 0x4015;
}

static bool adis16405_self_test(void) {
    // Self tests - using the internal testing routine - do all self test
    adis16405_write_u16(ADIS16405_REG_MSC_CTRL,0x0400);

    // Self test - Checks if bit has been cleared
    while ((adis16405_read_u16(ADIS16405_REG_MSC_CTRL) & 0x0400)) {
        chThdSleepMilliseconds(500);
    }

    uint16_t errors = adis16405_read_u16(0x3C);

    // Self test - Checks if error found when test run
    return errors == 0;
}

void adis16405_wakeup(EXTDriver *extp, expchannel_t channel) {
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chBSemSignalI(&adis16405_semaphore);
    chSysUnlockFromISR();
}

void adis16405_thread(void *arg) {
    (void)arg;
    const SPIConfig spi_cfg = {
        NULL,
        ADIS16405_SPI_CS_PORT,
        ADIS16405_SPI_CS_PIN,
        // CPOL, CPHA, MSB First, 16-bit frame
        // Clock rate should be <= 1 MHz for burst mode
        // I believe this sets it to 168000000 / 4 / 64 ~= 1MHz
        // TODO: Verify this
        SPI_CR1_BR_2 | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF
    };

    chBSemObjectInit(&adis16405_semaphore, true);

    chRegSetThreadName("ADIS16405");

    spiStart(&ADIS16405_SPID, &spi_cfg);

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

    adis16405_init();

    while (!adis16405_self_test()) {
        chThdSleepMilliseconds(100);
    }

    errors = adis16405_read_u16(0x3C);
    if (errors != 0) {
        COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_error);
        component_state_update(avionics_component_adis16405, state_error, errors);
        return;
    }

    COMPONENT_STATE_UPDATE(avionics_component_adis16405, state_ok);



    int16_t raw_data[12];
    float gyro[3],accel[3],magno[3], temperature;

    while(TRUE) {
        chSysLock();
        chBSemWaitTimeoutS(&adis16405_semaphore, 100);
        chSysUnlock();

        if (!adis16405_burst_read(raw_data))
            continue;

        // TODO check these
        // Factor on data sheet 3.33mg
        // Factor of 0.0125 is for when operating with 75 degrees/sec
        // Factor of 0.05 is for when operating with 300 degrees/sec
        for (int i = 0;i<3;i++)
            gyro[i] = 0.05f*PI/180.0f*(raw_data[i+1]);

        // Acceleration scale is 3.33,measured in mg
        for (int i = 0;i<3;i++)
            accel[i] = 3.33f/1000.0f*9.8f*(raw_data[i+4]);

        // Magno data in mgauss
        for (int i = 0;i<3;i++)
            magno[i] = 0.5f*(raw_data[i+7]);

        // Temperature scale 0.14 degrees, temp is measured in degrees
        temperature = 0.14f*raw_data[10];

        // TODO: Send off sensor data
    }
}
