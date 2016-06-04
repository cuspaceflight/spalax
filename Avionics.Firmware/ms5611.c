/*
 * MS5611-01BA03 Driver
 * M2FC
 * 2014 Adam Greig, Cambridge University Spaceflight
 * 2015 Eivind Roson Eide, Cambridge University Spaceflight
 */


#include "ms5611.h"
#include "hal.h"
#include "chprintf.h"
#include "badthinghandler.h"
#include "messaging.h"


#define MS5611_SPID        SPID2
#define MS5611_SPI_CS_PORT GPIOB
#define MS5611_SPI_CS_PIN  GPIOB_ALT_NSS


static void ms5611_spi_start(void);
static void ms5611_spi_stop(void);
static void ms5611_spi_select(void);
static void ms5611_spi_unselect(void);
static void ms5611_reset(void);
static void ms5611_read_u16(uint8_t adr, uint16_t* c);
static void ms5611_read_s24(uint8_t adr, int32_t* d);
static void ms5611_init(MS5611CalData* cal_data);
static void ms5611_read_cal(MS5611CalData* cal_data);
static void ms5611_read(MS5611CalData* cal_data,
			int32_t* temperature, int32_t* pressure);

static const SPIConfig spi_cfg = {
	NULL,
	MS5611_SPI_CS_PORT,
	MS5611_SPI_CS_PIN,
	SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

/* Acquire mutex and initialise SPI with DMA. */
static void ms5611_spi_start(){
	spiStart(&MS5611_SPID, &spi_cfg);
}

/* Release DMA and mutex. */
static void ms5611_spi_stop(){
	spiStop(&MS5611_SPID);
}

/* Assert CS without needing to have the SPI peripheral ready */
static void ms5611_spi_select(){
	palClearPad(MS5611_SPID.config->ssport, MS5611_SPID.config->sspad);
}

/* Deassert CS without needing to have the SPI peripheral ready */
static void ms5611_spi_unselect(){
	palSetPad(MS5611_SPID.config->ssport, MS5611_SPID.config->sspad);
}

/*
 * Resets the MS5611. Sends 0x1E, waits 5ms.
 */
static void ms5611_reset(){
	uint8_t adr = 0x1E;
	ms5611_spi_start();
	ms5611_spi_select();
	spiSend(&MS5611_SPID, 1, (void*)&adr);
	ms5611_spi_stop();
	chThdSleepMilliseconds(5);
	ms5611_spi_unselect();
}

/*
 * Reads a uint16 from the MS5611 address `adr`, stores it to `c`.
 */
static void ms5611_read_u16(uint8_t adr, uint16_t* c){
	uint8_t rx[2];
	ms5611_spi_start();
	ms5611_spi_select();
	spiSend(&MS5611_SPID, 1, (void*)&adr);
	spiReceive(&MS5611_SPID, 2, (void*)rx);
	ms5611_spi_unselect();
	ms5611_spi_stop();

	*c = rx[0] << 8 | rx[1];
}

/*
 * Reads an int24 from the MS5611 address `adr`, stores it to `d`.
 */
static void ms5611_read_s24(uint8_t adr, int32_t* d){
	uint8_t adc_adr = 0x00, rx[3];

	/* Start conversion */
	ms5611_spi_start();
	ms5611_spi_select();
	spiSend(&MS5611_SPID, 1, (void*)&adr);
	ms5611_spi_stop();

	/*
	 * Wait for conversion to complete. There doesn't appear to be any way
	 * to do this without timing it, unfortunately.
	 * This means we also lose out on rate - ideally we'd only pause for 0.6ms
	 * instead of this vaguely 1-2ms.
	 */
	chThdSleepMilliseconds(2);

	/* Deassert CS */
	ms5611_spi_unselect();

	/* Read ADC result */
	ms5611_spi_start();
	ms5611_spi_select();
	spiSend(&MS5611_SPID, 1, (void*)&adc_adr);
	spiReceive(&MS5611_SPID, 3, (void*)rx);
	ms5611_spi_unselect();
	ms5611_spi_stop();

	*d = rx[0] << 16 | rx[1] << 8 | rx[2];
}

/*
 * Reads MS5611 calibration data, writes it to `cal_data`.
 */
static void ms5611_read_cal(MS5611CalData* cal_data){
	uint16_t d0, d7;
	ms5611_read_u16(0xA0, &d0);
	ms5611_read_u16(0xA2, &(cal_data->c1));
	ms5611_read_u16(0xA4, &(cal_data->c2));
	ms5611_read_u16(0xA6, &(cal_data->c3));
	ms5611_read_u16(0xA8, &(cal_data->c4));
	ms5611_read_u16(0xAA, &(cal_data->c5));
	ms5611_read_u16(0xAC, &(cal_data->c6));
	ms5611_read_u16(0xAE, &d7);

	//log_u16(M2T_CH_CAL_BARO_1, d0, cal_data->c1, cal_data->c2, cal_data->c3);
	//log_u16(M2T_CH_CAL_BARO_2, cal_data->c4, cal_data->c5, cal_data->c6, d7);
}

/*
 * Initialise the MS5611.
 * Sends a RESET and then reads in the calibration data.
 *
 * Call this once system startup, before attempting ms5611_read.
 *
 * cal_data should be a pointer to some memory to store the calibration in.
 */
static void ms5611_init(MS5611CalData* cal_data){
	ms5611_reset();
	ms5611_read_cal(cal_data);
}

/*
 * Read and compensate a temperature and pressure from the MS5611.
 *
 * `cal_data` is previously read calibration data.
 * `temperature` and `pressure` are written to.
 *
 * `temperature` is in centidegrees Celcius,
 * `pressure` is in Pascals.
 */
static void ms5611_read(MS5611CalData* cal_data,
			int32_t* temperature, int32_t* pressure){
	int32_t d1, d2;
	int64_t off, sens, dt;
	int64_t t2 = 0, sens2 = 0, off2 = 0;
	ms5611_read_s24(0x40, &d1);
	ms5611_read_s24(0x50, &d2);

	/* Fetch and compute temperature */
	dt = (int64_t)d2 - ((int64_t)cal_data->c5 << 8);
	*temperature = 2000 + ((dt * (int64_t)cal_data->c6) >> 23);

	/* Compute offset and sensitivity */
	off = ((int64_t)cal_data->c2 << 16) + (((int64_t)cal_data->c4 * dt) >> 7);
	sens = ((int64_t)cal_data->c1 << 15) + (((int64_t)cal_data->c3 * dt) >> 8);

	/* Perform low temperature compensation */
	if (*temperature < 2000) {
		t2 = (dt * dt) >> 31;
		off2 = 5 * (*temperature - 2000) * (*temperature - 2000) >> 1;
		sens2 = off2 >> 1;
		if (*temperature < -1500) {
			off2 += 7 * (*temperature + 1500) * (*temperature + 1500);
			sens2 += 11 * (*temperature + 1500) * (*temperature + 1500) >> 1;
		}
		*temperature -= t2;
		off -= off2;
		sens -= sens2;
	}

	/* Compute and store new pressure and temperature */
	*pressure = (((d1 * sens) >> 21) - off) >> 15;
	//log_i32(M2T_CH_IMU_BARO, *pressure, *temperature);
	//m2status_set_baro(*pressure, *temperature);

	if (*pressure < 1000 || *pressure > 120000)
		COMPONENT_STATE_UPDATE(avionics_component_ms5611, state_error);
}

MESSAGING_PRODUCER(messaging_producer, telemetry_id_ms5611_data_raw, (sizeof(telemetry_header_t) + sizeof(ms5611data_t)) * 10)

/*
 * MS5611 main thread.
 * Resets the MS5611, reads cal data, then reads a pressure and temperature
 * in a loop.
 */
msg_t ms5611_thread(void *arg){
	(void)arg;
    chRegSetThreadName("MS5611");
	static MS5611CalData cal_data;
	ms5611data_t data;

	COMPONENT_STATE_UPDATE(avionics_component_ms5611, state_initializing);

	ms5611_init(&cal_data);
    messaging_producer_init(&messaging_producer);

	COMPONENT_STATE_UPDATE(avionics_component_ms5611, state_ok);

	while (TRUE) {
		ms5611_read(&cal_data, &data.temperature, &data.pressure);
        messaging_producer_send(&messaging_producer, 0, (const uint8_t*)&data, sizeof(data));
        chThdSleepMilliseconds(20);
	}
}
