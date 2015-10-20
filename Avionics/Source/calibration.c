#include "calibration.h"
#include <math.h>

#define ARDUIMU
//#define M2FC

#ifdef ARDUIMU

#define ACCEL_MXX 0.001186432230011
#define ACCEL_MXY -0.000000721270558
#define ACCEL_MXZ 0.000000006107108
#define ACCEL_MYY 0.001165610355608
#define ACCEL_MYZ -0.000000103273444
#define ACCEL_MZZ 0.001179905173114
#define ACCEL_OX -225.4809321801666
#define ACCEL_OY -195.4929288841663
#define ACCEL_OZ 1.5169317043500

#define MAG_MXX 0.001635422252442f
#define MAG_MXY 0.000037980055044f
#define MAG_MXZ 0.000095878830514f
#define MAG_MYY 0.001773860895132f
#define MAG_MYZ 0.000020643759832f
#define MAG_MZZ 0.001701760302961f
#define MAG_OX 15.0728348266064f
#define MAG_OY -24.2148733051456f
#define MAG_OZ 106.4663323513927f

// Gyro has range of 2000 degrees per second
// TODO: Calibrate this properly
#define GYRO_SCALE 2000.0f*0.01745329251f/INT16_MAX

#elif defined(M2FC)

// Very rough calibration - don't have board to actually calibrate properly
#define ACCEL_MXX 0.0386745557f
#define ACCEL_MXY 0
#define ACCEL_MXZ 0
#define ACCEL_MYY 0.0386745557f
#define ACCEL_MYZ 0
#define ACCEL_MZZ 0.0386745557f
#define ACCEL_OX 0 
#define ACCEL_OY 0
#define ACCEL_OZ 0

#define MAG_MXX 0
#define MAG_MXY 0
#define MAG_MXZ 0
#define MAG_MYY 0
#define MAG_MYZ 0
#define MAG_MZZ 0
#define MAG_OX 0
#define MAG_OY 0
#define MAG_OZ 0

#endif

void calibrate_accel(const int16_t in[3], float out[3]) {
	float x = (float)in[0] - ACCEL_OX;
	float y = (float)in[1] - ACCEL_OY;
	float z = (float)in[2] - ACCEL_OZ;
	
	out[0] = ACCEL_MXX*x + ACCEL_MXY*y + ACCEL_MXZ*z;
	out[1] = ACCEL_MXY*x + ACCEL_MYY*y + ACCEL_MYZ*z;
	out[2] = ACCEL_MXZ*x + ACCEL_MYZ*y + ACCEL_MZZ*z;
}

void calibrate_mag(const int16_t in[3], float out[3]) {
	float x = (float)in[0] - MAG_OX;
	float y = (float)in[1] - MAG_OY;
	float z = (float)in[2] - MAG_OZ;

	out[0] = MAG_MXX*x + MAG_MXY*y + MAG_MXZ*z;
	out[1] = MAG_MXY*x + MAG_MYY*y + MAG_MYZ*z;
	out[2] = MAG_MXZ*x + MAG_MYZ*y + MAG_MZZ*z;
}

void calibrate_gyro(const int16_t in[3], float out[3]) {
	out[0] = GYRO_SCALE*in[0];
	out[1] = GYRO_SCALE*in[1];
	out[2] = GYRO_SCALE*in[2];
}


/* Constants from the US Standard Atmosphere 1976 */
const float Rs = 8.31432f;
const float g0 = 9.80665f;
const float M = 0.0289644f;
const float Lb[7] = { -0.0065f, 0.0f, 0.001f, 0.0028f, 0.0f, -0.0028f, -0.002f };
const float Pb[7] = { 101325.0f, 22632.10f, 5474.89f, 868.02f, 110.91f, 66.94f, 3.96f };
const float Tb[7] = { 288.15f, 216.65f, 216.65f, 228.65f, 270.65f, 270.65f, 214.65f };
const float Hb[7] = { 0.0f, 11000.0f, 20000.0f, 32000.0f, 47000.0f, 51000.0f, 71000.0f };
volatile uint8_t calibration_trust_barometer = 1;


/*
* Convert a pressure and an atmospheric level b into an altitude.
* Reverses the standard equation for non-zero lapse regions,
* P = Pb (Tb / (Tb + Lb(h - hb)))^(M g0 / R* Lb)
*/
float state_estimation_p2a_nonzero_lapse(float pressure, int b)
{
	float lb = Lb[b];
	float hb = Hb[b];
	float pb = Pb[b];
	float tb = Tb[b];
	return hb + tb / lb * (powf(pressure / pb, (-Rs*lb) / (g0*M)) - 1.0f);
}
/* Convert a pressure and an atmospheric level b into an altitude.
* Reverses the standard equation for zero-lapse regions,
* P = Pb exp( -g0 M (h-hb) / R* Tb)
*/
float state_estimation_p2a_zero_lapse(float pressure, int b)
{
	float hb = Hb[b];
	float pb = Pb[b];
	float tb = Tb[b];
	return hb + (Rs * tb) / (g0 * M) * (logf(pressure) - logf(pb));
}

float state_estimation_pressure_to_altitude(float pressure)
{
	int b;
	/* For each level of the US Standard Atmosphere 1976, check if the pressure
	* is inside that level, and use the appropriate conversion based on lapse
	* rate at that level.
	*/
	for (b = 0; b < 6; b++) {
		if (pressure <= Pb[b] && pressure > Pb[b + 1]) {
			if (Lb[b] == 0.0f) {
				return state_estimation_p2a_zero_lapse(pressure, b);
			}
			else {
				return state_estimation_p2a_nonzero_lapse(pressure, b);
			}
		}
	}
	/* If no levels matched, something is wrong, returning -9999f will cause
	* this pressure value to be ignored.
	*/
	return -9999.0f;
}