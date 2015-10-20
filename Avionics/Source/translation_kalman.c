/*
* Translational State estimation and sensor fusion
* Avionics14
* 2014 Raphael Taylor-Davies, Cambridge University Spaceflight
*
* Pressure calibration, next state covariance matrix
* calculation and tuning constants taken from Martlet 2
* Written by Adam Greig, Cambridge University Spaceflight
*
*/

#include "translation_kalman.h"
#include <calibration.h>

// Current state - [x,y,z],[position,velocity,acceleration]
static float current_state[3][3];

//Covariance storage - [x,y,z],[row],[column]
// Initialise to 0 - we know the initial state with certainty
static float current_covariance[3][3][3] = {
		{ { 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f } },

		{ { 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f } },

		{ { 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f } }
};

/*
*
* Predicted State generated using SUVAT
*
* Next covariance matrix generated using method copied from Marlet 2
* Written by Adam Greig
*/
void translation_kalman_prediction_step(state_estimate_t* state, float dt) {
	//This will be computed in firmware but set at 1 ms for now for simplicity
	//float dt = 1;
	float dt2 = dt * dt;
	float dt3 = dt * dt2;
	float dt4 = dt * dt3;
	float dt5 = dt * dt4;
	float dt6 = dt * dt5;
	float dt2_2 = dt2 / 2.0f;

	/* TODO Determine this q-value */
	float q = 500.0;

	int i;
	for (i = 0; i < 3; i++) {
		float* x = current_state[i];
		float(*p)[3] = current_covariance[i];

		x[0] += dt * x[1] + dt2_2*x[2];
		x[1] += dt * x[2];

		/* Update covariance
		* P_{k|k-1} = F_k P_{k-1|k-1} F'_k + Q
		* Uses F.P.F' from above. We'll add Q later, this is just the FPF'.
		* Conveniently the form means we can update each element in-place.
		*/
		p[0][0] += (dt * p[1][0] + dt2_2 * p[2][0]
			+ dt * (p[0][1] + dt * p[1][1] * dt2_2 * p[2][1])
			+ dt2_2 * (p[0][2] + dt * p[1][2] + dt2_2 * p[2][2]));
		p[0][1] += (dt * p[1][1] + dt2_2 * p[2][1]
			+ dt * (p[0][2] + dt * p[1][2] + dt2_2 * p[2][2]));
		p[0][2] += (dt * p[1][2] + dt2_2 * p[2][2]);
		p[1][0] += (dt * p[2][0]
			+ dt * (p[1][1] + dt * p[2][1])
			+ dt2_2 * (p[1][2] + dt * p[2][2]));
		p[1][1] += (dt * p[2][1]
			+ dt * (p[1][2] + dt * p[2][2]));
		p[1][2] += (dt * p[2][2]);
		p[2][0] += (dt * p[2][1] + dt2_2 * p[2][2]);
		p[2][1] += (dt * p[2][2]);
		/* Add process noise to matrix above.
		* P_{k|k-1} += Q
		*/
		p[0][0] += q * dt6 / 36.0f;
		p[0][1] += q * dt5 / 12.0f;
		p[0][2] += q * dt4 / 6.0f;
		p[1][0] += q * dt5 / 12.0f;
		p[1][1] += q * dt4 / 4.0f;
		p[1][2] += q * dt3 / 2.0f;
		p[2][0] += q * dt4 / 6.0f;
		p[2][1] += q * dt3 / 2.0f;
		p[2][2] += q * dt2 / 1.0f;

		state->pos[i] = x[0];
		state->vel[i] = x[1];
		state->accel[i] = x[2];
	}
}


/*
*
* Pressure Update Equations
*
* Pressure only effects y axis so extract this first
*
* x = current_state[1]
*
* As the pressure reading gives us altitude H = [1, 0, 0]
*
* The covariance matrix P is a 3x3 matrix
*
* The residual covariance S is therefore [P[0][0] + R]
*
* Therefore K is the transpose of [P[0][0]/S,P[1][0]/S,P[2][0]/S]
*
* Working through the matrix multiplications:
*
*       [k[0] * P[0][0], k[0] * P[0][1], k[0] * P[0][2]]
* P -=  [k[1] * P[0][0], k[1] * P[0][1], k[1] * P[0][2]]
*       [k[2] * P[0][0], k[2] * P[0][1], k[2] * P[0][2]]
*
*       [k[0] * (Z - x[0])]
* x +=  [k[1] * (Z - x[0])]
*       [k[2] * (Z - x[0])]
*/

void translation_kalman_new_pressure_raw(float pressure) {
	/* Around 6.5Pa resolution on the barometer. */
	float baro_res = 6.5f;
	float h, hd;
	/* Discard data when mission control believes we are transonic. */
	if (!calibration_trust_barometer)
		return;
	/* Convert pressure reading into an altitude.
	* Run the same conversion for pressure + sensor resolution to get an idea
	* of the current noise variance in altitude terms for the filter.
	*/
	h = state_estimation_pressure_to_altitude(pressure);
	hd = state_estimation_pressure_to_altitude(pressure + baro_res);
	/* If there was an error (couldn't find suitable altitude band) for this
	* pressure, just don't use it. It's probably wrong. */
	if (h == -9999.0f || hd == -9999.0f)
		return;
	/* TODO: validate choice of r */
	float r = (h - hd) * (h - hd);

	float* x = current_state[1];
	float (*P)[3] = current_covariance[1];


	float s_inverse = 1 / (P[0][0] + r);

	float k[3];
	k[0] = P[0][0] * s_inverse;
	k[1] = P[1][0] * s_inverse;
	k[2] = P[2][0] * s_inverse;

	P[0][0] -= k[0] * P[0][0];
	P[0][1] -= k[0] * P[0][1];
	P[0][2] -= k[0] * P[0][2];
	P[1][0] -= k[1] * P[0][0];
	P[1][1] -= k[1] * P[0][1];
	P[1][2] -= k[1] * P[0][2];
	P[2][0] -= k[2] * P[0][0];
	P[2][1] -= k[2] * P[0][1];
	P[2][2] -= k[2] * P[0][2];

	float error = h - x[0];

	x[0] += k[0] * error;
	x[1] += k[1] * error;
	x[2] += k[2] * error;
}


/*
*
* Accelerometer Update Equations
*
* Each axis is independent so we handle them separately
*
* x = current_state[i]
* Z = accel[i]
* for i = 0,1,2
*
* As the accel reading gives us acceleration H = [0, 0, 1]
*
* The covariance matrix P is a 3x3 matrix
*
* The residual covariance S is therefore [P[2][2] + R]
*
* Therefore K is the transpose of [P[0][2]/S,P[1][2]/S,P[2][2]/S]
*
* Working through the matrix multiplications:
*
*       [k[0] * P[2][0], k[0] * P[2][1], k[0] * P[2][2]]
* P -=  [k[1] * P[2][0], k[1] * P[2][1], k[1] * P[2][2]]
*       [k[2] * P[2][0], k[2] * P[2][1], k[2] * P[2][2]]
*
*       [k[0] * (Z - x[0])]
* x +=  [k[1] * (Z - x[0])]
*       [k[2] * (Z - x[0])]
*/

void translation_kalman_new_accel(const float* accel) {
	const float r = 5.8229525e-07f;//0.2365f; //low-g value (high-g value is 7.6951f)

	int i;
	for (i = 0; i < 3; i++) {
		float* x = current_state[i];
		float(*P)[3] = current_covariance[i];

		float s_inverse = 1 / (P[2][2] + r);

		float k[3];
		k[0] = P[0][2] * s_inverse;
		k[1] = P[1][2] * s_inverse;
		k[2] = P[2][2] * s_inverse;

		P[0][0] -= k[0] * P[2][0];
		P[0][1] -= k[0] * P[2][1];
		P[0][2] -= k[0] * P[2][2];
		P[1][0] -= k[1] * P[2][0];
		P[1][1] -= k[1] * P[2][1];
		P[1][2] -= k[1] * P[2][2];
		P[2][0] -= k[2] * P[2][0];
		P[2][1] -= k[2] * P[2][1];
		P[2][2] -= k[2] * P[2][2];

		float error = accel[i] - x[2];

		x[0] += k[0] * error;
		x[1] += k[1] * error;
		x[2] += k[2] * error;
	}
}
