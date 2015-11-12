#include "orientation_kalman.h"
#include "math_utils.h"
#include "logging.h"

// The current state - {Angular Velocity(3), Quaternion(scalar,vector)}
float x[7] = { 0, 0, 0, 1, 0, 0, 0 };

// Covariance Storage
float P[7][7] = { { 0, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 0 } };

int checkForNan() {
	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 7; j++)
			if (P[i][j] != P[i][j])
				return 1;

	for (int i = 0; i < 7; i++)
		if (x[i] != x[i])
			return 1;

	return 0;
}


// Gyro Update
//
//	   [1 0 0 0 0 0 0];
// H = [0 1 0 0 0 0 0];
//	   [0 0 1 0 0 0 0];
//
void orientation_kalman_new_gyro(const float gyro[3]) {
	//PRINT("Gyro %f %f %f\n", gyro[0], gyro[1], gyro[2]);

	// TODO: Choose an appropriate value for this
	const float rk = 0.001f;

	// Temporary copy of p
	float pt[7][7];

	// (H * P * H' + Rk)
	float hphrk[3][3];
	float hphrk_inv[3][3];

	for (int i = 0; i < 7; i++) {
		for (int j = 0; j < 7; j++) {
			pt[i][j] = P[i][j];
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			hphrk[i][j] = pt[i][j];
		}
		hphrk[i][i] += rk;
	}

	mat3x3_inv(hphrk, hphrk_inv);

	// Kalman Gain
	float k[7][3];
	k[0][0] = hphrk_inv[0][0] * pt[0][0] + hphrk_inv[1][0] * pt[0][1] + hphrk_inv[2][0] * pt[0][2];
	k[0][1] = hphrk_inv[0][1] * pt[0][0] + hphrk_inv[1][1] * pt[0][1] + hphrk_inv[2][1] * pt[0][2];
	k[0][2] = hphrk_inv[0][2] * pt[0][0] + hphrk_inv[1][2] * pt[0][1] + hphrk_inv[2][2] * pt[0][2];
	k[1][0] = hphrk_inv[0][0] * pt[1][0] + hphrk_inv[1][0] * pt[1][1] + hphrk_inv[2][0] * pt[1][2];
	k[1][1] = hphrk_inv[0][1] * pt[1][0] + hphrk_inv[1][1] * pt[1][1] + hphrk_inv[2][1] * pt[1][2];
	k[1][2] = hphrk_inv[0][2] * pt[1][0] + hphrk_inv[1][2] * pt[1][1] + hphrk_inv[2][2] * pt[1][2];
	k[2][0] = hphrk_inv[0][0] * pt[2][0] + hphrk_inv[1][0] * pt[2][1] + hphrk_inv[2][0] * pt[2][2];
	k[2][1] = hphrk_inv[0][1] * pt[2][0] + hphrk_inv[1][1] * pt[2][1] + hphrk_inv[2][1] * pt[2][2];
	k[2][2] = hphrk_inv[0][2] * pt[2][0] + hphrk_inv[1][2] * pt[2][1] + hphrk_inv[2][2] * pt[2][2];
	k[3][0] = hphrk_inv[0][0] * pt[3][0] + hphrk_inv[1][0] * pt[3][1] + hphrk_inv[2][0] * pt[3][2];
	k[3][1] = hphrk_inv[0][1] * pt[3][0] + hphrk_inv[1][1] * pt[3][1] + hphrk_inv[2][1] * pt[3][2];
	k[3][2] = hphrk_inv[0][2] * pt[3][0] + hphrk_inv[1][2] * pt[3][1] + hphrk_inv[2][2] * pt[3][2];
	k[4][0] = hphrk_inv[0][0] * pt[4][0] + hphrk_inv[1][0] * pt[4][1] + hphrk_inv[2][0] * pt[4][2];
	k[4][1] = hphrk_inv[0][1] * pt[4][0] + hphrk_inv[1][1] * pt[4][1] + hphrk_inv[2][1] * pt[4][2];
	k[4][2] = hphrk_inv[0][2] * pt[4][0] + hphrk_inv[1][2] * pt[4][1] + hphrk_inv[2][2] * pt[4][2];
	k[5][0] = hphrk_inv[0][0] * pt[5][0] + hphrk_inv[1][0] * pt[5][1] + hphrk_inv[2][0] * pt[5][2];
	k[5][1] = hphrk_inv[0][1] * pt[5][0] + hphrk_inv[1][1] * pt[5][1] + hphrk_inv[2][1] * pt[5][2];
	k[5][2] = hphrk_inv[0][2] * pt[5][0] + hphrk_inv[1][2] * pt[5][1] + hphrk_inv[2][2] * pt[5][2];
	k[6][0] = hphrk_inv[0][0] * pt[6][0] + hphrk_inv[1][0] * pt[6][1] + hphrk_inv[2][0] * pt[6][2];
	k[6][1] = hphrk_inv[0][1] * pt[6][0] + hphrk_inv[1][1] * pt[6][1] + hphrk_inv[2][1] * pt[6][2];
	k[6][2] = hphrk_inv[0][2] * pt[6][0] + hphrk_inv[1][2] * pt[6][1] + hphrk_inv[2][2] * pt[6][2];

	// Update covariance

	P[0][0] = -pt[0][0] * (k[0][0] - 1) - k[0][1] * pt[1][0] - k[0][2] * pt[2][0];
	P[0][1] = -pt[0][1] * (k[0][0] - 1) - k[0][1] * pt[1][1] - k[0][2] * pt[2][1];
	P[0][2] = -pt[0][2] * (k[0][0] - 1) - k[0][1] * pt[1][2] - k[0][2] * pt[2][2];
	P[0][3] = -pt[0][3] * (k[0][0] - 1) - k[0][1] * pt[1][3] - k[0][2] * pt[2][3];
	P[0][4] = -pt[0][4] * (k[0][0] - 1) - k[0][1] * pt[1][4] - k[0][2] * pt[2][4];
	P[0][5] = -pt[0][5] * (k[0][0] - 1) - k[0][1] * pt[1][5] - k[0][2] * pt[2][5];
	P[0][6] = -pt[0][6] * (k[0][0] - 1) - k[0][1] * pt[1][6] - k[0][2] * pt[2][6];
	P[1][0] = -pt[1][0] * (k[1][1] - 1) - k[1][0] * pt[0][0] - k[1][2] * pt[2][0];
	P[1][1] = -pt[1][1] * (k[1][1] - 1) - k[1][0] * pt[0][1] - k[1][2] * pt[2][1];
	P[1][2] = -pt[1][2] * (k[1][1] - 1) - k[1][0] * pt[0][2] - k[1][2] * pt[2][2];
	P[1][3] = -pt[1][3] * (k[1][1] - 1) - k[1][0] * pt[0][3] - k[1][2] * pt[2][3];
	P[1][4] = -pt[1][4] * (k[1][1] - 1) - k[1][0] * pt[0][4] - k[1][2] * pt[2][4];
	P[1][5] = -pt[1][5] * (k[1][1] - 1) - k[1][0] * pt[0][5] - k[1][2] * pt[2][5];
	P[1][6] = -pt[1][6] * (k[1][1] - 1) - k[1][0] * pt[0][6] - k[1][2] * pt[2][6];
	P[2][0] = -pt[2][0] * (k[2][2] - 1) - k[2][0] * pt[0][0] - k[2][1] * pt[1][0];
	P[2][1] = -pt[2][1] * (k[2][2] - 1) - k[2][0] * pt[0][1] - k[2][1] * pt[1][1];
	P[2][2] = -pt[2][2] * (k[2][2] - 1) - k[2][0] * pt[0][2] - k[2][1] * pt[1][2];
	P[2][3] = -pt[2][3] * (k[2][2] - 1) - k[2][0] * pt[0][3] - k[2][1] * pt[1][3];
	P[2][4] = -pt[2][4] * (k[2][2] - 1) - k[2][0] * pt[0][4] - k[2][1] * pt[1][4];
	P[2][5] = -pt[2][5] * (k[2][2] - 1) - k[2][0] * pt[0][5] - k[2][1] * pt[1][5];
	P[2][6] = -pt[2][6] * (k[2][2] - 1) - k[2][0] * pt[0][6] - k[2][1] * pt[1][6];
	P[3][0] = pt[3][0] - k[3][0] * pt[0][0] - k[3][1] * pt[1][0] - k[3][2] * pt[2][0];
	P[3][1] = pt[3][1] - k[3][0] * pt[0][1] - k[3][1] * pt[1][1] - k[3][2] * pt[2][1];
	P[3][2] = pt[3][2] - k[3][0] * pt[0][2] - k[3][1] * pt[1][2] - k[3][2] * pt[2][2];
	P[3][3] = pt[3][3] - k[3][0] * pt[0][3] - k[3][1] * pt[1][3] - k[3][2] * pt[2][3];
	P[3][4] = pt[3][4] - k[3][0] * pt[0][4] - k[3][1] * pt[1][4] - k[3][2] * pt[2][4];
	P[3][5] = pt[3][5] - k[3][0] * pt[0][5] - k[3][1] * pt[1][5] - k[3][2] * pt[2][5];
	P[3][6] = pt[3][6] - k[3][0] * pt[0][6] - k[3][1] * pt[1][6] - k[3][2] * pt[2][6];
	P[4][0] = pt[4][0] - k[4][0] * pt[0][0] - k[4][1] * pt[1][0] - k[4][2] * pt[2][0];
	P[4][1] = pt[4][1] - k[4][0] * pt[0][1] - k[4][1] * pt[1][1] - k[4][2] * pt[2][1];
	P[4][2] = pt[4][2] - k[4][0] * pt[0][2] - k[4][1] * pt[1][2] - k[4][2] * pt[2][2];
	P[4][3] = pt[4][3] - k[4][0] * pt[0][3] - k[4][1] * pt[1][3] - k[4][2] * pt[2][3];
	P[4][4] = pt[4][4] - k[4][0] * pt[0][4] - k[4][1] * pt[1][4] - k[4][2] * pt[2][4];
	P[4][5] = pt[4][5] - k[4][0] * pt[0][5] - k[4][1] * pt[1][5] - k[4][2] * pt[2][5];
	P[4][6] = pt[4][6] - k[4][0] * pt[0][6] - k[4][1] * pt[1][6] - k[4][2] * pt[2][6];
	P[5][0] = pt[5][0] - k[5][0] * pt[0][0] - k[5][1] * pt[1][0] - k[5][2] * pt[2][0];
	P[5][1] = pt[5][1] - k[5][0] * pt[0][1] - k[5][1] * pt[1][1] - k[5][2] * pt[2][1];
	P[5][2] = pt[5][2] - k[5][0] * pt[0][2] - k[5][1] * pt[1][2] - k[5][2] * pt[2][2];
	P[5][3] = pt[5][3] - k[5][0] * pt[0][3] - k[5][1] * pt[1][3] - k[5][2] * pt[2][3];
	P[5][4] = pt[5][4] - k[5][0] * pt[0][4] - k[5][1] * pt[1][4] - k[5][2] * pt[2][4];
	P[5][5] = pt[5][5] - k[5][0] * pt[0][5] - k[5][1] * pt[1][5] - k[5][2] * pt[2][5];
	P[5][6] = pt[5][6] - k[5][0] * pt[0][6] - k[5][1] * pt[1][6] - k[5][2] * pt[2][6];
	P[6][0] = pt[6][0] - k[6][0] * pt[0][0] - k[6][1] * pt[1][0] - k[6][2] * pt[2][0];
	P[6][1] = pt[6][1] - k[6][0] * pt[0][1] - k[6][1] * pt[1][1] - k[6][2] * pt[2][1];
	P[6][2] = pt[6][2] - k[6][0] * pt[0][2] - k[6][1] * pt[1][2] - k[6][2] * pt[2][2];
	P[6][3] = pt[6][3] - k[6][0] * pt[0][3] - k[6][1] * pt[1][3] - k[6][2] * pt[2][3];
	P[6][4] = pt[6][4] - k[6][0] * pt[0][4] - k[6][1] * pt[1][4] - k[6][2] * pt[2][4];
	P[6][5] = pt[6][5] - k[6][0] * pt[0][5] - k[6][1] * pt[1][5] - k[6][2] * pt[2][5];
	P[6][6] = pt[6][6] - k[6][0] * pt[0][6] - k[6][1] * pt[1][6] - k[6][2] * pt[2][6];

	float x0 = x[0];
	float x1 = x[1];
	float x2 = x[2];

	// Update x
	x[0] += -k[0][0] * (x0 - gyro[0]) - k[0][1] * (x1 - gyro[1]) - k[0][2] * (x2 - gyro[2]);
	x[1] += -k[1][0] * (x0 - gyro[0]) - k[1][1] * (x1 - gyro[1]) - k[1][2] * (x2 - gyro[2]);
	x[2] += -k[2][0] * (x0 - gyro[0]) - k[2][1] * (x1 - gyro[1]) - k[2][2] * (x2 - gyro[2]);
	x[3] += -k[3][0] * (x0 - gyro[0]) - k[3][1] * (x1 - gyro[1]) - k[3][2] * (x2 - gyro[2]);
	x[4] += -k[4][0] * (x0 - gyro[0]) - k[4][1] * (x1 - gyro[1]) - k[4][2] * (x2 - gyro[2]);
	x[5] += -k[5][0] * (x0 - gyro[0]) - k[5][1] * (x1 - gyro[1]) - k[5][2] * (x2 - gyro[2]);
	x[6] += -k[6][0] * (x0 - gyro[0]) - k[6][1] * (x1 - gyro[1]) - k[6][2] * (x2 - gyro[2]);

	//if (checkForNan())
	//	PRINT("Gyro Update Nan");
}

void orientation_kalman_new_quaternion_wx_update(const float quat[4]) {
	const float Rk = 0.001f;
	float pt[7][7];
	float k[7][2];
	
	for (int i = 0; i < 7; i++) {
		for (int j = 0; j < 7; j++) {
			pt[i][j] = P[i][j];
		}
	}

	float det = Rk*Rk + pt[3][3] * pt[4][4] - pt[3][4] * pt[4][3] + Rk*pt[3][3] + Rk*pt[4][4];
	k[0][0] = (pt[0][3] * pt[4][4] - pt[0][4] * pt[4][3] + Rk*pt[0][3]) / det;
	k[0][1] = (pt[0][4] * pt[3][3] - pt[0][3] * pt[3][4] + Rk*pt[0][4]) / det;
	k[1][0] = (pt[1][3] * pt[4][4] - pt[1][4] * pt[4][3] + Rk*pt[1][3]) / det;
	k[1][1] = (pt[1][4] * pt[3][3] - pt[1][3] * pt[3][4] + Rk*pt[1][4]) / det;
	k[2][0] = (pt[2][3] * pt[4][4] - pt[2][4] * pt[4][3] + Rk*pt[2][3]) / det;
	k[2][1] = (pt[2][4] * pt[3][3] - pt[2][3] * pt[3][4] + Rk*pt[2][4]) / det;
	k[3][0] = (pt[3][3] * pt[4][4] - pt[3][4] * pt[4][3] + Rk*pt[3][3]) / det;
	k[3][1] = (Rk*pt[3][4]) / det;
	k[4][0] = (Rk*pt[4][3]) / det;
	k[4][1] = (pt[3][3] * pt[4][4] - pt[3][4] * pt[4][3] + Rk*pt[4][4]) / det;
	k[5][0] = (pt[4][4] * pt[5][3] - pt[4][3] * pt[5][4] + Rk*pt[5][3]) / det;
	k[5][1] = (pt[3][3] * pt[5][4] - pt[3][4] * pt[5][3] + Rk*pt[5][4]) / det;
	k[6][0] = (pt[4][4] * pt[6][3] - pt[4][3] * pt[6][4] + Rk*pt[6][3]) / det;
	k[6][1] = (pt[3][3] * pt[6][4] - pt[3][4] * pt[6][3] + Rk*pt[6][4]) / det;

	P[0][0] = pt[0][0] - k[0][0] * pt[3][0] - k[0][1] * pt[4][0];
	P[0][1] = pt[0][1] - k[0][0] * pt[3][1] - k[0][1] * pt[4][1];
	P[0][2] = pt[0][2] - k[0][0] * pt[3][2] - k[0][1] * pt[4][2];
	P[0][3] = pt[0][3] - k[0][0] * pt[3][3] - k[0][1] * pt[4][3];
	P[0][4] = pt[0][4] - k[0][0] * pt[3][4] - k[0][1] * pt[4][4];
	P[0][5] = pt[0][5] - k[0][0] * pt[3][5] - k[0][1] * pt[4][5];
	P[0][6] = pt[0][6] - k[0][0] * pt[3][6] - k[0][1] * pt[4][6];
	P[1][0] = pt[1][0] - k[1][0] * pt[3][0] - k[1][1] * pt[4][0];
	P[1][1] = pt[1][1] - k[1][0] * pt[3][1] - k[1][1] * pt[4][1];
	P[1][2] = pt[1][2] - k[1][0] * pt[3][2] - k[1][1] * pt[4][2];
	P[1][3] = pt[1][3] - k[1][0] * pt[3][3] - k[1][1] * pt[4][3];
	P[1][4] = pt[1][4] - k[1][0] * pt[3][4] - k[1][1] * pt[4][4];
	P[1][5] = pt[1][5] - k[1][0] * pt[3][5] - k[1][1] * pt[4][5];
	P[1][6] = pt[1][6] - k[1][0] * pt[3][6] - k[1][1] * pt[4][6];
	P[2][0] = pt[2][0] - k[2][0] * pt[3][0] - k[2][1] * pt[4][0];
	P[2][1] = pt[2][1] - k[2][0] * pt[3][1] - k[2][1] * pt[4][1];
	P[2][2] = pt[2][2] - k[2][0] * pt[3][2] - k[2][1] * pt[4][2];
	P[2][3] = pt[2][3] - k[2][0] * pt[3][3] - k[2][1] * pt[4][3];
	P[2][4] = pt[2][4] - k[2][0] * pt[3][4] - k[2][1] * pt[4][4];
	P[2][5] = pt[2][5] - k[2][0] * pt[3][5] - k[2][1] * pt[4][5];
	P[2][6] = pt[2][6] - k[2][0] * pt[3][6] - k[2][1] * pt[4][6];
	P[3][0] = -pt[3][0] * (k[3][0] - 1) - k[3][1] * pt[4][0];
	P[3][1] = -pt[3][1] * (k[3][0] - 1) - k[3][1] * pt[4][1];
	P[3][2] = -pt[3][2] * (k[3][0] - 1) - k[3][1] * pt[4][2];
	P[3][3] = -pt[3][3] * (k[3][0] - 1) - k[3][1] * pt[4][3];
	P[3][4] = -pt[3][4] * (k[3][0] - 1) - k[3][1] * pt[4][4];
	P[3][5] = -pt[3][5] * (k[3][0] - 1) - k[3][1] * pt[4][5];
	P[3][6] = -pt[3][6] * (k[3][0] - 1) - k[3][1] * pt[4][6];
	P[4][0] = -pt[4][0] * (k[4][1] - 1) - k[4][0] * pt[3][0];
	P[4][1] = -pt[4][1] * (k[4][1] - 1) - k[4][0] * pt[3][1];
	P[4][2] = -pt[4][2] * (k[4][1] - 1) - k[4][0] * pt[3][2];
	P[4][3] = -pt[4][3] * (k[4][1] - 1) - k[4][0] * pt[3][3];
	P[4][4] = -pt[4][4] * (k[4][1] - 1) - k[4][0] * pt[3][4];
	P[4][5] = -pt[4][5] * (k[4][1] - 1) - k[4][0] * pt[3][5];
	P[4][6] = -pt[4][6] * (k[4][1] - 1) - k[4][0] * pt[3][6];
	P[5][0] = pt[5][0] - k[5][0] * pt[3][0] - k[5][1] * pt[4][0];
	P[5][1] = pt[5][1] - k[5][0] * pt[3][1] - k[5][1] * pt[4][1];
	P[5][2] = pt[5][2] - k[5][0] * pt[3][2] - k[5][1] * pt[4][2];
	P[5][3] = pt[5][3] - k[5][0] * pt[3][3] - k[5][1] * pt[4][3];
	P[5][4] = pt[5][4] - k[5][0] * pt[3][4] - k[5][1] * pt[4][4];
	P[5][5] = pt[5][5] - k[5][0] * pt[3][5] - k[5][1] * pt[4][5];
	P[5][6] = pt[5][6] - k[5][0] * pt[3][6] - k[5][1] * pt[4][6];
	P[6][0] = pt[6][0] - k[6][0] * pt[3][0] - k[6][1] * pt[4][0];
	P[6][1] = pt[6][1] - k[6][0] * pt[3][1] - k[6][1] * pt[4][1];
	P[6][2] = pt[6][2] - k[6][0] * pt[3][2] - k[6][1] * pt[4][2];
	P[6][3] = pt[6][3] - k[6][0] * pt[3][3] - k[6][1] * pt[4][3];
	P[6][4] = pt[6][4] - k[6][0] * pt[3][4] - k[6][1] * pt[4][4];
	P[6][5] = pt[6][5] - k[6][0] * pt[3][5] - k[6][1] * pt[4][5];
	P[6][6] = pt[6][6] - k[6][0] * pt[3][6] - k[6][1] * pt[4][6];

	// NB swap quaternion components from (vector,scalar) to (scalar,vector)
	float delta0 = quat[3] - x[3];
	float delta1 = quat[0] - x[4];
	
	x[0] += k[0][0] * delta0 + k[0][1] * delta1;
	x[1] += k[1][0] * delta0 + k[1][1] * delta1;
	x[2] += k[2][0] * delta0 + k[2][1] * delta1;
	x[3] += k[3][0] * delta0 + k[3][1] * delta1;
	x[4] += k[4][0] * delta0 + k[4][1] * delta1;
	x[5] += k[5][0] * delta0 + k[5][1] * delta1;
	x[6] += k[6][0] * delta0 + k[6][1] * delta1;
}

void orientation_kalman_new_quaternion_yz_update(const float quat[4]) {
	const float Rk = 0.001f;
	float pt[7][7];
	float k[7][2];

	for (int i = 0; i < 7; i++) {
		for (int j = 0; j < 7; j++) {
			pt[i][j] = P[i][j];
		}
	}

	float det = Rk *Rk + pt[5][5] * pt[6][6] - pt[5][6] * pt[6][5] + Rk*pt[5][5] + Rk*pt[6][6];
	k[0][0] = (pt[0][5] * pt[6][6] - pt[0][6] * pt[6][5] + Rk*pt[0][5]) / det;
	k[0][1] = (pt[0][6] * pt[5][5] - pt[0][5] * pt[5][6] + Rk*pt[0][6]) / det;
	k[1][0] = (pt[1][5] * pt[6][6] - pt[1][6] * pt[6][5] + Rk*pt[1][5]) / det;
	k[1][1] = (pt[1][6] * pt[5][5] - pt[1][5] * pt[5][6] + Rk*pt[1][6]) / det;
	k[2][0] = (pt[2][5] * pt[6][6] - pt[2][6] * pt[6][5] + Rk*pt[2][5]) / det;
	k[2][1] = (pt[2][6] * pt[5][5] - pt[2][5] * pt[5][6] + Rk*pt[2][6]) / det;
	k[3][0] = (pt[3][5] * pt[6][6] - pt[3][6] * pt[6][5] + Rk*pt[3][5]) / det;
	k[3][1] = (pt[3][6] * pt[5][5] - pt[3][5] * pt[5][6] + Rk*pt[3][6]) / det;
	k[4][0] = (pt[4][5] * pt[6][6] - pt[4][6] * pt[6][5] + Rk*pt[4][5]) / det;
	k[4][1] = (pt[4][6] * pt[5][5] - pt[4][5] * pt[5][6] + Rk*pt[4][6]) / det;
	k[5][0] = (pt[5][5] * pt[6][6] - pt[5][6] * pt[6][5] + Rk*pt[5][5]) / det;
	k[5][1] = (Rk*pt[5][6]) / det;
	k[6][0] = (Rk*pt[6][5]) / det;
	k[6][1] = (pt[5][5] * pt[6][6] - pt[5][6] * pt[6][5] + Rk*pt[6][6]) / det;

	P[0][0] = pt[0][0] - k[0][0] * pt[5][0] - k[0][1] * pt[6][0];
	P[0][1] = pt[0][1] - k[0][0] * pt[5][1] - k[0][1] * pt[6][1];
	P[0][2] = pt[0][2] - k[0][0] * pt[5][2] - k[0][1] * pt[6][2];
	P[0][3] = pt[0][3] - k[0][0] * pt[5][3] - k[0][1] * pt[6][3];
	P[0][4] = pt[0][4] - k[0][0] * pt[5][4] - k[0][1] * pt[6][4];
	P[0][5] = pt[0][5] - k[0][0] * pt[5][5] - k[0][1] * pt[6][5];
	P[0][6] = pt[0][6] - k[0][0] * pt[5][6] - k[0][1] * pt[6][6];
	P[1][0] = pt[1][0] - k[1][0] * pt[5][0] - k[1][1] * pt[6][0];
	P[1][1] = pt[1][1] - k[1][0] * pt[5][1] - k[1][1] * pt[6][1];
	P[1][2] = pt[1][2] - k[1][0] * pt[5][2] - k[1][1] * pt[6][2];
	P[1][3] = pt[1][3] - k[1][0] * pt[5][3] - k[1][1] * pt[6][3];
	P[1][4] = pt[1][4] - k[1][0] * pt[5][4] - k[1][1] * pt[6][4];
	P[1][5] = pt[1][5] - k[1][0] * pt[5][5] - k[1][1] * pt[6][5];
	P[1][6] = pt[1][6] - k[1][0] * pt[5][6] - k[1][1] * pt[6][6];
	P[2][0] = pt[2][0] - k[2][0] * pt[5][0] - k[2][1] * pt[6][0];
	P[2][1] = pt[2][1] - k[2][0] * pt[5][1] - k[2][1] * pt[6][1];
	P[2][2] = pt[2][2] - k[2][0] * pt[5][2] - k[2][1] * pt[6][2];
	P[2][3] = pt[2][3] - k[2][0] * pt[5][3] - k[2][1] * pt[6][3];
	P[2][4] = pt[2][4] - k[2][0] * pt[5][4] - k[2][1] * pt[6][4];
	P[2][5] = pt[2][5] - k[2][0] * pt[5][5] - k[2][1] * pt[6][5];
	P[2][6] = pt[2][6] - k[2][0] * pt[5][6] - k[2][1] * pt[6][6];
	P[3][0] = pt[3][0] - k[3][0] * pt[5][0] - k[3][1] * pt[6][0];
	P[3][1] = pt[3][1] - k[3][0] * pt[5][1] - k[3][1] * pt[6][1];
	P[3][2] = pt[3][2] - k[3][0] * pt[5][2] - k[3][1] * pt[6][2];
	P[3][3] = pt[3][3] - k[3][0] * pt[5][3] - k[3][1] * pt[6][3];
	P[3][4] = pt[3][4] - k[3][0] * pt[5][4] - k[3][1] * pt[6][4];
	P[3][5] = pt[3][5] - k[3][0] * pt[5][5] - k[3][1] * pt[6][5];
	P[3][6] = pt[3][6] - k[3][0] * pt[5][6] - k[3][1] * pt[6][6];
	P[4][0] = pt[4][0] - k[4][0] * pt[5][0] - k[4][1] * pt[6][0];
	P[4][1] = pt[4][1] - k[4][0] * pt[5][1] - k[4][1] * pt[6][1];
	P[4][2] = pt[4][2] - k[4][0] * pt[5][2] - k[4][1] * pt[6][2];
	P[4][3] = pt[4][3] - k[4][0] * pt[5][3] - k[4][1] * pt[6][3];
	P[4][4] = pt[4][4] - k[4][0] * pt[5][4] - k[4][1] * pt[6][4];
	P[4][5] = pt[4][5] - k[4][0] * pt[5][5] - k[4][1] * pt[6][5];
	P[4][6] = pt[4][6] - k[4][0] * pt[5][6] - k[4][1] * pt[6][6];
	P[5][0] = -pt[5][0] * (k[5][0] - 1) - k[5][1] * pt[6][0];
	P[5][1] = -pt[5][1] * (k[5][0] - 1) - k[5][1] * pt[6][1];
	P[5][2] = -pt[5][2] * (k[5][0] - 1) - k[5][1] * pt[6][2];
	P[5][3] = -pt[5][3] * (k[5][0] - 1) - k[5][1] * pt[6][3];
	P[5][4] = -pt[5][4] * (k[5][0] - 1) - k[5][1] * pt[6][4];
	P[5][5] = -pt[5][5] * (k[5][0] - 1) - k[5][1] * pt[6][5];
	P[5][6] = -pt[5][6] * (k[5][0] - 1) - k[5][1] * pt[6][6];
	P[6][0] = -pt[6][0] * (k[6][1] - 1) - k[6][0] * pt[5][0];
	P[6][1] = -pt[6][1] * (k[6][1] - 1) - k[6][0] * pt[5][1];
	P[6][2] = -pt[6][2] * (k[6][1] - 1) - k[6][0] * pt[5][2];
	P[6][3] = -pt[6][3] * (k[6][1] - 1) - k[6][0] * pt[5][3];
	P[6][4] = -pt[6][4] * (k[6][1] - 1) - k[6][0] * pt[5][4];
	P[6][5] = -pt[6][5] * (k[6][1] - 1) - k[6][0] * pt[5][5];
	P[6][6] = -pt[6][6] * (k[6][1] - 1) - k[6][0] * pt[5][6];

	float delta2 = quat[1] - x[5];
	float delta3 = quat[2] - x[6];

	x[0] += k[0][0] * delta2 + k[0][1] * delta3;
	x[1] += k[1][0] * delta2 + k[1][1] * delta3;
	x[2] += k[2][0] * delta2 + k[2][1] * delta3;
	x[3] += k[3][0] * delta2 + k[3][1] * delta3;
	x[4] += k[4][0] * delta2 + k[4][1] * delta3;
	x[5] += k[5][0] * delta2 + k[5][1] * delta3;
	x[6] += k[6][0] * delta2 + k[6][1] * delta3;
}


// Runs update in one concerted update step
// This method runs into problems as the determinant of the residual covariance can be almost singular - resulting in Nan
// Use the separated method instead
void orientation_kalman_new_quaternion_single_update(const float quat[4]) {
	// TODO Choose an appropriate value for this
	const float rk = 0.001f;

	float hphrk[4][4];
	float hphrk_inv[4][4];
	float pt[7][7];

	for (int i = 0; i < 7; i++) {
		for (int j = 0; j < 7; j++) {
			pt[i][j] = P[i][j];
		}
	}

	for (int i = 3; i < 7; i++) {
		for (int j = 3; j < 7; j++) {
			hphrk[i - 3][j - 3] = pt[i][j];
		}
		hphrk[i-3][i-3] += rk;
	}

	mat4x4_inv(hphrk, hphrk_inv);

	for (int i = 0; i < 4; i++)
		PRINT("%f %f %f %f\n", hphrk_inv[0][i], hphrk_inv[1][i], hphrk_inv[2][i], hphrk_inv[3][i]);

	// Kalman Gain
	float k[7][4];
	k[0][0] = hphrk_inv[0][0] * pt[0][3] + hphrk_inv[1][0] * pt[0][4] + hphrk_inv[2][0] * pt[0][5] + hphrk_inv[3][0] * pt[0][6];
	k[0][1] = hphrk_inv[0][1] * pt[0][3] + hphrk_inv[1][1] * pt[0][4] + hphrk_inv[2][1] * pt[0][5] + hphrk_inv[3][1] * pt[0][6];
	k[0][2] = hphrk_inv[0][2] * pt[0][3] + hphrk_inv[1][2] * pt[0][4] + hphrk_inv[2][2] * pt[0][5] + hphrk_inv[3][2] * pt[0][6];
	k[0][3] = hphrk_inv[0][3] * pt[0][3] + hphrk_inv[1][3] * pt[0][4] + hphrk_inv[2][3] * pt[0][5] + hphrk_inv[3][3] * pt[0][6];
	k[1][0] = hphrk_inv[0][0] * pt[1][3] + hphrk_inv[1][0] * pt[1][4] + hphrk_inv[2][0] * pt[1][5] + hphrk_inv[3][0] * pt[1][6];
	k[1][1] = hphrk_inv[0][1] * pt[1][3] + hphrk_inv[1][1] * pt[1][4] + hphrk_inv[2][1] * pt[1][5] + hphrk_inv[3][1] * pt[1][6];
	k[1][2] = hphrk_inv[0][2] * pt[1][3] + hphrk_inv[1][2] * pt[1][4] + hphrk_inv[2][2] * pt[1][5] + hphrk_inv[3][2] * pt[1][6];
	k[1][3] = hphrk_inv[0][3] * pt[1][3] + hphrk_inv[1][3] * pt[1][4] + hphrk_inv[2][3] * pt[1][5] + hphrk_inv[3][3] * pt[1][6];
	k[2][0] = hphrk_inv[0][0] * pt[2][3] + hphrk_inv[1][0] * pt[2][4] + hphrk_inv[2][0] * pt[2][5] + hphrk_inv[3][0] * pt[2][6];
	k[2][1] = hphrk_inv[0][1] * pt[2][3] + hphrk_inv[1][1] * pt[2][4] + hphrk_inv[2][1] * pt[2][5] + hphrk_inv[3][1] * pt[2][6];
	k[2][2] = hphrk_inv[0][2] * pt[2][3] + hphrk_inv[1][2] * pt[2][4] + hphrk_inv[2][2] * pt[2][5] + hphrk_inv[3][2] * pt[2][6];
	k[2][3] = hphrk_inv[0][3] * pt[2][3] + hphrk_inv[1][3] * pt[2][4] + hphrk_inv[2][3] * pt[2][5] + hphrk_inv[3][3] * pt[2][6];
	k[3][0] = hphrk_inv[0][0] * pt[3][3] + hphrk_inv[1][0] * pt[3][4] + hphrk_inv[2][0] * pt[3][5] + hphrk_inv[3][0] * pt[3][6];
	k[3][1] = hphrk_inv[0][1] * pt[3][3] + hphrk_inv[1][1] * pt[3][4] + hphrk_inv[2][1] * pt[3][5] + hphrk_inv[3][1] * pt[3][6];
	k[3][2] = hphrk_inv[0][2] * pt[3][3] + hphrk_inv[1][2] * pt[3][4] + hphrk_inv[2][2] * pt[3][5] + hphrk_inv[3][2] * pt[3][6];
	k[3][3] = hphrk_inv[0][3] * pt[3][3] + hphrk_inv[1][3] * pt[3][4] + hphrk_inv[2][3] * pt[3][5] + hphrk_inv[3][3] * pt[3][6];
	k[4][0] = hphrk_inv[0][0] * pt[4][3] + hphrk_inv[1][0] * pt[4][4] + hphrk_inv[2][0] * pt[4][5] + hphrk_inv[3][0] * pt[4][6];
	k[4][1] = hphrk_inv[0][1] * pt[4][3] + hphrk_inv[1][1] * pt[4][4] + hphrk_inv[2][1] * pt[4][5] + hphrk_inv[3][1] * pt[4][6];
	k[4][2] = hphrk_inv[0][2] * pt[4][3] + hphrk_inv[1][2] * pt[4][4] + hphrk_inv[2][2] * pt[4][5] + hphrk_inv[3][2] * pt[4][6];
	k[4][3] = hphrk_inv[0][3] * pt[4][3] + hphrk_inv[1][3] * pt[4][4] + hphrk_inv[2][3] * pt[4][5] + hphrk_inv[3][3] * pt[4][6];
	k[5][0] = hphrk_inv[0][0] * pt[5][3] + hphrk_inv[1][0] * pt[5][4] + hphrk_inv[2][0] * pt[5][5] + hphrk_inv[3][0] * pt[5][6];
	k[5][1] = hphrk_inv[0][1] * pt[5][3] + hphrk_inv[1][1] * pt[5][4] + hphrk_inv[2][1] * pt[5][5] + hphrk_inv[3][1] * pt[5][6];
	k[5][2] = hphrk_inv[0][2] * pt[5][3] + hphrk_inv[1][2] * pt[5][4] + hphrk_inv[2][2] * pt[5][5] + hphrk_inv[3][2] * pt[5][6];
	k[5][3] = hphrk_inv[0][3] * pt[5][3] + hphrk_inv[1][3] * pt[5][4] + hphrk_inv[2][3] * pt[5][5] + hphrk_inv[3][3] * pt[5][6];
	k[6][0] = hphrk_inv[0][0] * pt[6][3] + hphrk_inv[1][0] * pt[6][4] + hphrk_inv[2][0] * pt[6][5] + hphrk_inv[3][0] * pt[6][6];
	k[6][1] = hphrk_inv[0][1] * pt[6][3] + hphrk_inv[1][1] * pt[6][4] + hphrk_inv[2][1] * pt[6][5] + hphrk_inv[3][1] * pt[6][6];
	k[6][2] = hphrk_inv[0][2] * pt[6][3] + hphrk_inv[1][2] * pt[6][4] + hphrk_inv[2][2] * pt[6][5] + hphrk_inv[3][2] * pt[6][6];
	k[6][3] = hphrk_inv[0][3] * pt[6][3] + hphrk_inv[1][3] * pt[6][4] + hphrk_inv[2][3] * pt[6][5] + hphrk_inv[3][3] * pt[6][6];

	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 4; j++)
			if (k[i][j] != k[i][j])
				PRINT("K Nan");

	// Update Covariance
	P[0][0] = pt[0][0] - k[0][0] * pt[3][0] - k[0][1] * pt[4][0] - k[0][2] * pt[5][0] - k[0][3] * pt[6][0];
	P[0][1] = pt[0][1] - k[0][0] * pt[3][1] - k[0][1] * pt[4][1] - k[0][2] * pt[5][1] - k[0][3] * pt[6][1];
	P[0][2] = pt[0][2] - k[0][0] * pt[3][2] - k[0][1] * pt[4][2] - k[0][2] * pt[5][2] - k[0][3] * pt[6][2];
	P[0][3] = pt[0][3] - k[0][0] * pt[3][3] - k[0][1] * pt[4][3] - k[0][2] * pt[5][3] - k[0][3] * pt[6][3];
	P[0][4] = pt[0][4] - k[0][0] * pt[3][4] - k[0][1] * pt[4][4] - k[0][2] * pt[5][4] - k[0][3] * pt[6][4];
	P[0][5] = pt[0][5] - k[0][0] * pt[3][5] - k[0][1] * pt[4][5] - k[0][2] * pt[5][5] - k[0][3] * pt[6][5];
	P[0][6] = pt[0][6] - k[0][0] * pt[3][6] - k[0][1] * pt[4][6] - k[0][2] * pt[5][6] - k[0][3] * pt[6][6];
	P[1][0] = pt[1][0] - k[1][0] * pt[3][0] - k[1][1] * pt[4][0] - k[1][2] * pt[5][0] - k[0][3] * pt[6][0];
	P[1][1] = pt[1][1] - k[1][0] * pt[3][1] - k[1][1] * pt[4][1] - k[1][2] * pt[5][1] - k[0][3] * pt[6][1];
	P[1][2] = pt[1][2] - k[1][0] * pt[3][2] - k[1][1] * pt[4][2] - k[1][2] * pt[5][2] - k[0][3] * pt[6][2];
	P[1][3] = pt[1][3] - k[1][0] * pt[3][3] - k[1][1] * pt[4][3] - k[1][2] * pt[5][3] - k[0][3] * pt[6][3];
	P[1][4] = pt[1][4] - k[1][0] * pt[3][4] - k[1][1] * pt[4][4] - k[1][2] * pt[5][4] - k[0][3] * pt[6][4];
	P[1][5] = pt[1][5] - k[1][0] * pt[3][5] - k[1][1] * pt[4][5] - k[1][2] * pt[5][5] - k[0][3] * pt[6][5];
	P[1][6] = pt[1][6] - k[1][0] * pt[3][6] - k[1][1] * pt[4][6] - k[1][2] * pt[5][6] - k[0][3] * pt[6][6];
	P[2][0] = pt[2][0] - k[2][0] * pt[3][0] - k[2][1] * pt[4][0] - k[0][3] * pt[6][0] - k[2][2] * pt[5][0];
	P[2][1] = pt[2][1] - k[2][0] * pt[3][1] - k[2][1] * pt[4][1] - k[0][3] * pt[6][1] - k[2][2] * pt[5][1];
	P[2][2] = pt[2][2] - k[2][0] * pt[3][2] - k[2][1] * pt[4][2] - k[0][3] * pt[6][2] - k[2][2] * pt[5][2];
	P[2][3] = pt[2][3] - k[2][0] * pt[3][3] - k[2][1] * pt[4][3] - k[0][3] * pt[6][3] - k[2][2] * pt[5][3];
	P[2][4] = pt[2][4] - k[2][0] * pt[3][4] - k[2][1] * pt[4][4] - k[0][3] * pt[6][4] - k[2][2] * pt[5][4];
	P[2][5] = pt[2][5] - k[2][0] * pt[3][5] - k[2][1] * pt[4][5] - k[0][3] * pt[6][5] - k[2][2] * pt[5][5];
	P[2][6] = pt[2][6] - k[2][0] * pt[3][6] - k[2][1] * pt[4][6] - k[0][3] * pt[6][6] - k[2][2] * pt[5][6];
	P[3][0] = -pt[3][0] * (k[3][0] - 1) - k[3][1] * pt[4][0] - k[0][3] * pt[6][0] - k[3][2] * pt[5][0];
	P[3][1] = -pt[3][1] * (k[3][0] - 1) - k[3][1] * pt[4][1] - k[0][3] * pt[6][1] - k[3][2] * pt[5][1];
	P[3][2] = -pt[3][2] * (k[3][0] - 1) - k[3][1] * pt[4][2] - k[0][3] * pt[6][2] - k[3][2] * pt[5][2];
	P[3][3] = -pt[3][3] * (k[3][0] - 1) - k[3][1] * pt[4][3] - k[0][3] * pt[6][3] - k[3][2] * pt[5][3];
	P[3][4] = -pt[3][4] * (k[3][0] - 1) - k[3][1] * pt[4][4] - k[0][3] * pt[6][4] - k[3][2] * pt[5][4];
	P[3][5] = -pt[3][5] * (k[3][0] - 1) - k[3][1] * pt[4][5] - k[0][3] * pt[6][5] - k[3][2] * pt[5][5];
	P[3][6] = -pt[3][6] * (k[3][0] - 1) - k[3][1] * pt[4][6] - k[0][3] * pt[6][6] - k[3][2] * pt[5][6];
	P[4][0] = -pt[4][0] * (k[4][1] - 1) - k[4][0] * pt[3][0] - k[0][3] * pt[6][0] - k[4][2] * pt[5][0];
	P[4][1] = -pt[4][1] * (k[4][1] - 1) - k[4][0] * pt[3][1] - k[0][3] * pt[6][1] - k[4][2] * pt[5][1];
	P[4][2] = -pt[4][2] * (k[4][1] - 1) - k[4][0] * pt[3][2] - k[0][3] * pt[6][2] - k[4][2] * pt[5][2];
	P[4][3] = -pt[4][3] * (k[4][1] - 1) - k[4][0] * pt[3][3] - k[0][3] * pt[6][3] - k[4][2] * pt[5][3];
	P[4][4] = -pt[4][4] * (k[4][1] - 1) - k[4][0] * pt[3][4] - k[0][3] * pt[6][4] - k[4][2] * pt[5][4];
	P[4][5] = -pt[4][5] * (k[4][1] - 1) - k[4][0] * pt[3][5] - k[0][3] * pt[6][5] - k[4][2] * pt[5][5];
	P[4][6] = -pt[4][6] * (k[4][1] - 1) - k[4][0] * pt[3][6] - k[0][3] * pt[6][6] - k[4][2] * pt[5][6];
	P[5][0] = -pt[5][0] * (k[5][2] - 1) - k[5][0] * pt[3][0] - k[0][3] * pt[6][0] - k[5][1] * pt[4][0];
	P[5][1] = -pt[5][1] * (k[5][2] - 1) - k[5][0] * pt[3][1] - k[0][3] * pt[6][1] - k[5][1] * pt[4][1];
	P[5][2] = -pt[5][2] * (k[5][2] - 1) - k[5][0] * pt[3][2] - k[0][3] * pt[6][2] - k[5][1] * pt[4][2];
	P[5][3] = -pt[5][3] * (k[5][2] - 1) - k[5][0] * pt[3][3] - k[0][3] * pt[6][3] - k[5][1] * pt[4][3];
	P[5][4] = -pt[5][4] * (k[5][2] - 1) - k[5][0] * pt[3][4] - k[0][3] * pt[6][4] - k[5][1] * pt[4][4];
	P[5][5] = -pt[5][5] * (k[5][2] - 1) - k[5][0] * pt[3][5] - k[0][3] * pt[6][5] - k[5][1] * pt[4][5];
	P[5][6] = -pt[5][6] * (k[5][2] - 1) - k[5][0] * pt[3][6] - k[0][3] * pt[6][6] - k[5][1] * pt[4][6];
	P[6][0] = -pt[6][0] * (k[0][3] - 1) - k[6][0] * pt[3][0] - k[6][1] * pt[4][0] - k[6][2] * pt[5][0];
	P[6][1] = -pt[6][1] * (k[0][3] - 1) - k[6][0] * pt[3][1] - k[6][1] * pt[4][1] - k[6][2] * pt[5][1];
	P[6][2] = -pt[6][2] * (k[0][3] - 1) - k[6][0] * pt[3][2] - k[6][1] * pt[4][2] - k[6][2] * pt[5][2];
	P[6][3] = -pt[6][3] * (k[0][3] - 1) - k[6][0] * pt[3][3] - k[6][1] * pt[4][3] - k[6][2] * pt[5][3];
	P[6][4] = -pt[6][4] * (k[0][3] - 1) - k[6][0] * pt[3][4] - k[6][1] * pt[4][4] - k[6][2] * pt[5][4];
	P[6][5] = -pt[6][5] * (k[0][3] - 1) - k[6][0] * pt[3][5] - k[6][1] * pt[4][5] - k[6][2] * pt[5][5];
	P[6][6] = -pt[6][6] * (k[0][3] - 1) - k[6][0] * pt[3][6] - k[6][1] * pt[4][6] - k[6][2] * pt[5][6];

	float x3 = x[3];
	float x4 = x[4];
	float x5 = x[5];
	float x6 = x[6];

	//Update x (quat swapped from (vector,scalar) to (scalar,vector)
	x[0] += -(x3 - quat[3])*k[0][0] - (x4 - quat[0])*k[0][1] - (x5 - quat[1])*k[0][2] - (x6 - quat[2])*k[0][3];
	x[1] += -(x3 - quat[3])*k[1][0] - (x4 - quat[0])*k[1][1] - (x5 - quat[1])*k[1][2] - (x6 - quat[2])*k[0][3];
	x[2] += -(x3 - quat[3])*k[2][0] - (x4 - quat[0])*k[2][1] - (x5 - quat[1])*k[2][2] - (x6 - quat[2])*k[0][3];
	x[3] += -(x3 - quat[3])*k[3][0] - (x4 - quat[0])*k[3][1] - (x5 - quat[1])*k[3][2] - (x6 - quat[2])*k[0][3];
	x[4] += -(x3 - quat[3])*k[4][0] - (x4 - quat[0])*k[4][1] - (x6 - quat[2])*k[0][3] - (x5 - quat[1])*k[4][2];
	x[5] += -(x3 - quat[3])*k[5][0] - (x4 - quat[0])*k[5][1] - (x6 - quat[2])*k[0][3] - (x5 - quat[1])*k[5][2];
	x[6] += -(x3 - quat[3])*k[6][0] - (x4 - quat[0])*k[6][1] - (x6 - quat[2])*k[0][3] - (x5 - quat[1])*k[6][2];

	if (checkForNan()) {
		PRINT("Quat Update Nan");

	}
}

void orientation_kalman_new_quaternion(const float quat[4]) {
	//orientation_kalman_new_quaternion_single_update(quat);
	orientation_kalman_new_quaternion_wx_update(quat);
	orientation_kalman_new_quaternion_yz_update(quat);
}


void orientation_kalman_prediction_step(state_estimate_t* estimate, float d) {
	const float tau = 0.5f;
	const float variance = 0.4f;
	float Q00 = variance / (2.0f*tau) * (1.0f - expf(-2.0f*d / tau));
	float Q11 = Q00;
	float Q22 = Q00;


	float edt0 = exp(-d / tau);
	float edt1 = edt0;
	float edt2 = edt0;
	float edt[3] = { edt0, edt1, edt2 };


	float pt[7][7];
	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 7; j++)
			pt[i][j] = P[i][j];

	float dxby2[7];
	float dxby2squared[7];
	for (int i = 0; i < 7; i++){
		dxby2[i] = x[i] * d / 2.0f;
		dxby2squared[i] = dxby2[i] * dxby2[i];
	}

	//TODO: Simplify out common terms from horrendous mess of code
	{
		P[0][0] = Q00 + edt0 * edt0 * pt[0][0];
		P[0][1] = edt0*edt1*pt[0][1];
		P[0][2] = edt0*edt2*pt[0][2];
		P[0][3] = edt0*pt[0][3] - edt0*(dxby2[0])*pt[0][4] - edt0*(dxby2[4])*pt[0][0] - edt0*(dxby2[1])*pt[0][5] - edt0*(dxby2[5])*pt[0][1] - edt0*(dxby2[2])*pt[0][6] - edt0*(dxby2[6])*pt[0][2];
		P[0][4] = edt0*pt[0][4] + edt0*(dxby2[0])*pt[0][3] + edt0*(dxby2[3])*pt[0][0] - edt0*(dxby2[1])*pt[0][6] + edt0*(dxby2[2])*pt[0][5] + edt0*(dxby2[5])*pt[0][2] - edt0*(dxby2[6])*pt[0][1];
		P[0][5] = edt0*pt[0][5] + edt0*(dxby2[1])*pt[0][3] + edt0*(dxby2[3])*pt[0][1] + edt0*(dxby2[0])*pt[0][6] - edt0*(dxby2[2])*pt[0][4] - edt0*(dxby2[4])*pt[0][2] + edt0*(dxby2[6])*pt[0][0];
		P[0][6] = edt0*pt[0][6] - edt0*(dxby2[0])*pt[0][5] + edt0*(dxby2[1])*pt[0][4] + edt0*(dxby2[2])*pt[0][3] + edt0*(dxby2[3])*pt[0][2] + edt0*(dxby2[4])*pt[0][1] - edt0*(dxby2[5])*pt[0][0];
		P[1][0] = edt0*edt1*pt[1][0];
		P[1][1] = Q11 + edt1 * edt1 * pt[1][1];
		P[1][2] = edt1*edt2*pt[1][2];
		P[1][3] = edt1*pt[1][3] - edt1*(dxby2[0])*pt[1][4] - edt1*(dxby2[4])*pt[1][0] - edt1*(dxby2[1])*pt[1][5] - edt1*(dxby2[5])*pt[1][1] - edt1*(dxby2[2])*pt[1][6] - edt1*(dxby2[6])*pt[1][2];
		P[1][4] = edt1*pt[1][4] + edt1*(dxby2[0])*pt[1][3] + edt1*(dxby2[3])*pt[1][0] - edt1*(dxby2[1])*pt[1][6] + edt1*(dxby2[2])*pt[1][5] + edt1*(dxby2[5])*pt[1][2] - edt1*(dxby2[6])*pt[1][1];
		P[1][5] = edt1*pt[1][5] + edt1*(dxby2[1])*pt[1][3] + edt1*(dxby2[3])*pt[1][1] + edt1*(dxby2[0])*pt[1][6] - edt1*(dxby2[2])*pt[1][4] - edt1*(dxby2[4])*pt[1][2] + edt1*(dxby2[6])*pt[1][0];
		P[1][6] = edt1*pt[1][6] - edt1*(dxby2[0])*pt[1][5] + edt1*(dxby2[1])*pt[1][4] + edt1*(dxby2[2])*pt[1][3] + edt1*(dxby2[3])*pt[1][2] + edt1*(dxby2[4])*pt[1][1] - edt1*(dxby2[5])*pt[1][0];
		P[2][0] = edt0*edt2*pt[2][0];
		P[2][1] = edt1*edt2*pt[2][1];
		P[2][2] = Q22 + edt2 * edt2 * pt[2][2];
		P[2][3] = edt2*pt[2][3] - edt2*(dxby2[0])*pt[2][4] - edt2*(dxby2[4])*pt[2][0] - edt2*(dxby2[1])*pt[2][5] - edt2*(dxby2[5])*pt[2][1] - edt2*(dxby2[2])*pt[2][6] - edt2*(dxby2[6])*pt[2][2];
		P[2][4] = edt2*pt[2][4] + edt2*(dxby2[0])*pt[2][3] + edt2*(dxby2[3])*pt[2][0] - edt2*(dxby2[1])*pt[2][6] + edt2*(dxby2[2])*pt[2][5] + edt2*(dxby2[5])*pt[2][2] - edt2*(dxby2[6])*pt[2][1];
		P[2][5] = edt2*pt[2][5] + edt2*(dxby2[1])*pt[2][3] + edt2*(dxby2[3])*pt[2][1] + edt2*(dxby2[0])*pt[2][6] - edt2*(dxby2[2])*pt[2][4] - edt2*(dxby2[4])*pt[2][2] + edt2*(dxby2[6])*pt[2][0];
		P[2][6] = edt2*pt[2][6] - edt2*(dxby2[0])*pt[2][5] + edt2*(dxby2[1])*pt[2][4] + edt2*(dxby2[2])*pt[2][3] + edt2*(dxby2[3])*pt[2][2] + edt2*(dxby2[4])*pt[2][1] - edt2*(dxby2[5])*pt[2][0];
		P[3][0] = -edt0*(pt[0][0] * dxby2[4] - pt[3][0] + pt[4][0] * dxby2[0] + pt[1][0] * dxby2[5] + pt[5][0] * dxby2[1] + pt[2][0] * dxby2[6] + pt[6][0] * dxby2[2]);
		P[3][1] = -edt1*(pt[0][1] * dxby2[4] - pt[3][1] + pt[4][1] * dxby2[0] + pt[1][1] * dxby2[5] + pt[5][1] * dxby2[1] + pt[2][1] * dxby2[6] + pt[6][1] * dxby2[2]);
		P[3][2] = -edt2*(pt[0][2] * dxby2[4] - pt[3][2] + pt[4][2] * dxby2[0] + pt[1][2] * dxby2[5] + pt[5][2] * dxby2[1] + pt[2][2] * dxby2[6] + pt[6][2] * dxby2[2]);
		P[3][3] = pt[3][3] - pt[0][3] * dxby2[4] - pt[4][3] * dxby2[0] - pt[1][3] * dxby2[5] - pt[5][3] * dxby2[1] - pt[2][3] * dxby2[6] - pt[6][3] * dxby2[2] + (dxby2[4])*(pt[0][0] * dxby2[4] - pt[3][0] + pt[4][0] * dxby2[0] + pt[1][0] * dxby2[5] + pt[5][0] * dxby2[1] + pt[2][0] * dxby2[6] + pt[6][0] * dxby2[2]) + (dxby2[5])*(pt[0][1] * dxby2[4] - pt[3][1] + pt[4][1] * dxby2[0] + pt[1][1] * dxby2[5] + pt[5][1] * dxby2[1] + pt[2][1] * dxby2[6] + pt[6][1] * dxby2[2]) + (dxby2[6])*(pt[0][2] * dxby2[4] - pt[3][2] + pt[4][2] * dxby2[0] + pt[1][2] * dxby2[5] + pt[5][2] * dxby2[1] + pt[2][2] * dxby2[6] + pt[6][2] * dxby2[2]) + (dxby2[0])*(pt[0][4] * dxby2[4] - pt[3][4] + pt[4][4] * dxby2[0] + pt[1][4] * dxby2[5] + pt[5][4] * dxby2[1] + pt[2][4] * dxby2[6] + pt[6][4] * dxby2[2]) + (dxby2[1])*(pt[0][5] * dxby2[4] - pt[3][5] + pt[4][5] * dxby2[0] + pt[1][5] * dxby2[5] + pt[5][5] * dxby2[1] + pt[2][5] * dxby2[6] + pt[6][5] * dxby2[2]) + (dxby2[2])*(pt[0][6] * dxby2[4] - pt[3][6] + pt[4][6] * dxby2[0] + pt[1][6] * dxby2[5] + pt[5][6] * dxby2[1] + pt[2][6] * dxby2[6] + pt[6][6] * dxby2[2]);
		P[3][4] = pt[3][4] - pt[0][4] * dxby2[4] - pt[4][4] * dxby2[0] - pt[1][4] * dxby2[5] - pt[5][4] * dxby2[1] - pt[2][4] * dxby2[6] - pt[6][4] * dxby2[2] - (dxby2[3])*(pt[0][0] * dxby2[4] - pt[3][0] + pt[4][0] * dxby2[0] + pt[1][0] * dxby2[5] + pt[5][0] * dxby2[1] + pt[2][0] * dxby2[6] + pt[6][0] * dxby2[2]) + (dxby2[6])*(pt[0][1] * dxby2[4] - pt[3][1] + pt[4][1] * dxby2[0] + pt[1][1] * dxby2[5] + pt[5][1] * dxby2[1] + pt[2][1] * dxby2[6] + pt[6][1] * dxby2[2]) - (dxby2[5])*(pt[0][2] * dxby2[4] - pt[3][2] + pt[4][2] * dxby2[0] + pt[1][2] * dxby2[5] + pt[5][2] * dxby2[1] + pt[2][2] * dxby2[6] + pt[6][2] * dxby2[2]) - (dxby2[0])*(pt[0][3] * dxby2[4] - pt[3][3] + pt[4][3] * dxby2[0] + pt[1][3] * dxby2[5] + pt[5][3] * dxby2[1] + pt[2][3] * dxby2[6] + pt[6][3] * dxby2[2]) - (dxby2[2])*(pt[0][5] * dxby2[4] - pt[3][5] + pt[4][5] * dxby2[0] + pt[1][5] * dxby2[5] + pt[5][5] * dxby2[1] + pt[2][5] * dxby2[6] + pt[6][5] * dxby2[2]) + (dxby2[1])*(pt[0][6] * dxby2[4] - pt[3][6] + pt[4][6] * dxby2[0] + pt[1][6] * dxby2[5] + pt[5][6] * dxby2[1] + pt[2][6] * dxby2[6] + pt[6][6] * dxby2[2]);
		P[3][5] = pt[3][5] - pt[0][5] * dxby2[4] - pt[4][5] * dxby2[0] - pt[1][5] * dxby2[5] - pt[5][5] * dxby2[1] - pt[2][5] * dxby2[6] - pt[6][5] * dxby2[2] - (dxby2[6])*(pt[0][0] * dxby2[4] - pt[3][0] + pt[4][0] * dxby2[0] + pt[1][0] * dxby2[5] + pt[5][0] * dxby2[1] + pt[2][0] * dxby2[6] + pt[6][0] * dxby2[2]) - (dxby2[3])*(pt[0][1] * dxby2[4] - pt[3][1] + pt[4][1] * dxby2[0] + pt[1][1] * dxby2[5] + pt[5][1] * dxby2[1] + pt[2][1] * dxby2[6] + pt[6][1] * dxby2[2]) + (dxby2[4])*(pt[0][2] * dxby2[4] - pt[3][2] + pt[4][2] * dxby2[0] + pt[1][2] * dxby2[5] + pt[5][2] * dxby2[1] + pt[2][2] * dxby2[6] + pt[6][2] * dxby2[2]) - (dxby2[1])*(pt[0][3] * dxby2[4] - pt[3][3] + pt[4][3] * dxby2[0] + pt[1][3] * dxby2[5] + pt[5][3] * dxby2[1] + pt[2][3] * dxby2[6] + pt[6][3] * dxby2[2]) + (dxby2[2])*(pt[0][4] * dxby2[4] - pt[3][4] + pt[4][4] * dxby2[0] + pt[1][4] * dxby2[5] + pt[5][4] * dxby2[1] + pt[2][4] * dxby2[6] + pt[6][4] * dxby2[2]) - (dxby2[0])*(pt[0][6] * dxby2[4] - pt[3][6] + pt[4][6] * dxby2[0] + pt[1][6] * dxby2[5] + pt[5][6] * dxby2[1] + pt[2][6] * dxby2[6] + pt[6][6] * dxby2[2]);
		P[3][6] = pt[3][6] - pt[0][6] * dxby2[4] - pt[4][6] * dxby2[0] - pt[1][6] * dxby2[5] - pt[5][6] * dxby2[1] - pt[2][6] * dxby2[6] - pt[6][6] * dxby2[2] + (dxby2[5])*(pt[0][0] * dxby2[4] - pt[3][0] + pt[4][0] * dxby2[0] + pt[1][0] * dxby2[5] + pt[5][0] * dxby2[1] + pt[2][0] * dxby2[6] + pt[6][0] * dxby2[2]) - (dxby2[4])*(pt[0][1] * dxby2[4] - pt[3][1] + pt[4][1] * dxby2[0] + pt[1][1] * dxby2[5] + pt[5][1] * dxby2[1] + pt[2][1] * dxby2[6] + pt[6][1] * dxby2[2]) - (dxby2[3])*(pt[0][2] * dxby2[4] - pt[3][2] + pt[4][2] * dxby2[0] + pt[1][2] * dxby2[5] + pt[5][2] * dxby2[1] + pt[2][2] * dxby2[6] + pt[6][2] * dxby2[2]) - (dxby2[2])*(pt[0][3] * dxby2[4] - pt[3][3] + pt[4][3] * dxby2[0] + pt[1][3] * dxby2[5] + pt[5][3] * dxby2[1] + pt[2][3] * dxby2[6] + pt[6][3] * dxby2[2]) - (dxby2[1])*(pt[0][4] * dxby2[4] - pt[3][4] + pt[4][4] * dxby2[0] + pt[1][4] * dxby2[5] + pt[5][4] * dxby2[1] + pt[2][4] * dxby2[6] + pt[6][4] * dxby2[2]) + (dxby2[0])*(pt[0][5] * dxby2[4] - pt[3][5] + pt[4][5] * dxby2[0] + pt[1][5] * dxby2[5] + pt[5][5] * dxby2[1] + pt[2][5] * dxby2[6] + pt[6][5] * dxby2[2]);
		P[4][0] = edt0*(pt[4][0] + pt[0][0] * dxby2[3] + pt[3][0] * dxby2[0] - pt[1][0] * dxby2[6] + pt[2][0] * dxby2[5] + pt[5][0] * dxby2[2] - pt[6][0] * dxby2[1]);
		P[4][1] = edt1*(pt[4][1] + pt[0][1] * dxby2[3] + pt[3][1] * dxby2[0] - pt[1][1] * dxby2[6] + pt[2][1] * dxby2[5] + pt[5][1] * dxby2[2] - pt[6][1] * dxby2[1]);
		P[4][2] = edt2*(pt[4][2] + pt[0][2] * dxby2[3] + pt[3][2] * dxby2[0] - pt[1][2] * dxby2[6] + pt[2][2] * dxby2[5] + pt[5][2] * dxby2[2] - pt[6][2] * dxby2[1]);
		P[4][3] = pt[4][3] - (dxby2[5])*(pt[4][1] + pt[0][1] * dxby2[3] + pt[3][1] * dxby2[0] - pt[1][1] * dxby2[6] + pt[2][1] * dxby2[5] + pt[5][1] * dxby2[2] - pt[6][1] * dxby2[1]) - (dxby2[6])*(pt[4][2] + pt[0][2] * dxby2[3] + pt[3][2] * dxby2[0] - pt[1][2] * dxby2[6] + pt[2][2] * dxby2[5] + pt[5][2] * dxby2[2] - pt[6][2] * dxby2[1]) - (dxby2[0])*(pt[4][4] + pt[0][4] * dxby2[3] + pt[3][4] * dxby2[0] - pt[1][4] * dxby2[6] + pt[2][4] * dxby2[5] + pt[5][4] * dxby2[2] - pt[6][4] * dxby2[1]) - (dxby2[1])*(pt[4][5] + pt[0][5] * dxby2[3] + pt[3][5] * dxby2[0] - pt[1][5] * dxby2[6] + pt[2][5] * dxby2[5] + pt[5][5] * dxby2[2] - pt[6][5] * dxby2[1]) - (dxby2[2])*(pt[4][6] + pt[0][6] * dxby2[3] + pt[3][6] * dxby2[0] - pt[1][6] * dxby2[6] + pt[2][6] * dxby2[5] + pt[5][6] * dxby2[2] - pt[6][6] * dxby2[1]) - (dxby2[4])*(pt[4][0] + pt[0][0] * dxby2[3] + pt[3][0] * dxby2[0] - pt[1][0] * dxby2[6] + pt[2][0] * dxby2[5] + pt[5][0] * dxby2[2] - pt[6][0] * dxby2[1]) + pt[0][3] * dxby2[3] + pt[3][3] * dxby2[0] - pt[1][3] * dxby2[6] + pt[2][3] * dxby2[5] + pt[5][3] * dxby2[2] - pt[6][3] * dxby2[1];
		P[4][4] = (dxby2[3])*(pt[4][0] + pt[0][0] * dxby2[3] + pt[3][0] * dxby2[0] - pt[1][0] * dxby2[6] + pt[2][0] * dxby2[5] + pt[5][0] * dxby2[2] - pt[6][0] * dxby2[1]) - (dxby2[6])*(pt[4][1] + pt[0][1] * dxby2[3] + pt[3][1] * dxby2[0] - pt[1][1] * dxby2[6] + pt[2][1] * dxby2[5] + pt[5][1] * dxby2[2] - pt[6][1] * dxby2[1]) + (dxby2[5])*(pt[4][2] + pt[0][2] * dxby2[3] + pt[3][2] * dxby2[0] - pt[1][2] * dxby2[6] + pt[2][2] * dxby2[5] + pt[5][2] * dxby2[2] - pt[6][2] * dxby2[1]) + (dxby2[0])*(pt[4][3] + pt[0][3] * dxby2[3] + pt[3][3] * dxby2[0] - pt[1][3] * dxby2[6] + pt[2][3] * dxby2[5] + pt[5][3] * dxby2[2] - pt[6][3] * dxby2[1]) + (dxby2[2])*(pt[4][5] + pt[0][5] * dxby2[3] + pt[3][5] * dxby2[0] - pt[1][5] * dxby2[6] + pt[2][5] * dxby2[5] + pt[5][5] * dxby2[2] - pt[6][5] * dxby2[1]) - (dxby2[1])*(pt[4][6] + pt[0][6] * dxby2[3] + pt[3][6] * dxby2[0] - pt[1][6] * dxby2[6] + pt[2][6] * dxby2[5] + pt[5][6] * dxby2[2] - pt[6][6] * dxby2[1]) + pt[4][4] + pt[0][4] * dxby2[3] + pt[3][4] * dxby2[0] - pt[1][4] * dxby2[6] + pt[2][4] * dxby2[5] + pt[5][4] * dxby2[2] - pt[6][4] * dxby2[1];
		P[4][5] = (dxby2[6])*(pt[4][0] + pt[0][0] * dxby2[3] + pt[3][0] * dxby2[0] - pt[1][0] * dxby2[6] + pt[2][0] * dxby2[5] + pt[5][0] * dxby2[2] - pt[6][0] * dxby2[1]) + (dxby2[3])*(pt[4][1] + pt[0][1] * dxby2[3] + pt[3][1] * dxby2[0] - pt[1][1] * dxby2[6] + pt[2][1] * dxby2[5] + pt[5][1] * dxby2[2] - pt[6][1] * dxby2[1]) - (dxby2[4])*(pt[4][2] + pt[0][2] * dxby2[3] + pt[3][2] * dxby2[0] - pt[1][2] * dxby2[6] + pt[2][2] * dxby2[5] + pt[5][2] * dxby2[2] - pt[6][2] * dxby2[1]) + (dxby2[1])*(pt[4][3] + pt[0][3] * dxby2[3] + pt[3][3] * dxby2[0] - pt[1][3] * dxby2[6] + pt[2][3] * dxby2[5] + pt[5][3] * dxby2[2] - pt[6][3] * dxby2[1]) - (dxby2[2])*(pt[4][4] + pt[0][4] * dxby2[3] + pt[3][4] * dxby2[0] - pt[1][4] * dxby2[6] + pt[2][4] * dxby2[5] + pt[5][4] * dxby2[2] - pt[6][4] * dxby2[1]) + (dxby2[0])*(pt[4][6] + pt[0][6] * dxby2[3] + pt[3][6] * dxby2[0] - pt[1][6] * dxby2[6] + pt[2][6] * dxby2[5] + pt[5][6] * dxby2[2] - pt[6][6] * dxby2[1]) + pt[4][5] + pt[0][5] * dxby2[3] + pt[3][5] * dxby2[0] - pt[1][5] * dxby2[6] + pt[2][5] * dxby2[5] + pt[5][5] * dxby2[2] - pt[6][5] * dxby2[1];
		P[4][6] = (dxby2[4])*(pt[4][1] + pt[0][1] * dxby2[3] + pt[3][1] * dxby2[0] - pt[1][1] * dxby2[6] + pt[2][1] * dxby2[5] + pt[5][1] * dxby2[2] - pt[6][1] * dxby2[1]) - (dxby2[5])*(pt[4][0] + pt[0][0] * dxby2[3] + pt[3][0] * dxby2[0] - pt[1][0] * dxby2[6] + pt[2][0] * dxby2[5] + pt[5][0] * dxby2[2] - pt[6][0] * dxby2[1]) + (dxby2[3])*(pt[4][2] + pt[0][2] * dxby2[3] + pt[3][2] * dxby2[0] - pt[1][2] * dxby2[6] + pt[2][2] * dxby2[5] + pt[5][2] * dxby2[2] - pt[6][2] * dxby2[1]) + (dxby2[2])*(pt[4][3] + pt[0][3] * dxby2[3] + pt[3][3] * dxby2[0] - pt[1][3] * dxby2[6] + pt[2][3] * dxby2[5] + pt[5][3] * dxby2[2] - pt[6][3] * dxby2[1]) + (dxby2[1])*(pt[4][4] + pt[0][4] * dxby2[3] + pt[3][4] * dxby2[0] - pt[1][4] * dxby2[6] + pt[2][4] * dxby2[5] + pt[5][4] * dxby2[2] - pt[6][4] * dxby2[1]) - (dxby2[0])*(pt[4][5] + pt[0][5] * dxby2[3] + pt[3][5] * dxby2[0] - pt[1][5] * dxby2[6] + pt[2][5] * dxby2[5] + pt[5][5] * dxby2[2] - pt[6][5] * dxby2[1]) + pt[4][6] + pt[0][6] * dxby2[3] + pt[3][6] * dxby2[0] - pt[1][6] * dxby2[6] + pt[2][6] * dxby2[5] + pt[5][6] * dxby2[2] - pt[6][6] * dxby2[1];
		P[5][0] = edt0*(pt[5][0] + pt[1][0] * dxby2[3] + pt[3][0] * dxby2[1] + pt[0][0] * dxby2[6] - pt[2][0] * dxby2[4] - pt[4][0] * dxby2[2] + pt[6][0] * dxby2[0]);
		P[5][1] = edt1*(pt[5][1] + pt[1][1] * dxby2[3] + pt[3][1] * dxby2[1] + pt[0][1] * dxby2[6] - pt[2][1] * dxby2[4] - pt[4][1] * dxby2[2] + pt[6][1] * dxby2[0]);
		P[5][2] = edt2*(pt[5][2] + pt[1][2] * dxby2[3] + pt[3][2] * dxby2[1] + pt[0][2] * dxby2[6] - pt[2][2] * dxby2[4] - pt[4][2] * dxby2[2] + pt[6][2] * dxby2[0]);
		P[5][3] = pt[5][3] - (dxby2[5])*(pt[5][1] + pt[1][1] * dxby2[3] + pt[3][1] * dxby2[1] + pt[0][1] * dxby2[6] - pt[2][1] * dxby2[4] - pt[4][1] * dxby2[2] + pt[6][1] * dxby2[0]) - (dxby2[6])*(pt[5][2] + pt[1][2] * dxby2[3] + pt[3][2] * dxby2[1] + pt[0][2] * dxby2[6] - pt[2][2] * dxby2[4] - pt[4][2] * dxby2[2] + pt[6][2] * dxby2[0]) - (dxby2[0])*(pt[5][4] + pt[1][4] * dxby2[3] + pt[3][4] * dxby2[1] + pt[0][4] * dxby2[6] - pt[2][4] * dxby2[4] - pt[4][4] * dxby2[2] + pt[6][4] * dxby2[0]) - (dxby2[1])*(pt[5][5] + pt[1][5] * dxby2[3] + pt[3][5] * dxby2[1] + pt[0][5] * dxby2[6] - pt[2][5] * dxby2[4] - pt[4][5] * dxby2[2] + pt[6][5] * dxby2[0]) - (dxby2[2])*(pt[5][6] + pt[1][6] * dxby2[3] + pt[3][6] * dxby2[1] + pt[0][6] * dxby2[6] - pt[2][6] * dxby2[4] - pt[4][6] * dxby2[2] + pt[6][6] * dxby2[0]) - (dxby2[4])*(pt[5][0] + pt[1][0] * dxby2[3] + pt[3][0] * dxby2[1] + pt[0][0] * dxby2[6] - pt[2][0] * dxby2[4] - pt[4][0] * dxby2[2] + pt[6][0] * dxby2[0]) + pt[1][3] * dxby2[3] + pt[3][3] * dxby2[1] + pt[0][3] * dxby2[6] - pt[2][3] * dxby2[4] - pt[4][3] * dxby2[2] + pt[6][3] * dxby2[0];
		P[5][4] = (dxby2[3])*(pt[5][0] + pt[1][0] * dxby2[3] + pt[3][0] * dxby2[1] + pt[0][0] * dxby2[6] - pt[2][0] * dxby2[4] - pt[4][0] * dxby2[2] + pt[6][0] * dxby2[0]) - (dxby2[6])*(pt[5][1] + pt[1][1] * dxby2[3] + pt[3][1] * dxby2[1] + pt[0][1] * dxby2[6] - pt[2][1] * dxby2[4] - pt[4][1] * dxby2[2] + pt[6][1] * dxby2[0]) + (dxby2[5])*(pt[5][2] + pt[1][2] * dxby2[3] + pt[3][2] * dxby2[1] + pt[0][2] * dxby2[6] - pt[2][2] * dxby2[4] - pt[4][2] * dxby2[2] + pt[6][2] * dxby2[0]) + (dxby2[0])*(pt[5][3] + pt[1][3] * dxby2[3] + pt[3][3] * dxby2[1] + pt[0][3] * dxby2[6] - pt[2][3] * dxby2[4] - pt[4][3] * dxby2[2] + pt[6][3] * dxby2[0]) + (dxby2[2])*(pt[5][5] + pt[1][5] * dxby2[3] + pt[3][5] * dxby2[1] + pt[0][5] * dxby2[6] - pt[2][5] * dxby2[4] - pt[4][5] * dxby2[2] + pt[6][5] * dxby2[0]) - (dxby2[1])*(pt[5][6] + pt[1][6] * dxby2[3] + pt[3][6] * dxby2[1] + pt[0][6] * dxby2[6] - pt[2][6] * dxby2[4] - pt[4][6] * dxby2[2] + pt[6][6] * dxby2[0]) + pt[5][4] + pt[1][4] * dxby2[3] + pt[3][4] * dxby2[1] + pt[0][4] * dxby2[6] - pt[2][4] * dxby2[4] - pt[4][4] * dxby2[2] + pt[6][4] * dxby2[0];
		P[5][5] = (dxby2[6])*(pt[5][0] + pt[1][0] * dxby2[3] + pt[3][0] * dxby2[1] + pt[0][0] * dxby2[6] - pt[2][0] * dxby2[4] - pt[4][0] * dxby2[2] + pt[6][0] * dxby2[0]) + (dxby2[3])*(pt[5][1] + pt[1][1] * dxby2[3] + pt[3][1] * dxby2[1] + pt[0][1] * dxby2[6] - pt[2][1] * dxby2[4] - pt[4][1] * dxby2[2] + pt[6][1] * dxby2[0]) - (dxby2[4])*(pt[5][2] + pt[1][2] * dxby2[3] + pt[3][2] * dxby2[1] + pt[0][2] * dxby2[6] - pt[2][2] * dxby2[4] - pt[4][2] * dxby2[2] + pt[6][2] * dxby2[0]) + (dxby2[1])*(pt[5][3] + pt[1][3] * dxby2[3] + pt[3][3] * dxby2[1] + pt[0][3] * dxby2[6] - pt[2][3] * dxby2[4] - pt[4][3] * dxby2[2] + pt[6][3] * dxby2[0]) - (dxby2[2])*(pt[5][4] + pt[1][4] * dxby2[3] + pt[3][4] * dxby2[1] + pt[0][4] * dxby2[6] - pt[2][4] * dxby2[4] - pt[4][4] * dxby2[2] + pt[6][4] * dxby2[0]) + (dxby2[0])*(pt[5][6] + pt[1][6] * dxby2[3] + pt[3][6] * dxby2[1] + pt[0][6] * dxby2[6] - pt[2][6] * dxby2[4] - pt[4][6] * dxby2[2] + pt[6][6] * dxby2[0]) + pt[5][5] + pt[1][5] * dxby2[3] + pt[3][5] * dxby2[1] + pt[0][5] * dxby2[6] - pt[2][5] * dxby2[4] - pt[4][5] * dxby2[2] + pt[6][5] * dxby2[0];
		P[5][6] = (dxby2[4])*(pt[5][1] + pt[1][1] * dxby2[3] + pt[3][1] * dxby2[1] + pt[0][1] * dxby2[6] - pt[2][1] * dxby2[4] - pt[4][1] * dxby2[2] + pt[6][1] * dxby2[0]) - (dxby2[5])*(pt[5][0] + pt[1][0] * dxby2[3] + pt[3][0] * dxby2[1] + pt[0][0] * dxby2[6] - pt[2][0] * dxby2[4] - pt[4][0] * dxby2[2] + pt[6][0] * dxby2[0]) + (dxby2[3])*(pt[5][2] + pt[1][2] * dxby2[3] + pt[3][2] * dxby2[1] + pt[0][2] * dxby2[6] - pt[2][2] * dxby2[4] - pt[4][2] * dxby2[2] + pt[6][2] * dxby2[0]) + (dxby2[2])*(pt[5][3] + pt[1][3] * dxby2[3] + pt[3][3] * dxby2[1] + pt[0][3] * dxby2[6] - pt[2][3] * dxby2[4] - pt[4][3] * dxby2[2] + pt[6][3] * dxby2[0]) + (dxby2[1])*(pt[5][4] + pt[1][4] * dxby2[3] + pt[3][4] * dxby2[1] + pt[0][4] * dxby2[6] - pt[2][4] * dxby2[4] - pt[4][4] * dxby2[2] + pt[6][4] * dxby2[0]) - (dxby2[0])*(pt[5][5] + pt[1][5] * dxby2[3] + pt[3][5] * dxby2[1] + pt[0][5] * dxby2[6] - pt[2][5] * dxby2[4] - pt[4][5] * dxby2[2] + pt[6][5] * dxby2[0]) + pt[5][6] + pt[1][6] * dxby2[3] + pt[3][6] * dxby2[1] + pt[0][6] * dxby2[6] - pt[2][6] * dxby2[4] - pt[4][6] * dxby2[2] + pt[6][6] * dxby2[0];
		P[6][0] = edt0*(pt[6][0] - pt[0][0] * dxby2[5] + pt[1][0] * dxby2[4] + pt[2][0] * dxby2[3] + pt[3][0] * dxby2[2] + pt[4][0] * dxby2[1] - pt[5][0] * dxby2[0]);
		P[6][1] = edt1*(pt[6][1] - pt[0][1] * dxby2[5] + pt[1][1] * dxby2[4] + pt[2][1] * dxby2[3] + pt[3][1] * dxby2[2] + pt[4][1] * dxby2[1] - pt[5][1] * dxby2[0]);
		P[6][2] = edt2*(pt[6][2] - pt[0][2] * dxby2[5] + pt[1][2] * dxby2[4] + pt[2][2] * dxby2[3] + pt[3][2] * dxby2[2] + pt[4][2] * dxby2[1] - pt[5][2] * dxby2[0]);
		P[6][3] = pt[6][3] - (dxby2[5])*(pt[6][1] - pt[0][1] * dxby2[5] + pt[1][1] * dxby2[4] + pt[2][1] * dxby2[3] + pt[3][1] * dxby2[2] + pt[4][1] * dxby2[1] - pt[5][1] * dxby2[0]) - (dxby2[6])*(pt[6][2] - pt[0][2] * dxby2[5] + pt[1][2] * dxby2[4] + pt[2][2] * dxby2[3] + pt[3][2] * dxby2[2] + pt[4][2] * dxby2[1] - pt[5][2] * dxby2[0]) - (dxby2[0])*(pt[6][4] - pt[0][4] * dxby2[5] + pt[1][4] * dxby2[4] + pt[2][4] * dxby2[3] + pt[3][4] * dxby2[2] + pt[4][4] * dxby2[1] - pt[5][4] * dxby2[0]) - (dxby2[1])*(pt[6][5] - pt[0][5] * dxby2[5] + pt[1][5] * dxby2[4] + pt[2][5] * dxby2[3] + pt[3][5] * dxby2[2] + pt[4][5] * dxby2[1] - pt[5][5] * dxby2[0]) - (dxby2[2])*(pt[6][6] - pt[0][6] * dxby2[5] + pt[1][6] * dxby2[4] + pt[2][6] * dxby2[3] + pt[3][6] * dxby2[2] + pt[4][6] * dxby2[1] - pt[5][6] * dxby2[0]) - (dxby2[4])*(pt[6][0] - pt[0][0] * dxby2[5] + pt[1][0] * dxby2[4] + pt[2][0] * dxby2[3] + pt[3][0] * dxby2[2] + pt[4][0] * dxby2[1] - pt[5][0] * dxby2[0]) - pt[0][3] * dxby2[5] + pt[1][3] * dxby2[4] + pt[2][3] * dxby2[3] + pt[3][3] * dxby2[2] + pt[4][3] * dxby2[1] - pt[5][3] * dxby2[0];
		P[6][4] = (dxby2[3])*(pt[6][0] - pt[0][0] * dxby2[5] + pt[1][0] * dxby2[4] + pt[2][0] * dxby2[3] + pt[3][0] * dxby2[2] + pt[4][0] * dxby2[1] - pt[5][0] * dxby2[0]) - (dxby2[6])*(pt[6][1] - pt[0][1] * dxby2[5] + pt[1][1] * dxby2[4] + pt[2][1] * dxby2[3] + pt[3][1] * dxby2[2] + pt[4][1] * dxby2[1] - pt[5][1] * dxby2[0]) + (dxby2[5])*(pt[6][2] - pt[0][2] * dxby2[5] + pt[1][2] * dxby2[4] + pt[2][2] * dxby2[3] + pt[3][2] * dxby2[2] + pt[4][2] * dxby2[1] - pt[5][2] * dxby2[0]) + (dxby2[0])*(pt[6][3] - pt[0][3] * dxby2[5] + pt[1][3] * dxby2[4] + pt[2][3] * dxby2[3] + pt[3][3] * dxby2[2] + pt[4][3] * dxby2[1] - pt[5][3] * dxby2[0]) + (dxby2[2])*(pt[6][5] - pt[0][5] * dxby2[5] + pt[1][5] * dxby2[4] + pt[2][5] * dxby2[3] + pt[3][5] * dxby2[2] + pt[4][5] * dxby2[1] - pt[5][5] * dxby2[0]) - (dxby2[1])*(pt[6][6] - pt[0][6] * dxby2[5] + pt[1][6] * dxby2[4] + pt[2][6] * dxby2[3] + pt[3][6] * dxby2[2] + pt[4][6] * dxby2[1] - pt[5][6] * dxby2[0]) + pt[6][4] - pt[0][4] * dxby2[5] + pt[1][4] * dxby2[4] + pt[2][4] * dxby2[3] + pt[3][4] * dxby2[2] + pt[4][4] * dxby2[1] - pt[5][4] * dxby2[0];
		P[6][5] = (dxby2[6])*(pt[6][0] - pt[0][0] * dxby2[5] + pt[1][0] * dxby2[4] + pt[2][0] * dxby2[3] + pt[3][0] * dxby2[2] + pt[4][0] * dxby2[1] - pt[5][0] * dxby2[0]) + (dxby2[3])*(pt[6][1] - pt[0][1] * dxby2[5] + pt[1][1] * dxby2[4] + pt[2][1] * dxby2[3] + pt[3][1] * dxby2[2] + pt[4][1] * dxby2[1] - pt[5][1] * dxby2[0]) - (dxby2[4])*(pt[6][2] - pt[0][2] * dxby2[5] + pt[1][2] * dxby2[4] + pt[2][2] * dxby2[3] + pt[3][2] * dxby2[2] + pt[4][2] * dxby2[1] - pt[5][2] * dxby2[0]) + (dxby2[1])*(pt[6][3] - pt[0][3] * dxby2[5] + pt[1][3] * dxby2[4] + pt[2][3] * dxby2[3] + pt[3][3] * dxby2[2] + pt[4][3] * dxby2[1] - pt[5][3] * dxby2[0]) - (dxby2[2])*(pt[6][4] - pt[0][4] * dxby2[5] + pt[1][4] * dxby2[4] + pt[2][4] * dxby2[3] + pt[3][4] * dxby2[2] + pt[4][4] * dxby2[1] - pt[5][4] * dxby2[0]) + (dxby2[0])*(pt[6][6] - pt[0][6] * dxby2[5] + pt[1][6] * dxby2[4] + pt[2][6] * dxby2[3] + pt[3][6] * dxby2[2] + pt[4][6] * dxby2[1] - pt[5][6] * dxby2[0]) + pt[6][5] - pt[0][5] * dxby2[5] + pt[1][5] * dxby2[4] + pt[2][5] * dxby2[3] + pt[3][5] * dxby2[2] + pt[4][5] * dxby2[1] - pt[5][5] * dxby2[0];
		P[6][6] = (dxby2[4])*(pt[6][1] - pt[0][1] * dxby2[5] + pt[1][1] * dxby2[4] + pt[2][1] * dxby2[3] + pt[3][1] * dxby2[2] + pt[4][1] * dxby2[1] - pt[5][1] * dxby2[0]) - (dxby2[5])*(pt[6][0] - pt[0][0] * dxby2[5] + pt[1][0] * dxby2[4] + pt[2][0] * dxby2[3] + pt[3][0] * dxby2[2] + pt[4][0] * dxby2[1] - pt[5][0] * dxby2[0]) + (dxby2[3])*(pt[6][2] - pt[0][2] * dxby2[5] + pt[1][2] * dxby2[4] + pt[2][2] * dxby2[3] + pt[3][2] * dxby2[2] + pt[4][2] * dxby2[1] - pt[5][2] * dxby2[0]) + (dxby2[2])*(pt[6][3] - pt[0][3] * dxby2[5] + pt[1][3] * dxby2[4] + pt[2][3] * dxby2[3] + pt[3][3] * dxby2[2] + pt[4][3] * dxby2[1] - pt[5][3] * dxby2[0]) + (dxby2[1])*(pt[6][4] - pt[0][4] * dxby2[5] + pt[1][4] * dxby2[4] + pt[2][4] * dxby2[3] + pt[3][4] * dxby2[2] + pt[4][4] * dxby2[1] - pt[5][4] * dxby2[0]) - (dxby2[0])*(pt[6][5] - pt[0][5] * dxby2[5] + pt[1][5] * dxby2[4] + pt[2][5] * dxby2[3] + pt[3][5] * dxby2[2] + pt[4][5] * dxby2[1] - pt[5][5] * dxby2[0]) + pt[6][6] - pt[0][6] * dxby2[5] + pt[1][6] * dxby2[4] + pt[2][6] * dxby2[3] + pt[3][6] * dxby2[2] + pt[4][6] * dxby2[1] - pt[5][6] * dxby2[0];

	}

	float xt[7];
	for (int i = 0; i < 7; i++) {
		xt[i] = x[i];
	}

	x[3] += d / 2.0f * (-xt[0] * xt[4] - xt[1] * xt[5] - xt[2] * xt[6]);
	x[4] += d / 2.0f * (xt[0] * xt[3] - xt[1] * xt[6] + xt[2] * xt[5]);
	x[5] += d / 2.0f * (xt[0] * xt[6] + xt[1] * xt[3] - xt[2] * xt[4]);
	x[6] += d / 2.0f * (-xt[0] * xt[5] + xt[1] * xt[4] + xt[2] * xt[3]);

	float quatnorm = sqrtf(x[4] * x[4] + x[5] * x[5] + x[6] * x[6] + x[3]*x[3]);
	x[3] = x[3] / quatnorm;
	x[4] = x[4] / quatnorm;
	x[5] = x[5] / quatnorm;
	x[6] = x[6] / quatnorm;

	estimate->orientation_q[0] = x[4];
	estimate->orientation_q[1] = x[5];
	estimate->orientation_q[2] = x[6];
	estimate->orientation_q[3] = x[3];

	estimate->angular_velocity[0] = x[0];
	estimate->angular_velocity[1] = x[1];
	estimate->angular_velocity[2] = x[2];

	//FTLOG("%f %f %f %f", x[4], x[5], x[6], x[3]);

	//quat_to_euler(estimate->orientation_q, estimate->orientation_euler);


	
	//if (checkForNan())
	//	PRINT("Prediction Nan");
}
