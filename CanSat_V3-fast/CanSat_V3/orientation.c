/*
 * orientation.c
 *
 * Created: 21.06.2018 13:45:46
 *  Author: b.moczala
 */ 

#include "struct.h"
#include "orientation.h"
#include "CanSat.h"
#include <math.h>

void imuQuaternionMultiplication(quaternion_t *q1, quaternion_t *q2, quaternion_t *result){
	float A = (q1->w + q1->x) * (q2->w + q2->x);
	float B = (q1->z - q1->y) * (q2->y - q2->z);
	float C = (q1->w - q1->x) * (q2->y + q2->z);
	float D = (q1->y + q1->z) * (q2->w - q2->x);
	float E = (q1->x + q1->z) * (q2->x + q2->y);
	float F = (q1->x - q1->z) * (q2->x - q2->y);
	float G = (q1->w + q1->y) * (q2->w - q2->z);
	float H = (q1->w - q1->y) * (q2->w + q2->z);

	result->w = B + (- E - F + G + H) / 2.0f;
	result->x = A - (+ E + F + G + H) / 2.0f;
	result->y = C + (+ E - F + G - H) / 2.0f;
	result->z = D + (+ E - F - G + H) / 2.0f;
}

void imuQuaternionComputeProducts(quaternion_t *quat, quaternionProd_t *quatProd){
	quatProd->ww = quat->w * quat->w;
	quatProd->wx = quat->w * quat->x;
	quatProd->wy = quat->w * quat->y;
	quatProd->wz = quat->w * quat->z;
	quatProd->xx = quat->x * quat->x;
	quatProd->xy = quat->x * quat->y;
	quatProd->xz = quat->x * quat->z;
	quatProd->yy = quat->y * quat->y;
	quatProd->yz = quat->y * quat->z;
	quatProd->zz = quat->z * quat->z;
}

void imuTransformVectorBodyToEarth(vector_t * v, vector_t *outVector, float angle[3][3]){	//OK
	// From body frame to earth frame
	float x = angle[0][0] * v->x + angle[0][1] * v->y + angle[0][2] * v->z;
	float y = angle[1][0] * v->x + angle[1][1] * v->y + angle[1][2] * v->z;
	float z = angle[2][0] * v->x + angle[2][1] * v->y + angle[2][2] * v->z;

	outVector->x = x;
	outVector->y = -y;
	outVector->z = z;
}

void imuUpdateEulerAngles(orientation_t * orient){	// OK
	quaternionProd_t buffer;

	orient->euler.roll = atan2f(orient->rMat[2][1], orient->rMat[2][2]) * (180.0f / M_PIf);
	orient->euler.pitch = ((0.5f * M_PIf) - acosf(-orient->rMat[2][0])) * (180.0f / M_PIf);
	orient->euler.yaw = (-atan2f(orient->rMat[1][0], orient->rMat[0][0]) * (180.0f / M_PIf));
}

float GetVectorLength(vector_t inVector){	//OK
	float s1,s2,s3;

	s1 = inVector.x * inVector.x;
	s2 = inVector.y * inVector.y;
	s3 = inVector.z * inVector.z;

	return sqrtf(s1 + s2 + s3);
}

void imuInitOrientation(orientation_t * orient){	//OK
	orient->quaternion.w = 1.0f;
	orient->quaternion.x = 0.0f;
	orient->quaternion.y = 0.0f;
	orient->quaternion.z = 0.0f;

	orient->rMat[0][0] = 1.0f;
	orient->rMat[0][1] = 0.0f;
	orient->rMat[0][2] = 0.0f;

	orient->rMat[1][0] = 0.0f;
	orient->rMat[1][1] = 1.0f;
	orient->rMat[1][2] = 0.0f;

	orient->rMat[2][0] = 0.0f;
	orient->rMat[2][1] = 0.0f;
	orient->rMat[2][2] = 1.0f;

	orient->euler.pitch = 0.0f;
	orient->euler.yaw   = 0.0f;
	orient->euler.roll  = 0.0f;

}

void imuMahonyAHRSupdate(float dt,
						vector_t gyro,
						uint8_t useAcc, vector_t acc,
						uint8_t useMag, vector_t mag,
						orientation_t * orient){

	float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

	// Calculate general spin rate (rad/s)
	float spin_rate = sqrtf(SQ(gyro.x) + SQ(gyro.y) + SQ(gyro.z));

	//Errors
	float ex = 0, ey = 0, ez = 0;

	// Use measured magnetic field vector
	float recipMagNorm = SQ(mag.x) + SQ(mag.y) + SQ(mag.z);
	if (useMag && recipMagNorm > 0.01f) {
		// Normalise magnetometer measurement
		recipMagNorm = sqrtf(recipMagNorm);
		mag.x /= recipMagNorm;
		mag.y /= recipMagNorm;
		mag.z /= recipMagNorm;

		// For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
		// This way magnetic field will only affect heading and wont mess roll/pitch angles

		// (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
		// (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
		float hx = orient->rMat[0][0] * mag.x + orient->rMat[0][1] * mag.y + orient->rMat[0][2] * mag.z;
		float hy = orient->rMat[1][0] * mag.x + orient->rMat[1][1] * mag.y + orient->rMat[1][2] * mag.z;
		float bx = sqrtf(SQ(hx) + SQ(hy));

		// magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
		float ez_ef = -(hy * bx);

		// Rotate mag error vector back to BF and accumulate
		ex += orient->rMat[2][0] * ez_ef;
		ey += orient->rMat[2][1] * ez_ef;
		ez += orient->rMat[2][2] * ez_ef;
	}

	// Use measured acceleration vector
	float recipAccNorm = SQ(acc.x) + SQ(acc.y) + SQ(acc.z);
	if (useAcc && recipAccNorm > 0.01f) {
		// Normalise accelerometer measurement
		recipAccNorm = sqrtf(recipAccNorm);
		acc.x /= recipAccNorm;
		acc.y /= recipAccNorm;
		acc.z /= recipAccNorm;

		// Error is sum of cross product between estimated direction and measured direction of gravity
		ex += (acc.y * orient->rMat[2][2] - acc.z * orient->rMat[2][1]);
		ey += (acc.z * orient->rMat[2][0] - acc.x * orient->rMat[2][2]);
		ez += (acc.x * orient->rMat[2][1] - acc.y * orient->rMat[2][0]);
	}

	// Compute and apply integral feedback if enabled  <<---------------------------------------------------------dodaæ ograniczenie wartoœci ca³kowania
	//if (imuRuntimeConfig.dcm_ki > 0.0f) {
	if (1) {
		// Stop integrating if spinning beyond the certain limit
		if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
			//float32_t dcmKiGain = imuRuntimeConfig.dcm_ki;	//<<------------------------------------------------tu wstawiæ zmienny gain Ki
			float dcmKiGain = 30.0f / 10000.0f;
			integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
			integralFBy += dcmKiGain * ey * dt;
			integralFBz += dcmKiGain * ez * dt;
		}
		} else {
		integralFBx = 0.0f;    // prevent integral windup
		integralFBy = 0.0f;
		integralFBz = 0.0f;
	}

	// Calculate kP gain. If we are acquiring initial attitude (not armed and within 20 sec from powerup) scale the kP to converge faster
	//float dcmKpGain = imuRuntimeConfig.dcm_kp * imuGetPGainScaleFactor(); //<<------------------------------------------------tu wstawiæ zmienny gain Kp
	float dcmKpGain = (2500.0f / 10000.0f) * 10.0f;

	// Apply proportional and integral feedback
	gyro.x += dcmKpGain * ex + integralFBx;
	gyro.y += dcmKpGain * ey + integralFBy;
	gyro.z += dcmKpGain * ez + integralFBz;

	// Integrate rate of change of quaternion
	gyro.x *= (0.5f * dt);
	gyro.y *= (0.5f * dt);
	gyro.z *= (0.5f * dt);

	// buforowanie starych q
	quaternion_t buffer;
	buffer.w = orient->quaternion.w;
	buffer.x = orient->quaternion.x;
	buffer.y = orient->quaternion.y;
	buffer.z = orient->quaternion.z;

	orient->quaternion.w += (-buffer.x * gyro.x - buffer.y * gyro.y - buffer.z * gyro.z);
	orient->quaternion.x += (+buffer.w * gyro.x + buffer.y * gyro.z - buffer.z * gyro.y);
	orient->quaternion.y += (+buffer.w * gyro.y - buffer.x * gyro.z + buffer.z * gyro.x);
	orient->quaternion.z += (+buffer.w * gyro.z + buffer.x * gyro.y - buffer.y * gyro.x);

	// Normalise quaternion
	float recipNorm = sqrtf(SQ(orient->quaternion.w) + SQ(orient->quaternion.x) + SQ(orient->quaternion.y) + SQ(orient->quaternion.z));
	orient->quaternion.w /= recipNorm;
	orient->quaternion.x /= recipNorm;
	orient->quaternion.y /= recipNorm;
	orient->quaternion.z /= recipNorm;

	// Pre-compute rotation matrix from quaternion
	imuComputeRotationMatrix(orient);
}

void imuComputeRotationMatrix(orientation_t * orient){
	quaternionProd_t qP;
	imuQuaternionComputeProducts(&(orient->quaternion), &qP);

	orient->rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
	orient->rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
	orient->rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

	orient->rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
	orient->rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
	orient->rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

	orient->rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
	orient->rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
	orient->rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
}

void imuCalculateAcceleration(SensorsData_t * sensor, orientation_t * orient, Inertial_t * kin){
	vector_t accel_ned;
	accel_ned.x = sensor->accel_x;
	accel_ned.y = sensor->accel_y;
	accel_ned.z = sensor->accel_z;

	imuTransformVectorBodyToEarth(&accel_ned, &accel_ned, orient->rMat);

	accel_ned.z -= 1.0;	//usuniêcie gravitacji

	kin->accelX_ng = accel_ned.x;
	kin->accelY_ng = accel_ned.y;
	kin->accelZ_ng = accel_ned.z;
}

void Orientation_CalcKinematics(Inertial_t * kin, float deltaT, orientation_t * orient, SensorsData_t * sensor){
	imuCalculateAcceleration(sensor, orient, kin);
}


void Orientation_CalcOrientation(SensorsData_t * sensor, orientation_t * orient, flightState_t flightState){
	vector_t accData;
	vector_t gyroData;
	vector_t magData;

	//---------------------------- Przepisanie pomiarów na zmienne lokalne ---------------------------
	accData.x = sensor->accel_x;
	accData.y = sensor->accel_y;
	accData.z = sensor->accel_z;

	// convert gyro data to radians
	gyroData.x = DEGREES_TO_RADIANS(sensor->gyro_x);
	gyroData.y = DEGREES_TO_RADIANS(sensor->gyro_y);
	gyroData.z = DEGREES_TO_RADIANS(sensor->gyro_z);

	magData.x = sensor->mag_x;
	magData.y = sensor->mag_y;
	magData.z = sensor->mag_z;

	//Flaga sygnalizuj¹ce u¿ycie danego czujnika do obliczeñ
	uint8_t useAcc = 1;
	uint8_t useMag = 1;

	//-------------------------- Uwzglêdnienie stanów lotu --------------------------------------------
	/* TODO Dodaæ wp³yw stanów lotu na parametry liczenia orientacji */

// 	switch(flightState){
// 		case STARTUP:
// 		//wysokie wzmocnienie, w³¹czone wszystkie czujniki
// 		break;
// 
// 		case INTERNAL_CHECK:
// 		case CALIBRATION:
// 		case PREFLIGHT:
// 		case PREPARATION_ERROR:
// 		//nisie wzmocnienie, wy³¹czony akcelerometr
// 		break;
// 
// 		case ME_STARTUP:
// 		case ME_ACCELERATING:
// 		case ME_ERROR:
// 		case MECO:
// 		case MECO_ERROR:
// 		case FREEFLIGH:
// 		case SE_STARTUP:
// 		case SE_ACCELRATING:
// 		case SE_ERROR:
// 		case SECO:
// 		case SECO_ERROR:
// 		case FREEFLIGHT2:
// 		case APOGEE:
// 		case FREEFALL:
// 		case DRAGSHUTE_DEPLOY:
// 		case DRAGSHUTE_ERROR:
// 		case MAINSHUTE_DEPLOY:
// 		//œrednie wzmocnienie, wy³¹czony akcelerometr
// 		break;
// 
// 		case DRAGSHUTE_FALL:
// 		case MAINSHUTE_FALL:
// 		//œrednie wzmocnienie, wszystkie czujniki w³¹czone
// 		break;
// 
// 		case LANDING:
// 		//niskie wzmocnienie, wszystkie czujniki w³¹czona
// 		break;
// 
// 		case ABORT:
// 		default:
// 		break;
// 	}

	//-------------------------- Obliczanie orientacji ------------------------------------------------
	imuMahonyAHRSupdate(1.0/sampling_rate,		//czas kroku
	gyroData,			//dane z ¿yroskopu
	useAcc, accData,	//dane z akcelerometru
	useMag, magData,	//dane z magnetometru
	orient);			//dotychczasowa orientacja

	//-------------------------- Aktualizacja k¹tów Eulera ---------------------------------------------
	imuUpdateEulerAngles(orient);
}