/*
 * orientation.h
 *
 * Created: 21.06.2018 13:46:03
 *  Author: b.moczala
 */ 


#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#define SQ(x) ((x)*(x))
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)
#define M_PIf 3.14159265358979323846f
#define SPIN_RATE_LIMIT 20	//limit prêdkoœci obrotowej <- do liczenia orientacji


void imuQuaternionMultiplication(quaternion_t *q1, quaternion_t *q2, quaternion_t *result);
void imuQuaternionComputeProducts(quaternion_t *quat, quaternionProd_t *quatProd);
void imuTransformVectorBodyToEarth(vector_t * v, vector_t *outVector, float angle[3][3]);
void imuUpdateEulerAngles(orientation_t * orient);
float GetVectorLength(vector_t inVector);
void imuInitOrientation(orientation_t * orient);
void imuMahonyAHRSupdate(float dt, vector_t gyro, uint8_t useAcc, vector_t acc, uint8_t useMag, vector_t mag, orientation_t * orient);
void imuComputeRotationMatrix(orientation_t * orient);
void imuCalculateAcceleration(SensorsData_t * sensor, orientation_t * orient, Inertial_t * kin);
void Orientation_CalcKinematics(Inertial_t * kin, float deltaT, orientation_t * orient, SensorsData_t * sensor);
void Orientation_CalcOrientation(SensorsData_t * sensor, orientation_t * orient, flightState_t flightState);
#endif /* ORIENTATION_H_ */
