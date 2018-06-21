



void imuQuaternionMultiplication(quaternion_t *q1, quaternion_t *q2, quaternion_t *result){
	float32_t A = (q1->w + q1->x) * (q2->w + q2->x);
	float32_t B = (q1->z - q1->y) * (q2->y - q2->z);
	float32_t C = (q1->w - q1->x) * (q2->y + q2->z);
	float32_t D = (q1->y + q1->z) * (q2->w - q2->x);
	float32_t E = (q1->x + q1->z) * (q2->x + q2->y);
	float32_t F = (q1->x - q1->z) * (q2->x - q2->y);
	float32_t G = (q1->w + q1->y) * (q2->w - q2->z);
	float32_t H = (q1->w - q1->y) * (q2->w + q2->z);

	result->w = B + (- E - F + G + H) / 2.0f;
	result->x = A - (+ E + F + G + H) / 2.0f;
	result->y = C + (+ E - F + G - H) / 2.0f;
	result->z = D + (+ E - F - G + H) / 2.0f;
}