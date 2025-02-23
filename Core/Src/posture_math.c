#include "posture_math.h"

float dot_product(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void normalize_vector(float v[3]) {
    float mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag > 0.0f) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    }
}

float get_angle_between_vectors(float vector1[3], float vector2[3]) {
    float v1[3] = {vector1[0], vector1[1], vector1[2]};
    float v2[3] = {vector2[0], vector2[1], vector2[2]};
    normalize_vector(v1);
    normalize_vector(v2);
    float dot = dot_product(v1, v2);
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    return acosf(dot);
}

float get_magnitude_of_vector(float vector[3]) {
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
} 