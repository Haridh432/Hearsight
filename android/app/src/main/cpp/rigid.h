#ifndef HSV2_RIGID_H
#define HSV2_RIGID_H

#include <cstddef>

// Rotation R (row-major) maximising sum_i q_i . (R p_i), from the
// cross-covariance S[a][b] = sum_i p_i[a] * q_i[b] (Horn's quaternion method).
void hornRotation(const double S[3][3], float R[9]);

// Rigid transform q = R p + t that best maps points p onto q (least squares).
// Returns false for fewer than 3 points.
bool fitRigid(const float (*p)[3], const float (*q)[3], size_t n, float R[9], float t[3]);

#endif  // HSV2_RIGID_H
