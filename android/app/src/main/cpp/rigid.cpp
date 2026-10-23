#include "rigid.h"

#include <cmath>

namespace {

// Largest-eigenvalue eigenvector of a symmetric 4x4 matrix (cyclic Jacobi).
void largestEigenvector4(double A[4][4], double out[4]) {
    double V[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        double off = 0;
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q) off += A[p][q] * A[p][q];
        if (off < 1e-18) break;
        for (int p = 0; p < 4; ++p) {
            for (int q = p + 1; q < 4; ++q) {
                if (std::fabs(A[p][q]) < 1e-15) continue;
                const double theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) /
                                 (std::fabs(theta) + std::sqrt(theta * theta + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 4; ++k) {
                    const double akp = A[k][p], akq = A[k][q];
                    A[k][p] = c * akp - s * akq;
                    A[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 4; ++k) {
                    const double apk = A[p][k], aqk = A[q][k];
                    A[p][k] = c * apk - s * aqk;
                    A[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 4; ++k) {
                    const double vkp = V[k][p], vkq = V[k][q];
                    V[k][p] = c * vkp - s * vkq;
                    V[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    int m = 0;
    for (int i = 1; i < 4; ++i)
        if (A[i][i] > A[m][m]) m = i;
    for (int k = 0; k < 4; ++k) out[k] = V[k][m];
}

}  // namespace

void hornRotation(const double S[3][3], float R[9]) {
    const double Sxx = S[0][0], Sxy = S[0][1], Sxz = S[0][2];
    const double Syx = S[1][0], Syy = S[1][1], Syz = S[1][2];
    const double Szx = S[2][0], Szy = S[2][1], Szz = S[2][2];
    double N[4][4] = {
        {Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx},
        {Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz},
        {Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy},
        {Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz}};
    double q[4];
    largestEigenvector4(N, q);
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = static_cast<float>(w * w + x * x - y * y - z * z);
    R[1] = static_cast<float>(2 * (x * y - w * z));
    R[2] = static_cast<float>(2 * (x * z + w * y));
    R[3] = static_cast<float>(2 * (x * y + w * z));
    R[4] = static_cast<float>(w * w - x * x + y * y - z * z);
    R[5] = static_cast<float>(2 * (y * z - w * x));
    R[6] = static_cast<float>(2 * (x * z - w * y));
    R[7] = static_cast<float>(2 * (y * z + w * x));
    R[8] = static_cast<float>(w * w - x * x - y * y + z * z);
}

bool fitRigid(const float (*p)[3], const float (*q)[3], size_t n, float R[9], float t[3]) {
    if (n < 3) return false;
    double pc[3] = {0, 0, 0}, qc[3] = {0, 0, 0};
    for (size_t i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k) { pc[k] += p[i][k]; qc[k] += q[i][k]; }
    for (int k = 0; k < 3; ++k) { pc[k] /= n; qc[k] /= n; }
    double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (size_t i = 0; i < n; ++i)
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) S[a][b] += (p[i][a] - pc[a]) * (q[i][b] - qc[b]);
    hornRotation(S, R);
    for (int k = 0; k < 3; ++k)
        t[k] = static_cast<float>(qc[k] - (R[k * 3] * pc[0] + R[k * 3 + 1] * pc[1] + R[k * 3 + 2] * pc[2]));
    return true;
}
