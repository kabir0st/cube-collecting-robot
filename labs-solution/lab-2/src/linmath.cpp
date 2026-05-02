#include <linmath.hpp>
#include <cmath>

static const float DET_EPS = 1e-6f;

static float det3(const float A[9]) {
    return A[0] * (A[4] * A[8] - A[5] * A[7])
         - A[1] * (A[3] * A[8] - A[5] * A[6])
         + A[2] * (A[3] * A[7] - A[4] * A[6]);
}

bool inverse3x3(const float A[9], float Ainv[9]) {
    const float d = det3(A);
    if (std::fabs(d) < DET_EPS) return false;
    const float inv = 1.0f / d;
    Ainv[0] =  (A[4] * A[8] - A[5] * A[7]) * inv;
    Ainv[1] = -(A[1] * A[8] - A[2] * A[7]) * inv;
    Ainv[2] =  (A[1] * A[5] - A[2] * A[4]) * inv;
    Ainv[3] = -(A[3] * A[8] - A[5] * A[6]) * inv;
    Ainv[4] =  (A[0] * A[8] - A[2] * A[6]) * inv;
    Ainv[5] = -(A[0] * A[5] - A[2] * A[3]) * inv;
    Ainv[6] =  (A[3] * A[7] - A[4] * A[6]) * inv;
    Ainv[7] = -(A[0] * A[7] - A[1] * A[6]) * inv;
    Ainv[8] =  (A[0] * A[4] - A[1] * A[3]) * inv;
    return true;
}

bool solve3x3(const float A[9], const float b[3], float x[3]) {
    float Ainv[9];
    if (!inverse3x3(A, Ainv)) return false;
    x[0] = Ainv[0] * b[0] + Ainv[1] * b[1] + Ainv[2] * b[2];
    x[1] = Ainv[3] * b[0] + Ainv[4] * b[1] + Ainv[5] * b[2];
    x[2] = Ainv[6] * b[0] + Ainv[7] * b[1] + Ainv[8] * b[2];
    return true;
}
