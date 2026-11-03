// Small vectors and matrices for the catch code (tracker, perception, game): 3-vectors and 3x3 rotations as arrays,
// a dynamic row-major matrix for the filter's 6x6 covariance and its measurement blocks. No allocation-heavy library:
// the same source builds in the ROS package and in test/core_check.cpp (g++ / zig c++).
#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace omni {

constexpr double kPiC = 3.14159265358979323846;

using V3 = std::array<double, 3>;
using M3 = std::array<double, 9>;           // row-major

inline V3 operator+(const V3& a, const V3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline V3 operator-(const V3& a, const V3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline V3 operator*(const V3& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
inline V3 operator*(double s, const V3& a) { return {s * a[0], s * a[1], s * a[2]}; }
inline V3 operator/(const V3& a, double s) { return {a[0] / s, a[1] / s, a[2] / s}; }
inline double dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline double norm(const V3& a) { return std::sqrt(dot(a, a)); }

inline V3 mul(const M3& R, const V3& v) {               // R @ v
    return {R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
            R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
}
inline V3 mul_t(const M3& R, const V3& v) {             // R.T @ v
    return {R[0] * v[0] + R[3] * v[1] + R[6] * v[2], R[1] * v[0] + R[4] * v[1] + R[7] * v[2],
            R[2] * v[0] + R[5] * v[1] + R[8] * v[2]};
}
inline M3 mul(const M3& A, const M3& B) {
    M3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
    return C;
}
inline double trace_t_mul(const M3& A, const M3& B) {  // trace(A.T @ B)
    double s = 0;
    for (int k = 0; k < 9; ++k) s += A[k] * B[k];
    return s;
}

// Python's a % b for floats (the result takes b's sign)
inline double py_mod(double a, double b) {
    double r = std::fmod(a, b);
    if (r != 0 && ((r < 0) != (b < 0))) r += b;
    return r;
}

struct Mat {
    int r = 0, c = 0;
    std::vector<double> a;
    Mat() = default;
    Mat(int r, int c, double v = 0.0) : r(r), c(c), a((size_t)r * c, v) {}
    static Mat eye(int n) {
        Mat m(n, n);
        for (int i = 0; i < n; ++i) m(i, i) = 1.0;
        return m;
    }
    double& operator()(int i, int j) { return a[(size_t)i * c + j]; }
    double operator()(int i, int j) const { return a[(size_t)i * c + j]; }
    bool empty() const { return a.empty(); }

    Mat t() const {
        Mat m(c, r);
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < c; ++j) m(j, i) = (*this)(i, j);
        return m;
    }
    Mat operator*(const Mat& B) const {
        Mat m(r, B.c);
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < B.c; ++j) {
                double s = 0;
                for (int k = 0; k < c; ++k) s += (*this)(i, k) * B(k, j);
                m(i, j) = s;
            }
        return m;
    }
    std::vector<double> operator*(const std::vector<double>& v) const {
        std::vector<double> o(r, 0.0);
        for (int i = 0; i < r; ++i) {
            double s = 0;
            for (int k = 0; k < c; ++k) s += (*this)(i, k) * v[k];
            o[i] = s;
        }
        return o;
    }
    Mat operator+(const Mat& B) const {
        Mat m = *this;
        for (size_t k = 0; k < a.size(); ++k) m.a[k] += B.a[k];
        return m;
    }
    Mat operator-(const Mat& B) const {
        Mat m = *this;
        for (size_t k = 0; k < a.size(); ++k) m.a[k] -= B.a[k];
        return m;
    }
    Mat operator*(double s) const {
        Mat m = *this;
        for (auto& x : m.a) x *= s;
        return m;
    }
    Mat block(int i0, int j0, int nr, int nc) const {
        Mat m(nr, nc);
        for (int i = 0; i < nr; ++i)
            for (int j = 0; j < nc; ++j) m(i, j) = (*this)(i0 + i, j0 + j);
        return m;
    }
    // inverse by Gauss-Jordan with partial pivoting (as LAPACK's LU pivots); throws if singular
    Mat inv() const {
        int n = r;
        Mat A = *this, I = eye(n);
        for (int col = 0; col < n; ++col) {
            int p = col;
            for (int i = col + 1; i < n; ++i)
                if (std::fabs(A(i, col)) > std::fabs(A(p, col))) p = i;
            if (A(p, col) == 0.0) throw std::runtime_error("singular matrix");
            if (p != col)
                for (int j = 0; j < n; ++j) std::swap(A(p, j), A(col, j)), std::swap(I(p, j), I(col, j));
            double d = A(col, col);
            for (int j = 0; j < n; ++j) A(col, j) /= d, I(col, j) /= d;
            for (int i = 0; i < n; ++i) {
                if (i == col) continue;
                double f = A(i, col);
                if (f == 0.0) continue;
                for (int j = 0; j < n; ++j) A(i, j) -= f * A(col, j), I(i, j) -= f * I(col, j);
            }
        }
        return I;
    }
    // x with A x = b (Gaussian elimination, partial pivoting)
    std::vector<double> solve(std::vector<double> b) const {
        int n = r;
        Mat A = *this;
        for (int col = 0; col < n; ++col) {
            int p = col;
            for (int i = col + 1; i < n; ++i)
                if (std::fabs(A(i, col)) > std::fabs(A(p, col))) p = i;
            if (A(p, col) == 0.0) throw std::runtime_error("singular matrix");
            if (p != col) {
                for (int j = 0; j < n; ++j) std::swap(A(p, j), A(col, j));
                std::swap(b[p], b[col]);
            }
            for (int i = col + 1; i < n; ++i) {
                double f = A(i, col) / A(col, col);
                for (int j = col; j < n; ++j) A(i, j) -= f * A(col, j);
                b[i] -= f * b[col];
            }
        }
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            double s = b[i];
            for (int j = i + 1; j < n; ++j) s -= A(i, j) * x[j];
            x[i] = s / A(i, i);
        }
        return x;
    }
};

// the larger eigenvalue of a symmetric 2x2 [[a, b], [b, d]] (np.linalg.eigvalsh(...)[-1]; numpy reads b below the diagonal)
inline double eig2_max(double a, double b, double d) {
    double m = 0.5 * (a + d), h = 0.5 * (a - d);
    return m + std::sqrt(h * h + b * b);
}

}  // namespace omni
