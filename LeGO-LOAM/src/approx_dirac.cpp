#include "approx_dirac.h" // 包含頭文件
#include <cmath>

double approx_dirac(double x) {
    const double epsilon = 1e-8; // 調整這個值以改變近似的精度和範圍
    return exp(-x * x / (2 * epsilon * epsilon)) / (sqrt(2 * M_PI) * epsilon);
}