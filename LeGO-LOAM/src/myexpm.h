// myexpm.h

#ifndef MYEXPM_H
#define MYEXPM_H

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <vector>

namespace boost { namespace numeric { namespace ublas {

typedef matrix<double, column_major, std::vector<double>> bmatrix;
}}}
// // 解線性方程組
// extern "C" void dgesv_(int *n, int *nrhs, double *A, int *lda, int *ipiv,
//                        double *b, int *ldb, int *info);

// // 平衡矩陣
// extern "C" void dgebal_(char* JOB, int *n, double *A, 
//                         int *lda, int *ilo, int *ihi, 
//                         double *scale, int *info);

// }}}
// using namespace boost::numeric::ublas;
// using std::cout;
// using std::endl;

// typedef matrix<double, column_major, std::vector<double> > bmatrix;


namespace myexpm {

using boost::numeric::ublas::bmatrix;

// 解線性方程組的包裝函數
void lsolve(bmatrix &A, bmatrix &BX);

// 平衡矩陣的包裝函數
void balanceMatrix(bmatrix &A, std::vector<double> &scale, int &ilo, int &ihi, bool Lbalance);

// 計算矩陣的指數
void expm_higham05(bmatrix &AeA);

// Najfeld和Havel的矩陣指數計算方法
void expm_najfeld_havel(bmatrix &AeA, bool doDP);

}

#endif // MYEXPM_H
