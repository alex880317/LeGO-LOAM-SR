//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: MeaCov2C.h
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 27-Feb-2024 15:53:12
//

#ifndef MEACOV2C_H
#define MEACOV2C_H

// Include Files
#include "MeaCov2C_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void MeaCov2C(const double P[324], const double V[4],
                     const double xtrue2err[342],
                     const struct0_T *AckermanBaseMea, double encoder_pri,
                     double theta_num, const double Ackermanparam[8],
                     double sampling_time_ack, double R[36]);

#endif
//
// File trailer for MeaCov2C.h
//
// [EOF]
//
