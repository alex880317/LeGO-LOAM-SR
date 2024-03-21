//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: MeaCov2C_initialize.cpp
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 27-Feb-2024 15:53:12
//

// Include Files
#include "MeaCov2C_initialize.h"
#include "MeaCov2C_data.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void MeaCov2C_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_MeaCov2C = true;
}

//
// File trailer for MeaCov2C_initialize.cpp
//
// [EOF]
//
