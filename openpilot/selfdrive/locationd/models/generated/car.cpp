#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1109859493199596240) {
   out_1109859493199596240[0] = delta_x[0] + nom_x[0];
   out_1109859493199596240[1] = delta_x[1] + nom_x[1];
   out_1109859493199596240[2] = delta_x[2] + nom_x[2];
   out_1109859493199596240[3] = delta_x[3] + nom_x[3];
   out_1109859493199596240[4] = delta_x[4] + nom_x[4];
   out_1109859493199596240[5] = delta_x[5] + nom_x[5];
   out_1109859493199596240[6] = delta_x[6] + nom_x[6];
   out_1109859493199596240[7] = delta_x[7] + nom_x[7];
   out_1109859493199596240[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5396602859633922124) {
   out_5396602859633922124[0] = -nom_x[0] + true_x[0];
   out_5396602859633922124[1] = -nom_x[1] + true_x[1];
   out_5396602859633922124[2] = -nom_x[2] + true_x[2];
   out_5396602859633922124[3] = -nom_x[3] + true_x[3];
   out_5396602859633922124[4] = -nom_x[4] + true_x[4];
   out_5396602859633922124[5] = -nom_x[5] + true_x[5];
   out_5396602859633922124[6] = -nom_x[6] + true_x[6];
   out_5396602859633922124[7] = -nom_x[7] + true_x[7];
   out_5396602859633922124[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3654488737269418491) {
   out_3654488737269418491[0] = 1.0;
   out_3654488737269418491[1] = 0.0;
   out_3654488737269418491[2] = 0.0;
   out_3654488737269418491[3] = 0.0;
   out_3654488737269418491[4] = 0.0;
   out_3654488737269418491[5] = 0.0;
   out_3654488737269418491[6] = 0.0;
   out_3654488737269418491[7] = 0.0;
   out_3654488737269418491[8] = 0.0;
   out_3654488737269418491[9] = 0.0;
   out_3654488737269418491[10] = 1.0;
   out_3654488737269418491[11] = 0.0;
   out_3654488737269418491[12] = 0.0;
   out_3654488737269418491[13] = 0.0;
   out_3654488737269418491[14] = 0.0;
   out_3654488737269418491[15] = 0.0;
   out_3654488737269418491[16] = 0.0;
   out_3654488737269418491[17] = 0.0;
   out_3654488737269418491[18] = 0.0;
   out_3654488737269418491[19] = 0.0;
   out_3654488737269418491[20] = 1.0;
   out_3654488737269418491[21] = 0.0;
   out_3654488737269418491[22] = 0.0;
   out_3654488737269418491[23] = 0.0;
   out_3654488737269418491[24] = 0.0;
   out_3654488737269418491[25] = 0.0;
   out_3654488737269418491[26] = 0.0;
   out_3654488737269418491[27] = 0.0;
   out_3654488737269418491[28] = 0.0;
   out_3654488737269418491[29] = 0.0;
   out_3654488737269418491[30] = 1.0;
   out_3654488737269418491[31] = 0.0;
   out_3654488737269418491[32] = 0.0;
   out_3654488737269418491[33] = 0.0;
   out_3654488737269418491[34] = 0.0;
   out_3654488737269418491[35] = 0.0;
   out_3654488737269418491[36] = 0.0;
   out_3654488737269418491[37] = 0.0;
   out_3654488737269418491[38] = 0.0;
   out_3654488737269418491[39] = 0.0;
   out_3654488737269418491[40] = 1.0;
   out_3654488737269418491[41] = 0.0;
   out_3654488737269418491[42] = 0.0;
   out_3654488737269418491[43] = 0.0;
   out_3654488737269418491[44] = 0.0;
   out_3654488737269418491[45] = 0.0;
   out_3654488737269418491[46] = 0.0;
   out_3654488737269418491[47] = 0.0;
   out_3654488737269418491[48] = 0.0;
   out_3654488737269418491[49] = 0.0;
   out_3654488737269418491[50] = 1.0;
   out_3654488737269418491[51] = 0.0;
   out_3654488737269418491[52] = 0.0;
   out_3654488737269418491[53] = 0.0;
   out_3654488737269418491[54] = 0.0;
   out_3654488737269418491[55] = 0.0;
   out_3654488737269418491[56] = 0.0;
   out_3654488737269418491[57] = 0.0;
   out_3654488737269418491[58] = 0.0;
   out_3654488737269418491[59] = 0.0;
   out_3654488737269418491[60] = 1.0;
   out_3654488737269418491[61] = 0.0;
   out_3654488737269418491[62] = 0.0;
   out_3654488737269418491[63] = 0.0;
   out_3654488737269418491[64] = 0.0;
   out_3654488737269418491[65] = 0.0;
   out_3654488737269418491[66] = 0.0;
   out_3654488737269418491[67] = 0.0;
   out_3654488737269418491[68] = 0.0;
   out_3654488737269418491[69] = 0.0;
   out_3654488737269418491[70] = 1.0;
   out_3654488737269418491[71] = 0.0;
   out_3654488737269418491[72] = 0.0;
   out_3654488737269418491[73] = 0.0;
   out_3654488737269418491[74] = 0.0;
   out_3654488737269418491[75] = 0.0;
   out_3654488737269418491[76] = 0.0;
   out_3654488737269418491[77] = 0.0;
   out_3654488737269418491[78] = 0.0;
   out_3654488737269418491[79] = 0.0;
   out_3654488737269418491[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2693061884706349106) {
   out_2693061884706349106[0] = state[0];
   out_2693061884706349106[1] = state[1];
   out_2693061884706349106[2] = state[2];
   out_2693061884706349106[3] = state[3];
   out_2693061884706349106[4] = state[4];
   out_2693061884706349106[5] = dt*(stiffness_front*(-state[2] - state[3] + state[7])*state[0]/state[1] + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/state[4] + ((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4])*state[6] - 9.8100000000000005*state[8]) + state[5];
   out_2693061884706349106[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2693061884706349106[7] = state[7];
   out_2693061884706349106[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4793267624379730768) {
   out_4793267624379730768[0] = 1;
   out_4793267624379730768[1] = 0;
   out_4793267624379730768[2] = 0;
   out_4793267624379730768[3] = 0;
   out_4793267624379730768[4] = 0;
   out_4793267624379730768[5] = 0;
   out_4793267624379730768[6] = 0;
   out_4793267624379730768[7] = 0;
   out_4793267624379730768[8] = 0;
   out_4793267624379730768[9] = 0;
   out_4793267624379730768[10] = 1;
   out_4793267624379730768[11] = 0;
   out_4793267624379730768[12] = 0;
   out_4793267624379730768[13] = 0;
   out_4793267624379730768[14] = 0;
   out_4793267624379730768[15] = 0;
   out_4793267624379730768[16] = 0;
   out_4793267624379730768[17] = 0;
   out_4793267624379730768[18] = 0;
   out_4793267624379730768[19] = 0;
   out_4793267624379730768[20] = 1;
   out_4793267624379730768[21] = 0;
   out_4793267624379730768[22] = 0;
   out_4793267624379730768[23] = 0;
   out_4793267624379730768[24] = 0;
   out_4793267624379730768[25] = 0;
   out_4793267624379730768[26] = 0;
   out_4793267624379730768[27] = 0;
   out_4793267624379730768[28] = 0;
   out_4793267624379730768[29] = 0;
   out_4793267624379730768[30] = 1;
   out_4793267624379730768[31] = 0;
   out_4793267624379730768[32] = 0;
   out_4793267624379730768[33] = 0;
   out_4793267624379730768[34] = 0;
   out_4793267624379730768[35] = 0;
   out_4793267624379730768[36] = 0;
   out_4793267624379730768[37] = 0;
   out_4793267624379730768[38] = 0;
   out_4793267624379730768[39] = 0;
   out_4793267624379730768[40] = 1;
   out_4793267624379730768[41] = 0;
   out_4793267624379730768[42] = 0;
   out_4793267624379730768[43] = 0;
   out_4793267624379730768[44] = 0;
   out_4793267624379730768[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/state[1] + (-stiffness_front - stiffness_rear)*state[5]/state[4] + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/state[4]);
   out_4793267624379730768[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/pow(state[1], 2);
   out_4793267624379730768[47] = -dt*stiffness_front*state[0]/state[1];
   out_4793267624379730768[48] = -dt*stiffness_front*state[0]/state[1];
   out_4793267624379730768[49] = dt*(-(-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/pow(state[4], 2) + (-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/pow(state[4], 2) - 1)*state[6]);
   out_4793267624379730768[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/state[4] + 1;
   out_4793267624379730768[51] = dt*((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4]);
   out_4793267624379730768[52] = dt*stiffness_front*state[0]/state[1];
   out_4793267624379730768[53] = -9.8100000000000005*dt;
   out_4793267624379730768[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4793267624379730768[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4793267624379730768[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4793267624379730768[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4793267624379730768[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4793267624379730768[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4793267624379730768[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4793267624379730768[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4793267624379730768[62] = 0;
   out_4793267624379730768[63] = 0;
   out_4793267624379730768[64] = 0;
   out_4793267624379730768[65] = 0;
   out_4793267624379730768[66] = 0;
   out_4793267624379730768[67] = 0;
   out_4793267624379730768[68] = 0;
   out_4793267624379730768[69] = 0;
   out_4793267624379730768[70] = 1;
   out_4793267624379730768[71] = 0;
   out_4793267624379730768[72] = 0;
   out_4793267624379730768[73] = 0;
   out_4793267624379730768[74] = 0;
   out_4793267624379730768[75] = 0;
   out_4793267624379730768[76] = 0;
   out_4793267624379730768[77] = 0;
   out_4793267624379730768[78] = 0;
   out_4793267624379730768[79] = 0;
   out_4793267624379730768[80] = 1;
}
void h_25(double *state, double *unused, double *out_7799640176173082715) {
   out_7799640176173082715[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3149154295069121779) {
   out_3149154295069121779[0] = 0;
   out_3149154295069121779[1] = 0;
   out_3149154295069121779[2] = 0;
   out_3149154295069121779[3] = 0;
   out_3149154295069121779[4] = 0;
   out_3149154295069121779[5] = 0;
   out_3149154295069121779[6] = 1;
   out_3149154295069121779[7] = 0;
   out_3149154295069121779[8] = 0;
}
void h_24(double *state, double *unused, double *out_7319244490573123957) {
   out_7319244490573123957[0] = state[4];
   out_7319244490573123957[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3426417511522396322) {
   out_3426417511522396322[0] = 0;
   out_3426417511522396322[1] = 0;
   out_3426417511522396322[2] = 0;
   out_3426417511522396322[3] = 0;
   out_3426417511522396322[4] = 1;
   out_3426417511522396322[5] = 0;
   out_3426417511522396322[6] = 0;
   out_3426417511522396322[7] = 0;
   out_3426417511522396322[8] = 0;
   out_3426417511522396322[9] = 0;
   out_3426417511522396322[10] = 0;
   out_3426417511522396322[11] = 0;
   out_3426417511522396322[12] = 0;
   out_3426417511522396322[13] = 0;
   out_3426417511522396322[14] = 1;
   out_3426417511522396322[15] = 0;
   out_3426417511522396322[16] = 0;
   out_3426417511522396322[17] = 0;
}
void h_30(double *state, double *unused, double *out_1885907149502334342) {
   out_1885907149502334342[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3767536046422494976) {
   out_3767536046422494976[0] = 0;
   out_3767536046422494976[1] = 0;
   out_3767536046422494976[2] = 0;
   out_3767536046422494976[3] = 0;
   out_3767536046422494976[4] = 1;
   out_3767536046422494976[5] = 0;
   out_3767536046422494976[6] = 0;
   out_3767536046422494976[7] = 0;
   out_3767536046422494976[8] = 0;
}
void h_26(double *state, double *unused, double *out_3115766118158511303) {
   out_3115766118158511303[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6890657613943178003) {
   out_6890657613943178003[0] = 0;
   out_6890657613943178003[1] = 0;
   out_6890657613943178003[2] = 0;
   out_6890657613943178003[3] = 0;
   out_6890657613943178003[4] = 0;
   out_6890657613943178003[5] = 0;
   out_6890657613943178003[6] = 0;
   out_6890657613943178003[7] = 1;
   out_6890657613943178003[8] = 0;
}
void h_27(double *state, double *unused, double *out_1400598658759233569) {
   out_1400598658759233569[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1592772734622070065) {
   out_1592772734622070065[0] = 0;
   out_1592772734622070065[1] = 0;
   out_1592772734622070065[2] = 0;
   out_1592772734622070065[3] = 1;
   out_1592772734622070065[4] = 0;
   out_1592772734622070065[5] = 0;
   out_1592772734622070065[6] = 0;
   out_1592772734622070065[7] = 0;
   out_1592772734622070065[8] = 0;
}
void h_29(double *state, double *unused, double *out_7096062022504506423) {
   out_7096062022504506423[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4277767390736887160) {
   out_4277767390736887160[0] = 0;
   out_4277767390736887160[1] = 1;
   out_4277767390736887160[2] = 0;
   out_4277767390736887160[3] = 0;
   out_4277767390736887160[4] = 0;
   out_4277767390736887160[5] = 0;
   out_4277767390736887160[6] = 0;
   out_4277767390736887160[7] = 0;
   out_4277767390736887160[8] = 0;
}
void h_28(double *state, double *unused, double *out_2003914543435711086) {
   out_2003914543435711086[0] = state[0];
}
void H_28(double *state, double *unused, double *out_804631626332643414) {
   out_804631626332643414[0] = 1;
   out_804631626332643414[1] = 0;
   out_804631626332643414[2] = 0;
   out_804631626332643414[3] = 0;
   out_804631626332643414[4] = 0;
   out_804631626332643414[5] = 0;
   out_804631626332643414[6] = 0;
   out_804631626332643414[7] = 0;
   out_804631626332643414[8] = 0;
}
void h_31(double *state, double *unused, double *out_4162190647919060365) {
   out_4162190647919060365[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3118508333192161351) {
   out_3118508333192161351[0] = 0;
   out_3118508333192161351[1] = 0;
   out_3118508333192161351[2] = 0;
   out_3118508333192161351[3] = 0;
   out_3118508333192161351[4] = 0;
   out_3118508333192161351[5] = 0;
   out_3118508333192161351[6] = 0;
   out_3118508333192161351[7] = 0;
   out_3118508333192161351[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1109859493199596240) {
  err_fun(nom_x, delta_x, out_1109859493199596240);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5396602859633922124) {
  inv_err_fun(nom_x, true_x, out_5396602859633922124);
}
void car_H_mod_fun(double *state, double *out_3654488737269418491) {
  H_mod_fun(state, out_3654488737269418491);
}
void car_f_fun(double *state, double dt, double *out_2693061884706349106) {
  f_fun(state,  dt, out_2693061884706349106);
}
void car_F_fun(double *state, double dt, double *out_4793267624379730768) {
  F_fun(state,  dt, out_4793267624379730768);
}
void car_h_25(double *state, double *unused, double *out_7799640176173082715) {
  h_25(state, unused, out_7799640176173082715);
}
void car_H_25(double *state, double *unused, double *out_3149154295069121779) {
  H_25(state, unused, out_3149154295069121779);
}
void car_h_24(double *state, double *unused, double *out_7319244490573123957) {
  h_24(state, unused, out_7319244490573123957);
}
void car_H_24(double *state, double *unused, double *out_3426417511522396322) {
  H_24(state, unused, out_3426417511522396322);
}
void car_h_30(double *state, double *unused, double *out_1885907149502334342) {
  h_30(state, unused, out_1885907149502334342);
}
void car_H_30(double *state, double *unused, double *out_3767536046422494976) {
  H_30(state, unused, out_3767536046422494976);
}
void car_h_26(double *state, double *unused, double *out_3115766118158511303) {
  h_26(state, unused, out_3115766118158511303);
}
void car_H_26(double *state, double *unused, double *out_6890657613943178003) {
  H_26(state, unused, out_6890657613943178003);
}
void car_h_27(double *state, double *unused, double *out_1400598658759233569) {
  h_27(state, unused, out_1400598658759233569);
}
void car_H_27(double *state, double *unused, double *out_1592772734622070065) {
  H_27(state, unused, out_1592772734622070065);
}
void car_h_29(double *state, double *unused, double *out_7096062022504506423) {
  h_29(state, unused, out_7096062022504506423);
}
void car_H_29(double *state, double *unused, double *out_4277767390736887160) {
  H_29(state, unused, out_4277767390736887160);
}
void car_h_28(double *state, double *unused, double *out_2003914543435711086) {
  h_28(state, unused, out_2003914543435711086);
}
void car_H_28(double *state, double *unused, double *out_804631626332643414) {
  H_28(state, unused, out_804631626332643414);
}
void car_h_31(double *state, double *unused, double *out_4162190647919060365) {
  h_31(state, unused, out_4162190647919060365);
}
void car_H_31(double *state, double *unused, double *out_3118508333192161351) {
  H_31(state, unused, out_3118508333192161351);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
