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
void err_fun(double *nom_x, double *delta_x, double *out_2894113938990965662) {
   out_2894113938990965662[0] = delta_x[0] + nom_x[0];
   out_2894113938990965662[1] = delta_x[1] + nom_x[1];
   out_2894113938990965662[2] = delta_x[2] + nom_x[2];
   out_2894113938990965662[3] = delta_x[3] + nom_x[3];
   out_2894113938990965662[4] = delta_x[4] + nom_x[4];
   out_2894113938990965662[5] = delta_x[5] + nom_x[5];
   out_2894113938990965662[6] = delta_x[6] + nom_x[6];
   out_2894113938990965662[7] = delta_x[7] + nom_x[7];
   out_2894113938990965662[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2381022520046862243) {
   out_2381022520046862243[0] = -nom_x[0] + true_x[0];
   out_2381022520046862243[1] = -nom_x[1] + true_x[1];
   out_2381022520046862243[2] = -nom_x[2] + true_x[2];
   out_2381022520046862243[3] = -nom_x[3] + true_x[3];
   out_2381022520046862243[4] = -nom_x[4] + true_x[4];
   out_2381022520046862243[5] = -nom_x[5] + true_x[5];
   out_2381022520046862243[6] = -nom_x[6] + true_x[6];
   out_2381022520046862243[7] = -nom_x[7] + true_x[7];
   out_2381022520046862243[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8073844072421287879) {
   out_8073844072421287879[0] = 1.0;
   out_8073844072421287879[1] = 0.0;
   out_8073844072421287879[2] = 0.0;
   out_8073844072421287879[3] = 0.0;
   out_8073844072421287879[4] = 0.0;
   out_8073844072421287879[5] = 0.0;
   out_8073844072421287879[6] = 0.0;
   out_8073844072421287879[7] = 0.0;
   out_8073844072421287879[8] = 0.0;
   out_8073844072421287879[9] = 0.0;
   out_8073844072421287879[10] = 1.0;
   out_8073844072421287879[11] = 0.0;
   out_8073844072421287879[12] = 0.0;
   out_8073844072421287879[13] = 0.0;
   out_8073844072421287879[14] = 0.0;
   out_8073844072421287879[15] = 0.0;
   out_8073844072421287879[16] = 0.0;
   out_8073844072421287879[17] = 0.0;
   out_8073844072421287879[18] = 0.0;
   out_8073844072421287879[19] = 0.0;
   out_8073844072421287879[20] = 1.0;
   out_8073844072421287879[21] = 0.0;
   out_8073844072421287879[22] = 0.0;
   out_8073844072421287879[23] = 0.0;
   out_8073844072421287879[24] = 0.0;
   out_8073844072421287879[25] = 0.0;
   out_8073844072421287879[26] = 0.0;
   out_8073844072421287879[27] = 0.0;
   out_8073844072421287879[28] = 0.0;
   out_8073844072421287879[29] = 0.0;
   out_8073844072421287879[30] = 1.0;
   out_8073844072421287879[31] = 0.0;
   out_8073844072421287879[32] = 0.0;
   out_8073844072421287879[33] = 0.0;
   out_8073844072421287879[34] = 0.0;
   out_8073844072421287879[35] = 0.0;
   out_8073844072421287879[36] = 0.0;
   out_8073844072421287879[37] = 0.0;
   out_8073844072421287879[38] = 0.0;
   out_8073844072421287879[39] = 0.0;
   out_8073844072421287879[40] = 1.0;
   out_8073844072421287879[41] = 0.0;
   out_8073844072421287879[42] = 0.0;
   out_8073844072421287879[43] = 0.0;
   out_8073844072421287879[44] = 0.0;
   out_8073844072421287879[45] = 0.0;
   out_8073844072421287879[46] = 0.0;
   out_8073844072421287879[47] = 0.0;
   out_8073844072421287879[48] = 0.0;
   out_8073844072421287879[49] = 0.0;
   out_8073844072421287879[50] = 1.0;
   out_8073844072421287879[51] = 0.0;
   out_8073844072421287879[52] = 0.0;
   out_8073844072421287879[53] = 0.0;
   out_8073844072421287879[54] = 0.0;
   out_8073844072421287879[55] = 0.0;
   out_8073844072421287879[56] = 0.0;
   out_8073844072421287879[57] = 0.0;
   out_8073844072421287879[58] = 0.0;
   out_8073844072421287879[59] = 0.0;
   out_8073844072421287879[60] = 1.0;
   out_8073844072421287879[61] = 0.0;
   out_8073844072421287879[62] = 0.0;
   out_8073844072421287879[63] = 0.0;
   out_8073844072421287879[64] = 0.0;
   out_8073844072421287879[65] = 0.0;
   out_8073844072421287879[66] = 0.0;
   out_8073844072421287879[67] = 0.0;
   out_8073844072421287879[68] = 0.0;
   out_8073844072421287879[69] = 0.0;
   out_8073844072421287879[70] = 1.0;
   out_8073844072421287879[71] = 0.0;
   out_8073844072421287879[72] = 0.0;
   out_8073844072421287879[73] = 0.0;
   out_8073844072421287879[74] = 0.0;
   out_8073844072421287879[75] = 0.0;
   out_8073844072421287879[76] = 0.0;
   out_8073844072421287879[77] = 0.0;
   out_8073844072421287879[78] = 0.0;
   out_8073844072421287879[79] = 0.0;
   out_8073844072421287879[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1835070724140985802) {
   out_1835070724140985802[0] = state[0];
   out_1835070724140985802[1] = state[1];
   out_1835070724140985802[2] = state[2];
   out_1835070724140985802[3] = state[3];
   out_1835070724140985802[4] = state[4];
   out_1835070724140985802[5] = dt*(stiffness_front*(-state[2] - state[3] + state[7])*state[0]/state[1] + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/state[4] + ((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4])*state[6] - 9.8100000000000005*state[8]) + state[5];
   out_1835070724140985802[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1835070724140985802[7] = state[7];
   out_1835070724140985802[8] = state[8];
}
void F_fun(double *state, double dt, double *out_672615182746371325) {
   out_672615182746371325[0] = 1;
   out_672615182746371325[1] = 0;
   out_672615182746371325[2] = 0;
   out_672615182746371325[3] = 0;
   out_672615182746371325[4] = 0;
   out_672615182746371325[5] = 0;
   out_672615182746371325[6] = 0;
   out_672615182746371325[7] = 0;
   out_672615182746371325[8] = 0;
   out_672615182746371325[9] = 0;
   out_672615182746371325[10] = 1;
   out_672615182746371325[11] = 0;
   out_672615182746371325[12] = 0;
   out_672615182746371325[13] = 0;
   out_672615182746371325[14] = 0;
   out_672615182746371325[15] = 0;
   out_672615182746371325[16] = 0;
   out_672615182746371325[17] = 0;
   out_672615182746371325[18] = 0;
   out_672615182746371325[19] = 0;
   out_672615182746371325[20] = 1;
   out_672615182746371325[21] = 0;
   out_672615182746371325[22] = 0;
   out_672615182746371325[23] = 0;
   out_672615182746371325[24] = 0;
   out_672615182746371325[25] = 0;
   out_672615182746371325[26] = 0;
   out_672615182746371325[27] = 0;
   out_672615182746371325[28] = 0;
   out_672615182746371325[29] = 0;
   out_672615182746371325[30] = 1;
   out_672615182746371325[31] = 0;
   out_672615182746371325[32] = 0;
   out_672615182746371325[33] = 0;
   out_672615182746371325[34] = 0;
   out_672615182746371325[35] = 0;
   out_672615182746371325[36] = 0;
   out_672615182746371325[37] = 0;
   out_672615182746371325[38] = 0;
   out_672615182746371325[39] = 0;
   out_672615182746371325[40] = 1;
   out_672615182746371325[41] = 0;
   out_672615182746371325[42] = 0;
   out_672615182746371325[43] = 0;
   out_672615182746371325[44] = 0;
   out_672615182746371325[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/state[1] + (-stiffness_front - stiffness_rear)*state[5]/state[4] + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/state[4]);
   out_672615182746371325[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/pow(state[1], 2);
   out_672615182746371325[47] = -dt*stiffness_front*state[0]/state[1];
   out_672615182746371325[48] = -dt*stiffness_front*state[0]/state[1];
   out_672615182746371325[49] = dt*(-(-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/pow(state[4], 2) + (-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/pow(state[4], 2) - 1)*state[6]);
   out_672615182746371325[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/state[4] + 1;
   out_672615182746371325[51] = dt*((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4]);
   out_672615182746371325[52] = dt*stiffness_front*state[0]/state[1];
   out_672615182746371325[53] = -9.8100000000000005*dt;
   out_672615182746371325[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_672615182746371325[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_672615182746371325[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_672615182746371325[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_672615182746371325[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_672615182746371325[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_672615182746371325[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_672615182746371325[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_672615182746371325[62] = 0;
   out_672615182746371325[63] = 0;
   out_672615182746371325[64] = 0;
   out_672615182746371325[65] = 0;
   out_672615182746371325[66] = 0;
   out_672615182746371325[67] = 0;
   out_672615182746371325[68] = 0;
   out_672615182746371325[69] = 0;
   out_672615182746371325[70] = 1;
   out_672615182746371325[71] = 0;
   out_672615182746371325[72] = 0;
   out_672615182746371325[73] = 0;
   out_672615182746371325[74] = 0;
   out_672615182746371325[75] = 0;
   out_672615182746371325[76] = 0;
   out_672615182746371325[77] = 0;
   out_672615182746371325[78] = 0;
   out_672615182746371325[79] = 0;
   out_672615182746371325[80] = 1;
}
void h_25(double *state, double *unused, double *out_2182680073942906551) {
   out_2182680073942906551[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1119344153490949358) {
   out_1119344153490949358[0] = 0;
   out_1119344153490949358[1] = 0;
   out_1119344153490949358[2] = 0;
   out_1119344153490949358[3] = 0;
   out_1119344153490949358[4] = 0;
   out_1119344153490949358[5] = 0;
   out_1119344153490949358[6] = 1;
   out_1119344153490949358[7] = 0;
   out_1119344153490949358[8] = 0;
}
void h_24(double *state, double *unused, double *out_3573452335236708456) {
   out_3573452335236708456[0] = state[4];
   out_3573452335236708456[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3291993752496448924) {
   out_3291993752496448924[0] = 0;
   out_3291993752496448924[1] = 0;
   out_3291993752496448924[2] = 0;
   out_3291993752496448924[3] = 0;
   out_3291993752496448924[4] = 1;
   out_3291993752496448924[5] = 0;
   out_3291993752496448924[6] = 0;
   out_3291993752496448924[7] = 0;
   out_3291993752496448924[8] = 0;
   out_3291993752496448924[9] = 0;
   out_3291993752496448924[10] = 0;
   out_3291993752496448924[11] = 0;
   out_3291993752496448924[12] = 0;
   out_3291993752496448924[13] = 0;
   out_3291993752496448924[14] = 1;
   out_3291993752496448924[15] = 0;
   out_3291993752496448924[16] = 0;
   out_3291993752496448924[17] = 0;
}
void h_30(double *state, double *unused, double *out_5502112638480461076) {
   out_5502112638480461076[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5797346188000667397) {
   out_5797346188000667397[0] = 0;
   out_5797346188000667397[1] = 0;
   out_5797346188000667397[2] = 0;
   out_5797346188000667397[3] = 0;
   out_5797346188000667397[4] = 1;
   out_5797346188000667397[5] = 0;
   out_5797346188000667397[6] = 0;
   out_5797346188000667397[7] = 0;
   out_5797346188000667397[8] = 0;
}
void h_26(double *state, double *unused, double *out_7942958167568244895) {
   out_7942958167568244895[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4860847472365005582) {
   out_4860847472365005582[0] = 0;
   out_4860847472365005582[1] = 0;
   out_4860847472365005582[2] = 0;
   out_4860847472365005582[3] = 0;
   out_4860847472365005582[4] = 0;
   out_4860847472365005582[5] = 0;
   out_4860847472365005582[6] = 0;
   out_4860847472365005582[7] = 1;
   out_4860847472365005582[8] = 0;
}
void h_27(double *state, double *unused, double *out_1014936272866764814) {
   out_1014936272866764814[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3622582876200242486) {
   out_3622582876200242486[0] = 0;
   out_3622582876200242486[1] = 0;
   out_3622582876200242486[2] = 0;
   out_3622582876200242486[3] = 1;
   out_3622582876200242486[4] = 0;
   out_3622582876200242486[5] = 0;
   out_3622582876200242486[6] = 0;
   out_3622582876200242486[7] = 0;
   out_3622582876200242486[8] = 0;
}
void h_29(double *state, double *unused, double *out_7624941203851777070) {
   out_7624941203851777070[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1909220149330691453) {
   out_1909220149330691453[0] = 0;
   out_1909220149330691453[1] = 1;
   out_1909220149330691453[2] = 0;
   out_1909220149330691453[3] = 0;
   out_1909220149330691453[4] = 0;
   out_1909220149330691453[5] = 0;
   out_1909220149330691453[6] = 0;
   out_1909220149330691453[7] = 0;
   out_1909220149330691453[8] = 0;
}
void h_28(double *state, double *unused, double *out_6016609540527610459) {
   out_6016609540527610459[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3173178867738839121) {
   out_3173178867738839121[0] = 1;
   out_3173178867738839121[1] = 0;
   out_3173178867738839121[2] = 0;
   out_3173178867738839121[3] = 0;
   out_3173178867738839121[4] = 0;
   out_3173178867738839121[5] = 0;
   out_3173178867738839121[6] = 0;
   out_3173178867738839121[7] = 0;
   out_3173178867738839121[8] = 0;
}
void h_31(double *state, double *unused, double *out_2339866742294035696) {
   out_2339866742294035696[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1088698191613988930) {
   out_1088698191613988930[0] = 0;
   out_1088698191613988930[1] = 0;
   out_1088698191613988930[2] = 0;
   out_1088698191613988930[3] = 0;
   out_1088698191613988930[4] = 0;
   out_1088698191613988930[5] = 0;
   out_1088698191613988930[6] = 0;
   out_1088698191613988930[7] = 0;
   out_1088698191613988930[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2894113938990965662) {
  err_fun(nom_x, delta_x, out_2894113938990965662);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2381022520046862243) {
  inv_err_fun(nom_x, true_x, out_2381022520046862243);
}
void car_H_mod_fun(double *state, double *out_8073844072421287879) {
  H_mod_fun(state, out_8073844072421287879);
}
void car_f_fun(double *state, double dt, double *out_1835070724140985802) {
  f_fun(state,  dt, out_1835070724140985802);
}
void car_F_fun(double *state, double dt, double *out_672615182746371325) {
  F_fun(state,  dt, out_672615182746371325);
}
void car_h_25(double *state, double *unused, double *out_2182680073942906551) {
  h_25(state, unused, out_2182680073942906551);
}
void car_H_25(double *state, double *unused, double *out_1119344153490949358) {
  H_25(state, unused, out_1119344153490949358);
}
void car_h_24(double *state, double *unused, double *out_3573452335236708456) {
  h_24(state, unused, out_3573452335236708456);
}
void car_H_24(double *state, double *unused, double *out_3291993752496448924) {
  H_24(state, unused, out_3291993752496448924);
}
void car_h_30(double *state, double *unused, double *out_5502112638480461076) {
  h_30(state, unused, out_5502112638480461076);
}
void car_H_30(double *state, double *unused, double *out_5797346188000667397) {
  H_30(state, unused, out_5797346188000667397);
}
void car_h_26(double *state, double *unused, double *out_7942958167568244895) {
  h_26(state, unused, out_7942958167568244895);
}
void car_H_26(double *state, double *unused, double *out_4860847472365005582) {
  H_26(state, unused, out_4860847472365005582);
}
void car_h_27(double *state, double *unused, double *out_1014936272866764814) {
  h_27(state, unused, out_1014936272866764814);
}
void car_H_27(double *state, double *unused, double *out_3622582876200242486) {
  H_27(state, unused, out_3622582876200242486);
}
void car_h_29(double *state, double *unused, double *out_7624941203851777070) {
  h_29(state, unused, out_7624941203851777070);
}
void car_H_29(double *state, double *unused, double *out_1909220149330691453) {
  H_29(state, unused, out_1909220149330691453);
}
void car_h_28(double *state, double *unused, double *out_6016609540527610459) {
  h_28(state, unused, out_6016609540527610459);
}
void car_H_28(double *state, double *unused, double *out_3173178867738839121) {
  H_28(state, unused, out_3173178867738839121);
}
void car_h_31(double *state, double *unused, double *out_2339866742294035696) {
  h_31(state, unused, out_2339866742294035696);
}
void car_H_31(double *state, double *unused, double *out_1088698191613988930) {
  H_31(state, unused, out_1088698191613988930);
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
