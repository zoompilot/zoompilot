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
void err_fun(double *nom_x, double *delta_x, double *out_7339197990201245439) {
   out_7339197990201245439[0] = delta_x[0] + nom_x[0];
   out_7339197990201245439[1] = delta_x[1] + nom_x[1];
   out_7339197990201245439[2] = delta_x[2] + nom_x[2];
   out_7339197990201245439[3] = delta_x[3] + nom_x[3];
   out_7339197990201245439[4] = delta_x[4] + nom_x[4];
   out_7339197990201245439[5] = delta_x[5] + nom_x[5];
   out_7339197990201245439[6] = delta_x[6] + nom_x[6];
   out_7339197990201245439[7] = delta_x[7] + nom_x[7];
   out_7339197990201245439[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6443059423535593112) {
   out_6443059423535593112[0] = -nom_x[0] + true_x[0];
   out_6443059423535593112[1] = -nom_x[1] + true_x[1];
   out_6443059423535593112[2] = -nom_x[2] + true_x[2];
   out_6443059423535593112[3] = -nom_x[3] + true_x[3];
   out_6443059423535593112[4] = -nom_x[4] + true_x[4];
   out_6443059423535593112[5] = -nom_x[5] + true_x[5];
   out_6443059423535593112[6] = -nom_x[6] + true_x[6];
   out_6443059423535593112[7] = -nom_x[7] + true_x[7];
   out_6443059423535593112[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3688601458501992314) {
   out_3688601458501992314[0] = 1.0;
   out_3688601458501992314[1] = 0.0;
   out_3688601458501992314[2] = 0.0;
   out_3688601458501992314[3] = 0.0;
   out_3688601458501992314[4] = 0.0;
   out_3688601458501992314[5] = 0.0;
   out_3688601458501992314[6] = 0.0;
   out_3688601458501992314[7] = 0.0;
   out_3688601458501992314[8] = 0.0;
   out_3688601458501992314[9] = 0.0;
   out_3688601458501992314[10] = 1.0;
   out_3688601458501992314[11] = 0.0;
   out_3688601458501992314[12] = 0.0;
   out_3688601458501992314[13] = 0.0;
   out_3688601458501992314[14] = 0.0;
   out_3688601458501992314[15] = 0.0;
   out_3688601458501992314[16] = 0.0;
   out_3688601458501992314[17] = 0.0;
   out_3688601458501992314[18] = 0.0;
   out_3688601458501992314[19] = 0.0;
   out_3688601458501992314[20] = 1.0;
   out_3688601458501992314[21] = 0.0;
   out_3688601458501992314[22] = 0.0;
   out_3688601458501992314[23] = 0.0;
   out_3688601458501992314[24] = 0.0;
   out_3688601458501992314[25] = 0.0;
   out_3688601458501992314[26] = 0.0;
   out_3688601458501992314[27] = 0.0;
   out_3688601458501992314[28] = 0.0;
   out_3688601458501992314[29] = 0.0;
   out_3688601458501992314[30] = 1.0;
   out_3688601458501992314[31] = 0.0;
   out_3688601458501992314[32] = 0.0;
   out_3688601458501992314[33] = 0.0;
   out_3688601458501992314[34] = 0.0;
   out_3688601458501992314[35] = 0.0;
   out_3688601458501992314[36] = 0.0;
   out_3688601458501992314[37] = 0.0;
   out_3688601458501992314[38] = 0.0;
   out_3688601458501992314[39] = 0.0;
   out_3688601458501992314[40] = 1.0;
   out_3688601458501992314[41] = 0.0;
   out_3688601458501992314[42] = 0.0;
   out_3688601458501992314[43] = 0.0;
   out_3688601458501992314[44] = 0.0;
   out_3688601458501992314[45] = 0.0;
   out_3688601458501992314[46] = 0.0;
   out_3688601458501992314[47] = 0.0;
   out_3688601458501992314[48] = 0.0;
   out_3688601458501992314[49] = 0.0;
   out_3688601458501992314[50] = 1.0;
   out_3688601458501992314[51] = 0.0;
   out_3688601458501992314[52] = 0.0;
   out_3688601458501992314[53] = 0.0;
   out_3688601458501992314[54] = 0.0;
   out_3688601458501992314[55] = 0.0;
   out_3688601458501992314[56] = 0.0;
   out_3688601458501992314[57] = 0.0;
   out_3688601458501992314[58] = 0.0;
   out_3688601458501992314[59] = 0.0;
   out_3688601458501992314[60] = 1.0;
   out_3688601458501992314[61] = 0.0;
   out_3688601458501992314[62] = 0.0;
   out_3688601458501992314[63] = 0.0;
   out_3688601458501992314[64] = 0.0;
   out_3688601458501992314[65] = 0.0;
   out_3688601458501992314[66] = 0.0;
   out_3688601458501992314[67] = 0.0;
   out_3688601458501992314[68] = 0.0;
   out_3688601458501992314[69] = 0.0;
   out_3688601458501992314[70] = 1.0;
   out_3688601458501992314[71] = 0.0;
   out_3688601458501992314[72] = 0.0;
   out_3688601458501992314[73] = 0.0;
   out_3688601458501992314[74] = 0.0;
   out_3688601458501992314[75] = 0.0;
   out_3688601458501992314[76] = 0.0;
   out_3688601458501992314[77] = 0.0;
   out_3688601458501992314[78] = 0.0;
   out_3688601458501992314[79] = 0.0;
   out_3688601458501992314[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8584571223880546781) {
   out_8584571223880546781[0] = state[0];
   out_8584571223880546781[1] = state[1];
   out_8584571223880546781[2] = state[2];
   out_8584571223880546781[3] = state[3];
   out_8584571223880546781[4] = state[4];
   out_8584571223880546781[5] = dt*(stiffness_front*(-state[2] - state[3] + state[7])*state[0]/state[1] + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/state[4] + ((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4])*state[6] - 9.8100000000000005*state[8]) + state[5];
   out_8584571223880546781[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8584571223880546781[7] = state[7];
   out_8584571223880546781[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1273353521868428473) {
   out_1273353521868428473[0] = 1;
   out_1273353521868428473[1] = 0;
   out_1273353521868428473[2] = 0;
   out_1273353521868428473[3] = 0;
   out_1273353521868428473[4] = 0;
   out_1273353521868428473[5] = 0;
   out_1273353521868428473[6] = 0;
   out_1273353521868428473[7] = 0;
   out_1273353521868428473[8] = 0;
   out_1273353521868428473[9] = 0;
   out_1273353521868428473[10] = 1;
   out_1273353521868428473[11] = 0;
   out_1273353521868428473[12] = 0;
   out_1273353521868428473[13] = 0;
   out_1273353521868428473[14] = 0;
   out_1273353521868428473[15] = 0;
   out_1273353521868428473[16] = 0;
   out_1273353521868428473[17] = 0;
   out_1273353521868428473[18] = 0;
   out_1273353521868428473[19] = 0;
   out_1273353521868428473[20] = 1;
   out_1273353521868428473[21] = 0;
   out_1273353521868428473[22] = 0;
   out_1273353521868428473[23] = 0;
   out_1273353521868428473[24] = 0;
   out_1273353521868428473[25] = 0;
   out_1273353521868428473[26] = 0;
   out_1273353521868428473[27] = 0;
   out_1273353521868428473[28] = 0;
   out_1273353521868428473[29] = 0;
   out_1273353521868428473[30] = 1;
   out_1273353521868428473[31] = 0;
   out_1273353521868428473[32] = 0;
   out_1273353521868428473[33] = 0;
   out_1273353521868428473[34] = 0;
   out_1273353521868428473[35] = 0;
   out_1273353521868428473[36] = 0;
   out_1273353521868428473[37] = 0;
   out_1273353521868428473[38] = 0;
   out_1273353521868428473[39] = 0;
   out_1273353521868428473[40] = 1;
   out_1273353521868428473[41] = 0;
   out_1273353521868428473[42] = 0;
   out_1273353521868428473[43] = 0;
   out_1273353521868428473[44] = 0;
   out_1273353521868428473[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/state[1] + (-stiffness_front - stiffness_rear)*state[5]/state[4] + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/state[4]);
   out_1273353521868428473[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/pow(state[1], 2);
   out_1273353521868428473[47] = -dt*stiffness_front*state[0]/state[1];
   out_1273353521868428473[48] = -dt*stiffness_front*state[0]/state[1];
   out_1273353521868428473[49] = dt*(-(-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/pow(state[4], 2) + (-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/pow(state[4], 2) - 1)*state[6]);
   out_1273353521868428473[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/state[4] + 1;
   out_1273353521868428473[51] = dt*((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4]);
   out_1273353521868428473[52] = dt*stiffness_front*state[0]/state[1];
   out_1273353521868428473[53] = -9.8100000000000005*dt;
   out_1273353521868428473[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1273353521868428473[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1273353521868428473[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1273353521868428473[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1273353521868428473[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1273353521868428473[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1273353521868428473[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1273353521868428473[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1273353521868428473[62] = 0;
   out_1273353521868428473[63] = 0;
   out_1273353521868428473[64] = 0;
   out_1273353521868428473[65] = 0;
   out_1273353521868428473[66] = 0;
   out_1273353521868428473[67] = 0;
   out_1273353521868428473[68] = 0;
   out_1273353521868428473[69] = 0;
   out_1273353521868428473[70] = 1;
   out_1273353521868428473[71] = 0;
   out_1273353521868428473[72] = 0;
   out_1273353521868428473[73] = 0;
   out_1273353521868428473[74] = 0;
   out_1273353521868428473[75] = 0;
   out_1273353521868428473[76] = 0;
   out_1273353521868428473[77] = 0;
   out_1273353521868428473[78] = 0;
   out_1273353521868428473[79] = 0;
   out_1273353521868428473[80] = 1;
}
void h_25(double *state, double *unused, double *out_1943557172694771114) {
   out_1943557172694771114[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8261119655317529351) {
   out_8261119655317529351[0] = 0;
   out_8261119655317529351[1] = 0;
   out_8261119655317529351[2] = 0;
   out_8261119655317529351[3] = 0;
   out_8261119655317529351[4] = 0;
   out_8261119655317529351[5] = 0;
   out_8261119655317529351[6] = 1;
   out_8261119655317529351[7] = 0;
   out_8261119655317529351[8] = 0;
}
void h_24(double *state, double *unused, double *out_3447843149587576073) {
   out_3447843149587576073[0] = state[4];
   out_3447843149587576073[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3392304790289822499) {
   out_3392304790289822499[0] = 0;
   out_3392304790289822499[1] = 0;
   out_3392304790289822499[2] = 0;
   out_3392304790289822499[3] = 0;
   out_3392304790289822499[4] = 1;
   out_3392304790289822499[5] = 0;
   out_3392304790289822499[6] = 0;
   out_3392304790289822499[7] = 0;
   out_3392304790289822499[8] = 0;
   out_3392304790289822499[9] = 0;
   out_3392304790289822499[10] = 0;
   out_3392304790289822499[11] = 0;
   out_3392304790289822499[12] = 0;
   out_3392304790289822499[13] = 0;
   out_3392304790289822499[14] = 1;
   out_3392304790289822499[15] = 0;
   out_3392304790289822499[16] = 0;
   out_3392304790289822499[17] = 0;
}
void h_30(double *state, double *unused, double *out_8006526219751040510) {
   out_8006526219751040510[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3733423325189921153) {
   out_3733423325189921153[0] = 0;
   out_3733423325189921153[1] = 0;
   out_3733423325189921153[2] = 0;
   out_3733423325189921153[3] = 0;
   out_3733423325189921153[4] = 1;
   out_3733423325189921153[5] = 0;
   out_3733423325189921153[6] = 0;
   out_3733423325189921153[7] = 0;
   out_3733423325189921153[8] = 0;
}
void h_26(double *state, double *unused, double *out_789783244514956630) {
   out_789783244514956630[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4519616336443473127) {
   out_4519616336443473127[0] = 0;
   out_4519616336443473127[1] = 0;
   out_4519616336443473127[2] = 0;
   out_4519616336443473127[3] = 0;
   out_4519616336443473127[4] = 0;
   out_4519616336443473127[5] = 0;
   out_4519616336443473127[6] = 0;
   out_4519616336443473127[7] = 1;
   out_4519616336443473127[8] = 0;
}
void h_27(double *state, double *unused, double *out_9068043466586079689) {
   out_9068043466586079689[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5957017396373864370) {
   out_5957017396373864370[0] = 0;
   out_5957017396373864370[1] = 0;
   out_5957017396373864370[2] = 0;
   out_5957017396373864370[3] = 1;
   out_5957017396373864370[4] = 0;
   out_5957017396373864370[5] = 0;
   out_5957017396373864370[6] = 0;
   out_5957017396373864370[7] = 0;
   out_5957017396373864370[8] = 0;
}
void h_29(double *state, double *unused, double *out_1607484702337191364) {
   out_1607484702337191364[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4243654669504313337) {
   out_4243654669504313337[0] = 0;
   out_4243654669504313337[1] = 1;
   out_4243654669504313337[2] = 0;
   out_4243654669504313337[3] = 0;
   out_4243654669504313337[4] = 0;
   out_4243654669504313337[5] = 0;
   out_4243654669504313337[6] = 0;
   out_4243654669504313337[7] = 0;
   out_4243654669504313337[8] = 0;
}
void h_28(double *state, double *unused, double *out_886620595116332338) {
   out_886620595116332338[0] = state[0];
}
void H_28(double *state, double *unused, double *out_838744347565217237) {
   out_838744347565217237[0] = 1;
   out_838744347565217237[1] = 0;
   out_838744347565217237[2] = 0;
   out_838744347565217237[3] = 0;
   out_838744347565217237[4] = 0;
   out_838744347565217237[5] = 0;
   out_838744347565217237[6] = 0;
   out_838744347565217237[7] = 0;
   out_838744347565217237[8] = 0;
}
void h_31(double *state, double *unused, double *out_2218751234979277003) {
   out_2218751234979277003[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8291765617194489779) {
   out_8291765617194489779[0] = 0;
   out_8291765617194489779[1] = 0;
   out_8291765617194489779[2] = 0;
   out_8291765617194489779[3] = 0;
   out_8291765617194489779[4] = 0;
   out_8291765617194489779[5] = 0;
   out_8291765617194489779[6] = 0;
   out_8291765617194489779[7] = 0;
   out_8291765617194489779[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7339197990201245439) {
  err_fun(nom_x, delta_x, out_7339197990201245439);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6443059423535593112) {
  inv_err_fun(nom_x, true_x, out_6443059423535593112);
}
void car_H_mod_fun(double *state, double *out_3688601458501992314) {
  H_mod_fun(state, out_3688601458501992314);
}
void car_f_fun(double *state, double dt, double *out_8584571223880546781) {
  f_fun(state,  dt, out_8584571223880546781);
}
void car_F_fun(double *state, double dt, double *out_1273353521868428473) {
  F_fun(state,  dt, out_1273353521868428473);
}
void car_h_25(double *state, double *unused, double *out_1943557172694771114) {
  h_25(state, unused, out_1943557172694771114);
}
void car_H_25(double *state, double *unused, double *out_8261119655317529351) {
  H_25(state, unused, out_8261119655317529351);
}
void car_h_24(double *state, double *unused, double *out_3447843149587576073) {
  h_24(state, unused, out_3447843149587576073);
}
void car_H_24(double *state, double *unused, double *out_3392304790289822499) {
  H_24(state, unused, out_3392304790289822499);
}
void car_h_30(double *state, double *unused, double *out_8006526219751040510) {
  h_30(state, unused, out_8006526219751040510);
}
void car_H_30(double *state, double *unused, double *out_3733423325189921153) {
  H_30(state, unused, out_3733423325189921153);
}
void car_h_26(double *state, double *unused, double *out_789783244514956630) {
  h_26(state, unused, out_789783244514956630);
}
void car_H_26(double *state, double *unused, double *out_4519616336443473127) {
  H_26(state, unused, out_4519616336443473127);
}
void car_h_27(double *state, double *unused, double *out_9068043466586079689) {
  h_27(state, unused, out_9068043466586079689);
}
void car_H_27(double *state, double *unused, double *out_5957017396373864370) {
  H_27(state, unused, out_5957017396373864370);
}
void car_h_29(double *state, double *unused, double *out_1607484702337191364) {
  h_29(state, unused, out_1607484702337191364);
}
void car_H_29(double *state, double *unused, double *out_4243654669504313337) {
  H_29(state, unused, out_4243654669504313337);
}
void car_h_28(double *state, double *unused, double *out_886620595116332338) {
  h_28(state, unused, out_886620595116332338);
}
void car_H_28(double *state, double *unused, double *out_838744347565217237) {
  H_28(state, unused, out_838744347565217237);
}
void car_h_31(double *state, double *unused, double *out_2218751234979277003) {
  h_31(state, unused, out_2218751234979277003);
}
void car_H_31(double *state, double *unused, double *out_8291765617194489779) {
  H_31(state, unused, out_8291765617194489779);
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
