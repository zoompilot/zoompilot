#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

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
void err_fun(double *nom_x, double *delta_x, double *out_6117935916544501599) {
   out_6117935916544501599[0] = delta_x[0] + nom_x[0];
   out_6117935916544501599[1] = delta_x[1] + nom_x[1];
   out_6117935916544501599[2] = delta_x[2] + nom_x[2];
   out_6117935916544501599[3] = delta_x[3] + nom_x[3];
   out_6117935916544501599[4] = delta_x[4] + nom_x[4];
   out_6117935916544501599[5] = delta_x[5] + nom_x[5];
   out_6117935916544501599[6] = delta_x[6] + nom_x[6];
   out_6117935916544501599[7] = delta_x[7] + nom_x[7];
   out_6117935916544501599[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6194813894654517410) {
   out_6194813894654517410[0] = -nom_x[0] + true_x[0];
   out_6194813894654517410[1] = -nom_x[1] + true_x[1];
   out_6194813894654517410[2] = -nom_x[2] + true_x[2];
   out_6194813894654517410[3] = -nom_x[3] + true_x[3];
   out_6194813894654517410[4] = -nom_x[4] + true_x[4];
   out_6194813894654517410[5] = -nom_x[5] + true_x[5];
   out_6194813894654517410[6] = -nom_x[6] + true_x[6];
   out_6194813894654517410[7] = -nom_x[7] + true_x[7];
   out_6194813894654517410[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2062526067607976898) {
   out_2062526067607976898[0] = 1.0;
   out_2062526067607976898[1] = 0.0;
   out_2062526067607976898[2] = 0.0;
   out_2062526067607976898[3] = 0.0;
   out_2062526067607976898[4] = 0.0;
   out_2062526067607976898[5] = 0.0;
   out_2062526067607976898[6] = 0.0;
   out_2062526067607976898[7] = 0.0;
   out_2062526067607976898[8] = 0.0;
   out_2062526067607976898[9] = 0.0;
   out_2062526067607976898[10] = 1.0;
   out_2062526067607976898[11] = 0.0;
   out_2062526067607976898[12] = 0.0;
   out_2062526067607976898[13] = 0.0;
   out_2062526067607976898[14] = 0.0;
   out_2062526067607976898[15] = 0.0;
   out_2062526067607976898[16] = 0.0;
   out_2062526067607976898[17] = 0.0;
   out_2062526067607976898[18] = 0.0;
   out_2062526067607976898[19] = 0.0;
   out_2062526067607976898[20] = 1.0;
   out_2062526067607976898[21] = 0.0;
   out_2062526067607976898[22] = 0.0;
   out_2062526067607976898[23] = 0.0;
   out_2062526067607976898[24] = 0.0;
   out_2062526067607976898[25] = 0.0;
   out_2062526067607976898[26] = 0.0;
   out_2062526067607976898[27] = 0.0;
   out_2062526067607976898[28] = 0.0;
   out_2062526067607976898[29] = 0.0;
   out_2062526067607976898[30] = 1.0;
   out_2062526067607976898[31] = 0.0;
   out_2062526067607976898[32] = 0.0;
   out_2062526067607976898[33] = 0.0;
   out_2062526067607976898[34] = 0.0;
   out_2062526067607976898[35] = 0.0;
   out_2062526067607976898[36] = 0.0;
   out_2062526067607976898[37] = 0.0;
   out_2062526067607976898[38] = 0.0;
   out_2062526067607976898[39] = 0.0;
   out_2062526067607976898[40] = 1.0;
   out_2062526067607976898[41] = 0.0;
   out_2062526067607976898[42] = 0.0;
   out_2062526067607976898[43] = 0.0;
   out_2062526067607976898[44] = 0.0;
   out_2062526067607976898[45] = 0.0;
   out_2062526067607976898[46] = 0.0;
   out_2062526067607976898[47] = 0.0;
   out_2062526067607976898[48] = 0.0;
   out_2062526067607976898[49] = 0.0;
   out_2062526067607976898[50] = 1.0;
   out_2062526067607976898[51] = 0.0;
   out_2062526067607976898[52] = 0.0;
   out_2062526067607976898[53] = 0.0;
   out_2062526067607976898[54] = 0.0;
   out_2062526067607976898[55] = 0.0;
   out_2062526067607976898[56] = 0.0;
   out_2062526067607976898[57] = 0.0;
   out_2062526067607976898[58] = 0.0;
   out_2062526067607976898[59] = 0.0;
   out_2062526067607976898[60] = 1.0;
   out_2062526067607976898[61] = 0.0;
   out_2062526067607976898[62] = 0.0;
   out_2062526067607976898[63] = 0.0;
   out_2062526067607976898[64] = 0.0;
   out_2062526067607976898[65] = 0.0;
   out_2062526067607976898[66] = 0.0;
   out_2062526067607976898[67] = 0.0;
   out_2062526067607976898[68] = 0.0;
   out_2062526067607976898[69] = 0.0;
   out_2062526067607976898[70] = 1.0;
   out_2062526067607976898[71] = 0.0;
   out_2062526067607976898[72] = 0.0;
   out_2062526067607976898[73] = 0.0;
   out_2062526067607976898[74] = 0.0;
   out_2062526067607976898[75] = 0.0;
   out_2062526067607976898[76] = 0.0;
   out_2062526067607976898[77] = 0.0;
   out_2062526067607976898[78] = 0.0;
   out_2062526067607976898[79] = 0.0;
   out_2062526067607976898[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4206235638268058655) {
   out_4206235638268058655[0] = state[0];
   out_4206235638268058655[1] = state[1];
   out_4206235638268058655[2] = state[2];
   out_4206235638268058655[3] = state[3];
   out_4206235638268058655[4] = state[4];
   out_4206235638268058655[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4206235638268058655[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4206235638268058655[7] = state[7];
   out_4206235638268058655[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2317020392373099243) {
   out_2317020392373099243[0] = 1;
   out_2317020392373099243[1] = 0;
   out_2317020392373099243[2] = 0;
   out_2317020392373099243[3] = 0;
   out_2317020392373099243[4] = 0;
   out_2317020392373099243[5] = 0;
   out_2317020392373099243[6] = 0;
   out_2317020392373099243[7] = 0;
   out_2317020392373099243[8] = 0;
   out_2317020392373099243[9] = 0;
   out_2317020392373099243[10] = 1;
   out_2317020392373099243[11] = 0;
   out_2317020392373099243[12] = 0;
   out_2317020392373099243[13] = 0;
   out_2317020392373099243[14] = 0;
   out_2317020392373099243[15] = 0;
   out_2317020392373099243[16] = 0;
   out_2317020392373099243[17] = 0;
   out_2317020392373099243[18] = 0;
   out_2317020392373099243[19] = 0;
   out_2317020392373099243[20] = 1;
   out_2317020392373099243[21] = 0;
   out_2317020392373099243[22] = 0;
   out_2317020392373099243[23] = 0;
   out_2317020392373099243[24] = 0;
   out_2317020392373099243[25] = 0;
   out_2317020392373099243[26] = 0;
   out_2317020392373099243[27] = 0;
   out_2317020392373099243[28] = 0;
   out_2317020392373099243[29] = 0;
   out_2317020392373099243[30] = 1;
   out_2317020392373099243[31] = 0;
   out_2317020392373099243[32] = 0;
   out_2317020392373099243[33] = 0;
   out_2317020392373099243[34] = 0;
   out_2317020392373099243[35] = 0;
   out_2317020392373099243[36] = 0;
   out_2317020392373099243[37] = 0;
   out_2317020392373099243[38] = 0;
   out_2317020392373099243[39] = 0;
   out_2317020392373099243[40] = 1;
   out_2317020392373099243[41] = 0;
   out_2317020392373099243[42] = 0;
   out_2317020392373099243[43] = 0;
   out_2317020392373099243[44] = 0;
   out_2317020392373099243[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2317020392373099243[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2317020392373099243[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2317020392373099243[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2317020392373099243[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2317020392373099243[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2317020392373099243[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2317020392373099243[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2317020392373099243[53] = -9.8100000000000005*dt;
   out_2317020392373099243[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2317020392373099243[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2317020392373099243[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2317020392373099243[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2317020392373099243[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2317020392373099243[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2317020392373099243[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2317020392373099243[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2317020392373099243[62] = 0;
   out_2317020392373099243[63] = 0;
   out_2317020392373099243[64] = 0;
   out_2317020392373099243[65] = 0;
   out_2317020392373099243[66] = 0;
   out_2317020392373099243[67] = 0;
   out_2317020392373099243[68] = 0;
   out_2317020392373099243[69] = 0;
   out_2317020392373099243[70] = 1;
   out_2317020392373099243[71] = 0;
   out_2317020392373099243[72] = 0;
   out_2317020392373099243[73] = 0;
   out_2317020392373099243[74] = 0;
   out_2317020392373099243[75] = 0;
   out_2317020392373099243[76] = 0;
   out_2317020392373099243[77] = 0;
   out_2317020392373099243[78] = 0;
   out_2317020392373099243[79] = 0;
   out_2317020392373099243[80] = 1;
}
void h_25(double *state, double *unused, double *out_7935216587365566452) {
   out_7935216587365566452[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2303274409466923571) {
   out_2303274409466923571[0] = 0;
   out_2303274409466923571[1] = 0;
   out_2303274409466923571[2] = 0;
   out_2303274409466923571[3] = 0;
   out_2303274409466923571[4] = 0;
   out_2303274409466923571[5] = 0;
   out_2303274409466923571[6] = 1;
   out_2303274409466923571[7] = 0;
   out_2303274409466923571[8] = 0;
}
void h_24(double *state, double *unused, double *out_8324933285601970249) {
   out_8324933285601970249[0] = state[4];
   out_8324933285601970249[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5861200008269246707) {
   out_5861200008269246707[0] = 0;
   out_5861200008269246707[1] = 0;
   out_5861200008269246707[2] = 0;
   out_5861200008269246707[3] = 0;
   out_5861200008269246707[4] = 1;
   out_5861200008269246707[5] = 0;
   out_5861200008269246707[6] = 0;
   out_5861200008269246707[7] = 0;
   out_5861200008269246707[8] = 0;
   out_5861200008269246707[9] = 0;
   out_5861200008269246707[10] = 0;
   out_5861200008269246707[11] = 0;
   out_5861200008269246707[12] = 0;
   out_5861200008269246707[13] = 0;
   out_5861200008269246707[14] = 1;
   out_5861200008269246707[15] = 0;
   out_5861200008269246707[16] = 0;
   out_5861200008269246707[17] = 0;
}
void h_30(double *state, double *unused, double *out_2689948998131531790) {
   out_2689948998131531790[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9219964750958540326) {
   out_9219964750958540326[0] = 0;
   out_9219964750958540326[1] = 0;
   out_9219964750958540326[2] = 0;
   out_9219964750958540326[3] = 0;
   out_9219964750958540326[4] = 1;
   out_9219964750958540326[5] = 0;
   out_9219964750958540326[6] = 0;
   out_9219964750958540326[7] = 0;
   out_9219964750958540326[8] = 0;
}
void h_26(double *state, double *unused, double *out_2535958217703793157) {
   out_2535958217703793157[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5607800379227724172) {
   out_5607800379227724172[0] = 0;
   out_5607800379227724172[1] = 0;
   out_5607800379227724172[2] = 0;
   out_5607800379227724172[3] = 0;
   out_5607800379227724172[4] = 0;
   out_5607800379227724172[5] = 0;
   out_5607800379227724172[6] = 0;
   out_5607800379227724172[7] = 1;
   out_5607800379227724172[8] = 0;
}
void h_27(double *state, double *unused, double *out_1247011134770884164) {
   out_1247011134770884164[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7045201439158115415) {
   out_7045201439158115415[0] = 0;
   out_7045201439158115415[1] = 0;
   out_7045201439158115415[2] = 0;
   out_7045201439158115415[3] = 1;
   out_7045201439158115415[4] = 0;
   out_7045201439158115415[5] = 0;
   out_7045201439158115415[6] = 0;
   out_7045201439158115415[7] = 0;
   out_7045201439158115415[8] = 0;
}
void h_29(double *state, double *unused, double *out_6960485611312022686) {
   out_6960485611312022686[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8716547978436619106) {
   out_8716547978436619106[0] = 0;
   out_8716547978436619106[1] = 1;
   out_8716547978436619106[2] = 0;
   out_8716547978436619106[3] = 0;
   out_8716547978436619106[4] = 0;
   out_8716547978436619106[5] = 0;
   out_8716547978436619106[6] = 0;
   out_8716547978436619106[7] = 0;
   out_8716547978436619106[8] = 0;
}
void h_28(double *state, double *unused, double *out_443436699964796796) {
   out_443436699964796796[0] = state[0];
}
void H_28(double *state, double *unused, double *out_249439695219033808) {
   out_249439695219033808[0] = 1;
   out_249439695219033808[1] = 0;
   out_249439695219033808[2] = 0;
   out_249439695219033808[3] = 0;
   out_249439695219033808[4] = 0;
   out_249439695219033808[5] = 0;
   out_249439695219033808[6] = 0;
   out_249439695219033808[7] = 0;
   out_249439695219033808[8] = 0;
}
void h_31(double *state, double *unused, double *out_3740692259993634228) {
   out_3740692259993634228[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9066794413730810792) {
   out_9066794413730810792[0] = 0;
   out_9066794413730810792[1] = 0;
   out_9066794413730810792[2] = 0;
   out_9066794413730810792[3] = 0;
   out_9066794413730810792[4] = 0;
   out_9066794413730810792[5] = 0;
   out_9066794413730810792[6] = 0;
   out_9066794413730810792[7] = 0;
   out_9066794413730810792[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6117935916544501599) {
  err_fun(nom_x, delta_x, out_6117935916544501599);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6194813894654517410) {
  inv_err_fun(nom_x, true_x, out_6194813894654517410);
}
void car_H_mod_fun(double *state, double *out_2062526067607976898) {
  H_mod_fun(state, out_2062526067607976898);
}
void car_f_fun(double *state, double dt, double *out_4206235638268058655) {
  f_fun(state,  dt, out_4206235638268058655);
}
void car_F_fun(double *state, double dt, double *out_2317020392373099243) {
  F_fun(state,  dt, out_2317020392373099243);
}
void car_h_25(double *state, double *unused, double *out_7935216587365566452) {
  h_25(state, unused, out_7935216587365566452);
}
void car_H_25(double *state, double *unused, double *out_2303274409466923571) {
  H_25(state, unused, out_2303274409466923571);
}
void car_h_24(double *state, double *unused, double *out_8324933285601970249) {
  h_24(state, unused, out_8324933285601970249);
}
void car_H_24(double *state, double *unused, double *out_5861200008269246707) {
  H_24(state, unused, out_5861200008269246707);
}
void car_h_30(double *state, double *unused, double *out_2689948998131531790) {
  h_30(state, unused, out_2689948998131531790);
}
void car_H_30(double *state, double *unused, double *out_9219964750958540326) {
  H_30(state, unused, out_9219964750958540326);
}
void car_h_26(double *state, double *unused, double *out_2535958217703793157) {
  h_26(state, unused, out_2535958217703793157);
}
void car_H_26(double *state, double *unused, double *out_5607800379227724172) {
  H_26(state, unused, out_5607800379227724172);
}
void car_h_27(double *state, double *unused, double *out_1247011134770884164) {
  h_27(state, unused, out_1247011134770884164);
}
void car_H_27(double *state, double *unused, double *out_7045201439158115415) {
  H_27(state, unused, out_7045201439158115415);
}
void car_h_29(double *state, double *unused, double *out_6960485611312022686) {
  h_29(state, unused, out_6960485611312022686);
}
void car_H_29(double *state, double *unused, double *out_8716547978436619106) {
  H_29(state, unused, out_8716547978436619106);
}
void car_h_28(double *state, double *unused, double *out_443436699964796796) {
  h_28(state, unused, out_443436699964796796);
}
void car_H_28(double *state, double *unused, double *out_249439695219033808) {
  H_28(state, unused, out_249439695219033808);
}
void car_h_31(double *state, double *unused, double *out_3740692259993634228) {
  h_31(state, unused, out_3740692259993634228);
}
void car_H_31(double *state, double *unused, double *out_9066794413730810792) {
  H_31(state, unused, out_9066794413730810792);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
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
    { "mass", car_set_mass },
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
