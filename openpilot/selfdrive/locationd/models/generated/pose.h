#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4384279722743490608);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_107855010463002620);
void pose_H_mod_fun(double *state, double *out_402663571807210546);
void pose_f_fun(double *state, double dt, double *out_2303112348537759179);
void pose_F_fun(double *state, double dt, double *out_3747527490433827778);
void pose_h_4(double *state, double *unused, double *out_8983849997392014675);
void pose_H_4(double *state, double *unused, double *out_2127501724899142132);
void pose_h_10(double *state, double *unused, double *out_7432581086548000889);
void pose_H_10(double *state, double *unused, double *out_9051257725730722421);
void pose_h_13(double *state, double *unused, double *out_537443524052514409);
void pose_H_13(double *state, double *unused, double *out_8708611140493708555);
void pose_h_14(double *state, double *unused, double *out_7069210550597015765);
void pose_H_14(double *state, double *unused, double *out_955286707396230164);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}