#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2966425499024310265);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4677272284422365493);
void pose_H_mod_fun(double *state, double *out_6471686313161485195);
void pose_f_fun(double *state, double dt, double *out_8577303503121138630);
void pose_F_fun(double *state, double dt, double *out_5893284074989477049);
void pose_h_4(double *state, double *unused, double *out_3588243949097045624);
void pose_H_4(double *state, double *unused, double *out_2472675352896336147);
void pose_h_10(double *state, double *unused, double *out_6829459391907355466);
void pose_H_10(double *state, double *unused, double *out_5498212507308944349);
void pose_h_13(double *state, double *unused, double *out_8252438701567181479);
void pose_H_13(double *state, double *unused, double *out_6306430816198860171);
void pose_h_14(double *state, double *unused, double *out_7741604625342005453);
void pose_H_14(double *state, double *unused, double *out_5555463785191708443);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}