#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3980012004147667955);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6720818933782490088);
void pose_H_mod_fun(double *state, double *out_3217080649686513056);
void pose_f_fun(double *state, double dt, double *out_3363301103592732445);
void pose_F_fun(double *state, double dt, double *out_1157141240317607749);
void pose_h_4(double *state, double *unused, double *out_5928686857747843496);
void pose_H_4(double *state, double *unused, double *out_6988887859530420628);
void pose_h_10(double *state, double *unused, double *out_2233713730666020117);
void pose_H_10(double *state, double *unused, double *out_8215253873719460024);
void pose_h_13(double *state, double *unused, double *out_3078021598904732947);
void pose_H_13(double *state, double *unused, double *out_3847225005862430059);
void pose_h_14(double *state, double *unused, double *out_3755611037136954903);
void pose_H_14(double *state, double *unused, double *out_3906099427235048332);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}