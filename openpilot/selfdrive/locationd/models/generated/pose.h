#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1238705429075317501);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3460389199729958721);
void pose_H_mod_fun(double *state, double *out_5700165656230282766);
void pose_f_fun(double *state, double dt, double *out_4518422097938204125);
void pose_F_fun(double *state, double dt, double *out_794072923466735339);
void pose_h_4(double *state, double *unused, double *out_3695677144055718255);
void pose_H_4(double *state, double *unused, double *out_6454326658284995373);
void pose_h_10(double *state, double *unused, double *out_6296017263622027777);
void pose_H_10(double *state, double *unused, double *out_740468345994452368);
void pose_h_13(double *state, double *unused, double *out_8962027605949315770);
void pose_H_13(double *state, double *unused, double *out_2620571194982471349);
void pose_h_14(double *state, double *unused, double *out_3731961672478710821);
void pose_H_14(double *state, double *unused, double *out_3371538225989623077);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}