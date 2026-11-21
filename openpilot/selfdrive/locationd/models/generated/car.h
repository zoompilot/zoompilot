#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7339197990201245439);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6443059423535593112);
void car_H_mod_fun(double *state, double *out_3688601458501992314);
void car_f_fun(double *state, double dt, double *out_8584571223880546781);
void car_F_fun(double *state, double dt, double *out_1273353521868428473);
void car_h_25(double *state, double *unused, double *out_1943557172694771114);
void car_H_25(double *state, double *unused, double *out_8261119655317529351);
void car_h_24(double *state, double *unused, double *out_3447843149587576073);
void car_H_24(double *state, double *unused, double *out_3392304790289822499);
void car_h_30(double *state, double *unused, double *out_8006526219751040510);
void car_H_30(double *state, double *unused, double *out_3733423325189921153);
void car_h_26(double *state, double *unused, double *out_789783244514956630);
void car_H_26(double *state, double *unused, double *out_4519616336443473127);
void car_h_27(double *state, double *unused, double *out_9068043466586079689);
void car_H_27(double *state, double *unused, double *out_5957017396373864370);
void car_h_29(double *state, double *unused, double *out_1607484702337191364);
void car_H_29(double *state, double *unused, double *out_4243654669504313337);
void car_h_28(double *state, double *unused, double *out_886620595116332338);
void car_H_28(double *state, double *unused, double *out_838744347565217237);
void car_h_31(double *state, double *unused, double *out_2218751234979277003);
void car_H_31(double *state, double *unused, double *out_8291765617194489779);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}