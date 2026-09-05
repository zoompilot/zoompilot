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
void car_err_fun(double *nom_x, double *delta_x, double *out_6117935916544501599);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6194813894654517410);
void car_H_mod_fun(double *state, double *out_2062526067607976898);
void car_f_fun(double *state, double dt, double *out_4206235638268058655);
void car_F_fun(double *state, double dt, double *out_2317020392373099243);
void car_h_25(double *state, double *unused, double *out_7935216587365566452);
void car_H_25(double *state, double *unused, double *out_2303274409466923571);
void car_h_24(double *state, double *unused, double *out_8324933285601970249);
void car_H_24(double *state, double *unused, double *out_5861200008269246707);
void car_h_30(double *state, double *unused, double *out_2689948998131531790);
void car_H_30(double *state, double *unused, double *out_9219964750958540326);
void car_h_26(double *state, double *unused, double *out_2535958217703793157);
void car_H_26(double *state, double *unused, double *out_5607800379227724172);
void car_h_27(double *state, double *unused, double *out_1247011134770884164);
void car_H_27(double *state, double *unused, double *out_7045201439158115415);
void car_h_29(double *state, double *unused, double *out_6960485611312022686);
void car_H_29(double *state, double *unused, double *out_8716547978436619106);
void car_h_28(double *state, double *unused, double *out_443436699964796796);
void car_H_28(double *state, double *unused, double *out_249439695219033808);
void car_h_31(double *state, double *unused, double *out_3740692259993634228);
void car_H_31(double *state, double *unused, double *out_9066794413730810792);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}