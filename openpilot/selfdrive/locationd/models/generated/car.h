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
void car_err_fun(double *nom_x, double *delta_x, double *out_1109859493199596240);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5396602859633922124);
void car_H_mod_fun(double *state, double *out_3654488737269418491);
void car_f_fun(double *state, double dt, double *out_2693061884706349106);
void car_F_fun(double *state, double dt, double *out_4793267624379730768);
void car_h_25(double *state, double *unused, double *out_7799640176173082715);
void car_H_25(double *state, double *unused, double *out_3149154295069121779);
void car_h_24(double *state, double *unused, double *out_7319244490573123957);
void car_H_24(double *state, double *unused, double *out_3426417511522396322);
void car_h_30(double *state, double *unused, double *out_1885907149502334342);
void car_H_30(double *state, double *unused, double *out_3767536046422494976);
void car_h_26(double *state, double *unused, double *out_3115766118158511303);
void car_H_26(double *state, double *unused, double *out_6890657613943178003);
void car_h_27(double *state, double *unused, double *out_1400598658759233569);
void car_H_27(double *state, double *unused, double *out_1592772734622070065);
void car_h_29(double *state, double *unused, double *out_7096062022504506423);
void car_H_29(double *state, double *unused, double *out_4277767390736887160);
void car_h_28(double *state, double *unused, double *out_2003914543435711086);
void car_H_28(double *state, double *unused, double *out_804631626332643414);
void car_h_31(double *state, double *unused, double *out_4162190647919060365);
void car_H_31(double *state, double *unused, double *out_3118508333192161351);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}