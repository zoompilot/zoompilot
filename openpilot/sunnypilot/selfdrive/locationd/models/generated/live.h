#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7127357533330379995);
void live_err_fun(double *nom_x, double *delta_x, double *out_5733526440875031090);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5450789964659067411);
void live_H_mod_fun(double *state, double *out_1844576855017461930);
void live_f_fun(double *state, double dt, double *out_9154957731331470588);
void live_F_fun(double *state, double dt, double *out_8056420783522387589);
void live_h_4(double *state, double *unused, double *out_4415134660603741535);
void live_H_4(double *state, double *unused, double *out_6187724249356015607);
void live_h_9(double *state, double *unused, double *out_5755112323731165661);
void live_H_9(double *state, double *unused, double *out_1099494685908431863);
void live_h_10(double *state, double *unused, double *out_7003160357881100886);
void live_H_10(double *state, double *unused, double *out_7595251612948454581);
void live_h_12(double *state, double *unused, double *out_9071225180501111988);
void live_H_12(double *state, double *unused, double *out_1479404064326434885);
void live_h_35(double *state, double *unused, double *out_7226969160488583769);
void live_H_35(double *state, double *unused, double *out_2821062191983408231);
void live_h_32(double *state, double *unused, double *out_924003840371607839);
void live_H_32(double *state, double *unused, double *out_5846612876889480996);
void live_h_13(double *state, double *unused, double *out_3215338349328053209);
void live_H_13(double *state, double *unused, double *out_4665796487479573837);
void live_h_14(double *state, double *unused, double *out_5755112323731165661);
void live_H_14(double *state, double *unused, double *out_1099494685908431863);
void live_h_33(double *state, double *unused, double *out_7549954339202035725);
void live_H_33(double *state, double *unused, double *out_329494812655449373);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}