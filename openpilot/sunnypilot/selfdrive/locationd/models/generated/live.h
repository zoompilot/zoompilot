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
void live_H(double *in_vec, double *out_7390718901985399711);
void live_err_fun(double *nom_x, double *delta_x, double *out_7767618672553834553);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1252517421750402912);
void live_H_mod_fun(double *state, double *out_1948677792468621013);
void live_f_fun(double *state, double dt, double *out_3983732259728233237);
void live_F_fun(double *state, double dt, double *out_49019992945217905);
void live_h_4(double *state, double *unused, double *out_1548717302932279775);
void live_H_4(double *state, double *unused, double *out_31946828515027326);
void live_h_9(double *state, double *unused, double *out_574521155003578487);
void live_H_9(double *state, double *unused, double *out_7255272106749420144);
void live_h_10(double *state, double *unused, double *out_612063108390815116);
void live_H_10(double *state, double *unused, double *out_8774573615931127055);
void live_h_12(double *state, double *unused, double *out_5312902188037404870);
void live_H_12(double *state, double *unused, double *out_4987509579516934469);
void live_h_35(double *state, double *unused, double *out_3084272713708530517);
void live_H_35(double *state, double *unused, double *out_7733072611841948178);
void live_h_32(double *state, double *unused, double *out_2759554303922909555);
void live_H_32(double *state, double *unused, double *out_4846591188993493682);
void live_h_13(double *state, double *unused, double *out_641240902070438255);
void live_H_13(double *state, double *unused, double *out_3636209941299068821);
void live_h_14(double *state, double *unused, double *out_574521155003578487);
void live_H_14(double *state, double *unused, double *out_7255272106749420144);
void live_h_33(double *state, double *unused, double *out_6155452159725340248);
void live_H_33(double *state, double *unused, double *out_7563114457228745834);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}