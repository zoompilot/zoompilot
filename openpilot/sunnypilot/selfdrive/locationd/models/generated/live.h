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
void live_H(double *in_vec, double *out_7913246842055150495);
void live_err_fun(double *nom_x, double *delta_x, double *out_8220292187028522364);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1155679063593018162);
void live_H_mod_fun(double *state, double *out_8071831103445423949);
void live_f_fun(double *state, double dt, double *out_8007022477658346717);
void live_F_fun(double *state, double dt, double *out_4124212876548391657);
void live_h_4(double *state, double *unused, double *out_7719679033564044774);
void live_H_4(double *state, double *unused, double *out_5818615371373158901);
void live_h_9(double *state, double *unused, double *out_4003208543120263903);
void live_H_9(double *state, double *unused, double *out_5340909767071945245);
void live_h_10(double *state, double *unused, double *out_1493995218883652641);
void live_H_10(double *state, double *unused, double *out_7824853844914814855);
void live_h_12(double *state, double *unused, double *out_7765727755675671973);
void live_H_12(double *state, double *unused, double *out_562643005669574095);
void live_h_35(double *state, double *unused, double *out_42049044738198602);
void live_H_35(double *state, double *unused, double *out_2182920026655439614);
void live_h_32(double *state, double *unused, double *out_4815101869690107855);
void live_H_32(double *state, double *unused, double *out_4966578191073857207);
void live_h_13(double *state, double *unused, double *out_5134938513956407549);
void live_H_13(double *state, double *unused, double *out_3009850677634594653);
void live_h_14(double *state, double *unused, double *out_4003208543120263903);
void live_H_14(double *state, double *unused, double *out_5340909767071945245);
void live_h_33(double *state, double *unused, double *out_6796979617862572815);
void live_H_33(double *state, double *unused, double *out_5333477031294297218);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}