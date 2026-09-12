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
void live_H(double *in_vec, double *out_279507552812078921);
void live_err_fun(double *nom_x, double *delta_x, double *out_7287045377931086126);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4461859850580319781);
void live_H_mod_fun(double *state, double *out_8801086248045647772);
void live_f_fun(double *state, double dt, double *out_834831074738186548);
void live_F_fun(double *state, double dt, double *out_2719023923646998880);
void live_h_4(double *state, double *unused, double *out_5407760545237060454);
void live_H_4(double *state, double *unused, double *out_4996074715023384530);
void live_h_9(double *state, double *unused, double *out_935430128420636855);
void live_H_9(double *state, double *unused, double *out_4754885068393793885);
void live_h_10(double *state, double *unused, double *out_8199568084199206038);
void live_H_10(double *state, double *unused, double *out_8780983890396670122);
void live_h_12(double *state, double *unused, double *out_4078164043252404420);
void live_H_12(double *state, double *unused, double *out_23381693008577265);
void live_h_35(double *state, double *unused, double *out_6376732515625190229);
void live_H_35(double *state, double *unused, double *out_1629412657650777154);
void live_h_32(double *state, double *unused, double *out_1093608941673163340);
void live_H_32(double *state, double *unused, double *out_1498247872710036685);
void live_h_13(double *state, double *unused, double *out_98987599675754647);
void live_H_13(double *state, double *unused, double *out_1455756144988918162);
void live_h_14(double *state, double *unused, double *out_935430128420636855);
void live_H_14(double *state, double *unused, double *out_4754885068393793885);
void live_h_33(double *state, double *unused, double *out_1476059775988479524);
void live_H_33(double *state, double *unused, double *out_1521144346988080450);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}