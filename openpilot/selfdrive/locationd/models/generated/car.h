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
void car_err_fun(double *nom_x, double *delta_x, double *out_2894113938990965662);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2381022520046862243);
void car_H_mod_fun(double *state, double *out_8073844072421287879);
void car_f_fun(double *state, double dt, double *out_1835070724140985802);
void car_F_fun(double *state, double dt, double *out_672615182746371325);
void car_h_25(double *state, double *unused, double *out_2182680073942906551);
void car_H_25(double *state, double *unused, double *out_1119344153490949358);
void car_h_24(double *state, double *unused, double *out_3573452335236708456);
void car_H_24(double *state, double *unused, double *out_3291993752496448924);
void car_h_30(double *state, double *unused, double *out_5502112638480461076);
void car_H_30(double *state, double *unused, double *out_5797346188000667397);
void car_h_26(double *state, double *unused, double *out_7942958167568244895);
void car_H_26(double *state, double *unused, double *out_4860847472365005582);
void car_h_27(double *state, double *unused, double *out_1014936272866764814);
void car_H_27(double *state, double *unused, double *out_3622582876200242486);
void car_h_29(double *state, double *unused, double *out_7624941203851777070);
void car_H_29(double *state, double *unused, double *out_1909220149330691453);
void car_h_28(double *state, double *unused, double *out_6016609540527610459);
void car_H_28(double *state, double *unused, double *out_3173178867738839121);
void car_h_31(double *state, double *unused, double *out_2339866742294035696);
void car_H_31(double *state, double *unused, double *out_1088698191613988930);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}