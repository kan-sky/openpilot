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
void car_err_fun(double *nom_x, double *delta_x, double *out_3627095079095666957);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_23504288492248099);
void car_H_mod_fun(double *state, double *out_6172561962072649303);
void car_f_fun(double *state, double dt, double *out_2282682727056561244);
void car_F_fun(double *state, double dt, double *out_4989797238579589534);
void car_h_25(double *state, double *unused, double *out_1239437150094003777);
void car_H_25(double *state, double *unused, double *out_7370490286452237473);
void car_h_24(double *state, double *unused, double *out_7987796621209786173);
void car_H_24(double *state, double *unused, double *out_794918479860719372);
void car_h_30(double *state, double *unused, double *out_7257223531169736215);
void car_H_30(double *state, double *unused, double *out_453799944960620718);
void car_h_26(double *state, double *unused, double *out_6567334513083707087);
void car_H_26(double *state, double *unused, double *out_7334750468383257919);
void car_h_27(double *state, double *unused, double *out_5448768302500678511);
void car_H_27(double *state, double *unused, double *out_2628563256761045629);
void car_h_29(double *state, double *unused, double *out_2255550263508890570);
void car_H_29(double *state, double *unused, double *out_4341925983630596662);
void car_h_28(double *state, double *unused, double *out_7139216137236359471);
void car_H_28(double *state, double *unused, double *out_9022419073009424380);
void car_h_31(double *state, double *unused, double *out_964243087809497888);
void car_H_31(double *state, double *unused, double *out_7339844324575277045);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}