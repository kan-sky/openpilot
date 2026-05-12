#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6074688492235971640);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7256018833813533895);
void pose_H_mod_fun(double *state, double *out_5581698035203868742);
void pose_f_fun(double *state, double dt, double *out_2476147660253971784);
void pose_F_fun(double *state, double dt, double *out_6325063450262357255);
void pose_h_4(double *state, double *unused, double *out_5863910480806422069);
void pose_H_4(double *state, double *unused, double *out_7712527653466602139);
void pose_h_10(double *state, double *unused, double *out_5277016606615583295);
void pose_H_10(double *state, double *unused, double *out_2723658752885143762);
void pose_h_13(double *state, double *unused, double *out_4417683070087264532);
void pose_H_13(double *state, double *unused, double *out_6900460956940425453);
void pose_h_14(double *state, double *unused, double *out_7264994596821899571);
void pose_H_14(double *state, double *unused, double *out_7651427987947577181);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}