#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3627095079095666957) {
   out_3627095079095666957[0] = delta_x[0] + nom_x[0];
   out_3627095079095666957[1] = delta_x[1] + nom_x[1];
   out_3627095079095666957[2] = delta_x[2] + nom_x[2];
   out_3627095079095666957[3] = delta_x[3] + nom_x[3];
   out_3627095079095666957[4] = delta_x[4] + nom_x[4];
   out_3627095079095666957[5] = delta_x[5] + nom_x[5];
   out_3627095079095666957[6] = delta_x[6] + nom_x[6];
   out_3627095079095666957[7] = delta_x[7] + nom_x[7];
   out_3627095079095666957[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_23504288492248099) {
   out_23504288492248099[0] = -nom_x[0] + true_x[0];
   out_23504288492248099[1] = -nom_x[1] + true_x[1];
   out_23504288492248099[2] = -nom_x[2] + true_x[2];
   out_23504288492248099[3] = -nom_x[3] + true_x[3];
   out_23504288492248099[4] = -nom_x[4] + true_x[4];
   out_23504288492248099[5] = -nom_x[5] + true_x[5];
   out_23504288492248099[6] = -nom_x[6] + true_x[6];
   out_23504288492248099[7] = -nom_x[7] + true_x[7];
   out_23504288492248099[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6172561962072649303) {
   out_6172561962072649303[0] = 1.0;
   out_6172561962072649303[1] = 0.0;
   out_6172561962072649303[2] = 0.0;
   out_6172561962072649303[3] = 0.0;
   out_6172561962072649303[4] = 0.0;
   out_6172561962072649303[5] = 0.0;
   out_6172561962072649303[6] = 0.0;
   out_6172561962072649303[7] = 0.0;
   out_6172561962072649303[8] = 0.0;
   out_6172561962072649303[9] = 0.0;
   out_6172561962072649303[10] = 1.0;
   out_6172561962072649303[11] = 0.0;
   out_6172561962072649303[12] = 0.0;
   out_6172561962072649303[13] = 0.0;
   out_6172561962072649303[14] = 0.0;
   out_6172561962072649303[15] = 0.0;
   out_6172561962072649303[16] = 0.0;
   out_6172561962072649303[17] = 0.0;
   out_6172561962072649303[18] = 0.0;
   out_6172561962072649303[19] = 0.0;
   out_6172561962072649303[20] = 1.0;
   out_6172561962072649303[21] = 0.0;
   out_6172561962072649303[22] = 0.0;
   out_6172561962072649303[23] = 0.0;
   out_6172561962072649303[24] = 0.0;
   out_6172561962072649303[25] = 0.0;
   out_6172561962072649303[26] = 0.0;
   out_6172561962072649303[27] = 0.0;
   out_6172561962072649303[28] = 0.0;
   out_6172561962072649303[29] = 0.0;
   out_6172561962072649303[30] = 1.0;
   out_6172561962072649303[31] = 0.0;
   out_6172561962072649303[32] = 0.0;
   out_6172561962072649303[33] = 0.0;
   out_6172561962072649303[34] = 0.0;
   out_6172561962072649303[35] = 0.0;
   out_6172561962072649303[36] = 0.0;
   out_6172561962072649303[37] = 0.0;
   out_6172561962072649303[38] = 0.0;
   out_6172561962072649303[39] = 0.0;
   out_6172561962072649303[40] = 1.0;
   out_6172561962072649303[41] = 0.0;
   out_6172561962072649303[42] = 0.0;
   out_6172561962072649303[43] = 0.0;
   out_6172561962072649303[44] = 0.0;
   out_6172561962072649303[45] = 0.0;
   out_6172561962072649303[46] = 0.0;
   out_6172561962072649303[47] = 0.0;
   out_6172561962072649303[48] = 0.0;
   out_6172561962072649303[49] = 0.0;
   out_6172561962072649303[50] = 1.0;
   out_6172561962072649303[51] = 0.0;
   out_6172561962072649303[52] = 0.0;
   out_6172561962072649303[53] = 0.0;
   out_6172561962072649303[54] = 0.0;
   out_6172561962072649303[55] = 0.0;
   out_6172561962072649303[56] = 0.0;
   out_6172561962072649303[57] = 0.0;
   out_6172561962072649303[58] = 0.0;
   out_6172561962072649303[59] = 0.0;
   out_6172561962072649303[60] = 1.0;
   out_6172561962072649303[61] = 0.0;
   out_6172561962072649303[62] = 0.0;
   out_6172561962072649303[63] = 0.0;
   out_6172561962072649303[64] = 0.0;
   out_6172561962072649303[65] = 0.0;
   out_6172561962072649303[66] = 0.0;
   out_6172561962072649303[67] = 0.0;
   out_6172561962072649303[68] = 0.0;
   out_6172561962072649303[69] = 0.0;
   out_6172561962072649303[70] = 1.0;
   out_6172561962072649303[71] = 0.0;
   out_6172561962072649303[72] = 0.0;
   out_6172561962072649303[73] = 0.0;
   out_6172561962072649303[74] = 0.0;
   out_6172561962072649303[75] = 0.0;
   out_6172561962072649303[76] = 0.0;
   out_6172561962072649303[77] = 0.0;
   out_6172561962072649303[78] = 0.0;
   out_6172561962072649303[79] = 0.0;
   out_6172561962072649303[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2282682727056561244) {
   out_2282682727056561244[0] = state[0];
   out_2282682727056561244[1] = state[1];
   out_2282682727056561244[2] = state[2];
   out_2282682727056561244[3] = state[3];
   out_2282682727056561244[4] = state[4];
   out_2282682727056561244[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2282682727056561244[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2282682727056561244[7] = state[7];
   out_2282682727056561244[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4989797238579589534) {
   out_4989797238579589534[0] = 1;
   out_4989797238579589534[1] = 0;
   out_4989797238579589534[2] = 0;
   out_4989797238579589534[3] = 0;
   out_4989797238579589534[4] = 0;
   out_4989797238579589534[5] = 0;
   out_4989797238579589534[6] = 0;
   out_4989797238579589534[7] = 0;
   out_4989797238579589534[8] = 0;
   out_4989797238579589534[9] = 0;
   out_4989797238579589534[10] = 1;
   out_4989797238579589534[11] = 0;
   out_4989797238579589534[12] = 0;
   out_4989797238579589534[13] = 0;
   out_4989797238579589534[14] = 0;
   out_4989797238579589534[15] = 0;
   out_4989797238579589534[16] = 0;
   out_4989797238579589534[17] = 0;
   out_4989797238579589534[18] = 0;
   out_4989797238579589534[19] = 0;
   out_4989797238579589534[20] = 1;
   out_4989797238579589534[21] = 0;
   out_4989797238579589534[22] = 0;
   out_4989797238579589534[23] = 0;
   out_4989797238579589534[24] = 0;
   out_4989797238579589534[25] = 0;
   out_4989797238579589534[26] = 0;
   out_4989797238579589534[27] = 0;
   out_4989797238579589534[28] = 0;
   out_4989797238579589534[29] = 0;
   out_4989797238579589534[30] = 1;
   out_4989797238579589534[31] = 0;
   out_4989797238579589534[32] = 0;
   out_4989797238579589534[33] = 0;
   out_4989797238579589534[34] = 0;
   out_4989797238579589534[35] = 0;
   out_4989797238579589534[36] = 0;
   out_4989797238579589534[37] = 0;
   out_4989797238579589534[38] = 0;
   out_4989797238579589534[39] = 0;
   out_4989797238579589534[40] = 1;
   out_4989797238579589534[41] = 0;
   out_4989797238579589534[42] = 0;
   out_4989797238579589534[43] = 0;
   out_4989797238579589534[44] = 0;
   out_4989797238579589534[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4989797238579589534[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4989797238579589534[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4989797238579589534[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4989797238579589534[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4989797238579589534[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4989797238579589534[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4989797238579589534[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4989797238579589534[53] = -9.8100000000000005*dt;
   out_4989797238579589534[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4989797238579589534[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4989797238579589534[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4989797238579589534[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4989797238579589534[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4989797238579589534[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4989797238579589534[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4989797238579589534[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4989797238579589534[62] = 0;
   out_4989797238579589534[63] = 0;
   out_4989797238579589534[64] = 0;
   out_4989797238579589534[65] = 0;
   out_4989797238579589534[66] = 0;
   out_4989797238579589534[67] = 0;
   out_4989797238579589534[68] = 0;
   out_4989797238579589534[69] = 0;
   out_4989797238579589534[70] = 1;
   out_4989797238579589534[71] = 0;
   out_4989797238579589534[72] = 0;
   out_4989797238579589534[73] = 0;
   out_4989797238579589534[74] = 0;
   out_4989797238579589534[75] = 0;
   out_4989797238579589534[76] = 0;
   out_4989797238579589534[77] = 0;
   out_4989797238579589534[78] = 0;
   out_4989797238579589534[79] = 0;
   out_4989797238579589534[80] = 1;
}
void h_25(double *state, double *unused, double *out_1239437150094003777) {
   out_1239437150094003777[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7370490286452237473) {
   out_7370490286452237473[0] = 0;
   out_7370490286452237473[1] = 0;
   out_7370490286452237473[2] = 0;
   out_7370490286452237473[3] = 0;
   out_7370490286452237473[4] = 0;
   out_7370490286452237473[5] = 0;
   out_7370490286452237473[6] = 1;
   out_7370490286452237473[7] = 0;
   out_7370490286452237473[8] = 0;
}
void h_24(double *state, double *unused, double *out_7987796621209786173) {
   out_7987796621209786173[0] = state[4];
   out_7987796621209786173[1] = state[5];
}
void H_24(double *state, double *unused, double *out_794918479860719372) {
   out_794918479860719372[0] = 0;
   out_794918479860719372[1] = 0;
   out_794918479860719372[2] = 0;
   out_794918479860719372[3] = 0;
   out_794918479860719372[4] = 1;
   out_794918479860719372[5] = 0;
   out_794918479860719372[6] = 0;
   out_794918479860719372[7] = 0;
   out_794918479860719372[8] = 0;
   out_794918479860719372[9] = 0;
   out_794918479860719372[10] = 0;
   out_794918479860719372[11] = 0;
   out_794918479860719372[12] = 0;
   out_794918479860719372[13] = 0;
   out_794918479860719372[14] = 1;
   out_794918479860719372[15] = 0;
   out_794918479860719372[16] = 0;
   out_794918479860719372[17] = 0;
}
void h_30(double *state, double *unused, double *out_7257223531169736215) {
   out_7257223531169736215[0] = state[4];
}
void H_30(double *state, double *unused, double *out_453799944960620718) {
   out_453799944960620718[0] = 0;
   out_453799944960620718[1] = 0;
   out_453799944960620718[2] = 0;
   out_453799944960620718[3] = 0;
   out_453799944960620718[4] = 1;
   out_453799944960620718[5] = 0;
   out_453799944960620718[6] = 0;
   out_453799944960620718[7] = 0;
   out_453799944960620718[8] = 0;
}
void h_26(double *state, double *unused, double *out_6567334513083707087) {
   out_6567334513083707087[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7334750468383257919) {
   out_7334750468383257919[0] = 0;
   out_7334750468383257919[1] = 0;
   out_7334750468383257919[2] = 0;
   out_7334750468383257919[3] = 0;
   out_7334750468383257919[4] = 0;
   out_7334750468383257919[5] = 0;
   out_7334750468383257919[6] = 0;
   out_7334750468383257919[7] = 1;
   out_7334750468383257919[8] = 0;
}
void h_27(double *state, double *unused, double *out_5448768302500678511) {
   out_5448768302500678511[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2628563256761045629) {
   out_2628563256761045629[0] = 0;
   out_2628563256761045629[1] = 0;
   out_2628563256761045629[2] = 0;
   out_2628563256761045629[3] = 1;
   out_2628563256761045629[4] = 0;
   out_2628563256761045629[5] = 0;
   out_2628563256761045629[6] = 0;
   out_2628563256761045629[7] = 0;
   out_2628563256761045629[8] = 0;
}
void h_29(double *state, double *unused, double *out_2255550263508890570) {
   out_2255550263508890570[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4341925983630596662) {
   out_4341925983630596662[0] = 0;
   out_4341925983630596662[1] = 1;
   out_4341925983630596662[2] = 0;
   out_4341925983630596662[3] = 0;
   out_4341925983630596662[4] = 0;
   out_4341925983630596662[5] = 0;
   out_4341925983630596662[6] = 0;
   out_4341925983630596662[7] = 0;
   out_4341925983630596662[8] = 0;
}
void h_28(double *state, double *unused, double *out_7139216137236359471) {
   out_7139216137236359471[0] = state[0];
}
void H_28(double *state, double *unused, double *out_9022419073009424380) {
   out_9022419073009424380[0] = 1;
   out_9022419073009424380[1] = 0;
   out_9022419073009424380[2] = 0;
   out_9022419073009424380[3] = 0;
   out_9022419073009424380[4] = 0;
   out_9022419073009424380[5] = 0;
   out_9022419073009424380[6] = 0;
   out_9022419073009424380[7] = 0;
   out_9022419073009424380[8] = 0;
}
void h_31(double *state, double *unused, double *out_964243087809497888) {
   out_964243087809497888[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7339844324575277045) {
   out_7339844324575277045[0] = 0;
   out_7339844324575277045[1] = 0;
   out_7339844324575277045[2] = 0;
   out_7339844324575277045[3] = 0;
   out_7339844324575277045[4] = 0;
   out_7339844324575277045[5] = 0;
   out_7339844324575277045[6] = 0;
   out_7339844324575277045[7] = 0;
   out_7339844324575277045[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3627095079095666957) {
  err_fun(nom_x, delta_x, out_3627095079095666957);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_23504288492248099) {
  inv_err_fun(nom_x, true_x, out_23504288492248099);
}
void car_H_mod_fun(double *state, double *out_6172561962072649303) {
  H_mod_fun(state, out_6172561962072649303);
}
void car_f_fun(double *state, double dt, double *out_2282682727056561244) {
  f_fun(state,  dt, out_2282682727056561244);
}
void car_F_fun(double *state, double dt, double *out_4989797238579589534) {
  F_fun(state,  dt, out_4989797238579589534);
}
void car_h_25(double *state, double *unused, double *out_1239437150094003777) {
  h_25(state, unused, out_1239437150094003777);
}
void car_H_25(double *state, double *unused, double *out_7370490286452237473) {
  H_25(state, unused, out_7370490286452237473);
}
void car_h_24(double *state, double *unused, double *out_7987796621209786173) {
  h_24(state, unused, out_7987796621209786173);
}
void car_H_24(double *state, double *unused, double *out_794918479860719372) {
  H_24(state, unused, out_794918479860719372);
}
void car_h_30(double *state, double *unused, double *out_7257223531169736215) {
  h_30(state, unused, out_7257223531169736215);
}
void car_H_30(double *state, double *unused, double *out_453799944960620718) {
  H_30(state, unused, out_453799944960620718);
}
void car_h_26(double *state, double *unused, double *out_6567334513083707087) {
  h_26(state, unused, out_6567334513083707087);
}
void car_H_26(double *state, double *unused, double *out_7334750468383257919) {
  H_26(state, unused, out_7334750468383257919);
}
void car_h_27(double *state, double *unused, double *out_5448768302500678511) {
  h_27(state, unused, out_5448768302500678511);
}
void car_H_27(double *state, double *unused, double *out_2628563256761045629) {
  H_27(state, unused, out_2628563256761045629);
}
void car_h_29(double *state, double *unused, double *out_2255550263508890570) {
  h_29(state, unused, out_2255550263508890570);
}
void car_H_29(double *state, double *unused, double *out_4341925983630596662) {
  H_29(state, unused, out_4341925983630596662);
}
void car_h_28(double *state, double *unused, double *out_7139216137236359471) {
  h_28(state, unused, out_7139216137236359471);
}
void car_H_28(double *state, double *unused, double *out_9022419073009424380) {
  H_28(state, unused, out_9022419073009424380);
}
void car_h_31(double *state, double *unused, double *out_964243087809497888) {
  h_31(state, unused, out_964243087809497888);
}
void car_H_31(double *state, double *unused, double *out_7339844324575277045) {
  H_31(state, unused, out_7339844324575277045);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
