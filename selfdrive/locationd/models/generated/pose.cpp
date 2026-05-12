#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6074688492235971640) {
   out_6074688492235971640[0] = delta_x[0] + nom_x[0];
   out_6074688492235971640[1] = delta_x[1] + nom_x[1];
   out_6074688492235971640[2] = delta_x[2] + nom_x[2];
   out_6074688492235971640[3] = delta_x[3] + nom_x[3];
   out_6074688492235971640[4] = delta_x[4] + nom_x[4];
   out_6074688492235971640[5] = delta_x[5] + nom_x[5];
   out_6074688492235971640[6] = delta_x[6] + nom_x[6];
   out_6074688492235971640[7] = delta_x[7] + nom_x[7];
   out_6074688492235971640[8] = delta_x[8] + nom_x[8];
   out_6074688492235971640[9] = delta_x[9] + nom_x[9];
   out_6074688492235971640[10] = delta_x[10] + nom_x[10];
   out_6074688492235971640[11] = delta_x[11] + nom_x[11];
   out_6074688492235971640[12] = delta_x[12] + nom_x[12];
   out_6074688492235971640[13] = delta_x[13] + nom_x[13];
   out_6074688492235971640[14] = delta_x[14] + nom_x[14];
   out_6074688492235971640[15] = delta_x[15] + nom_x[15];
   out_6074688492235971640[16] = delta_x[16] + nom_x[16];
   out_6074688492235971640[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7256018833813533895) {
   out_7256018833813533895[0] = -nom_x[0] + true_x[0];
   out_7256018833813533895[1] = -nom_x[1] + true_x[1];
   out_7256018833813533895[2] = -nom_x[2] + true_x[2];
   out_7256018833813533895[3] = -nom_x[3] + true_x[3];
   out_7256018833813533895[4] = -nom_x[4] + true_x[4];
   out_7256018833813533895[5] = -nom_x[5] + true_x[5];
   out_7256018833813533895[6] = -nom_x[6] + true_x[6];
   out_7256018833813533895[7] = -nom_x[7] + true_x[7];
   out_7256018833813533895[8] = -nom_x[8] + true_x[8];
   out_7256018833813533895[9] = -nom_x[9] + true_x[9];
   out_7256018833813533895[10] = -nom_x[10] + true_x[10];
   out_7256018833813533895[11] = -nom_x[11] + true_x[11];
   out_7256018833813533895[12] = -nom_x[12] + true_x[12];
   out_7256018833813533895[13] = -nom_x[13] + true_x[13];
   out_7256018833813533895[14] = -nom_x[14] + true_x[14];
   out_7256018833813533895[15] = -nom_x[15] + true_x[15];
   out_7256018833813533895[16] = -nom_x[16] + true_x[16];
   out_7256018833813533895[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5581698035203868742) {
   out_5581698035203868742[0] = 1.0;
   out_5581698035203868742[1] = 0.0;
   out_5581698035203868742[2] = 0.0;
   out_5581698035203868742[3] = 0.0;
   out_5581698035203868742[4] = 0.0;
   out_5581698035203868742[5] = 0.0;
   out_5581698035203868742[6] = 0.0;
   out_5581698035203868742[7] = 0.0;
   out_5581698035203868742[8] = 0.0;
   out_5581698035203868742[9] = 0.0;
   out_5581698035203868742[10] = 0.0;
   out_5581698035203868742[11] = 0.0;
   out_5581698035203868742[12] = 0.0;
   out_5581698035203868742[13] = 0.0;
   out_5581698035203868742[14] = 0.0;
   out_5581698035203868742[15] = 0.0;
   out_5581698035203868742[16] = 0.0;
   out_5581698035203868742[17] = 0.0;
   out_5581698035203868742[18] = 0.0;
   out_5581698035203868742[19] = 1.0;
   out_5581698035203868742[20] = 0.0;
   out_5581698035203868742[21] = 0.0;
   out_5581698035203868742[22] = 0.0;
   out_5581698035203868742[23] = 0.0;
   out_5581698035203868742[24] = 0.0;
   out_5581698035203868742[25] = 0.0;
   out_5581698035203868742[26] = 0.0;
   out_5581698035203868742[27] = 0.0;
   out_5581698035203868742[28] = 0.0;
   out_5581698035203868742[29] = 0.0;
   out_5581698035203868742[30] = 0.0;
   out_5581698035203868742[31] = 0.0;
   out_5581698035203868742[32] = 0.0;
   out_5581698035203868742[33] = 0.0;
   out_5581698035203868742[34] = 0.0;
   out_5581698035203868742[35] = 0.0;
   out_5581698035203868742[36] = 0.0;
   out_5581698035203868742[37] = 0.0;
   out_5581698035203868742[38] = 1.0;
   out_5581698035203868742[39] = 0.0;
   out_5581698035203868742[40] = 0.0;
   out_5581698035203868742[41] = 0.0;
   out_5581698035203868742[42] = 0.0;
   out_5581698035203868742[43] = 0.0;
   out_5581698035203868742[44] = 0.0;
   out_5581698035203868742[45] = 0.0;
   out_5581698035203868742[46] = 0.0;
   out_5581698035203868742[47] = 0.0;
   out_5581698035203868742[48] = 0.0;
   out_5581698035203868742[49] = 0.0;
   out_5581698035203868742[50] = 0.0;
   out_5581698035203868742[51] = 0.0;
   out_5581698035203868742[52] = 0.0;
   out_5581698035203868742[53] = 0.0;
   out_5581698035203868742[54] = 0.0;
   out_5581698035203868742[55] = 0.0;
   out_5581698035203868742[56] = 0.0;
   out_5581698035203868742[57] = 1.0;
   out_5581698035203868742[58] = 0.0;
   out_5581698035203868742[59] = 0.0;
   out_5581698035203868742[60] = 0.0;
   out_5581698035203868742[61] = 0.0;
   out_5581698035203868742[62] = 0.0;
   out_5581698035203868742[63] = 0.0;
   out_5581698035203868742[64] = 0.0;
   out_5581698035203868742[65] = 0.0;
   out_5581698035203868742[66] = 0.0;
   out_5581698035203868742[67] = 0.0;
   out_5581698035203868742[68] = 0.0;
   out_5581698035203868742[69] = 0.0;
   out_5581698035203868742[70] = 0.0;
   out_5581698035203868742[71] = 0.0;
   out_5581698035203868742[72] = 0.0;
   out_5581698035203868742[73] = 0.0;
   out_5581698035203868742[74] = 0.0;
   out_5581698035203868742[75] = 0.0;
   out_5581698035203868742[76] = 1.0;
   out_5581698035203868742[77] = 0.0;
   out_5581698035203868742[78] = 0.0;
   out_5581698035203868742[79] = 0.0;
   out_5581698035203868742[80] = 0.0;
   out_5581698035203868742[81] = 0.0;
   out_5581698035203868742[82] = 0.0;
   out_5581698035203868742[83] = 0.0;
   out_5581698035203868742[84] = 0.0;
   out_5581698035203868742[85] = 0.0;
   out_5581698035203868742[86] = 0.0;
   out_5581698035203868742[87] = 0.0;
   out_5581698035203868742[88] = 0.0;
   out_5581698035203868742[89] = 0.0;
   out_5581698035203868742[90] = 0.0;
   out_5581698035203868742[91] = 0.0;
   out_5581698035203868742[92] = 0.0;
   out_5581698035203868742[93] = 0.0;
   out_5581698035203868742[94] = 0.0;
   out_5581698035203868742[95] = 1.0;
   out_5581698035203868742[96] = 0.0;
   out_5581698035203868742[97] = 0.0;
   out_5581698035203868742[98] = 0.0;
   out_5581698035203868742[99] = 0.0;
   out_5581698035203868742[100] = 0.0;
   out_5581698035203868742[101] = 0.0;
   out_5581698035203868742[102] = 0.0;
   out_5581698035203868742[103] = 0.0;
   out_5581698035203868742[104] = 0.0;
   out_5581698035203868742[105] = 0.0;
   out_5581698035203868742[106] = 0.0;
   out_5581698035203868742[107] = 0.0;
   out_5581698035203868742[108] = 0.0;
   out_5581698035203868742[109] = 0.0;
   out_5581698035203868742[110] = 0.0;
   out_5581698035203868742[111] = 0.0;
   out_5581698035203868742[112] = 0.0;
   out_5581698035203868742[113] = 0.0;
   out_5581698035203868742[114] = 1.0;
   out_5581698035203868742[115] = 0.0;
   out_5581698035203868742[116] = 0.0;
   out_5581698035203868742[117] = 0.0;
   out_5581698035203868742[118] = 0.0;
   out_5581698035203868742[119] = 0.0;
   out_5581698035203868742[120] = 0.0;
   out_5581698035203868742[121] = 0.0;
   out_5581698035203868742[122] = 0.0;
   out_5581698035203868742[123] = 0.0;
   out_5581698035203868742[124] = 0.0;
   out_5581698035203868742[125] = 0.0;
   out_5581698035203868742[126] = 0.0;
   out_5581698035203868742[127] = 0.0;
   out_5581698035203868742[128] = 0.0;
   out_5581698035203868742[129] = 0.0;
   out_5581698035203868742[130] = 0.0;
   out_5581698035203868742[131] = 0.0;
   out_5581698035203868742[132] = 0.0;
   out_5581698035203868742[133] = 1.0;
   out_5581698035203868742[134] = 0.0;
   out_5581698035203868742[135] = 0.0;
   out_5581698035203868742[136] = 0.0;
   out_5581698035203868742[137] = 0.0;
   out_5581698035203868742[138] = 0.0;
   out_5581698035203868742[139] = 0.0;
   out_5581698035203868742[140] = 0.0;
   out_5581698035203868742[141] = 0.0;
   out_5581698035203868742[142] = 0.0;
   out_5581698035203868742[143] = 0.0;
   out_5581698035203868742[144] = 0.0;
   out_5581698035203868742[145] = 0.0;
   out_5581698035203868742[146] = 0.0;
   out_5581698035203868742[147] = 0.0;
   out_5581698035203868742[148] = 0.0;
   out_5581698035203868742[149] = 0.0;
   out_5581698035203868742[150] = 0.0;
   out_5581698035203868742[151] = 0.0;
   out_5581698035203868742[152] = 1.0;
   out_5581698035203868742[153] = 0.0;
   out_5581698035203868742[154] = 0.0;
   out_5581698035203868742[155] = 0.0;
   out_5581698035203868742[156] = 0.0;
   out_5581698035203868742[157] = 0.0;
   out_5581698035203868742[158] = 0.0;
   out_5581698035203868742[159] = 0.0;
   out_5581698035203868742[160] = 0.0;
   out_5581698035203868742[161] = 0.0;
   out_5581698035203868742[162] = 0.0;
   out_5581698035203868742[163] = 0.0;
   out_5581698035203868742[164] = 0.0;
   out_5581698035203868742[165] = 0.0;
   out_5581698035203868742[166] = 0.0;
   out_5581698035203868742[167] = 0.0;
   out_5581698035203868742[168] = 0.0;
   out_5581698035203868742[169] = 0.0;
   out_5581698035203868742[170] = 0.0;
   out_5581698035203868742[171] = 1.0;
   out_5581698035203868742[172] = 0.0;
   out_5581698035203868742[173] = 0.0;
   out_5581698035203868742[174] = 0.0;
   out_5581698035203868742[175] = 0.0;
   out_5581698035203868742[176] = 0.0;
   out_5581698035203868742[177] = 0.0;
   out_5581698035203868742[178] = 0.0;
   out_5581698035203868742[179] = 0.0;
   out_5581698035203868742[180] = 0.0;
   out_5581698035203868742[181] = 0.0;
   out_5581698035203868742[182] = 0.0;
   out_5581698035203868742[183] = 0.0;
   out_5581698035203868742[184] = 0.0;
   out_5581698035203868742[185] = 0.0;
   out_5581698035203868742[186] = 0.0;
   out_5581698035203868742[187] = 0.0;
   out_5581698035203868742[188] = 0.0;
   out_5581698035203868742[189] = 0.0;
   out_5581698035203868742[190] = 1.0;
   out_5581698035203868742[191] = 0.0;
   out_5581698035203868742[192] = 0.0;
   out_5581698035203868742[193] = 0.0;
   out_5581698035203868742[194] = 0.0;
   out_5581698035203868742[195] = 0.0;
   out_5581698035203868742[196] = 0.0;
   out_5581698035203868742[197] = 0.0;
   out_5581698035203868742[198] = 0.0;
   out_5581698035203868742[199] = 0.0;
   out_5581698035203868742[200] = 0.0;
   out_5581698035203868742[201] = 0.0;
   out_5581698035203868742[202] = 0.0;
   out_5581698035203868742[203] = 0.0;
   out_5581698035203868742[204] = 0.0;
   out_5581698035203868742[205] = 0.0;
   out_5581698035203868742[206] = 0.0;
   out_5581698035203868742[207] = 0.0;
   out_5581698035203868742[208] = 0.0;
   out_5581698035203868742[209] = 1.0;
   out_5581698035203868742[210] = 0.0;
   out_5581698035203868742[211] = 0.0;
   out_5581698035203868742[212] = 0.0;
   out_5581698035203868742[213] = 0.0;
   out_5581698035203868742[214] = 0.0;
   out_5581698035203868742[215] = 0.0;
   out_5581698035203868742[216] = 0.0;
   out_5581698035203868742[217] = 0.0;
   out_5581698035203868742[218] = 0.0;
   out_5581698035203868742[219] = 0.0;
   out_5581698035203868742[220] = 0.0;
   out_5581698035203868742[221] = 0.0;
   out_5581698035203868742[222] = 0.0;
   out_5581698035203868742[223] = 0.0;
   out_5581698035203868742[224] = 0.0;
   out_5581698035203868742[225] = 0.0;
   out_5581698035203868742[226] = 0.0;
   out_5581698035203868742[227] = 0.0;
   out_5581698035203868742[228] = 1.0;
   out_5581698035203868742[229] = 0.0;
   out_5581698035203868742[230] = 0.0;
   out_5581698035203868742[231] = 0.0;
   out_5581698035203868742[232] = 0.0;
   out_5581698035203868742[233] = 0.0;
   out_5581698035203868742[234] = 0.0;
   out_5581698035203868742[235] = 0.0;
   out_5581698035203868742[236] = 0.0;
   out_5581698035203868742[237] = 0.0;
   out_5581698035203868742[238] = 0.0;
   out_5581698035203868742[239] = 0.0;
   out_5581698035203868742[240] = 0.0;
   out_5581698035203868742[241] = 0.0;
   out_5581698035203868742[242] = 0.0;
   out_5581698035203868742[243] = 0.0;
   out_5581698035203868742[244] = 0.0;
   out_5581698035203868742[245] = 0.0;
   out_5581698035203868742[246] = 0.0;
   out_5581698035203868742[247] = 1.0;
   out_5581698035203868742[248] = 0.0;
   out_5581698035203868742[249] = 0.0;
   out_5581698035203868742[250] = 0.0;
   out_5581698035203868742[251] = 0.0;
   out_5581698035203868742[252] = 0.0;
   out_5581698035203868742[253] = 0.0;
   out_5581698035203868742[254] = 0.0;
   out_5581698035203868742[255] = 0.0;
   out_5581698035203868742[256] = 0.0;
   out_5581698035203868742[257] = 0.0;
   out_5581698035203868742[258] = 0.0;
   out_5581698035203868742[259] = 0.0;
   out_5581698035203868742[260] = 0.0;
   out_5581698035203868742[261] = 0.0;
   out_5581698035203868742[262] = 0.0;
   out_5581698035203868742[263] = 0.0;
   out_5581698035203868742[264] = 0.0;
   out_5581698035203868742[265] = 0.0;
   out_5581698035203868742[266] = 1.0;
   out_5581698035203868742[267] = 0.0;
   out_5581698035203868742[268] = 0.0;
   out_5581698035203868742[269] = 0.0;
   out_5581698035203868742[270] = 0.0;
   out_5581698035203868742[271] = 0.0;
   out_5581698035203868742[272] = 0.0;
   out_5581698035203868742[273] = 0.0;
   out_5581698035203868742[274] = 0.0;
   out_5581698035203868742[275] = 0.0;
   out_5581698035203868742[276] = 0.0;
   out_5581698035203868742[277] = 0.0;
   out_5581698035203868742[278] = 0.0;
   out_5581698035203868742[279] = 0.0;
   out_5581698035203868742[280] = 0.0;
   out_5581698035203868742[281] = 0.0;
   out_5581698035203868742[282] = 0.0;
   out_5581698035203868742[283] = 0.0;
   out_5581698035203868742[284] = 0.0;
   out_5581698035203868742[285] = 1.0;
   out_5581698035203868742[286] = 0.0;
   out_5581698035203868742[287] = 0.0;
   out_5581698035203868742[288] = 0.0;
   out_5581698035203868742[289] = 0.0;
   out_5581698035203868742[290] = 0.0;
   out_5581698035203868742[291] = 0.0;
   out_5581698035203868742[292] = 0.0;
   out_5581698035203868742[293] = 0.0;
   out_5581698035203868742[294] = 0.0;
   out_5581698035203868742[295] = 0.0;
   out_5581698035203868742[296] = 0.0;
   out_5581698035203868742[297] = 0.0;
   out_5581698035203868742[298] = 0.0;
   out_5581698035203868742[299] = 0.0;
   out_5581698035203868742[300] = 0.0;
   out_5581698035203868742[301] = 0.0;
   out_5581698035203868742[302] = 0.0;
   out_5581698035203868742[303] = 0.0;
   out_5581698035203868742[304] = 1.0;
   out_5581698035203868742[305] = 0.0;
   out_5581698035203868742[306] = 0.0;
   out_5581698035203868742[307] = 0.0;
   out_5581698035203868742[308] = 0.0;
   out_5581698035203868742[309] = 0.0;
   out_5581698035203868742[310] = 0.0;
   out_5581698035203868742[311] = 0.0;
   out_5581698035203868742[312] = 0.0;
   out_5581698035203868742[313] = 0.0;
   out_5581698035203868742[314] = 0.0;
   out_5581698035203868742[315] = 0.0;
   out_5581698035203868742[316] = 0.0;
   out_5581698035203868742[317] = 0.0;
   out_5581698035203868742[318] = 0.0;
   out_5581698035203868742[319] = 0.0;
   out_5581698035203868742[320] = 0.0;
   out_5581698035203868742[321] = 0.0;
   out_5581698035203868742[322] = 0.0;
   out_5581698035203868742[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2476147660253971784) {
   out_2476147660253971784[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2476147660253971784[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2476147660253971784[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2476147660253971784[3] = dt*state[12] + state[3];
   out_2476147660253971784[4] = dt*state[13] + state[4];
   out_2476147660253971784[5] = dt*state[14] + state[5];
   out_2476147660253971784[6] = state[6];
   out_2476147660253971784[7] = state[7];
   out_2476147660253971784[8] = state[8];
   out_2476147660253971784[9] = state[9];
   out_2476147660253971784[10] = state[10];
   out_2476147660253971784[11] = state[11];
   out_2476147660253971784[12] = state[12];
   out_2476147660253971784[13] = state[13];
   out_2476147660253971784[14] = state[14];
   out_2476147660253971784[15] = state[15];
   out_2476147660253971784[16] = state[16];
   out_2476147660253971784[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6325063450262357255) {
   out_6325063450262357255[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6325063450262357255[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6325063450262357255[2] = 0;
   out_6325063450262357255[3] = 0;
   out_6325063450262357255[4] = 0;
   out_6325063450262357255[5] = 0;
   out_6325063450262357255[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6325063450262357255[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6325063450262357255[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6325063450262357255[9] = 0;
   out_6325063450262357255[10] = 0;
   out_6325063450262357255[11] = 0;
   out_6325063450262357255[12] = 0;
   out_6325063450262357255[13] = 0;
   out_6325063450262357255[14] = 0;
   out_6325063450262357255[15] = 0;
   out_6325063450262357255[16] = 0;
   out_6325063450262357255[17] = 0;
   out_6325063450262357255[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6325063450262357255[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6325063450262357255[20] = 0;
   out_6325063450262357255[21] = 0;
   out_6325063450262357255[22] = 0;
   out_6325063450262357255[23] = 0;
   out_6325063450262357255[24] = 0;
   out_6325063450262357255[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6325063450262357255[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6325063450262357255[27] = 0;
   out_6325063450262357255[28] = 0;
   out_6325063450262357255[29] = 0;
   out_6325063450262357255[30] = 0;
   out_6325063450262357255[31] = 0;
   out_6325063450262357255[32] = 0;
   out_6325063450262357255[33] = 0;
   out_6325063450262357255[34] = 0;
   out_6325063450262357255[35] = 0;
   out_6325063450262357255[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6325063450262357255[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6325063450262357255[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6325063450262357255[39] = 0;
   out_6325063450262357255[40] = 0;
   out_6325063450262357255[41] = 0;
   out_6325063450262357255[42] = 0;
   out_6325063450262357255[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6325063450262357255[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6325063450262357255[45] = 0;
   out_6325063450262357255[46] = 0;
   out_6325063450262357255[47] = 0;
   out_6325063450262357255[48] = 0;
   out_6325063450262357255[49] = 0;
   out_6325063450262357255[50] = 0;
   out_6325063450262357255[51] = 0;
   out_6325063450262357255[52] = 0;
   out_6325063450262357255[53] = 0;
   out_6325063450262357255[54] = 0;
   out_6325063450262357255[55] = 0;
   out_6325063450262357255[56] = 0;
   out_6325063450262357255[57] = 1;
   out_6325063450262357255[58] = 0;
   out_6325063450262357255[59] = 0;
   out_6325063450262357255[60] = 0;
   out_6325063450262357255[61] = 0;
   out_6325063450262357255[62] = 0;
   out_6325063450262357255[63] = 0;
   out_6325063450262357255[64] = 0;
   out_6325063450262357255[65] = 0;
   out_6325063450262357255[66] = dt;
   out_6325063450262357255[67] = 0;
   out_6325063450262357255[68] = 0;
   out_6325063450262357255[69] = 0;
   out_6325063450262357255[70] = 0;
   out_6325063450262357255[71] = 0;
   out_6325063450262357255[72] = 0;
   out_6325063450262357255[73] = 0;
   out_6325063450262357255[74] = 0;
   out_6325063450262357255[75] = 0;
   out_6325063450262357255[76] = 1;
   out_6325063450262357255[77] = 0;
   out_6325063450262357255[78] = 0;
   out_6325063450262357255[79] = 0;
   out_6325063450262357255[80] = 0;
   out_6325063450262357255[81] = 0;
   out_6325063450262357255[82] = 0;
   out_6325063450262357255[83] = 0;
   out_6325063450262357255[84] = 0;
   out_6325063450262357255[85] = dt;
   out_6325063450262357255[86] = 0;
   out_6325063450262357255[87] = 0;
   out_6325063450262357255[88] = 0;
   out_6325063450262357255[89] = 0;
   out_6325063450262357255[90] = 0;
   out_6325063450262357255[91] = 0;
   out_6325063450262357255[92] = 0;
   out_6325063450262357255[93] = 0;
   out_6325063450262357255[94] = 0;
   out_6325063450262357255[95] = 1;
   out_6325063450262357255[96] = 0;
   out_6325063450262357255[97] = 0;
   out_6325063450262357255[98] = 0;
   out_6325063450262357255[99] = 0;
   out_6325063450262357255[100] = 0;
   out_6325063450262357255[101] = 0;
   out_6325063450262357255[102] = 0;
   out_6325063450262357255[103] = 0;
   out_6325063450262357255[104] = dt;
   out_6325063450262357255[105] = 0;
   out_6325063450262357255[106] = 0;
   out_6325063450262357255[107] = 0;
   out_6325063450262357255[108] = 0;
   out_6325063450262357255[109] = 0;
   out_6325063450262357255[110] = 0;
   out_6325063450262357255[111] = 0;
   out_6325063450262357255[112] = 0;
   out_6325063450262357255[113] = 0;
   out_6325063450262357255[114] = 1;
   out_6325063450262357255[115] = 0;
   out_6325063450262357255[116] = 0;
   out_6325063450262357255[117] = 0;
   out_6325063450262357255[118] = 0;
   out_6325063450262357255[119] = 0;
   out_6325063450262357255[120] = 0;
   out_6325063450262357255[121] = 0;
   out_6325063450262357255[122] = 0;
   out_6325063450262357255[123] = 0;
   out_6325063450262357255[124] = 0;
   out_6325063450262357255[125] = 0;
   out_6325063450262357255[126] = 0;
   out_6325063450262357255[127] = 0;
   out_6325063450262357255[128] = 0;
   out_6325063450262357255[129] = 0;
   out_6325063450262357255[130] = 0;
   out_6325063450262357255[131] = 0;
   out_6325063450262357255[132] = 0;
   out_6325063450262357255[133] = 1;
   out_6325063450262357255[134] = 0;
   out_6325063450262357255[135] = 0;
   out_6325063450262357255[136] = 0;
   out_6325063450262357255[137] = 0;
   out_6325063450262357255[138] = 0;
   out_6325063450262357255[139] = 0;
   out_6325063450262357255[140] = 0;
   out_6325063450262357255[141] = 0;
   out_6325063450262357255[142] = 0;
   out_6325063450262357255[143] = 0;
   out_6325063450262357255[144] = 0;
   out_6325063450262357255[145] = 0;
   out_6325063450262357255[146] = 0;
   out_6325063450262357255[147] = 0;
   out_6325063450262357255[148] = 0;
   out_6325063450262357255[149] = 0;
   out_6325063450262357255[150] = 0;
   out_6325063450262357255[151] = 0;
   out_6325063450262357255[152] = 1;
   out_6325063450262357255[153] = 0;
   out_6325063450262357255[154] = 0;
   out_6325063450262357255[155] = 0;
   out_6325063450262357255[156] = 0;
   out_6325063450262357255[157] = 0;
   out_6325063450262357255[158] = 0;
   out_6325063450262357255[159] = 0;
   out_6325063450262357255[160] = 0;
   out_6325063450262357255[161] = 0;
   out_6325063450262357255[162] = 0;
   out_6325063450262357255[163] = 0;
   out_6325063450262357255[164] = 0;
   out_6325063450262357255[165] = 0;
   out_6325063450262357255[166] = 0;
   out_6325063450262357255[167] = 0;
   out_6325063450262357255[168] = 0;
   out_6325063450262357255[169] = 0;
   out_6325063450262357255[170] = 0;
   out_6325063450262357255[171] = 1;
   out_6325063450262357255[172] = 0;
   out_6325063450262357255[173] = 0;
   out_6325063450262357255[174] = 0;
   out_6325063450262357255[175] = 0;
   out_6325063450262357255[176] = 0;
   out_6325063450262357255[177] = 0;
   out_6325063450262357255[178] = 0;
   out_6325063450262357255[179] = 0;
   out_6325063450262357255[180] = 0;
   out_6325063450262357255[181] = 0;
   out_6325063450262357255[182] = 0;
   out_6325063450262357255[183] = 0;
   out_6325063450262357255[184] = 0;
   out_6325063450262357255[185] = 0;
   out_6325063450262357255[186] = 0;
   out_6325063450262357255[187] = 0;
   out_6325063450262357255[188] = 0;
   out_6325063450262357255[189] = 0;
   out_6325063450262357255[190] = 1;
   out_6325063450262357255[191] = 0;
   out_6325063450262357255[192] = 0;
   out_6325063450262357255[193] = 0;
   out_6325063450262357255[194] = 0;
   out_6325063450262357255[195] = 0;
   out_6325063450262357255[196] = 0;
   out_6325063450262357255[197] = 0;
   out_6325063450262357255[198] = 0;
   out_6325063450262357255[199] = 0;
   out_6325063450262357255[200] = 0;
   out_6325063450262357255[201] = 0;
   out_6325063450262357255[202] = 0;
   out_6325063450262357255[203] = 0;
   out_6325063450262357255[204] = 0;
   out_6325063450262357255[205] = 0;
   out_6325063450262357255[206] = 0;
   out_6325063450262357255[207] = 0;
   out_6325063450262357255[208] = 0;
   out_6325063450262357255[209] = 1;
   out_6325063450262357255[210] = 0;
   out_6325063450262357255[211] = 0;
   out_6325063450262357255[212] = 0;
   out_6325063450262357255[213] = 0;
   out_6325063450262357255[214] = 0;
   out_6325063450262357255[215] = 0;
   out_6325063450262357255[216] = 0;
   out_6325063450262357255[217] = 0;
   out_6325063450262357255[218] = 0;
   out_6325063450262357255[219] = 0;
   out_6325063450262357255[220] = 0;
   out_6325063450262357255[221] = 0;
   out_6325063450262357255[222] = 0;
   out_6325063450262357255[223] = 0;
   out_6325063450262357255[224] = 0;
   out_6325063450262357255[225] = 0;
   out_6325063450262357255[226] = 0;
   out_6325063450262357255[227] = 0;
   out_6325063450262357255[228] = 1;
   out_6325063450262357255[229] = 0;
   out_6325063450262357255[230] = 0;
   out_6325063450262357255[231] = 0;
   out_6325063450262357255[232] = 0;
   out_6325063450262357255[233] = 0;
   out_6325063450262357255[234] = 0;
   out_6325063450262357255[235] = 0;
   out_6325063450262357255[236] = 0;
   out_6325063450262357255[237] = 0;
   out_6325063450262357255[238] = 0;
   out_6325063450262357255[239] = 0;
   out_6325063450262357255[240] = 0;
   out_6325063450262357255[241] = 0;
   out_6325063450262357255[242] = 0;
   out_6325063450262357255[243] = 0;
   out_6325063450262357255[244] = 0;
   out_6325063450262357255[245] = 0;
   out_6325063450262357255[246] = 0;
   out_6325063450262357255[247] = 1;
   out_6325063450262357255[248] = 0;
   out_6325063450262357255[249] = 0;
   out_6325063450262357255[250] = 0;
   out_6325063450262357255[251] = 0;
   out_6325063450262357255[252] = 0;
   out_6325063450262357255[253] = 0;
   out_6325063450262357255[254] = 0;
   out_6325063450262357255[255] = 0;
   out_6325063450262357255[256] = 0;
   out_6325063450262357255[257] = 0;
   out_6325063450262357255[258] = 0;
   out_6325063450262357255[259] = 0;
   out_6325063450262357255[260] = 0;
   out_6325063450262357255[261] = 0;
   out_6325063450262357255[262] = 0;
   out_6325063450262357255[263] = 0;
   out_6325063450262357255[264] = 0;
   out_6325063450262357255[265] = 0;
   out_6325063450262357255[266] = 1;
   out_6325063450262357255[267] = 0;
   out_6325063450262357255[268] = 0;
   out_6325063450262357255[269] = 0;
   out_6325063450262357255[270] = 0;
   out_6325063450262357255[271] = 0;
   out_6325063450262357255[272] = 0;
   out_6325063450262357255[273] = 0;
   out_6325063450262357255[274] = 0;
   out_6325063450262357255[275] = 0;
   out_6325063450262357255[276] = 0;
   out_6325063450262357255[277] = 0;
   out_6325063450262357255[278] = 0;
   out_6325063450262357255[279] = 0;
   out_6325063450262357255[280] = 0;
   out_6325063450262357255[281] = 0;
   out_6325063450262357255[282] = 0;
   out_6325063450262357255[283] = 0;
   out_6325063450262357255[284] = 0;
   out_6325063450262357255[285] = 1;
   out_6325063450262357255[286] = 0;
   out_6325063450262357255[287] = 0;
   out_6325063450262357255[288] = 0;
   out_6325063450262357255[289] = 0;
   out_6325063450262357255[290] = 0;
   out_6325063450262357255[291] = 0;
   out_6325063450262357255[292] = 0;
   out_6325063450262357255[293] = 0;
   out_6325063450262357255[294] = 0;
   out_6325063450262357255[295] = 0;
   out_6325063450262357255[296] = 0;
   out_6325063450262357255[297] = 0;
   out_6325063450262357255[298] = 0;
   out_6325063450262357255[299] = 0;
   out_6325063450262357255[300] = 0;
   out_6325063450262357255[301] = 0;
   out_6325063450262357255[302] = 0;
   out_6325063450262357255[303] = 0;
   out_6325063450262357255[304] = 1;
   out_6325063450262357255[305] = 0;
   out_6325063450262357255[306] = 0;
   out_6325063450262357255[307] = 0;
   out_6325063450262357255[308] = 0;
   out_6325063450262357255[309] = 0;
   out_6325063450262357255[310] = 0;
   out_6325063450262357255[311] = 0;
   out_6325063450262357255[312] = 0;
   out_6325063450262357255[313] = 0;
   out_6325063450262357255[314] = 0;
   out_6325063450262357255[315] = 0;
   out_6325063450262357255[316] = 0;
   out_6325063450262357255[317] = 0;
   out_6325063450262357255[318] = 0;
   out_6325063450262357255[319] = 0;
   out_6325063450262357255[320] = 0;
   out_6325063450262357255[321] = 0;
   out_6325063450262357255[322] = 0;
   out_6325063450262357255[323] = 1;
}
void h_4(double *state, double *unused, double *out_5863910480806422069) {
   out_5863910480806422069[0] = state[6] + state[9];
   out_5863910480806422069[1] = state[7] + state[10];
   out_5863910480806422069[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7712527653466602139) {
   out_7712527653466602139[0] = 0;
   out_7712527653466602139[1] = 0;
   out_7712527653466602139[2] = 0;
   out_7712527653466602139[3] = 0;
   out_7712527653466602139[4] = 0;
   out_7712527653466602139[5] = 0;
   out_7712527653466602139[6] = 1;
   out_7712527653466602139[7] = 0;
   out_7712527653466602139[8] = 0;
   out_7712527653466602139[9] = 1;
   out_7712527653466602139[10] = 0;
   out_7712527653466602139[11] = 0;
   out_7712527653466602139[12] = 0;
   out_7712527653466602139[13] = 0;
   out_7712527653466602139[14] = 0;
   out_7712527653466602139[15] = 0;
   out_7712527653466602139[16] = 0;
   out_7712527653466602139[17] = 0;
   out_7712527653466602139[18] = 0;
   out_7712527653466602139[19] = 0;
   out_7712527653466602139[20] = 0;
   out_7712527653466602139[21] = 0;
   out_7712527653466602139[22] = 0;
   out_7712527653466602139[23] = 0;
   out_7712527653466602139[24] = 0;
   out_7712527653466602139[25] = 1;
   out_7712527653466602139[26] = 0;
   out_7712527653466602139[27] = 0;
   out_7712527653466602139[28] = 1;
   out_7712527653466602139[29] = 0;
   out_7712527653466602139[30] = 0;
   out_7712527653466602139[31] = 0;
   out_7712527653466602139[32] = 0;
   out_7712527653466602139[33] = 0;
   out_7712527653466602139[34] = 0;
   out_7712527653466602139[35] = 0;
   out_7712527653466602139[36] = 0;
   out_7712527653466602139[37] = 0;
   out_7712527653466602139[38] = 0;
   out_7712527653466602139[39] = 0;
   out_7712527653466602139[40] = 0;
   out_7712527653466602139[41] = 0;
   out_7712527653466602139[42] = 0;
   out_7712527653466602139[43] = 0;
   out_7712527653466602139[44] = 1;
   out_7712527653466602139[45] = 0;
   out_7712527653466602139[46] = 0;
   out_7712527653466602139[47] = 1;
   out_7712527653466602139[48] = 0;
   out_7712527653466602139[49] = 0;
   out_7712527653466602139[50] = 0;
   out_7712527653466602139[51] = 0;
   out_7712527653466602139[52] = 0;
   out_7712527653466602139[53] = 0;
}
void h_10(double *state, double *unused, double *out_5277016606615583295) {
   out_5277016606615583295[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5277016606615583295[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5277016606615583295[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2723658752885143762) {
   out_2723658752885143762[0] = 0;
   out_2723658752885143762[1] = 9.8100000000000005*cos(state[1]);
   out_2723658752885143762[2] = 0;
   out_2723658752885143762[3] = 0;
   out_2723658752885143762[4] = -state[8];
   out_2723658752885143762[5] = state[7];
   out_2723658752885143762[6] = 0;
   out_2723658752885143762[7] = state[5];
   out_2723658752885143762[8] = -state[4];
   out_2723658752885143762[9] = 0;
   out_2723658752885143762[10] = 0;
   out_2723658752885143762[11] = 0;
   out_2723658752885143762[12] = 1;
   out_2723658752885143762[13] = 0;
   out_2723658752885143762[14] = 0;
   out_2723658752885143762[15] = 1;
   out_2723658752885143762[16] = 0;
   out_2723658752885143762[17] = 0;
   out_2723658752885143762[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2723658752885143762[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2723658752885143762[20] = 0;
   out_2723658752885143762[21] = state[8];
   out_2723658752885143762[22] = 0;
   out_2723658752885143762[23] = -state[6];
   out_2723658752885143762[24] = -state[5];
   out_2723658752885143762[25] = 0;
   out_2723658752885143762[26] = state[3];
   out_2723658752885143762[27] = 0;
   out_2723658752885143762[28] = 0;
   out_2723658752885143762[29] = 0;
   out_2723658752885143762[30] = 0;
   out_2723658752885143762[31] = 1;
   out_2723658752885143762[32] = 0;
   out_2723658752885143762[33] = 0;
   out_2723658752885143762[34] = 1;
   out_2723658752885143762[35] = 0;
   out_2723658752885143762[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2723658752885143762[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2723658752885143762[38] = 0;
   out_2723658752885143762[39] = -state[7];
   out_2723658752885143762[40] = state[6];
   out_2723658752885143762[41] = 0;
   out_2723658752885143762[42] = state[4];
   out_2723658752885143762[43] = -state[3];
   out_2723658752885143762[44] = 0;
   out_2723658752885143762[45] = 0;
   out_2723658752885143762[46] = 0;
   out_2723658752885143762[47] = 0;
   out_2723658752885143762[48] = 0;
   out_2723658752885143762[49] = 0;
   out_2723658752885143762[50] = 1;
   out_2723658752885143762[51] = 0;
   out_2723658752885143762[52] = 0;
   out_2723658752885143762[53] = 1;
}
void h_13(double *state, double *unused, double *out_4417683070087264532) {
   out_4417683070087264532[0] = state[3];
   out_4417683070087264532[1] = state[4];
   out_4417683070087264532[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6900460956940425453) {
   out_6900460956940425453[0] = 0;
   out_6900460956940425453[1] = 0;
   out_6900460956940425453[2] = 0;
   out_6900460956940425453[3] = 1;
   out_6900460956940425453[4] = 0;
   out_6900460956940425453[5] = 0;
   out_6900460956940425453[6] = 0;
   out_6900460956940425453[7] = 0;
   out_6900460956940425453[8] = 0;
   out_6900460956940425453[9] = 0;
   out_6900460956940425453[10] = 0;
   out_6900460956940425453[11] = 0;
   out_6900460956940425453[12] = 0;
   out_6900460956940425453[13] = 0;
   out_6900460956940425453[14] = 0;
   out_6900460956940425453[15] = 0;
   out_6900460956940425453[16] = 0;
   out_6900460956940425453[17] = 0;
   out_6900460956940425453[18] = 0;
   out_6900460956940425453[19] = 0;
   out_6900460956940425453[20] = 0;
   out_6900460956940425453[21] = 0;
   out_6900460956940425453[22] = 1;
   out_6900460956940425453[23] = 0;
   out_6900460956940425453[24] = 0;
   out_6900460956940425453[25] = 0;
   out_6900460956940425453[26] = 0;
   out_6900460956940425453[27] = 0;
   out_6900460956940425453[28] = 0;
   out_6900460956940425453[29] = 0;
   out_6900460956940425453[30] = 0;
   out_6900460956940425453[31] = 0;
   out_6900460956940425453[32] = 0;
   out_6900460956940425453[33] = 0;
   out_6900460956940425453[34] = 0;
   out_6900460956940425453[35] = 0;
   out_6900460956940425453[36] = 0;
   out_6900460956940425453[37] = 0;
   out_6900460956940425453[38] = 0;
   out_6900460956940425453[39] = 0;
   out_6900460956940425453[40] = 0;
   out_6900460956940425453[41] = 1;
   out_6900460956940425453[42] = 0;
   out_6900460956940425453[43] = 0;
   out_6900460956940425453[44] = 0;
   out_6900460956940425453[45] = 0;
   out_6900460956940425453[46] = 0;
   out_6900460956940425453[47] = 0;
   out_6900460956940425453[48] = 0;
   out_6900460956940425453[49] = 0;
   out_6900460956940425453[50] = 0;
   out_6900460956940425453[51] = 0;
   out_6900460956940425453[52] = 0;
   out_6900460956940425453[53] = 0;
}
void h_14(double *state, double *unused, double *out_7264994596821899571) {
   out_7264994596821899571[0] = state[6];
   out_7264994596821899571[1] = state[7];
   out_7264994596821899571[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7651427987947577181) {
   out_7651427987947577181[0] = 0;
   out_7651427987947577181[1] = 0;
   out_7651427987947577181[2] = 0;
   out_7651427987947577181[3] = 0;
   out_7651427987947577181[4] = 0;
   out_7651427987947577181[5] = 0;
   out_7651427987947577181[6] = 1;
   out_7651427987947577181[7] = 0;
   out_7651427987947577181[8] = 0;
   out_7651427987947577181[9] = 0;
   out_7651427987947577181[10] = 0;
   out_7651427987947577181[11] = 0;
   out_7651427987947577181[12] = 0;
   out_7651427987947577181[13] = 0;
   out_7651427987947577181[14] = 0;
   out_7651427987947577181[15] = 0;
   out_7651427987947577181[16] = 0;
   out_7651427987947577181[17] = 0;
   out_7651427987947577181[18] = 0;
   out_7651427987947577181[19] = 0;
   out_7651427987947577181[20] = 0;
   out_7651427987947577181[21] = 0;
   out_7651427987947577181[22] = 0;
   out_7651427987947577181[23] = 0;
   out_7651427987947577181[24] = 0;
   out_7651427987947577181[25] = 1;
   out_7651427987947577181[26] = 0;
   out_7651427987947577181[27] = 0;
   out_7651427987947577181[28] = 0;
   out_7651427987947577181[29] = 0;
   out_7651427987947577181[30] = 0;
   out_7651427987947577181[31] = 0;
   out_7651427987947577181[32] = 0;
   out_7651427987947577181[33] = 0;
   out_7651427987947577181[34] = 0;
   out_7651427987947577181[35] = 0;
   out_7651427987947577181[36] = 0;
   out_7651427987947577181[37] = 0;
   out_7651427987947577181[38] = 0;
   out_7651427987947577181[39] = 0;
   out_7651427987947577181[40] = 0;
   out_7651427987947577181[41] = 0;
   out_7651427987947577181[42] = 0;
   out_7651427987947577181[43] = 0;
   out_7651427987947577181[44] = 1;
   out_7651427987947577181[45] = 0;
   out_7651427987947577181[46] = 0;
   out_7651427987947577181[47] = 0;
   out_7651427987947577181[48] = 0;
   out_7651427987947577181[49] = 0;
   out_7651427987947577181[50] = 0;
   out_7651427987947577181[51] = 0;
   out_7651427987947577181[52] = 0;
   out_7651427987947577181[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_6074688492235971640) {
  err_fun(nom_x, delta_x, out_6074688492235971640);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7256018833813533895) {
  inv_err_fun(nom_x, true_x, out_7256018833813533895);
}
void pose_H_mod_fun(double *state, double *out_5581698035203868742) {
  H_mod_fun(state, out_5581698035203868742);
}
void pose_f_fun(double *state, double dt, double *out_2476147660253971784) {
  f_fun(state,  dt, out_2476147660253971784);
}
void pose_F_fun(double *state, double dt, double *out_6325063450262357255) {
  F_fun(state,  dt, out_6325063450262357255);
}
void pose_h_4(double *state, double *unused, double *out_5863910480806422069) {
  h_4(state, unused, out_5863910480806422069);
}
void pose_H_4(double *state, double *unused, double *out_7712527653466602139) {
  H_4(state, unused, out_7712527653466602139);
}
void pose_h_10(double *state, double *unused, double *out_5277016606615583295) {
  h_10(state, unused, out_5277016606615583295);
}
void pose_H_10(double *state, double *unused, double *out_2723658752885143762) {
  H_10(state, unused, out_2723658752885143762);
}
void pose_h_13(double *state, double *unused, double *out_4417683070087264532) {
  h_13(state, unused, out_4417683070087264532);
}
void pose_H_13(double *state, double *unused, double *out_6900460956940425453) {
  H_13(state, unused, out_6900460956940425453);
}
void pose_h_14(double *state, double *unused, double *out_7264994596821899571) {
  h_14(state, unused, out_7264994596821899571);
}
void pose_H_14(double *state, double *unused, double *out_7651427987947577181) {
  H_14(state, unused, out_7651427987947577181);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
