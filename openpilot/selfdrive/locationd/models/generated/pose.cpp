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
void err_fun(double *nom_x, double *delta_x, double *out_1238705429075317501) {
   out_1238705429075317501[0] = delta_x[0] + nom_x[0];
   out_1238705429075317501[1] = delta_x[1] + nom_x[1];
   out_1238705429075317501[2] = delta_x[2] + nom_x[2];
   out_1238705429075317501[3] = delta_x[3] + nom_x[3];
   out_1238705429075317501[4] = delta_x[4] + nom_x[4];
   out_1238705429075317501[5] = delta_x[5] + nom_x[5];
   out_1238705429075317501[6] = delta_x[6] + nom_x[6];
   out_1238705429075317501[7] = delta_x[7] + nom_x[7];
   out_1238705429075317501[8] = delta_x[8] + nom_x[8];
   out_1238705429075317501[9] = delta_x[9] + nom_x[9];
   out_1238705429075317501[10] = delta_x[10] + nom_x[10];
   out_1238705429075317501[11] = delta_x[11] + nom_x[11];
   out_1238705429075317501[12] = delta_x[12] + nom_x[12];
   out_1238705429075317501[13] = delta_x[13] + nom_x[13];
   out_1238705429075317501[14] = delta_x[14] + nom_x[14];
   out_1238705429075317501[15] = delta_x[15] + nom_x[15];
   out_1238705429075317501[16] = delta_x[16] + nom_x[16];
   out_1238705429075317501[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3460389199729958721) {
   out_3460389199729958721[0] = -nom_x[0] + true_x[0];
   out_3460389199729958721[1] = -nom_x[1] + true_x[1];
   out_3460389199729958721[2] = -nom_x[2] + true_x[2];
   out_3460389199729958721[3] = -nom_x[3] + true_x[3];
   out_3460389199729958721[4] = -nom_x[4] + true_x[4];
   out_3460389199729958721[5] = -nom_x[5] + true_x[5];
   out_3460389199729958721[6] = -nom_x[6] + true_x[6];
   out_3460389199729958721[7] = -nom_x[7] + true_x[7];
   out_3460389199729958721[8] = -nom_x[8] + true_x[8];
   out_3460389199729958721[9] = -nom_x[9] + true_x[9];
   out_3460389199729958721[10] = -nom_x[10] + true_x[10];
   out_3460389199729958721[11] = -nom_x[11] + true_x[11];
   out_3460389199729958721[12] = -nom_x[12] + true_x[12];
   out_3460389199729958721[13] = -nom_x[13] + true_x[13];
   out_3460389199729958721[14] = -nom_x[14] + true_x[14];
   out_3460389199729958721[15] = -nom_x[15] + true_x[15];
   out_3460389199729958721[16] = -nom_x[16] + true_x[16];
   out_3460389199729958721[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5700165656230282766) {
   out_5700165656230282766[0] = 1.0;
   out_5700165656230282766[1] = 0.0;
   out_5700165656230282766[2] = 0.0;
   out_5700165656230282766[3] = 0.0;
   out_5700165656230282766[4] = 0.0;
   out_5700165656230282766[5] = 0.0;
   out_5700165656230282766[6] = 0.0;
   out_5700165656230282766[7] = 0.0;
   out_5700165656230282766[8] = 0.0;
   out_5700165656230282766[9] = 0.0;
   out_5700165656230282766[10] = 0.0;
   out_5700165656230282766[11] = 0.0;
   out_5700165656230282766[12] = 0.0;
   out_5700165656230282766[13] = 0.0;
   out_5700165656230282766[14] = 0.0;
   out_5700165656230282766[15] = 0.0;
   out_5700165656230282766[16] = 0.0;
   out_5700165656230282766[17] = 0.0;
   out_5700165656230282766[18] = 0.0;
   out_5700165656230282766[19] = 1.0;
   out_5700165656230282766[20] = 0.0;
   out_5700165656230282766[21] = 0.0;
   out_5700165656230282766[22] = 0.0;
   out_5700165656230282766[23] = 0.0;
   out_5700165656230282766[24] = 0.0;
   out_5700165656230282766[25] = 0.0;
   out_5700165656230282766[26] = 0.0;
   out_5700165656230282766[27] = 0.0;
   out_5700165656230282766[28] = 0.0;
   out_5700165656230282766[29] = 0.0;
   out_5700165656230282766[30] = 0.0;
   out_5700165656230282766[31] = 0.0;
   out_5700165656230282766[32] = 0.0;
   out_5700165656230282766[33] = 0.0;
   out_5700165656230282766[34] = 0.0;
   out_5700165656230282766[35] = 0.0;
   out_5700165656230282766[36] = 0.0;
   out_5700165656230282766[37] = 0.0;
   out_5700165656230282766[38] = 1.0;
   out_5700165656230282766[39] = 0.0;
   out_5700165656230282766[40] = 0.0;
   out_5700165656230282766[41] = 0.0;
   out_5700165656230282766[42] = 0.0;
   out_5700165656230282766[43] = 0.0;
   out_5700165656230282766[44] = 0.0;
   out_5700165656230282766[45] = 0.0;
   out_5700165656230282766[46] = 0.0;
   out_5700165656230282766[47] = 0.0;
   out_5700165656230282766[48] = 0.0;
   out_5700165656230282766[49] = 0.0;
   out_5700165656230282766[50] = 0.0;
   out_5700165656230282766[51] = 0.0;
   out_5700165656230282766[52] = 0.0;
   out_5700165656230282766[53] = 0.0;
   out_5700165656230282766[54] = 0.0;
   out_5700165656230282766[55] = 0.0;
   out_5700165656230282766[56] = 0.0;
   out_5700165656230282766[57] = 1.0;
   out_5700165656230282766[58] = 0.0;
   out_5700165656230282766[59] = 0.0;
   out_5700165656230282766[60] = 0.0;
   out_5700165656230282766[61] = 0.0;
   out_5700165656230282766[62] = 0.0;
   out_5700165656230282766[63] = 0.0;
   out_5700165656230282766[64] = 0.0;
   out_5700165656230282766[65] = 0.0;
   out_5700165656230282766[66] = 0.0;
   out_5700165656230282766[67] = 0.0;
   out_5700165656230282766[68] = 0.0;
   out_5700165656230282766[69] = 0.0;
   out_5700165656230282766[70] = 0.0;
   out_5700165656230282766[71] = 0.0;
   out_5700165656230282766[72] = 0.0;
   out_5700165656230282766[73] = 0.0;
   out_5700165656230282766[74] = 0.0;
   out_5700165656230282766[75] = 0.0;
   out_5700165656230282766[76] = 1.0;
   out_5700165656230282766[77] = 0.0;
   out_5700165656230282766[78] = 0.0;
   out_5700165656230282766[79] = 0.0;
   out_5700165656230282766[80] = 0.0;
   out_5700165656230282766[81] = 0.0;
   out_5700165656230282766[82] = 0.0;
   out_5700165656230282766[83] = 0.0;
   out_5700165656230282766[84] = 0.0;
   out_5700165656230282766[85] = 0.0;
   out_5700165656230282766[86] = 0.0;
   out_5700165656230282766[87] = 0.0;
   out_5700165656230282766[88] = 0.0;
   out_5700165656230282766[89] = 0.0;
   out_5700165656230282766[90] = 0.0;
   out_5700165656230282766[91] = 0.0;
   out_5700165656230282766[92] = 0.0;
   out_5700165656230282766[93] = 0.0;
   out_5700165656230282766[94] = 0.0;
   out_5700165656230282766[95] = 1.0;
   out_5700165656230282766[96] = 0.0;
   out_5700165656230282766[97] = 0.0;
   out_5700165656230282766[98] = 0.0;
   out_5700165656230282766[99] = 0.0;
   out_5700165656230282766[100] = 0.0;
   out_5700165656230282766[101] = 0.0;
   out_5700165656230282766[102] = 0.0;
   out_5700165656230282766[103] = 0.0;
   out_5700165656230282766[104] = 0.0;
   out_5700165656230282766[105] = 0.0;
   out_5700165656230282766[106] = 0.0;
   out_5700165656230282766[107] = 0.0;
   out_5700165656230282766[108] = 0.0;
   out_5700165656230282766[109] = 0.0;
   out_5700165656230282766[110] = 0.0;
   out_5700165656230282766[111] = 0.0;
   out_5700165656230282766[112] = 0.0;
   out_5700165656230282766[113] = 0.0;
   out_5700165656230282766[114] = 1.0;
   out_5700165656230282766[115] = 0.0;
   out_5700165656230282766[116] = 0.0;
   out_5700165656230282766[117] = 0.0;
   out_5700165656230282766[118] = 0.0;
   out_5700165656230282766[119] = 0.0;
   out_5700165656230282766[120] = 0.0;
   out_5700165656230282766[121] = 0.0;
   out_5700165656230282766[122] = 0.0;
   out_5700165656230282766[123] = 0.0;
   out_5700165656230282766[124] = 0.0;
   out_5700165656230282766[125] = 0.0;
   out_5700165656230282766[126] = 0.0;
   out_5700165656230282766[127] = 0.0;
   out_5700165656230282766[128] = 0.0;
   out_5700165656230282766[129] = 0.0;
   out_5700165656230282766[130] = 0.0;
   out_5700165656230282766[131] = 0.0;
   out_5700165656230282766[132] = 0.0;
   out_5700165656230282766[133] = 1.0;
   out_5700165656230282766[134] = 0.0;
   out_5700165656230282766[135] = 0.0;
   out_5700165656230282766[136] = 0.0;
   out_5700165656230282766[137] = 0.0;
   out_5700165656230282766[138] = 0.0;
   out_5700165656230282766[139] = 0.0;
   out_5700165656230282766[140] = 0.0;
   out_5700165656230282766[141] = 0.0;
   out_5700165656230282766[142] = 0.0;
   out_5700165656230282766[143] = 0.0;
   out_5700165656230282766[144] = 0.0;
   out_5700165656230282766[145] = 0.0;
   out_5700165656230282766[146] = 0.0;
   out_5700165656230282766[147] = 0.0;
   out_5700165656230282766[148] = 0.0;
   out_5700165656230282766[149] = 0.0;
   out_5700165656230282766[150] = 0.0;
   out_5700165656230282766[151] = 0.0;
   out_5700165656230282766[152] = 1.0;
   out_5700165656230282766[153] = 0.0;
   out_5700165656230282766[154] = 0.0;
   out_5700165656230282766[155] = 0.0;
   out_5700165656230282766[156] = 0.0;
   out_5700165656230282766[157] = 0.0;
   out_5700165656230282766[158] = 0.0;
   out_5700165656230282766[159] = 0.0;
   out_5700165656230282766[160] = 0.0;
   out_5700165656230282766[161] = 0.0;
   out_5700165656230282766[162] = 0.0;
   out_5700165656230282766[163] = 0.0;
   out_5700165656230282766[164] = 0.0;
   out_5700165656230282766[165] = 0.0;
   out_5700165656230282766[166] = 0.0;
   out_5700165656230282766[167] = 0.0;
   out_5700165656230282766[168] = 0.0;
   out_5700165656230282766[169] = 0.0;
   out_5700165656230282766[170] = 0.0;
   out_5700165656230282766[171] = 1.0;
   out_5700165656230282766[172] = 0.0;
   out_5700165656230282766[173] = 0.0;
   out_5700165656230282766[174] = 0.0;
   out_5700165656230282766[175] = 0.0;
   out_5700165656230282766[176] = 0.0;
   out_5700165656230282766[177] = 0.0;
   out_5700165656230282766[178] = 0.0;
   out_5700165656230282766[179] = 0.0;
   out_5700165656230282766[180] = 0.0;
   out_5700165656230282766[181] = 0.0;
   out_5700165656230282766[182] = 0.0;
   out_5700165656230282766[183] = 0.0;
   out_5700165656230282766[184] = 0.0;
   out_5700165656230282766[185] = 0.0;
   out_5700165656230282766[186] = 0.0;
   out_5700165656230282766[187] = 0.0;
   out_5700165656230282766[188] = 0.0;
   out_5700165656230282766[189] = 0.0;
   out_5700165656230282766[190] = 1.0;
   out_5700165656230282766[191] = 0.0;
   out_5700165656230282766[192] = 0.0;
   out_5700165656230282766[193] = 0.0;
   out_5700165656230282766[194] = 0.0;
   out_5700165656230282766[195] = 0.0;
   out_5700165656230282766[196] = 0.0;
   out_5700165656230282766[197] = 0.0;
   out_5700165656230282766[198] = 0.0;
   out_5700165656230282766[199] = 0.0;
   out_5700165656230282766[200] = 0.0;
   out_5700165656230282766[201] = 0.0;
   out_5700165656230282766[202] = 0.0;
   out_5700165656230282766[203] = 0.0;
   out_5700165656230282766[204] = 0.0;
   out_5700165656230282766[205] = 0.0;
   out_5700165656230282766[206] = 0.0;
   out_5700165656230282766[207] = 0.0;
   out_5700165656230282766[208] = 0.0;
   out_5700165656230282766[209] = 1.0;
   out_5700165656230282766[210] = 0.0;
   out_5700165656230282766[211] = 0.0;
   out_5700165656230282766[212] = 0.0;
   out_5700165656230282766[213] = 0.0;
   out_5700165656230282766[214] = 0.0;
   out_5700165656230282766[215] = 0.0;
   out_5700165656230282766[216] = 0.0;
   out_5700165656230282766[217] = 0.0;
   out_5700165656230282766[218] = 0.0;
   out_5700165656230282766[219] = 0.0;
   out_5700165656230282766[220] = 0.0;
   out_5700165656230282766[221] = 0.0;
   out_5700165656230282766[222] = 0.0;
   out_5700165656230282766[223] = 0.0;
   out_5700165656230282766[224] = 0.0;
   out_5700165656230282766[225] = 0.0;
   out_5700165656230282766[226] = 0.0;
   out_5700165656230282766[227] = 0.0;
   out_5700165656230282766[228] = 1.0;
   out_5700165656230282766[229] = 0.0;
   out_5700165656230282766[230] = 0.0;
   out_5700165656230282766[231] = 0.0;
   out_5700165656230282766[232] = 0.0;
   out_5700165656230282766[233] = 0.0;
   out_5700165656230282766[234] = 0.0;
   out_5700165656230282766[235] = 0.0;
   out_5700165656230282766[236] = 0.0;
   out_5700165656230282766[237] = 0.0;
   out_5700165656230282766[238] = 0.0;
   out_5700165656230282766[239] = 0.0;
   out_5700165656230282766[240] = 0.0;
   out_5700165656230282766[241] = 0.0;
   out_5700165656230282766[242] = 0.0;
   out_5700165656230282766[243] = 0.0;
   out_5700165656230282766[244] = 0.0;
   out_5700165656230282766[245] = 0.0;
   out_5700165656230282766[246] = 0.0;
   out_5700165656230282766[247] = 1.0;
   out_5700165656230282766[248] = 0.0;
   out_5700165656230282766[249] = 0.0;
   out_5700165656230282766[250] = 0.0;
   out_5700165656230282766[251] = 0.0;
   out_5700165656230282766[252] = 0.0;
   out_5700165656230282766[253] = 0.0;
   out_5700165656230282766[254] = 0.0;
   out_5700165656230282766[255] = 0.0;
   out_5700165656230282766[256] = 0.0;
   out_5700165656230282766[257] = 0.0;
   out_5700165656230282766[258] = 0.0;
   out_5700165656230282766[259] = 0.0;
   out_5700165656230282766[260] = 0.0;
   out_5700165656230282766[261] = 0.0;
   out_5700165656230282766[262] = 0.0;
   out_5700165656230282766[263] = 0.0;
   out_5700165656230282766[264] = 0.0;
   out_5700165656230282766[265] = 0.0;
   out_5700165656230282766[266] = 1.0;
   out_5700165656230282766[267] = 0.0;
   out_5700165656230282766[268] = 0.0;
   out_5700165656230282766[269] = 0.0;
   out_5700165656230282766[270] = 0.0;
   out_5700165656230282766[271] = 0.0;
   out_5700165656230282766[272] = 0.0;
   out_5700165656230282766[273] = 0.0;
   out_5700165656230282766[274] = 0.0;
   out_5700165656230282766[275] = 0.0;
   out_5700165656230282766[276] = 0.0;
   out_5700165656230282766[277] = 0.0;
   out_5700165656230282766[278] = 0.0;
   out_5700165656230282766[279] = 0.0;
   out_5700165656230282766[280] = 0.0;
   out_5700165656230282766[281] = 0.0;
   out_5700165656230282766[282] = 0.0;
   out_5700165656230282766[283] = 0.0;
   out_5700165656230282766[284] = 0.0;
   out_5700165656230282766[285] = 1.0;
   out_5700165656230282766[286] = 0.0;
   out_5700165656230282766[287] = 0.0;
   out_5700165656230282766[288] = 0.0;
   out_5700165656230282766[289] = 0.0;
   out_5700165656230282766[290] = 0.0;
   out_5700165656230282766[291] = 0.0;
   out_5700165656230282766[292] = 0.0;
   out_5700165656230282766[293] = 0.0;
   out_5700165656230282766[294] = 0.0;
   out_5700165656230282766[295] = 0.0;
   out_5700165656230282766[296] = 0.0;
   out_5700165656230282766[297] = 0.0;
   out_5700165656230282766[298] = 0.0;
   out_5700165656230282766[299] = 0.0;
   out_5700165656230282766[300] = 0.0;
   out_5700165656230282766[301] = 0.0;
   out_5700165656230282766[302] = 0.0;
   out_5700165656230282766[303] = 0.0;
   out_5700165656230282766[304] = 1.0;
   out_5700165656230282766[305] = 0.0;
   out_5700165656230282766[306] = 0.0;
   out_5700165656230282766[307] = 0.0;
   out_5700165656230282766[308] = 0.0;
   out_5700165656230282766[309] = 0.0;
   out_5700165656230282766[310] = 0.0;
   out_5700165656230282766[311] = 0.0;
   out_5700165656230282766[312] = 0.0;
   out_5700165656230282766[313] = 0.0;
   out_5700165656230282766[314] = 0.0;
   out_5700165656230282766[315] = 0.0;
   out_5700165656230282766[316] = 0.0;
   out_5700165656230282766[317] = 0.0;
   out_5700165656230282766[318] = 0.0;
   out_5700165656230282766[319] = 0.0;
   out_5700165656230282766[320] = 0.0;
   out_5700165656230282766[321] = 0.0;
   out_5700165656230282766[322] = 0.0;
   out_5700165656230282766[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4518422097938204125) {
   out_4518422097938204125[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4518422097938204125[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4518422097938204125[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4518422097938204125[3] = dt*state[12] + state[3];
   out_4518422097938204125[4] = dt*state[13] + state[4];
   out_4518422097938204125[5] = dt*state[14] + state[5];
   out_4518422097938204125[6] = state[6];
   out_4518422097938204125[7] = state[7];
   out_4518422097938204125[8] = state[8];
   out_4518422097938204125[9] = state[9];
   out_4518422097938204125[10] = state[10];
   out_4518422097938204125[11] = state[11];
   out_4518422097938204125[12] = state[12];
   out_4518422097938204125[13] = state[13];
   out_4518422097938204125[14] = state[14];
   out_4518422097938204125[15] = state[15];
   out_4518422097938204125[16] = state[16];
   out_4518422097938204125[17] = state[17];
}
void F_fun(double *state, double dt, double *out_794072923466735339) {
   out_794072923466735339[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_794072923466735339[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_794072923466735339[2] = 0;
   out_794072923466735339[3] = 0;
   out_794072923466735339[4] = 0;
   out_794072923466735339[5] = 0;
   out_794072923466735339[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_794072923466735339[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_794072923466735339[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_794072923466735339[9] = 0;
   out_794072923466735339[10] = 0;
   out_794072923466735339[11] = 0;
   out_794072923466735339[12] = 0;
   out_794072923466735339[13] = 0;
   out_794072923466735339[14] = 0;
   out_794072923466735339[15] = 0;
   out_794072923466735339[16] = 0;
   out_794072923466735339[17] = 0;
   out_794072923466735339[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_794072923466735339[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_794072923466735339[20] = 0;
   out_794072923466735339[21] = 0;
   out_794072923466735339[22] = 0;
   out_794072923466735339[23] = 0;
   out_794072923466735339[24] = 0;
   out_794072923466735339[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_794072923466735339[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_794072923466735339[27] = 0;
   out_794072923466735339[28] = 0;
   out_794072923466735339[29] = 0;
   out_794072923466735339[30] = 0;
   out_794072923466735339[31] = 0;
   out_794072923466735339[32] = 0;
   out_794072923466735339[33] = 0;
   out_794072923466735339[34] = 0;
   out_794072923466735339[35] = 0;
   out_794072923466735339[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_794072923466735339[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_794072923466735339[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_794072923466735339[39] = 0;
   out_794072923466735339[40] = 0;
   out_794072923466735339[41] = 0;
   out_794072923466735339[42] = 0;
   out_794072923466735339[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_794072923466735339[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_794072923466735339[45] = 0;
   out_794072923466735339[46] = 0;
   out_794072923466735339[47] = 0;
   out_794072923466735339[48] = 0;
   out_794072923466735339[49] = 0;
   out_794072923466735339[50] = 0;
   out_794072923466735339[51] = 0;
   out_794072923466735339[52] = 0;
   out_794072923466735339[53] = 0;
   out_794072923466735339[54] = 0;
   out_794072923466735339[55] = 0;
   out_794072923466735339[56] = 0;
   out_794072923466735339[57] = 1;
   out_794072923466735339[58] = 0;
   out_794072923466735339[59] = 0;
   out_794072923466735339[60] = 0;
   out_794072923466735339[61] = 0;
   out_794072923466735339[62] = 0;
   out_794072923466735339[63] = 0;
   out_794072923466735339[64] = 0;
   out_794072923466735339[65] = 0;
   out_794072923466735339[66] = dt;
   out_794072923466735339[67] = 0;
   out_794072923466735339[68] = 0;
   out_794072923466735339[69] = 0;
   out_794072923466735339[70] = 0;
   out_794072923466735339[71] = 0;
   out_794072923466735339[72] = 0;
   out_794072923466735339[73] = 0;
   out_794072923466735339[74] = 0;
   out_794072923466735339[75] = 0;
   out_794072923466735339[76] = 1;
   out_794072923466735339[77] = 0;
   out_794072923466735339[78] = 0;
   out_794072923466735339[79] = 0;
   out_794072923466735339[80] = 0;
   out_794072923466735339[81] = 0;
   out_794072923466735339[82] = 0;
   out_794072923466735339[83] = 0;
   out_794072923466735339[84] = 0;
   out_794072923466735339[85] = dt;
   out_794072923466735339[86] = 0;
   out_794072923466735339[87] = 0;
   out_794072923466735339[88] = 0;
   out_794072923466735339[89] = 0;
   out_794072923466735339[90] = 0;
   out_794072923466735339[91] = 0;
   out_794072923466735339[92] = 0;
   out_794072923466735339[93] = 0;
   out_794072923466735339[94] = 0;
   out_794072923466735339[95] = 1;
   out_794072923466735339[96] = 0;
   out_794072923466735339[97] = 0;
   out_794072923466735339[98] = 0;
   out_794072923466735339[99] = 0;
   out_794072923466735339[100] = 0;
   out_794072923466735339[101] = 0;
   out_794072923466735339[102] = 0;
   out_794072923466735339[103] = 0;
   out_794072923466735339[104] = dt;
   out_794072923466735339[105] = 0;
   out_794072923466735339[106] = 0;
   out_794072923466735339[107] = 0;
   out_794072923466735339[108] = 0;
   out_794072923466735339[109] = 0;
   out_794072923466735339[110] = 0;
   out_794072923466735339[111] = 0;
   out_794072923466735339[112] = 0;
   out_794072923466735339[113] = 0;
   out_794072923466735339[114] = 1;
   out_794072923466735339[115] = 0;
   out_794072923466735339[116] = 0;
   out_794072923466735339[117] = 0;
   out_794072923466735339[118] = 0;
   out_794072923466735339[119] = 0;
   out_794072923466735339[120] = 0;
   out_794072923466735339[121] = 0;
   out_794072923466735339[122] = 0;
   out_794072923466735339[123] = 0;
   out_794072923466735339[124] = 0;
   out_794072923466735339[125] = 0;
   out_794072923466735339[126] = 0;
   out_794072923466735339[127] = 0;
   out_794072923466735339[128] = 0;
   out_794072923466735339[129] = 0;
   out_794072923466735339[130] = 0;
   out_794072923466735339[131] = 0;
   out_794072923466735339[132] = 0;
   out_794072923466735339[133] = 1;
   out_794072923466735339[134] = 0;
   out_794072923466735339[135] = 0;
   out_794072923466735339[136] = 0;
   out_794072923466735339[137] = 0;
   out_794072923466735339[138] = 0;
   out_794072923466735339[139] = 0;
   out_794072923466735339[140] = 0;
   out_794072923466735339[141] = 0;
   out_794072923466735339[142] = 0;
   out_794072923466735339[143] = 0;
   out_794072923466735339[144] = 0;
   out_794072923466735339[145] = 0;
   out_794072923466735339[146] = 0;
   out_794072923466735339[147] = 0;
   out_794072923466735339[148] = 0;
   out_794072923466735339[149] = 0;
   out_794072923466735339[150] = 0;
   out_794072923466735339[151] = 0;
   out_794072923466735339[152] = 1;
   out_794072923466735339[153] = 0;
   out_794072923466735339[154] = 0;
   out_794072923466735339[155] = 0;
   out_794072923466735339[156] = 0;
   out_794072923466735339[157] = 0;
   out_794072923466735339[158] = 0;
   out_794072923466735339[159] = 0;
   out_794072923466735339[160] = 0;
   out_794072923466735339[161] = 0;
   out_794072923466735339[162] = 0;
   out_794072923466735339[163] = 0;
   out_794072923466735339[164] = 0;
   out_794072923466735339[165] = 0;
   out_794072923466735339[166] = 0;
   out_794072923466735339[167] = 0;
   out_794072923466735339[168] = 0;
   out_794072923466735339[169] = 0;
   out_794072923466735339[170] = 0;
   out_794072923466735339[171] = 1;
   out_794072923466735339[172] = 0;
   out_794072923466735339[173] = 0;
   out_794072923466735339[174] = 0;
   out_794072923466735339[175] = 0;
   out_794072923466735339[176] = 0;
   out_794072923466735339[177] = 0;
   out_794072923466735339[178] = 0;
   out_794072923466735339[179] = 0;
   out_794072923466735339[180] = 0;
   out_794072923466735339[181] = 0;
   out_794072923466735339[182] = 0;
   out_794072923466735339[183] = 0;
   out_794072923466735339[184] = 0;
   out_794072923466735339[185] = 0;
   out_794072923466735339[186] = 0;
   out_794072923466735339[187] = 0;
   out_794072923466735339[188] = 0;
   out_794072923466735339[189] = 0;
   out_794072923466735339[190] = 1;
   out_794072923466735339[191] = 0;
   out_794072923466735339[192] = 0;
   out_794072923466735339[193] = 0;
   out_794072923466735339[194] = 0;
   out_794072923466735339[195] = 0;
   out_794072923466735339[196] = 0;
   out_794072923466735339[197] = 0;
   out_794072923466735339[198] = 0;
   out_794072923466735339[199] = 0;
   out_794072923466735339[200] = 0;
   out_794072923466735339[201] = 0;
   out_794072923466735339[202] = 0;
   out_794072923466735339[203] = 0;
   out_794072923466735339[204] = 0;
   out_794072923466735339[205] = 0;
   out_794072923466735339[206] = 0;
   out_794072923466735339[207] = 0;
   out_794072923466735339[208] = 0;
   out_794072923466735339[209] = 1;
   out_794072923466735339[210] = 0;
   out_794072923466735339[211] = 0;
   out_794072923466735339[212] = 0;
   out_794072923466735339[213] = 0;
   out_794072923466735339[214] = 0;
   out_794072923466735339[215] = 0;
   out_794072923466735339[216] = 0;
   out_794072923466735339[217] = 0;
   out_794072923466735339[218] = 0;
   out_794072923466735339[219] = 0;
   out_794072923466735339[220] = 0;
   out_794072923466735339[221] = 0;
   out_794072923466735339[222] = 0;
   out_794072923466735339[223] = 0;
   out_794072923466735339[224] = 0;
   out_794072923466735339[225] = 0;
   out_794072923466735339[226] = 0;
   out_794072923466735339[227] = 0;
   out_794072923466735339[228] = 1;
   out_794072923466735339[229] = 0;
   out_794072923466735339[230] = 0;
   out_794072923466735339[231] = 0;
   out_794072923466735339[232] = 0;
   out_794072923466735339[233] = 0;
   out_794072923466735339[234] = 0;
   out_794072923466735339[235] = 0;
   out_794072923466735339[236] = 0;
   out_794072923466735339[237] = 0;
   out_794072923466735339[238] = 0;
   out_794072923466735339[239] = 0;
   out_794072923466735339[240] = 0;
   out_794072923466735339[241] = 0;
   out_794072923466735339[242] = 0;
   out_794072923466735339[243] = 0;
   out_794072923466735339[244] = 0;
   out_794072923466735339[245] = 0;
   out_794072923466735339[246] = 0;
   out_794072923466735339[247] = 1;
   out_794072923466735339[248] = 0;
   out_794072923466735339[249] = 0;
   out_794072923466735339[250] = 0;
   out_794072923466735339[251] = 0;
   out_794072923466735339[252] = 0;
   out_794072923466735339[253] = 0;
   out_794072923466735339[254] = 0;
   out_794072923466735339[255] = 0;
   out_794072923466735339[256] = 0;
   out_794072923466735339[257] = 0;
   out_794072923466735339[258] = 0;
   out_794072923466735339[259] = 0;
   out_794072923466735339[260] = 0;
   out_794072923466735339[261] = 0;
   out_794072923466735339[262] = 0;
   out_794072923466735339[263] = 0;
   out_794072923466735339[264] = 0;
   out_794072923466735339[265] = 0;
   out_794072923466735339[266] = 1;
   out_794072923466735339[267] = 0;
   out_794072923466735339[268] = 0;
   out_794072923466735339[269] = 0;
   out_794072923466735339[270] = 0;
   out_794072923466735339[271] = 0;
   out_794072923466735339[272] = 0;
   out_794072923466735339[273] = 0;
   out_794072923466735339[274] = 0;
   out_794072923466735339[275] = 0;
   out_794072923466735339[276] = 0;
   out_794072923466735339[277] = 0;
   out_794072923466735339[278] = 0;
   out_794072923466735339[279] = 0;
   out_794072923466735339[280] = 0;
   out_794072923466735339[281] = 0;
   out_794072923466735339[282] = 0;
   out_794072923466735339[283] = 0;
   out_794072923466735339[284] = 0;
   out_794072923466735339[285] = 1;
   out_794072923466735339[286] = 0;
   out_794072923466735339[287] = 0;
   out_794072923466735339[288] = 0;
   out_794072923466735339[289] = 0;
   out_794072923466735339[290] = 0;
   out_794072923466735339[291] = 0;
   out_794072923466735339[292] = 0;
   out_794072923466735339[293] = 0;
   out_794072923466735339[294] = 0;
   out_794072923466735339[295] = 0;
   out_794072923466735339[296] = 0;
   out_794072923466735339[297] = 0;
   out_794072923466735339[298] = 0;
   out_794072923466735339[299] = 0;
   out_794072923466735339[300] = 0;
   out_794072923466735339[301] = 0;
   out_794072923466735339[302] = 0;
   out_794072923466735339[303] = 0;
   out_794072923466735339[304] = 1;
   out_794072923466735339[305] = 0;
   out_794072923466735339[306] = 0;
   out_794072923466735339[307] = 0;
   out_794072923466735339[308] = 0;
   out_794072923466735339[309] = 0;
   out_794072923466735339[310] = 0;
   out_794072923466735339[311] = 0;
   out_794072923466735339[312] = 0;
   out_794072923466735339[313] = 0;
   out_794072923466735339[314] = 0;
   out_794072923466735339[315] = 0;
   out_794072923466735339[316] = 0;
   out_794072923466735339[317] = 0;
   out_794072923466735339[318] = 0;
   out_794072923466735339[319] = 0;
   out_794072923466735339[320] = 0;
   out_794072923466735339[321] = 0;
   out_794072923466735339[322] = 0;
   out_794072923466735339[323] = 1;
}
void h_4(double *state, double *unused, double *out_3695677144055718255) {
   out_3695677144055718255[0] = state[6] + state[9];
   out_3695677144055718255[1] = state[7] + state[10];
   out_3695677144055718255[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6454326658284995373) {
   out_6454326658284995373[0] = 0;
   out_6454326658284995373[1] = 0;
   out_6454326658284995373[2] = 0;
   out_6454326658284995373[3] = 0;
   out_6454326658284995373[4] = 0;
   out_6454326658284995373[5] = 0;
   out_6454326658284995373[6] = 1;
   out_6454326658284995373[7] = 0;
   out_6454326658284995373[8] = 0;
   out_6454326658284995373[9] = 1;
   out_6454326658284995373[10] = 0;
   out_6454326658284995373[11] = 0;
   out_6454326658284995373[12] = 0;
   out_6454326658284995373[13] = 0;
   out_6454326658284995373[14] = 0;
   out_6454326658284995373[15] = 0;
   out_6454326658284995373[16] = 0;
   out_6454326658284995373[17] = 0;
   out_6454326658284995373[18] = 0;
   out_6454326658284995373[19] = 0;
   out_6454326658284995373[20] = 0;
   out_6454326658284995373[21] = 0;
   out_6454326658284995373[22] = 0;
   out_6454326658284995373[23] = 0;
   out_6454326658284995373[24] = 0;
   out_6454326658284995373[25] = 1;
   out_6454326658284995373[26] = 0;
   out_6454326658284995373[27] = 0;
   out_6454326658284995373[28] = 1;
   out_6454326658284995373[29] = 0;
   out_6454326658284995373[30] = 0;
   out_6454326658284995373[31] = 0;
   out_6454326658284995373[32] = 0;
   out_6454326658284995373[33] = 0;
   out_6454326658284995373[34] = 0;
   out_6454326658284995373[35] = 0;
   out_6454326658284995373[36] = 0;
   out_6454326658284995373[37] = 0;
   out_6454326658284995373[38] = 0;
   out_6454326658284995373[39] = 0;
   out_6454326658284995373[40] = 0;
   out_6454326658284995373[41] = 0;
   out_6454326658284995373[42] = 0;
   out_6454326658284995373[43] = 0;
   out_6454326658284995373[44] = 1;
   out_6454326658284995373[45] = 0;
   out_6454326658284995373[46] = 0;
   out_6454326658284995373[47] = 1;
   out_6454326658284995373[48] = 0;
   out_6454326658284995373[49] = 0;
   out_6454326658284995373[50] = 0;
   out_6454326658284995373[51] = 0;
   out_6454326658284995373[52] = 0;
   out_6454326658284995373[53] = 0;
}
void h_10(double *state, double *unused, double *out_6296017263622027777) {
   out_6296017263622027777[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6296017263622027777[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6296017263622027777[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_740468345994452368) {
   out_740468345994452368[0] = 0;
   out_740468345994452368[1] = 9.8100000000000005*cos(state[1]);
   out_740468345994452368[2] = 0;
   out_740468345994452368[3] = 0;
   out_740468345994452368[4] = -state[8];
   out_740468345994452368[5] = state[7];
   out_740468345994452368[6] = 0;
   out_740468345994452368[7] = state[5];
   out_740468345994452368[8] = -state[4];
   out_740468345994452368[9] = 0;
   out_740468345994452368[10] = 0;
   out_740468345994452368[11] = 0;
   out_740468345994452368[12] = 1;
   out_740468345994452368[13] = 0;
   out_740468345994452368[14] = 0;
   out_740468345994452368[15] = 1;
   out_740468345994452368[16] = 0;
   out_740468345994452368[17] = 0;
   out_740468345994452368[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_740468345994452368[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_740468345994452368[20] = 0;
   out_740468345994452368[21] = state[8];
   out_740468345994452368[22] = 0;
   out_740468345994452368[23] = -state[6];
   out_740468345994452368[24] = -state[5];
   out_740468345994452368[25] = 0;
   out_740468345994452368[26] = state[3];
   out_740468345994452368[27] = 0;
   out_740468345994452368[28] = 0;
   out_740468345994452368[29] = 0;
   out_740468345994452368[30] = 0;
   out_740468345994452368[31] = 1;
   out_740468345994452368[32] = 0;
   out_740468345994452368[33] = 0;
   out_740468345994452368[34] = 1;
   out_740468345994452368[35] = 0;
   out_740468345994452368[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_740468345994452368[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_740468345994452368[38] = 0;
   out_740468345994452368[39] = -state[7];
   out_740468345994452368[40] = state[6];
   out_740468345994452368[41] = 0;
   out_740468345994452368[42] = state[4];
   out_740468345994452368[43] = -state[3];
   out_740468345994452368[44] = 0;
   out_740468345994452368[45] = 0;
   out_740468345994452368[46] = 0;
   out_740468345994452368[47] = 0;
   out_740468345994452368[48] = 0;
   out_740468345994452368[49] = 0;
   out_740468345994452368[50] = 1;
   out_740468345994452368[51] = 0;
   out_740468345994452368[52] = 0;
   out_740468345994452368[53] = 1;
}
void h_13(double *state, double *unused, double *out_8962027605949315770) {
   out_8962027605949315770[0] = state[3];
   out_8962027605949315770[1] = state[4];
   out_8962027605949315770[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2620571194982471349) {
   out_2620571194982471349[0] = 0;
   out_2620571194982471349[1] = 0;
   out_2620571194982471349[2] = 0;
   out_2620571194982471349[3] = 1;
   out_2620571194982471349[4] = 0;
   out_2620571194982471349[5] = 0;
   out_2620571194982471349[6] = 0;
   out_2620571194982471349[7] = 0;
   out_2620571194982471349[8] = 0;
   out_2620571194982471349[9] = 0;
   out_2620571194982471349[10] = 0;
   out_2620571194982471349[11] = 0;
   out_2620571194982471349[12] = 0;
   out_2620571194982471349[13] = 0;
   out_2620571194982471349[14] = 0;
   out_2620571194982471349[15] = 0;
   out_2620571194982471349[16] = 0;
   out_2620571194982471349[17] = 0;
   out_2620571194982471349[18] = 0;
   out_2620571194982471349[19] = 0;
   out_2620571194982471349[20] = 0;
   out_2620571194982471349[21] = 0;
   out_2620571194982471349[22] = 1;
   out_2620571194982471349[23] = 0;
   out_2620571194982471349[24] = 0;
   out_2620571194982471349[25] = 0;
   out_2620571194982471349[26] = 0;
   out_2620571194982471349[27] = 0;
   out_2620571194982471349[28] = 0;
   out_2620571194982471349[29] = 0;
   out_2620571194982471349[30] = 0;
   out_2620571194982471349[31] = 0;
   out_2620571194982471349[32] = 0;
   out_2620571194982471349[33] = 0;
   out_2620571194982471349[34] = 0;
   out_2620571194982471349[35] = 0;
   out_2620571194982471349[36] = 0;
   out_2620571194982471349[37] = 0;
   out_2620571194982471349[38] = 0;
   out_2620571194982471349[39] = 0;
   out_2620571194982471349[40] = 0;
   out_2620571194982471349[41] = 1;
   out_2620571194982471349[42] = 0;
   out_2620571194982471349[43] = 0;
   out_2620571194982471349[44] = 0;
   out_2620571194982471349[45] = 0;
   out_2620571194982471349[46] = 0;
   out_2620571194982471349[47] = 0;
   out_2620571194982471349[48] = 0;
   out_2620571194982471349[49] = 0;
   out_2620571194982471349[50] = 0;
   out_2620571194982471349[51] = 0;
   out_2620571194982471349[52] = 0;
   out_2620571194982471349[53] = 0;
}
void h_14(double *state, double *unused, double *out_3731961672478710821) {
   out_3731961672478710821[0] = state[6];
   out_3731961672478710821[1] = state[7];
   out_3731961672478710821[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3371538225989623077) {
   out_3371538225989623077[0] = 0;
   out_3371538225989623077[1] = 0;
   out_3371538225989623077[2] = 0;
   out_3371538225989623077[3] = 0;
   out_3371538225989623077[4] = 0;
   out_3371538225989623077[5] = 0;
   out_3371538225989623077[6] = 1;
   out_3371538225989623077[7] = 0;
   out_3371538225989623077[8] = 0;
   out_3371538225989623077[9] = 0;
   out_3371538225989623077[10] = 0;
   out_3371538225989623077[11] = 0;
   out_3371538225989623077[12] = 0;
   out_3371538225989623077[13] = 0;
   out_3371538225989623077[14] = 0;
   out_3371538225989623077[15] = 0;
   out_3371538225989623077[16] = 0;
   out_3371538225989623077[17] = 0;
   out_3371538225989623077[18] = 0;
   out_3371538225989623077[19] = 0;
   out_3371538225989623077[20] = 0;
   out_3371538225989623077[21] = 0;
   out_3371538225989623077[22] = 0;
   out_3371538225989623077[23] = 0;
   out_3371538225989623077[24] = 0;
   out_3371538225989623077[25] = 1;
   out_3371538225989623077[26] = 0;
   out_3371538225989623077[27] = 0;
   out_3371538225989623077[28] = 0;
   out_3371538225989623077[29] = 0;
   out_3371538225989623077[30] = 0;
   out_3371538225989623077[31] = 0;
   out_3371538225989623077[32] = 0;
   out_3371538225989623077[33] = 0;
   out_3371538225989623077[34] = 0;
   out_3371538225989623077[35] = 0;
   out_3371538225989623077[36] = 0;
   out_3371538225989623077[37] = 0;
   out_3371538225989623077[38] = 0;
   out_3371538225989623077[39] = 0;
   out_3371538225989623077[40] = 0;
   out_3371538225989623077[41] = 0;
   out_3371538225989623077[42] = 0;
   out_3371538225989623077[43] = 0;
   out_3371538225989623077[44] = 1;
   out_3371538225989623077[45] = 0;
   out_3371538225989623077[46] = 0;
   out_3371538225989623077[47] = 0;
   out_3371538225989623077[48] = 0;
   out_3371538225989623077[49] = 0;
   out_3371538225989623077[50] = 0;
   out_3371538225989623077[51] = 0;
   out_3371538225989623077[52] = 0;
   out_3371538225989623077[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_1238705429075317501) {
  err_fun(nom_x, delta_x, out_1238705429075317501);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3460389199729958721) {
  inv_err_fun(nom_x, true_x, out_3460389199729958721);
}
void pose_H_mod_fun(double *state, double *out_5700165656230282766) {
  H_mod_fun(state, out_5700165656230282766);
}
void pose_f_fun(double *state, double dt, double *out_4518422097938204125) {
  f_fun(state,  dt, out_4518422097938204125);
}
void pose_F_fun(double *state, double dt, double *out_794072923466735339) {
  F_fun(state,  dt, out_794072923466735339);
}
void pose_h_4(double *state, double *unused, double *out_3695677144055718255) {
  h_4(state, unused, out_3695677144055718255);
}
void pose_H_4(double *state, double *unused, double *out_6454326658284995373) {
  H_4(state, unused, out_6454326658284995373);
}
void pose_h_10(double *state, double *unused, double *out_6296017263622027777) {
  h_10(state, unused, out_6296017263622027777);
}
void pose_H_10(double *state, double *unused, double *out_740468345994452368) {
  H_10(state, unused, out_740468345994452368);
}
void pose_h_13(double *state, double *unused, double *out_8962027605949315770) {
  h_13(state, unused, out_8962027605949315770);
}
void pose_H_13(double *state, double *unused, double *out_2620571194982471349) {
  H_13(state, unused, out_2620571194982471349);
}
void pose_h_14(double *state, double *unused, double *out_3731961672478710821) {
  h_14(state, unused, out_3731961672478710821);
}
void pose_H_14(double *state, double *unused, double *out_3371538225989623077) {
  H_14(state, unused, out_3371538225989623077);
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
