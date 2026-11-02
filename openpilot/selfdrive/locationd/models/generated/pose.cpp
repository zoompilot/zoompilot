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
void err_fun(double *nom_x, double *delta_x, double *out_3980012004147667955) {
   out_3980012004147667955[0] = delta_x[0] + nom_x[0];
   out_3980012004147667955[1] = delta_x[1] + nom_x[1];
   out_3980012004147667955[2] = delta_x[2] + nom_x[2];
   out_3980012004147667955[3] = delta_x[3] + nom_x[3];
   out_3980012004147667955[4] = delta_x[4] + nom_x[4];
   out_3980012004147667955[5] = delta_x[5] + nom_x[5];
   out_3980012004147667955[6] = delta_x[6] + nom_x[6];
   out_3980012004147667955[7] = delta_x[7] + nom_x[7];
   out_3980012004147667955[8] = delta_x[8] + nom_x[8];
   out_3980012004147667955[9] = delta_x[9] + nom_x[9];
   out_3980012004147667955[10] = delta_x[10] + nom_x[10];
   out_3980012004147667955[11] = delta_x[11] + nom_x[11];
   out_3980012004147667955[12] = delta_x[12] + nom_x[12];
   out_3980012004147667955[13] = delta_x[13] + nom_x[13];
   out_3980012004147667955[14] = delta_x[14] + nom_x[14];
   out_3980012004147667955[15] = delta_x[15] + nom_x[15];
   out_3980012004147667955[16] = delta_x[16] + nom_x[16];
   out_3980012004147667955[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6720818933782490088) {
   out_6720818933782490088[0] = -nom_x[0] + true_x[0];
   out_6720818933782490088[1] = -nom_x[1] + true_x[1];
   out_6720818933782490088[2] = -nom_x[2] + true_x[2];
   out_6720818933782490088[3] = -nom_x[3] + true_x[3];
   out_6720818933782490088[4] = -nom_x[4] + true_x[4];
   out_6720818933782490088[5] = -nom_x[5] + true_x[5];
   out_6720818933782490088[6] = -nom_x[6] + true_x[6];
   out_6720818933782490088[7] = -nom_x[7] + true_x[7];
   out_6720818933782490088[8] = -nom_x[8] + true_x[8];
   out_6720818933782490088[9] = -nom_x[9] + true_x[9];
   out_6720818933782490088[10] = -nom_x[10] + true_x[10];
   out_6720818933782490088[11] = -nom_x[11] + true_x[11];
   out_6720818933782490088[12] = -nom_x[12] + true_x[12];
   out_6720818933782490088[13] = -nom_x[13] + true_x[13];
   out_6720818933782490088[14] = -nom_x[14] + true_x[14];
   out_6720818933782490088[15] = -nom_x[15] + true_x[15];
   out_6720818933782490088[16] = -nom_x[16] + true_x[16];
   out_6720818933782490088[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3217080649686513056) {
   out_3217080649686513056[0] = 1.0;
   out_3217080649686513056[1] = 0.0;
   out_3217080649686513056[2] = 0.0;
   out_3217080649686513056[3] = 0.0;
   out_3217080649686513056[4] = 0.0;
   out_3217080649686513056[5] = 0.0;
   out_3217080649686513056[6] = 0.0;
   out_3217080649686513056[7] = 0.0;
   out_3217080649686513056[8] = 0.0;
   out_3217080649686513056[9] = 0.0;
   out_3217080649686513056[10] = 0.0;
   out_3217080649686513056[11] = 0.0;
   out_3217080649686513056[12] = 0.0;
   out_3217080649686513056[13] = 0.0;
   out_3217080649686513056[14] = 0.0;
   out_3217080649686513056[15] = 0.0;
   out_3217080649686513056[16] = 0.0;
   out_3217080649686513056[17] = 0.0;
   out_3217080649686513056[18] = 0.0;
   out_3217080649686513056[19] = 1.0;
   out_3217080649686513056[20] = 0.0;
   out_3217080649686513056[21] = 0.0;
   out_3217080649686513056[22] = 0.0;
   out_3217080649686513056[23] = 0.0;
   out_3217080649686513056[24] = 0.0;
   out_3217080649686513056[25] = 0.0;
   out_3217080649686513056[26] = 0.0;
   out_3217080649686513056[27] = 0.0;
   out_3217080649686513056[28] = 0.0;
   out_3217080649686513056[29] = 0.0;
   out_3217080649686513056[30] = 0.0;
   out_3217080649686513056[31] = 0.0;
   out_3217080649686513056[32] = 0.0;
   out_3217080649686513056[33] = 0.0;
   out_3217080649686513056[34] = 0.0;
   out_3217080649686513056[35] = 0.0;
   out_3217080649686513056[36] = 0.0;
   out_3217080649686513056[37] = 0.0;
   out_3217080649686513056[38] = 1.0;
   out_3217080649686513056[39] = 0.0;
   out_3217080649686513056[40] = 0.0;
   out_3217080649686513056[41] = 0.0;
   out_3217080649686513056[42] = 0.0;
   out_3217080649686513056[43] = 0.0;
   out_3217080649686513056[44] = 0.0;
   out_3217080649686513056[45] = 0.0;
   out_3217080649686513056[46] = 0.0;
   out_3217080649686513056[47] = 0.0;
   out_3217080649686513056[48] = 0.0;
   out_3217080649686513056[49] = 0.0;
   out_3217080649686513056[50] = 0.0;
   out_3217080649686513056[51] = 0.0;
   out_3217080649686513056[52] = 0.0;
   out_3217080649686513056[53] = 0.0;
   out_3217080649686513056[54] = 0.0;
   out_3217080649686513056[55] = 0.0;
   out_3217080649686513056[56] = 0.0;
   out_3217080649686513056[57] = 1.0;
   out_3217080649686513056[58] = 0.0;
   out_3217080649686513056[59] = 0.0;
   out_3217080649686513056[60] = 0.0;
   out_3217080649686513056[61] = 0.0;
   out_3217080649686513056[62] = 0.0;
   out_3217080649686513056[63] = 0.0;
   out_3217080649686513056[64] = 0.0;
   out_3217080649686513056[65] = 0.0;
   out_3217080649686513056[66] = 0.0;
   out_3217080649686513056[67] = 0.0;
   out_3217080649686513056[68] = 0.0;
   out_3217080649686513056[69] = 0.0;
   out_3217080649686513056[70] = 0.0;
   out_3217080649686513056[71] = 0.0;
   out_3217080649686513056[72] = 0.0;
   out_3217080649686513056[73] = 0.0;
   out_3217080649686513056[74] = 0.0;
   out_3217080649686513056[75] = 0.0;
   out_3217080649686513056[76] = 1.0;
   out_3217080649686513056[77] = 0.0;
   out_3217080649686513056[78] = 0.0;
   out_3217080649686513056[79] = 0.0;
   out_3217080649686513056[80] = 0.0;
   out_3217080649686513056[81] = 0.0;
   out_3217080649686513056[82] = 0.0;
   out_3217080649686513056[83] = 0.0;
   out_3217080649686513056[84] = 0.0;
   out_3217080649686513056[85] = 0.0;
   out_3217080649686513056[86] = 0.0;
   out_3217080649686513056[87] = 0.0;
   out_3217080649686513056[88] = 0.0;
   out_3217080649686513056[89] = 0.0;
   out_3217080649686513056[90] = 0.0;
   out_3217080649686513056[91] = 0.0;
   out_3217080649686513056[92] = 0.0;
   out_3217080649686513056[93] = 0.0;
   out_3217080649686513056[94] = 0.0;
   out_3217080649686513056[95] = 1.0;
   out_3217080649686513056[96] = 0.0;
   out_3217080649686513056[97] = 0.0;
   out_3217080649686513056[98] = 0.0;
   out_3217080649686513056[99] = 0.0;
   out_3217080649686513056[100] = 0.0;
   out_3217080649686513056[101] = 0.0;
   out_3217080649686513056[102] = 0.0;
   out_3217080649686513056[103] = 0.0;
   out_3217080649686513056[104] = 0.0;
   out_3217080649686513056[105] = 0.0;
   out_3217080649686513056[106] = 0.0;
   out_3217080649686513056[107] = 0.0;
   out_3217080649686513056[108] = 0.0;
   out_3217080649686513056[109] = 0.0;
   out_3217080649686513056[110] = 0.0;
   out_3217080649686513056[111] = 0.0;
   out_3217080649686513056[112] = 0.0;
   out_3217080649686513056[113] = 0.0;
   out_3217080649686513056[114] = 1.0;
   out_3217080649686513056[115] = 0.0;
   out_3217080649686513056[116] = 0.0;
   out_3217080649686513056[117] = 0.0;
   out_3217080649686513056[118] = 0.0;
   out_3217080649686513056[119] = 0.0;
   out_3217080649686513056[120] = 0.0;
   out_3217080649686513056[121] = 0.0;
   out_3217080649686513056[122] = 0.0;
   out_3217080649686513056[123] = 0.0;
   out_3217080649686513056[124] = 0.0;
   out_3217080649686513056[125] = 0.0;
   out_3217080649686513056[126] = 0.0;
   out_3217080649686513056[127] = 0.0;
   out_3217080649686513056[128] = 0.0;
   out_3217080649686513056[129] = 0.0;
   out_3217080649686513056[130] = 0.0;
   out_3217080649686513056[131] = 0.0;
   out_3217080649686513056[132] = 0.0;
   out_3217080649686513056[133] = 1.0;
   out_3217080649686513056[134] = 0.0;
   out_3217080649686513056[135] = 0.0;
   out_3217080649686513056[136] = 0.0;
   out_3217080649686513056[137] = 0.0;
   out_3217080649686513056[138] = 0.0;
   out_3217080649686513056[139] = 0.0;
   out_3217080649686513056[140] = 0.0;
   out_3217080649686513056[141] = 0.0;
   out_3217080649686513056[142] = 0.0;
   out_3217080649686513056[143] = 0.0;
   out_3217080649686513056[144] = 0.0;
   out_3217080649686513056[145] = 0.0;
   out_3217080649686513056[146] = 0.0;
   out_3217080649686513056[147] = 0.0;
   out_3217080649686513056[148] = 0.0;
   out_3217080649686513056[149] = 0.0;
   out_3217080649686513056[150] = 0.0;
   out_3217080649686513056[151] = 0.0;
   out_3217080649686513056[152] = 1.0;
   out_3217080649686513056[153] = 0.0;
   out_3217080649686513056[154] = 0.0;
   out_3217080649686513056[155] = 0.0;
   out_3217080649686513056[156] = 0.0;
   out_3217080649686513056[157] = 0.0;
   out_3217080649686513056[158] = 0.0;
   out_3217080649686513056[159] = 0.0;
   out_3217080649686513056[160] = 0.0;
   out_3217080649686513056[161] = 0.0;
   out_3217080649686513056[162] = 0.0;
   out_3217080649686513056[163] = 0.0;
   out_3217080649686513056[164] = 0.0;
   out_3217080649686513056[165] = 0.0;
   out_3217080649686513056[166] = 0.0;
   out_3217080649686513056[167] = 0.0;
   out_3217080649686513056[168] = 0.0;
   out_3217080649686513056[169] = 0.0;
   out_3217080649686513056[170] = 0.0;
   out_3217080649686513056[171] = 1.0;
   out_3217080649686513056[172] = 0.0;
   out_3217080649686513056[173] = 0.0;
   out_3217080649686513056[174] = 0.0;
   out_3217080649686513056[175] = 0.0;
   out_3217080649686513056[176] = 0.0;
   out_3217080649686513056[177] = 0.0;
   out_3217080649686513056[178] = 0.0;
   out_3217080649686513056[179] = 0.0;
   out_3217080649686513056[180] = 0.0;
   out_3217080649686513056[181] = 0.0;
   out_3217080649686513056[182] = 0.0;
   out_3217080649686513056[183] = 0.0;
   out_3217080649686513056[184] = 0.0;
   out_3217080649686513056[185] = 0.0;
   out_3217080649686513056[186] = 0.0;
   out_3217080649686513056[187] = 0.0;
   out_3217080649686513056[188] = 0.0;
   out_3217080649686513056[189] = 0.0;
   out_3217080649686513056[190] = 1.0;
   out_3217080649686513056[191] = 0.0;
   out_3217080649686513056[192] = 0.0;
   out_3217080649686513056[193] = 0.0;
   out_3217080649686513056[194] = 0.0;
   out_3217080649686513056[195] = 0.0;
   out_3217080649686513056[196] = 0.0;
   out_3217080649686513056[197] = 0.0;
   out_3217080649686513056[198] = 0.0;
   out_3217080649686513056[199] = 0.0;
   out_3217080649686513056[200] = 0.0;
   out_3217080649686513056[201] = 0.0;
   out_3217080649686513056[202] = 0.0;
   out_3217080649686513056[203] = 0.0;
   out_3217080649686513056[204] = 0.0;
   out_3217080649686513056[205] = 0.0;
   out_3217080649686513056[206] = 0.0;
   out_3217080649686513056[207] = 0.0;
   out_3217080649686513056[208] = 0.0;
   out_3217080649686513056[209] = 1.0;
   out_3217080649686513056[210] = 0.0;
   out_3217080649686513056[211] = 0.0;
   out_3217080649686513056[212] = 0.0;
   out_3217080649686513056[213] = 0.0;
   out_3217080649686513056[214] = 0.0;
   out_3217080649686513056[215] = 0.0;
   out_3217080649686513056[216] = 0.0;
   out_3217080649686513056[217] = 0.0;
   out_3217080649686513056[218] = 0.0;
   out_3217080649686513056[219] = 0.0;
   out_3217080649686513056[220] = 0.0;
   out_3217080649686513056[221] = 0.0;
   out_3217080649686513056[222] = 0.0;
   out_3217080649686513056[223] = 0.0;
   out_3217080649686513056[224] = 0.0;
   out_3217080649686513056[225] = 0.0;
   out_3217080649686513056[226] = 0.0;
   out_3217080649686513056[227] = 0.0;
   out_3217080649686513056[228] = 1.0;
   out_3217080649686513056[229] = 0.0;
   out_3217080649686513056[230] = 0.0;
   out_3217080649686513056[231] = 0.0;
   out_3217080649686513056[232] = 0.0;
   out_3217080649686513056[233] = 0.0;
   out_3217080649686513056[234] = 0.0;
   out_3217080649686513056[235] = 0.0;
   out_3217080649686513056[236] = 0.0;
   out_3217080649686513056[237] = 0.0;
   out_3217080649686513056[238] = 0.0;
   out_3217080649686513056[239] = 0.0;
   out_3217080649686513056[240] = 0.0;
   out_3217080649686513056[241] = 0.0;
   out_3217080649686513056[242] = 0.0;
   out_3217080649686513056[243] = 0.0;
   out_3217080649686513056[244] = 0.0;
   out_3217080649686513056[245] = 0.0;
   out_3217080649686513056[246] = 0.0;
   out_3217080649686513056[247] = 1.0;
   out_3217080649686513056[248] = 0.0;
   out_3217080649686513056[249] = 0.0;
   out_3217080649686513056[250] = 0.0;
   out_3217080649686513056[251] = 0.0;
   out_3217080649686513056[252] = 0.0;
   out_3217080649686513056[253] = 0.0;
   out_3217080649686513056[254] = 0.0;
   out_3217080649686513056[255] = 0.0;
   out_3217080649686513056[256] = 0.0;
   out_3217080649686513056[257] = 0.0;
   out_3217080649686513056[258] = 0.0;
   out_3217080649686513056[259] = 0.0;
   out_3217080649686513056[260] = 0.0;
   out_3217080649686513056[261] = 0.0;
   out_3217080649686513056[262] = 0.0;
   out_3217080649686513056[263] = 0.0;
   out_3217080649686513056[264] = 0.0;
   out_3217080649686513056[265] = 0.0;
   out_3217080649686513056[266] = 1.0;
   out_3217080649686513056[267] = 0.0;
   out_3217080649686513056[268] = 0.0;
   out_3217080649686513056[269] = 0.0;
   out_3217080649686513056[270] = 0.0;
   out_3217080649686513056[271] = 0.0;
   out_3217080649686513056[272] = 0.0;
   out_3217080649686513056[273] = 0.0;
   out_3217080649686513056[274] = 0.0;
   out_3217080649686513056[275] = 0.0;
   out_3217080649686513056[276] = 0.0;
   out_3217080649686513056[277] = 0.0;
   out_3217080649686513056[278] = 0.0;
   out_3217080649686513056[279] = 0.0;
   out_3217080649686513056[280] = 0.0;
   out_3217080649686513056[281] = 0.0;
   out_3217080649686513056[282] = 0.0;
   out_3217080649686513056[283] = 0.0;
   out_3217080649686513056[284] = 0.0;
   out_3217080649686513056[285] = 1.0;
   out_3217080649686513056[286] = 0.0;
   out_3217080649686513056[287] = 0.0;
   out_3217080649686513056[288] = 0.0;
   out_3217080649686513056[289] = 0.0;
   out_3217080649686513056[290] = 0.0;
   out_3217080649686513056[291] = 0.0;
   out_3217080649686513056[292] = 0.0;
   out_3217080649686513056[293] = 0.0;
   out_3217080649686513056[294] = 0.0;
   out_3217080649686513056[295] = 0.0;
   out_3217080649686513056[296] = 0.0;
   out_3217080649686513056[297] = 0.0;
   out_3217080649686513056[298] = 0.0;
   out_3217080649686513056[299] = 0.0;
   out_3217080649686513056[300] = 0.0;
   out_3217080649686513056[301] = 0.0;
   out_3217080649686513056[302] = 0.0;
   out_3217080649686513056[303] = 0.0;
   out_3217080649686513056[304] = 1.0;
   out_3217080649686513056[305] = 0.0;
   out_3217080649686513056[306] = 0.0;
   out_3217080649686513056[307] = 0.0;
   out_3217080649686513056[308] = 0.0;
   out_3217080649686513056[309] = 0.0;
   out_3217080649686513056[310] = 0.0;
   out_3217080649686513056[311] = 0.0;
   out_3217080649686513056[312] = 0.0;
   out_3217080649686513056[313] = 0.0;
   out_3217080649686513056[314] = 0.0;
   out_3217080649686513056[315] = 0.0;
   out_3217080649686513056[316] = 0.0;
   out_3217080649686513056[317] = 0.0;
   out_3217080649686513056[318] = 0.0;
   out_3217080649686513056[319] = 0.0;
   out_3217080649686513056[320] = 0.0;
   out_3217080649686513056[321] = 0.0;
   out_3217080649686513056[322] = 0.0;
   out_3217080649686513056[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3363301103592732445) {
   out_3363301103592732445[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3363301103592732445[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3363301103592732445[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3363301103592732445[3] = dt*state[12] + state[3];
   out_3363301103592732445[4] = dt*state[13] + state[4];
   out_3363301103592732445[5] = dt*state[14] + state[5];
   out_3363301103592732445[6] = state[6];
   out_3363301103592732445[7] = state[7];
   out_3363301103592732445[8] = state[8];
   out_3363301103592732445[9] = state[9];
   out_3363301103592732445[10] = state[10];
   out_3363301103592732445[11] = state[11];
   out_3363301103592732445[12] = state[12];
   out_3363301103592732445[13] = state[13];
   out_3363301103592732445[14] = state[14];
   out_3363301103592732445[15] = state[15];
   out_3363301103592732445[16] = state[16];
   out_3363301103592732445[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1157141240317607749) {
   out_1157141240317607749[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1157141240317607749[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1157141240317607749[2] = 0;
   out_1157141240317607749[3] = 0;
   out_1157141240317607749[4] = 0;
   out_1157141240317607749[5] = 0;
   out_1157141240317607749[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1157141240317607749[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1157141240317607749[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1157141240317607749[9] = 0;
   out_1157141240317607749[10] = 0;
   out_1157141240317607749[11] = 0;
   out_1157141240317607749[12] = 0;
   out_1157141240317607749[13] = 0;
   out_1157141240317607749[14] = 0;
   out_1157141240317607749[15] = 0;
   out_1157141240317607749[16] = 0;
   out_1157141240317607749[17] = 0;
   out_1157141240317607749[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1157141240317607749[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1157141240317607749[20] = 0;
   out_1157141240317607749[21] = 0;
   out_1157141240317607749[22] = 0;
   out_1157141240317607749[23] = 0;
   out_1157141240317607749[24] = 0;
   out_1157141240317607749[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1157141240317607749[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1157141240317607749[27] = 0;
   out_1157141240317607749[28] = 0;
   out_1157141240317607749[29] = 0;
   out_1157141240317607749[30] = 0;
   out_1157141240317607749[31] = 0;
   out_1157141240317607749[32] = 0;
   out_1157141240317607749[33] = 0;
   out_1157141240317607749[34] = 0;
   out_1157141240317607749[35] = 0;
   out_1157141240317607749[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1157141240317607749[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1157141240317607749[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1157141240317607749[39] = 0;
   out_1157141240317607749[40] = 0;
   out_1157141240317607749[41] = 0;
   out_1157141240317607749[42] = 0;
   out_1157141240317607749[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1157141240317607749[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1157141240317607749[45] = 0;
   out_1157141240317607749[46] = 0;
   out_1157141240317607749[47] = 0;
   out_1157141240317607749[48] = 0;
   out_1157141240317607749[49] = 0;
   out_1157141240317607749[50] = 0;
   out_1157141240317607749[51] = 0;
   out_1157141240317607749[52] = 0;
   out_1157141240317607749[53] = 0;
   out_1157141240317607749[54] = 0;
   out_1157141240317607749[55] = 0;
   out_1157141240317607749[56] = 0;
   out_1157141240317607749[57] = 1;
   out_1157141240317607749[58] = 0;
   out_1157141240317607749[59] = 0;
   out_1157141240317607749[60] = 0;
   out_1157141240317607749[61] = 0;
   out_1157141240317607749[62] = 0;
   out_1157141240317607749[63] = 0;
   out_1157141240317607749[64] = 0;
   out_1157141240317607749[65] = 0;
   out_1157141240317607749[66] = dt;
   out_1157141240317607749[67] = 0;
   out_1157141240317607749[68] = 0;
   out_1157141240317607749[69] = 0;
   out_1157141240317607749[70] = 0;
   out_1157141240317607749[71] = 0;
   out_1157141240317607749[72] = 0;
   out_1157141240317607749[73] = 0;
   out_1157141240317607749[74] = 0;
   out_1157141240317607749[75] = 0;
   out_1157141240317607749[76] = 1;
   out_1157141240317607749[77] = 0;
   out_1157141240317607749[78] = 0;
   out_1157141240317607749[79] = 0;
   out_1157141240317607749[80] = 0;
   out_1157141240317607749[81] = 0;
   out_1157141240317607749[82] = 0;
   out_1157141240317607749[83] = 0;
   out_1157141240317607749[84] = 0;
   out_1157141240317607749[85] = dt;
   out_1157141240317607749[86] = 0;
   out_1157141240317607749[87] = 0;
   out_1157141240317607749[88] = 0;
   out_1157141240317607749[89] = 0;
   out_1157141240317607749[90] = 0;
   out_1157141240317607749[91] = 0;
   out_1157141240317607749[92] = 0;
   out_1157141240317607749[93] = 0;
   out_1157141240317607749[94] = 0;
   out_1157141240317607749[95] = 1;
   out_1157141240317607749[96] = 0;
   out_1157141240317607749[97] = 0;
   out_1157141240317607749[98] = 0;
   out_1157141240317607749[99] = 0;
   out_1157141240317607749[100] = 0;
   out_1157141240317607749[101] = 0;
   out_1157141240317607749[102] = 0;
   out_1157141240317607749[103] = 0;
   out_1157141240317607749[104] = dt;
   out_1157141240317607749[105] = 0;
   out_1157141240317607749[106] = 0;
   out_1157141240317607749[107] = 0;
   out_1157141240317607749[108] = 0;
   out_1157141240317607749[109] = 0;
   out_1157141240317607749[110] = 0;
   out_1157141240317607749[111] = 0;
   out_1157141240317607749[112] = 0;
   out_1157141240317607749[113] = 0;
   out_1157141240317607749[114] = 1;
   out_1157141240317607749[115] = 0;
   out_1157141240317607749[116] = 0;
   out_1157141240317607749[117] = 0;
   out_1157141240317607749[118] = 0;
   out_1157141240317607749[119] = 0;
   out_1157141240317607749[120] = 0;
   out_1157141240317607749[121] = 0;
   out_1157141240317607749[122] = 0;
   out_1157141240317607749[123] = 0;
   out_1157141240317607749[124] = 0;
   out_1157141240317607749[125] = 0;
   out_1157141240317607749[126] = 0;
   out_1157141240317607749[127] = 0;
   out_1157141240317607749[128] = 0;
   out_1157141240317607749[129] = 0;
   out_1157141240317607749[130] = 0;
   out_1157141240317607749[131] = 0;
   out_1157141240317607749[132] = 0;
   out_1157141240317607749[133] = 1;
   out_1157141240317607749[134] = 0;
   out_1157141240317607749[135] = 0;
   out_1157141240317607749[136] = 0;
   out_1157141240317607749[137] = 0;
   out_1157141240317607749[138] = 0;
   out_1157141240317607749[139] = 0;
   out_1157141240317607749[140] = 0;
   out_1157141240317607749[141] = 0;
   out_1157141240317607749[142] = 0;
   out_1157141240317607749[143] = 0;
   out_1157141240317607749[144] = 0;
   out_1157141240317607749[145] = 0;
   out_1157141240317607749[146] = 0;
   out_1157141240317607749[147] = 0;
   out_1157141240317607749[148] = 0;
   out_1157141240317607749[149] = 0;
   out_1157141240317607749[150] = 0;
   out_1157141240317607749[151] = 0;
   out_1157141240317607749[152] = 1;
   out_1157141240317607749[153] = 0;
   out_1157141240317607749[154] = 0;
   out_1157141240317607749[155] = 0;
   out_1157141240317607749[156] = 0;
   out_1157141240317607749[157] = 0;
   out_1157141240317607749[158] = 0;
   out_1157141240317607749[159] = 0;
   out_1157141240317607749[160] = 0;
   out_1157141240317607749[161] = 0;
   out_1157141240317607749[162] = 0;
   out_1157141240317607749[163] = 0;
   out_1157141240317607749[164] = 0;
   out_1157141240317607749[165] = 0;
   out_1157141240317607749[166] = 0;
   out_1157141240317607749[167] = 0;
   out_1157141240317607749[168] = 0;
   out_1157141240317607749[169] = 0;
   out_1157141240317607749[170] = 0;
   out_1157141240317607749[171] = 1;
   out_1157141240317607749[172] = 0;
   out_1157141240317607749[173] = 0;
   out_1157141240317607749[174] = 0;
   out_1157141240317607749[175] = 0;
   out_1157141240317607749[176] = 0;
   out_1157141240317607749[177] = 0;
   out_1157141240317607749[178] = 0;
   out_1157141240317607749[179] = 0;
   out_1157141240317607749[180] = 0;
   out_1157141240317607749[181] = 0;
   out_1157141240317607749[182] = 0;
   out_1157141240317607749[183] = 0;
   out_1157141240317607749[184] = 0;
   out_1157141240317607749[185] = 0;
   out_1157141240317607749[186] = 0;
   out_1157141240317607749[187] = 0;
   out_1157141240317607749[188] = 0;
   out_1157141240317607749[189] = 0;
   out_1157141240317607749[190] = 1;
   out_1157141240317607749[191] = 0;
   out_1157141240317607749[192] = 0;
   out_1157141240317607749[193] = 0;
   out_1157141240317607749[194] = 0;
   out_1157141240317607749[195] = 0;
   out_1157141240317607749[196] = 0;
   out_1157141240317607749[197] = 0;
   out_1157141240317607749[198] = 0;
   out_1157141240317607749[199] = 0;
   out_1157141240317607749[200] = 0;
   out_1157141240317607749[201] = 0;
   out_1157141240317607749[202] = 0;
   out_1157141240317607749[203] = 0;
   out_1157141240317607749[204] = 0;
   out_1157141240317607749[205] = 0;
   out_1157141240317607749[206] = 0;
   out_1157141240317607749[207] = 0;
   out_1157141240317607749[208] = 0;
   out_1157141240317607749[209] = 1;
   out_1157141240317607749[210] = 0;
   out_1157141240317607749[211] = 0;
   out_1157141240317607749[212] = 0;
   out_1157141240317607749[213] = 0;
   out_1157141240317607749[214] = 0;
   out_1157141240317607749[215] = 0;
   out_1157141240317607749[216] = 0;
   out_1157141240317607749[217] = 0;
   out_1157141240317607749[218] = 0;
   out_1157141240317607749[219] = 0;
   out_1157141240317607749[220] = 0;
   out_1157141240317607749[221] = 0;
   out_1157141240317607749[222] = 0;
   out_1157141240317607749[223] = 0;
   out_1157141240317607749[224] = 0;
   out_1157141240317607749[225] = 0;
   out_1157141240317607749[226] = 0;
   out_1157141240317607749[227] = 0;
   out_1157141240317607749[228] = 1;
   out_1157141240317607749[229] = 0;
   out_1157141240317607749[230] = 0;
   out_1157141240317607749[231] = 0;
   out_1157141240317607749[232] = 0;
   out_1157141240317607749[233] = 0;
   out_1157141240317607749[234] = 0;
   out_1157141240317607749[235] = 0;
   out_1157141240317607749[236] = 0;
   out_1157141240317607749[237] = 0;
   out_1157141240317607749[238] = 0;
   out_1157141240317607749[239] = 0;
   out_1157141240317607749[240] = 0;
   out_1157141240317607749[241] = 0;
   out_1157141240317607749[242] = 0;
   out_1157141240317607749[243] = 0;
   out_1157141240317607749[244] = 0;
   out_1157141240317607749[245] = 0;
   out_1157141240317607749[246] = 0;
   out_1157141240317607749[247] = 1;
   out_1157141240317607749[248] = 0;
   out_1157141240317607749[249] = 0;
   out_1157141240317607749[250] = 0;
   out_1157141240317607749[251] = 0;
   out_1157141240317607749[252] = 0;
   out_1157141240317607749[253] = 0;
   out_1157141240317607749[254] = 0;
   out_1157141240317607749[255] = 0;
   out_1157141240317607749[256] = 0;
   out_1157141240317607749[257] = 0;
   out_1157141240317607749[258] = 0;
   out_1157141240317607749[259] = 0;
   out_1157141240317607749[260] = 0;
   out_1157141240317607749[261] = 0;
   out_1157141240317607749[262] = 0;
   out_1157141240317607749[263] = 0;
   out_1157141240317607749[264] = 0;
   out_1157141240317607749[265] = 0;
   out_1157141240317607749[266] = 1;
   out_1157141240317607749[267] = 0;
   out_1157141240317607749[268] = 0;
   out_1157141240317607749[269] = 0;
   out_1157141240317607749[270] = 0;
   out_1157141240317607749[271] = 0;
   out_1157141240317607749[272] = 0;
   out_1157141240317607749[273] = 0;
   out_1157141240317607749[274] = 0;
   out_1157141240317607749[275] = 0;
   out_1157141240317607749[276] = 0;
   out_1157141240317607749[277] = 0;
   out_1157141240317607749[278] = 0;
   out_1157141240317607749[279] = 0;
   out_1157141240317607749[280] = 0;
   out_1157141240317607749[281] = 0;
   out_1157141240317607749[282] = 0;
   out_1157141240317607749[283] = 0;
   out_1157141240317607749[284] = 0;
   out_1157141240317607749[285] = 1;
   out_1157141240317607749[286] = 0;
   out_1157141240317607749[287] = 0;
   out_1157141240317607749[288] = 0;
   out_1157141240317607749[289] = 0;
   out_1157141240317607749[290] = 0;
   out_1157141240317607749[291] = 0;
   out_1157141240317607749[292] = 0;
   out_1157141240317607749[293] = 0;
   out_1157141240317607749[294] = 0;
   out_1157141240317607749[295] = 0;
   out_1157141240317607749[296] = 0;
   out_1157141240317607749[297] = 0;
   out_1157141240317607749[298] = 0;
   out_1157141240317607749[299] = 0;
   out_1157141240317607749[300] = 0;
   out_1157141240317607749[301] = 0;
   out_1157141240317607749[302] = 0;
   out_1157141240317607749[303] = 0;
   out_1157141240317607749[304] = 1;
   out_1157141240317607749[305] = 0;
   out_1157141240317607749[306] = 0;
   out_1157141240317607749[307] = 0;
   out_1157141240317607749[308] = 0;
   out_1157141240317607749[309] = 0;
   out_1157141240317607749[310] = 0;
   out_1157141240317607749[311] = 0;
   out_1157141240317607749[312] = 0;
   out_1157141240317607749[313] = 0;
   out_1157141240317607749[314] = 0;
   out_1157141240317607749[315] = 0;
   out_1157141240317607749[316] = 0;
   out_1157141240317607749[317] = 0;
   out_1157141240317607749[318] = 0;
   out_1157141240317607749[319] = 0;
   out_1157141240317607749[320] = 0;
   out_1157141240317607749[321] = 0;
   out_1157141240317607749[322] = 0;
   out_1157141240317607749[323] = 1;
}
void h_4(double *state, double *unused, double *out_5928686857747843496) {
   out_5928686857747843496[0] = state[6] + state[9];
   out_5928686857747843496[1] = state[7] + state[10];
   out_5928686857747843496[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6988887859530420628) {
   out_6988887859530420628[0] = 0;
   out_6988887859530420628[1] = 0;
   out_6988887859530420628[2] = 0;
   out_6988887859530420628[3] = 0;
   out_6988887859530420628[4] = 0;
   out_6988887859530420628[5] = 0;
   out_6988887859530420628[6] = 1;
   out_6988887859530420628[7] = 0;
   out_6988887859530420628[8] = 0;
   out_6988887859530420628[9] = 1;
   out_6988887859530420628[10] = 0;
   out_6988887859530420628[11] = 0;
   out_6988887859530420628[12] = 0;
   out_6988887859530420628[13] = 0;
   out_6988887859530420628[14] = 0;
   out_6988887859530420628[15] = 0;
   out_6988887859530420628[16] = 0;
   out_6988887859530420628[17] = 0;
   out_6988887859530420628[18] = 0;
   out_6988887859530420628[19] = 0;
   out_6988887859530420628[20] = 0;
   out_6988887859530420628[21] = 0;
   out_6988887859530420628[22] = 0;
   out_6988887859530420628[23] = 0;
   out_6988887859530420628[24] = 0;
   out_6988887859530420628[25] = 1;
   out_6988887859530420628[26] = 0;
   out_6988887859530420628[27] = 0;
   out_6988887859530420628[28] = 1;
   out_6988887859530420628[29] = 0;
   out_6988887859530420628[30] = 0;
   out_6988887859530420628[31] = 0;
   out_6988887859530420628[32] = 0;
   out_6988887859530420628[33] = 0;
   out_6988887859530420628[34] = 0;
   out_6988887859530420628[35] = 0;
   out_6988887859530420628[36] = 0;
   out_6988887859530420628[37] = 0;
   out_6988887859530420628[38] = 0;
   out_6988887859530420628[39] = 0;
   out_6988887859530420628[40] = 0;
   out_6988887859530420628[41] = 0;
   out_6988887859530420628[42] = 0;
   out_6988887859530420628[43] = 0;
   out_6988887859530420628[44] = 1;
   out_6988887859530420628[45] = 0;
   out_6988887859530420628[46] = 0;
   out_6988887859530420628[47] = 1;
   out_6988887859530420628[48] = 0;
   out_6988887859530420628[49] = 0;
   out_6988887859530420628[50] = 0;
   out_6988887859530420628[51] = 0;
   out_6988887859530420628[52] = 0;
   out_6988887859530420628[53] = 0;
}
void h_10(double *state, double *unused, double *out_2233713730666020117) {
   out_2233713730666020117[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2233713730666020117[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2233713730666020117[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8215253873719460024) {
   out_8215253873719460024[0] = 0;
   out_8215253873719460024[1] = 9.8100000000000005*cos(state[1]);
   out_8215253873719460024[2] = 0;
   out_8215253873719460024[3] = 0;
   out_8215253873719460024[4] = -state[8];
   out_8215253873719460024[5] = state[7];
   out_8215253873719460024[6] = 0;
   out_8215253873719460024[7] = state[5];
   out_8215253873719460024[8] = -state[4];
   out_8215253873719460024[9] = 0;
   out_8215253873719460024[10] = 0;
   out_8215253873719460024[11] = 0;
   out_8215253873719460024[12] = 1;
   out_8215253873719460024[13] = 0;
   out_8215253873719460024[14] = 0;
   out_8215253873719460024[15] = 1;
   out_8215253873719460024[16] = 0;
   out_8215253873719460024[17] = 0;
   out_8215253873719460024[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8215253873719460024[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8215253873719460024[20] = 0;
   out_8215253873719460024[21] = state[8];
   out_8215253873719460024[22] = 0;
   out_8215253873719460024[23] = -state[6];
   out_8215253873719460024[24] = -state[5];
   out_8215253873719460024[25] = 0;
   out_8215253873719460024[26] = state[3];
   out_8215253873719460024[27] = 0;
   out_8215253873719460024[28] = 0;
   out_8215253873719460024[29] = 0;
   out_8215253873719460024[30] = 0;
   out_8215253873719460024[31] = 1;
   out_8215253873719460024[32] = 0;
   out_8215253873719460024[33] = 0;
   out_8215253873719460024[34] = 1;
   out_8215253873719460024[35] = 0;
   out_8215253873719460024[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8215253873719460024[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8215253873719460024[38] = 0;
   out_8215253873719460024[39] = -state[7];
   out_8215253873719460024[40] = state[6];
   out_8215253873719460024[41] = 0;
   out_8215253873719460024[42] = state[4];
   out_8215253873719460024[43] = -state[3];
   out_8215253873719460024[44] = 0;
   out_8215253873719460024[45] = 0;
   out_8215253873719460024[46] = 0;
   out_8215253873719460024[47] = 0;
   out_8215253873719460024[48] = 0;
   out_8215253873719460024[49] = 0;
   out_8215253873719460024[50] = 1;
   out_8215253873719460024[51] = 0;
   out_8215253873719460024[52] = 0;
   out_8215253873719460024[53] = 1;
}
void h_13(double *state, double *unused, double *out_3078021598904732947) {
   out_3078021598904732947[0] = state[3];
   out_3078021598904732947[1] = state[4];
   out_3078021598904732947[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3847225005862430059) {
   out_3847225005862430059[0] = 0;
   out_3847225005862430059[1] = 0;
   out_3847225005862430059[2] = 0;
   out_3847225005862430059[3] = 1;
   out_3847225005862430059[4] = 0;
   out_3847225005862430059[5] = 0;
   out_3847225005862430059[6] = 0;
   out_3847225005862430059[7] = 0;
   out_3847225005862430059[8] = 0;
   out_3847225005862430059[9] = 0;
   out_3847225005862430059[10] = 0;
   out_3847225005862430059[11] = 0;
   out_3847225005862430059[12] = 0;
   out_3847225005862430059[13] = 0;
   out_3847225005862430059[14] = 0;
   out_3847225005862430059[15] = 0;
   out_3847225005862430059[16] = 0;
   out_3847225005862430059[17] = 0;
   out_3847225005862430059[18] = 0;
   out_3847225005862430059[19] = 0;
   out_3847225005862430059[20] = 0;
   out_3847225005862430059[21] = 0;
   out_3847225005862430059[22] = 1;
   out_3847225005862430059[23] = 0;
   out_3847225005862430059[24] = 0;
   out_3847225005862430059[25] = 0;
   out_3847225005862430059[26] = 0;
   out_3847225005862430059[27] = 0;
   out_3847225005862430059[28] = 0;
   out_3847225005862430059[29] = 0;
   out_3847225005862430059[30] = 0;
   out_3847225005862430059[31] = 0;
   out_3847225005862430059[32] = 0;
   out_3847225005862430059[33] = 0;
   out_3847225005862430059[34] = 0;
   out_3847225005862430059[35] = 0;
   out_3847225005862430059[36] = 0;
   out_3847225005862430059[37] = 0;
   out_3847225005862430059[38] = 0;
   out_3847225005862430059[39] = 0;
   out_3847225005862430059[40] = 0;
   out_3847225005862430059[41] = 1;
   out_3847225005862430059[42] = 0;
   out_3847225005862430059[43] = 0;
   out_3847225005862430059[44] = 0;
   out_3847225005862430059[45] = 0;
   out_3847225005862430059[46] = 0;
   out_3847225005862430059[47] = 0;
   out_3847225005862430059[48] = 0;
   out_3847225005862430059[49] = 0;
   out_3847225005862430059[50] = 0;
   out_3847225005862430059[51] = 0;
   out_3847225005862430059[52] = 0;
   out_3847225005862430059[53] = 0;
}
void h_14(double *state, double *unused, double *out_3755611037136954903) {
   out_3755611037136954903[0] = state[6];
   out_3755611037136954903[1] = state[7];
   out_3755611037136954903[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3906099427235048332) {
   out_3906099427235048332[0] = 0;
   out_3906099427235048332[1] = 0;
   out_3906099427235048332[2] = 0;
   out_3906099427235048332[3] = 0;
   out_3906099427235048332[4] = 0;
   out_3906099427235048332[5] = 0;
   out_3906099427235048332[6] = 1;
   out_3906099427235048332[7] = 0;
   out_3906099427235048332[8] = 0;
   out_3906099427235048332[9] = 0;
   out_3906099427235048332[10] = 0;
   out_3906099427235048332[11] = 0;
   out_3906099427235048332[12] = 0;
   out_3906099427235048332[13] = 0;
   out_3906099427235048332[14] = 0;
   out_3906099427235048332[15] = 0;
   out_3906099427235048332[16] = 0;
   out_3906099427235048332[17] = 0;
   out_3906099427235048332[18] = 0;
   out_3906099427235048332[19] = 0;
   out_3906099427235048332[20] = 0;
   out_3906099427235048332[21] = 0;
   out_3906099427235048332[22] = 0;
   out_3906099427235048332[23] = 0;
   out_3906099427235048332[24] = 0;
   out_3906099427235048332[25] = 1;
   out_3906099427235048332[26] = 0;
   out_3906099427235048332[27] = 0;
   out_3906099427235048332[28] = 0;
   out_3906099427235048332[29] = 0;
   out_3906099427235048332[30] = 0;
   out_3906099427235048332[31] = 0;
   out_3906099427235048332[32] = 0;
   out_3906099427235048332[33] = 0;
   out_3906099427235048332[34] = 0;
   out_3906099427235048332[35] = 0;
   out_3906099427235048332[36] = 0;
   out_3906099427235048332[37] = 0;
   out_3906099427235048332[38] = 0;
   out_3906099427235048332[39] = 0;
   out_3906099427235048332[40] = 0;
   out_3906099427235048332[41] = 0;
   out_3906099427235048332[42] = 0;
   out_3906099427235048332[43] = 0;
   out_3906099427235048332[44] = 1;
   out_3906099427235048332[45] = 0;
   out_3906099427235048332[46] = 0;
   out_3906099427235048332[47] = 0;
   out_3906099427235048332[48] = 0;
   out_3906099427235048332[49] = 0;
   out_3906099427235048332[50] = 0;
   out_3906099427235048332[51] = 0;
   out_3906099427235048332[52] = 0;
   out_3906099427235048332[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3980012004147667955) {
  err_fun(nom_x, delta_x, out_3980012004147667955);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6720818933782490088) {
  inv_err_fun(nom_x, true_x, out_6720818933782490088);
}
void pose_H_mod_fun(double *state, double *out_3217080649686513056) {
  H_mod_fun(state, out_3217080649686513056);
}
void pose_f_fun(double *state, double dt, double *out_3363301103592732445) {
  f_fun(state,  dt, out_3363301103592732445);
}
void pose_F_fun(double *state, double dt, double *out_1157141240317607749) {
  F_fun(state,  dt, out_1157141240317607749);
}
void pose_h_4(double *state, double *unused, double *out_5928686857747843496) {
  h_4(state, unused, out_5928686857747843496);
}
void pose_H_4(double *state, double *unused, double *out_6988887859530420628) {
  H_4(state, unused, out_6988887859530420628);
}
void pose_h_10(double *state, double *unused, double *out_2233713730666020117) {
  h_10(state, unused, out_2233713730666020117);
}
void pose_H_10(double *state, double *unused, double *out_8215253873719460024) {
  H_10(state, unused, out_8215253873719460024);
}
void pose_h_13(double *state, double *unused, double *out_3078021598904732947) {
  h_13(state, unused, out_3078021598904732947);
}
void pose_H_13(double *state, double *unused, double *out_3847225005862430059) {
  H_13(state, unused, out_3847225005862430059);
}
void pose_h_14(double *state, double *unused, double *out_3755611037136954903) {
  h_14(state, unused, out_3755611037136954903);
}
void pose_H_14(double *state, double *unused, double *out_3906099427235048332) {
  H_14(state, unused, out_3906099427235048332);
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
