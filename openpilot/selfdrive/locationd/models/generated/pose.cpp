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
void err_fun(double *nom_x, double *delta_x, double *out_2966425499024310265) {
   out_2966425499024310265[0] = delta_x[0] + nom_x[0];
   out_2966425499024310265[1] = delta_x[1] + nom_x[1];
   out_2966425499024310265[2] = delta_x[2] + nom_x[2];
   out_2966425499024310265[3] = delta_x[3] + nom_x[3];
   out_2966425499024310265[4] = delta_x[4] + nom_x[4];
   out_2966425499024310265[5] = delta_x[5] + nom_x[5];
   out_2966425499024310265[6] = delta_x[6] + nom_x[6];
   out_2966425499024310265[7] = delta_x[7] + nom_x[7];
   out_2966425499024310265[8] = delta_x[8] + nom_x[8];
   out_2966425499024310265[9] = delta_x[9] + nom_x[9];
   out_2966425499024310265[10] = delta_x[10] + nom_x[10];
   out_2966425499024310265[11] = delta_x[11] + nom_x[11];
   out_2966425499024310265[12] = delta_x[12] + nom_x[12];
   out_2966425499024310265[13] = delta_x[13] + nom_x[13];
   out_2966425499024310265[14] = delta_x[14] + nom_x[14];
   out_2966425499024310265[15] = delta_x[15] + nom_x[15];
   out_2966425499024310265[16] = delta_x[16] + nom_x[16];
   out_2966425499024310265[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4677272284422365493) {
   out_4677272284422365493[0] = -nom_x[0] + true_x[0];
   out_4677272284422365493[1] = -nom_x[1] + true_x[1];
   out_4677272284422365493[2] = -nom_x[2] + true_x[2];
   out_4677272284422365493[3] = -nom_x[3] + true_x[3];
   out_4677272284422365493[4] = -nom_x[4] + true_x[4];
   out_4677272284422365493[5] = -nom_x[5] + true_x[5];
   out_4677272284422365493[6] = -nom_x[6] + true_x[6];
   out_4677272284422365493[7] = -nom_x[7] + true_x[7];
   out_4677272284422365493[8] = -nom_x[8] + true_x[8];
   out_4677272284422365493[9] = -nom_x[9] + true_x[9];
   out_4677272284422365493[10] = -nom_x[10] + true_x[10];
   out_4677272284422365493[11] = -nom_x[11] + true_x[11];
   out_4677272284422365493[12] = -nom_x[12] + true_x[12];
   out_4677272284422365493[13] = -nom_x[13] + true_x[13];
   out_4677272284422365493[14] = -nom_x[14] + true_x[14];
   out_4677272284422365493[15] = -nom_x[15] + true_x[15];
   out_4677272284422365493[16] = -nom_x[16] + true_x[16];
   out_4677272284422365493[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6471686313161485195) {
   out_6471686313161485195[0] = 1.0;
   out_6471686313161485195[1] = 0.0;
   out_6471686313161485195[2] = 0.0;
   out_6471686313161485195[3] = 0.0;
   out_6471686313161485195[4] = 0.0;
   out_6471686313161485195[5] = 0.0;
   out_6471686313161485195[6] = 0.0;
   out_6471686313161485195[7] = 0.0;
   out_6471686313161485195[8] = 0.0;
   out_6471686313161485195[9] = 0.0;
   out_6471686313161485195[10] = 0.0;
   out_6471686313161485195[11] = 0.0;
   out_6471686313161485195[12] = 0.0;
   out_6471686313161485195[13] = 0.0;
   out_6471686313161485195[14] = 0.0;
   out_6471686313161485195[15] = 0.0;
   out_6471686313161485195[16] = 0.0;
   out_6471686313161485195[17] = 0.0;
   out_6471686313161485195[18] = 0.0;
   out_6471686313161485195[19] = 1.0;
   out_6471686313161485195[20] = 0.0;
   out_6471686313161485195[21] = 0.0;
   out_6471686313161485195[22] = 0.0;
   out_6471686313161485195[23] = 0.0;
   out_6471686313161485195[24] = 0.0;
   out_6471686313161485195[25] = 0.0;
   out_6471686313161485195[26] = 0.0;
   out_6471686313161485195[27] = 0.0;
   out_6471686313161485195[28] = 0.0;
   out_6471686313161485195[29] = 0.0;
   out_6471686313161485195[30] = 0.0;
   out_6471686313161485195[31] = 0.0;
   out_6471686313161485195[32] = 0.0;
   out_6471686313161485195[33] = 0.0;
   out_6471686313161485195[34] = 0.0;
   out_6471686313161485195[35] = 0.0;
   out_6471686313161485195[36] = 0.0;
   out_6471686313161485195[37] = 0.0;
   out_6471686313161485195[38] = 1.0;
   out_6471686313161485195[39] = 0.0;
   out_6471686313161485195[40] = 0.0;
   out_6471686313161485195[41] = 0.0;
   out_6471686313161485195[42] = 0.0;
   out_6471686313161485195[43] = 0.0;
   out_6471686313161485195[44] = 0.0;
   out_6471686313161485195[45] = 0.0;
   out_6471686313161485195[46] = 0.0;
   out_6471686313161485195[47] = 0.0;
   out_6471686313161485195[48] = 0.0;
   out_6471686313161485195[49] = 0.0;
   out_6471686313161485195[50] = 0.0;
   out_6471686313161485195[51] = 0.0;
   out_6471686313161485195[52] = 0.0;
   out_6471686313161485195[53] = 0.0;
   out_6471686313161485195[54] = 0.0;
   out_6471686313161485195[55] = 0.0;
   out_6471686313161485195[56] = 0.0;
   out_6471686313161485195[57] = 1.0;
   out_6471686313161485195[58] = 0.0;
   out_6471686313161485195[59] = 0.0;
   out_6471686313161485195[60] = 0.0;
   out_6471686313161485195[61] = 0.0;
   out_6471686313161485195[62] = 0.0;
   out_6471686313161485195[63] = 0.0;
   out_6471686313161485195[64] = 0.0;
   out_6471686313161485195[65] = 0.0;
   out_6471686313161485195[66] = 0.0;
   out_6471686313161485195[67] = 0.0;
   out_6471686313161485195[68] = 0.0;
   out_6471686313161485195[69] = 0.0;
   out_6471686313161485195[70] = 0.0;
   out_6471686313161485195[71] = 0.0;
   out_6471686313161485195[72] = 0.0;
   out_6471686313161485195[73] = 0.0;
   out_6471686313161485195[74] = 0.0;
   out_6471686313161485195[75] = 0.0;
   out_6471686313161485195[76] = 1.0;
   out_6471686313161485195[77] = 0.0;
   out_6471686313161485195[78] = 0.0;
   out_6471686313161485195[79] = 0.0;
   out_6471686313161485195[80] = 0.0;
   out_6471686313161485195[81] = 0.0;
   out_6471686313161485195[82] = 0.0;
   out_6471686313161485195[83] = 0.0;
   out_6471686313161485195[84] = 0.0;
   out_6471686313161485195[85] = 0.0;
   out_6471686313161485195[86] = 0.0;
   out_6471686313161485195[87] = 0.0;
   out_6471686313161485195[88] = 0.0;
   out_6471686313161485195[89] = 0.0;
   out_6471686313161485195[90] = 0.0;
   out_6471686313161485195[91] = 0.0;
   out_6471686313161485195[92] = 0.0;
   out_6471686313161485195[93] = 0.0;
   out_6471686313161485195[94] = 0.0;
   out_6471686313161485195[95] = 1.0;
   out_6471686313161485195[96] = 0.0;
   out_6471686313161485195[97] = 0.0;
   out_6471686313161485195[98] = 0.0;
   out_6471686313161485195[99] = 0.0;
   out_6471686313161485195[100] = 0.0;
   out_6471686313161485195[101] = 0.0;
   out_6471686313161485195[102] = 0.0;
   out_6471686313161485195[103] = 0.0;
   out_6471686313161485195[104] = 0.0;
   out_6471686313161485195[105] = 0.0;
   out_6471686313161485195[106] = 0.0;
   out_6471686313161485195[107] = 0.0;
   out_6471686313161485195[108] = 0.0;
   out_6471686313161485195[109] = 0.0;
   out_6471686313161485195[110] = 0.0;
   out_6471686313161485195[111] = 0.0;
   out_6471686313161485195[112] = 0.0;
   out_6471686313161485195[113] = 0.0;
   out_6471686313161485195[114] = 1.0;
   out_6471686313161485195[115] = 0.0;
   out_6471686313161485195[116] = 0.0;
   out_6471686313161485195[117] = 0.0;
   out_6471686313161485195[118] = 0.0;
   out_6471686313161485195[119] = 0.0;
   out_6471686313161485195[120] = 0.0;
   out_6471686313161485195[121] = 0.0;
   out_6471686313161485195[122] = 0.0;
   out_6471686313161485195[123] = 0.0;
   out_6471686313161485195[124] = 0.0;
   out_6471686313161485195[125] = 0.0;
   out_6471686313161485195[126] = 0.0;
   out_6471686313161485195[127] = 0.0;
   out_6471686313161485195[128] = 0.0;
   out_6471686313161485195[129] = 0.0;
   out_6471686313161485195[130] = 0.0;
   out_6471686313161485195[131] = 0.0;
   out_6471686313161485195[132] = 0.0;
   out_6471686313161485195[133] = 1.0;
   out_6471686313161485195[134] = 0.0;
   out_6471686313161485195[135] = 0.0;
   out_6471686313161485195[136] = 0.0;
   out_6471686313161485195[137] = 0.0;
   out_6471686313161485195[138] = 0.0;
   out_6471686313161485195[139] = 0.0;
   out_6471686313161485195[140] = 0.0;
   out_6471686313161485195[141] = 0.0;
   out_6471686313161485195[142] = 0.0;
   out_6471686313161485195[143] = 0.0;
   out_6471686313161485195[144] = 0.0;
   out_6471686313161485195[145] = 0.0;
   out_6471686313161485195[146] = 0.0;
   out_6471686313161485195[147] = 0.0;
   out_6471686313161485195[148] = 0.0;
   out_6471686313161485195[149] = 0.0;
   out_6471686313161485195[150] = 0.0;
   out_6471686313161485195[151] = 0.0;
   out_6471686313161485195[152] = 1.0;
   out_6471686313161485195[153] = 0.0;
   out_6471686313161485195[154] = 0.0;
   out_6471686313161485195[155] = 0.0;
   out_6471686313161485195[156] = 0.0;
   out_6471686313161485195[157] = 0.0;
   out_6471686313161485195[158] = 0.0;
   out_6471686313161485195[159] = 0.0;
   out_6471686313161485195[160] = 0.0;
   out_6471686313161485195[161] = 0.0;
   out_6471686313161485195[162] = 0.0;
   out_6471686313161485195[163] = 0.0;
   out_6471686313161485195[164] = 0.0;
   out_6471686313161485195[165] = 0.0;
   out_6471686313161485195[166] = 0.0;
   out_6471686313161485195[167] = 0.0;
   out_6471686313161485195[168] = 0.0;
   out_6471686313161485195[169] = 0.0;
   out_6471686313161485195[170] = 0.0;
   out_6471686313161485195[171] = 1.0;
   out_6471686313161485195[172] = 0.0;
   out_6471686313161485195[173] = 0.0;
   out_6471686313161485195[174] = 0.0;
   out_6471686313161485195[175] = 0.0;
   out_6471686313161485195[176] = 0.0;
   out_6471686313161485195[177] = 0.0;
   out_6471686313161485195[178] = 0.0;
   out_6471686313161485195[179] = 0.0;
   out_6471686313161485195[180] = 0.0;
   out_6471686313161485195[181] = 0.0;
   out_6471686313161485195[182] = 0.0;
   out_6471686313161485195[183] = 0.0;
   out_6471686313161485195[184] = 0.0;
   out_6471686313161485195[185] = 0.0;
   out_6471686313161485195[186] = 0.0;
   out_6471686313161485195[187] = 0.0;
   out_6471686313161485195[188] = 0.0;
   out_6471686313161485195[189] = 0.0;
   out_6471686313161485195[190] = 1.0;
   out_6471686313161485195[191] = 0.0;
   out_6471686313161485195[192] = 0.0;
   out_6471686313161485195[193] = 0.0;
   out_6471686313161485195[194] = 0.0;
   out_6471686313161485195[195] = 0.0;
   out_6471686313161485195[196] = 0.0;
   out_6471686313161485195[197] = 0.0;
   out_6471686313161485195[198] = 0.0;
   out_6471686313161485195[199] = 0.0;
   out_6471686313161485195[200] = 0.0;
   out_6471686313161485195[201] = 0.0;
   out_6471686313161485195[202] = 0.0;
   out_6471686313161485195[203] = 0.0;
   out_6471686313161485195[204] = 0.0;
   out_6471686313161485195[205] = 0.0;
   out_6471686313161485195[206] = 0.0;
   out_6471686313161485195[207] = 0.0;
   out_6471686313161485195[208] = 0.0;
   out_6471686313161485195[209] = 1.0;
   out_6471686313161485195[210] = 0.0;
   out_6471686313161485195[211] = 0.0;
   out_6471686313161485195[212] = 0.0;
   out_6471686313161485195[213] = 0.0;
   out_6471686313161485195[214] = 0.0;
   out_6471686313161485195[215] = 0.0;
   out_6471686313161485195[216] = 0.0;
   out_6471686313161485195[217] = 0.0;
   out_6471686313161485195[218] = 0.0;
   out_6471686313161485195[219] = 0.0;
   out_6471686313161485195[220] = 0.0;
   out_6471686313161485195[221] = 0.0;
   out_6471686313161485195[222] = 0.0;
   out_6471686313161485195[223] = 0.0;
   out_6471686313161485195[224] = 0.0;
   out_6471686313161485195[225] = 0.0;
   out_6471686313161485195[226] = 0.0;
   out_6471686313161485195[227] = 0.0;
   out_6471686313161485195[228] = 1.0;
   out_6471686313161485195[229] = 0.0;
   out_6471686313161485195[230] = 0.0;
   out_6471686313161485195[231] = 0.0;
   out_6471686313161485195[232] = 0.0;
   out_6471686313161485195[233] = 0.0;
   out_6471686313161485195[234] = 0.0;
   out_6471686313161485195[235] = 0.0;
   out_6471686313161485195[236] = 0.0;
   out_6471686313161485195[237] = 0.0;
   out_6471686313161485195[238] = 0.0;
   out_6471686313161485195[239] = 0.0;
   out_6471686313161485195[240] = 0.0;
   out_6471686313161485195[241] = 0.0;
   out_6471686313161485195[242] = 0.0;
   out_6471686313161485195[243] = 0.0;
   out_6471686313161485195[244] = 0.0;
   out_6471686313161485195[245] = 0.0;
   out_6471686313161485195[246] = 0.0;
   out_6471686313161485195[247] = 1.0;
   out_6471686313161485195[248] = 0.0;
   out_6471686313161485195[249] = 0.0;
   out_6471686313161485195[250] = 0.0;
   out_6471686313161485195[251] = 0.0;
   out_6471686313161485195[252] = 0.0;
   out_6471686313161485195[253] = 0.0;
   out_6471686313161485195[254] = 0.0;
   out_6471686313161485195[255] = 0.0;
   out_6471686313161485195[256] = 0.0;
   out_6471686313161485195[257] = 0.0;
   out_6471686313161485195[258] = 0.0;
   out_6471686313161485195[259] = 0.0;
   out_6471686313161485195[260] = 0.0;
   out_6471686313161485195[261] = 0.0;
   out_6471686313161485195[262] = 0.0;
   out_6471686313161485195[263] = 0.0;
   out_6471686313161485195[264] = 0.0;
   out_6471686313161485195[265] = 0.0;
   out_6471686313161485195[266] = 1.0;
   out_6471686313161485195[267] = 0.0;
   out_6471686313161485195[268] = 0.0;
   out_6471686313161485195[269] = 0.0;
   out_6471686313161485195[270] = 0.0;
   out_6471686313161485195[271] = 0.0;
   out_6471686313161485195[272] = 0.0;
   out_6471686313161485195[273] = 0.0;
   out_6471686313161485195[274] = 0.0;
   out_6471686313161485195[275] = 0.0;
   out_6471686313161485195[276] = 0.0;
   out_6471686313161485195[277] = 0.0;
   out_6471686313161485195[278] = 0.0;
   out_6471686313161485195[279] = 0.0;
   out_6471686313161485195[280] = 0.0;
   out_6471686313161485195[281] = 0.0;
   out_6471686313161485195[282] = 0.0;
   out_6471686313161485195[283] = 0.0;
   out_6471686313161485195[284] = 0.0;
   out_6471686313161485195[285] = 1.0;
   out_6471686313161485195[286] = 0.0;
   out_6471686313161485195[287] = 0.0;
   out_6471686313161485195[288] = 0.0;
   out_6471686313161485195[289] = 0.0;
   out_6471686313161485195[290] = 0.0;
   out_6471686313161485195[291] = 0.0;
   out_6471686313161485195[292] = 0.0;
   out_6471686313161485195[293] = 0.0;
   out_6471686313161485195[294] = 0.0;
   out_6471686313161485195[295] = 0.0;
   out_6471686313161485195[296] = 0.0;
   out_6471686313161485195[297] = 0.0;
   out_6471686313161485195[298] = 0.0;
   out_6471686313161485195[299] = 0.0;
   out_6471686313161485195[300] = 0.0;
   out_6471686313161485195[301] = 0.0;
   out_6471686313161485195[302] = 0.0;
   out_6471686313161485195[303] = 0.0;
   out_6471686313161485195[304] = 1.0;
   out_6471686313161485195[305] = 0.0;
   out_6471686313161485195[306] = 0.0;
   out_6471686313161485195[307] = 0.0;
   out_6471686313161485195[308] = 0.0;
   out_6471686313161485195[309] = 0.0;
   out_6471686313161485195[310] = 0.0;
   out_6471686313161485195[311] = 0.0;
   out_6471686313161485195[312] = 0.0;
   out_6471686313161485195[313] = 0.0;
   out_6471686313161485195[314] = 0.0;
   out_6471686313161485195[315] = 0.0;
   out_6471686313161485195[316] = 0.0;
   out_6471686313161485195[317] = 0.0;
   out_6471686313161485195[318] = 0.0;
   out_6471686313161485195[319] = 0.0;
   out_6471686313161485195[320] = 0.0;
   out_6471686313161485195[321] = 0.0;
   out_6471686313161485195[322] = 0.0;
   out_6471686313161485195[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8577303503121138630) {
   out_8577303503121138630[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8577303503121138630[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8577303503121138630[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8577303503121138630[3] = dt*state[12] + state[3];
   out_8577303503121138630[4] = dt*state[13] + state[4];
   out_8577303503121138630[5] = dt*state[14] + state[5];
   out_8577303503121138630[6] = state[6];
   out_8577303503121138630[7] = state[7];
   out_8577303503121138630[8] = state[8];
   out_8577303503121138630[9] = state[9];
   out_8577303503121138630[10] = state[10];
   out_8577303503121138630[11] = state[11];
   out_8577303503121138630[12] = state[12];
   out_8577303503121138630[13] = state[13];
   out_8577303503121138630[14] = state[14];
   out_8577303503121138630[15] = state[15];
   out_8577303503121138630[16] = state[16];
   out_8577303503121138630[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5893284074989477049) {
   out_5893284074989477049[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5893284074989477049[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5893284074989477049[2] = 0;
   out_5893284074989477049[3] = 0;
   out_5893284074989477049[4] = 0;
   out_5893284074989477049[5] = 0;
   out_5893284074989477049[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5893284074989477049[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5893284074989477049[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5893284074989477049[9] = 0;
   out_5893284074989477049[10] = 0;
   out_5893284074989477049[11] = 0;
   out_5893284074989477049[12] = 0;
   out_5893284074989477049[13] = 0;
   out_5893284074989477049[14] = 0;
   out_5893284074989477049[15] = 0;
   out_5893284074989477049[16] = 0;
   out_5893284074989477049[17] = 0;
   out_5893284074989477049[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5893284074989477049[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5893284074989477049[20] = 0;
   out_5893284074989477049[21] = 0;
   out_5893284074989477049[22] = 0;
   out_5893284074989477049[23] = 0;
   out_5893284074989477049[24] = 0;
   out_5893284074989477049[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5893284074989477049[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5893284074989477049[27] = 0;
   out_5893284074989477049[28] = 0;
   out_5893284074989477049[29] = 0;
   out_5893284074989477049[30] = 0;
   out_5893284074989477049[31] = 0;
   out_5893284074989477049[32] = 0;
   out_5893284074989477049[33] = 0;
   out_5893284074989477049[34] = 0;
   out_5893284074989477049[35] = 0;
   out_5893284074989477049[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5893284074989477049[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5893284074989477049[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5893284074989477049[39] = 0;
   out_5893284074989477049[40] = 0;
   out_5893284074989477049[41] = 0;
   out_5893284074989477049[42] = 0;
   out_5893284074989477049[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5893284074989477049[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5893284074989477049[45] = 0;
   out_5893284074989477049[46] = 0;
   out_5893284074989477049[47] = 0;
   out_5893284074989477049[48] = 0;
   out_5893284074989477049[49] = 0;
   out_5893284074989477049[50] = 0;
   out_5893284074989477049[51] = 0;
   out_5893284074989477049[52] = 0;
   out_5893284074989477049[53] = 0;
   out_5893284074989477049[54] = 0;
   out_5893284074989477049[55] = 0;
   out_5893284074989477049[56] = 0;
   out_5893284074989477049[57] = 1;
   out_5893284074989477049[58] = 0;
   out_5893284074989477049[59] = 0;
   out_5893284074989477049[60] = 0;
   out_5893284074989477049[61] = 0;
   out_5893284074989477049[62] = 0;
   out_5893284074989477049[63] = 0;
   out_5893284074989477049[64] = 0;
   out_5893284074989477049[65] = 0;
   out_5893284074989477049[66] = dt;
   out_5893284074989477049[67] = 0;
   out_5893284074989477049[68] = 0;
   out_5893284074989477049[69] = 0;
   out_5893284074989477049[70] = 0;
   out_5893284074989477049[71] = 0;
   out_5893284074989477049[72] = 0;
   out_5893284074989477049[73] = 0;
   out_5893284074989477049[74] = 0;
   out_5893284074989477049[75] = 0;
   out_5893284074989477049[76] = 1;
   out_5893284074989477049[77] = 0;
   out_5893284074989477049[78] = 0;
   out_5893284074989477049[79] = 0;
   out_5893284074989477049[80] = 0;
   out_5893284074989477049[81] = 0;
   out_5893284074989477049[82] = 0;
   out_5893284074989477049[83] = 0;
   out_5893284074989477049[84] = 0;
   out_5893284074989477049[85] = dt;
   out_5893284074989477049[86] = 0;
   out_5893284074989477049[87] = 0;
   out_5893284074989477049[88] = 0;
   out_5893284074989477049[89] = 0;
   out_5893284074989477049[90] = 0;
   out_5893284074989477049[91] = 0;
   out_5893284074989477049[92] = 0;
   out_5893284074989477049[93] = 0;
   out_5893284074989477049[94] = 0;
   out_5893284074989477049[95] = 1;
   out_5893284074989477049[96] = 0;
   out_5893284074989477049[97] = 0;
   out_5893284074989477049[98] = 0;
   out_5893284074989477049[99] = 0;
   out_5893284074989477049[100] = 0;
   out_5893284074989477049[101] = 0;
   out_5893284074989477049[102] = 0;
   out_5893284074989477049[103] = 0;
   out_5893284074989477049[104] = dt;
   out_5893284074989477049[105] = 0;
   out_5893284074989477049[106] = 0;
   out_5893284074989477049[107] = 0;
   out_5893284074989477049[108] = 0;
   out_5893284074989477049[109] = 0;
   out_5893284074989477049[110] = 0;
   out_5893284074989477049[111] = 0;
   out_5893284074989477049[112] = 0;
   out_5893284074989477049[113] = 0;
   out_5893284074989477049[114] = 1;
   out_5893284074989477049[115] = 0;
   out_5893284074989477049[116] = 0;
   out_5893284074989477049[117] = 0;
   out_5893284074989477049[118] = 0;
   out_5893284074989477049[119] = 0;
   out_5893284074989477049[120] = 0;
   out_5893284074989477049[121] = 0;
   out_5893284074989477049[122] = 0;
   out_5893284074989477049[123] = 0;
   out_5893284074989477049[124] = 0;
   out_5893284074989477049[125] = 0;
   out_5893284074989477049[126] = 0;
   out_5893284074989477049[127] = 0;
   out_5893284074989477049[128] = 0;
   out_5893284074989477049[129] = 0;
   out_5893284074989477049[130] = 0;
   out_5893284074989477049[131] = 0;
   out_5893284074989477049[132] = 0;
   out_5893284074989477049[133] = 1;
   out_5893284074989477049[134] = 0;
   out_5893284074989477049[135] = 0;
   out_5893284074989477049[136] = 0;
   out_5893284074989477049[137] = 0;
   out_5893284074989477049[138] = 0;
   out_5893284074989477049[139] = 0;
   out_5893284074989477049[140] = 0;
   out_5893284074989477049[141] = 0;
   out_5893284074989477049[142] = 0;
   out_5893284074989477049[143] = 0;
   out_5893284074989477049[144] = 0;
   out_5893284074989477049[145] = 0;
   out_5893284074989477049[146] = 0;
   out_5893284074989477049[147] = 0;
   out_5893284074989477049[148] = 0;
   out_5893284074989477049[149] = 0;
   out_5893284074989477049[150] = 0;
   out_5893284074989477049[151] = 0;
   out_5893284074989477049[152] = 1;
   out_5893284074989477049[153] = 0;
   out_5893284074989477049[154] = 0;
   out_5893284074989477049[155] = 0;
   out_5893284074989477049[156] = 0;
   out_5893284074989477049[157] = 0;
   out_5893284074989477049[158] = 0;
   out_5893284074989477049[159] = 0;
   out_5893284074989477049[160] = 0;
   out_5893284074989477049[161] = 0;
   out_5893284074989477049[162] = 0;
   out_5893284074989477049[163] = 0;
   out_5893284074989477049[164] = 0;
   out_5893284074989477049[165] = 0;
   out_5893284074989477049[166] = 0;
   out_5893284074989477049[167] = 0;
   out_5893284074989477049[168] = 0;
   out_5893284074989477049[169] = 0;
   out_5893284074989477049[170] = 0;
   out_5893284074989477049[171] = 1;
   out_5893284074989477049[172] = 0;
   out_5893284074989477049[173] = 0;
   out_5893284074989477049[174] = 0;
   out_5893284074989477049[175] = 0;
   out_5893284074989477049[176] = 0;
   out_5893284074989477049[177] = 0;
   out_5893284074989477049[178] = 0;
   out_5893284074989477049[179] = 0;
   out_5893284074989477049[180] = 0;
   out_5893284074989477049[181] = 0;
   out_5893284074989477049[182] = 0;
   out_5893284074989477049[183] = 0;
   out_5893284074989477049[184] = 0;
   out_5893284074989477049[185] = 0;
   out_5893284074989477049[186] = 0;
   out_5893284074989477049[187] = 0;
   out_5893284074989477049[188] = 0;
   out_5893284074989477049[189] = 0;
   out_5893284074989477049[190] = 1;
   out_5893284074989477049[191] = 0;
   out_5893284074989477049[192] = 0;
   out_5893284074989477049[193] = 0;
   out_5893284074989477049[194] = 0;
   out_5893284074989477049[195] = 0;
   out_5893284074989477049[196] = 0;
   out_5893284074989477049[197] = 0;
   out_5893284074989477049[198] = 0;
   out_5893284074989477049[199] = 0;
   out_5893284074989477049[200] = 0;
   out_5893284074989477049[201] = 0;
   out_5893284074989477049[202] = 0;
   out_5893284074989477049[203] = 0;
   out_5893284074989477049[204] = 0;
   out_5893284074989477049[205] = 0;
   out_5893284074989477049[206] = 0;
   out_5893284074989477049[207] = 0;
   out_5893284074989477049[208] = 0;
   out_5893284074989477049[209] = 1;
   out_5893284074989477049[210] = 0;
   out_5893284074989477049[211] = 0;
   out_5893284074989477049[212] = 0;
   out_5893284074989477049[213] = 0;
   out_5893284074989477049[214] = 0;
   out_5893284074989477049[215] = 0;
   out_5893284074989477049[216] = 0;
   out_5893284074989477049[217] = 0;
   out_5893284074989477049[218] = 0;
   out_5893284074989477049[219] = 0;
   out_5893284074989477049[220] = 0;
   out_5893284074989477049[221] = 0;
   out_5893284074989477049[222] = 0;
   out_5893284074989477049[223] = 0;
   out_5893284074989477049[224] = 0;
   out_5893284074989477049[225] = 0;
   out_5893284074989477049[226] = 0;
   out_5893284074989477049[227] = 0;
   out_5893284074989477049[228] = 1;
   out_5893284074989477049[229] = 0;
   out_5893284074989477049[230] = 0;
   out_5893284074989477049[231] = 0;
   out_5893284074989477049[232] = 0;
   out_5893284074989477049[233] = 0;
   out_5893284074989477049[234] = 0;
   out_5893284074989477049[235] = 0;
   out_5893284074989477049[236] = 0;
   out_5893284074989477049[237] = 0;
   out_5893284074989477049[238] = 0;
   out_5893284074989477049[239] = 0;
   out_5893284074989477049[240] = 0;
   out_5893284074989477049[241] = 0;
   out_5893284074989477049[242] = 0;
   out_5893284074989477049[243] = 0;
   out_5893284074989477049[244] = 0;
   out_5893284074989477049[245] = 0;
   out_5893284074989477049[246] = 0;
   out_5893284074989477049[247] = 1;
   out_5893284074989477049[248] = 0;
   out_5893284074989477049[249] = 0;
   out_5893284074989477049[250] = 0;
   out_5893284074989477049[251] = 0;
   out_5893284074989477049[252] = 0;
   out_5893284074989477049[253] = 0;
   out_5893284074989477049[254] = 0;
   out_5893284074989477049[255] = 0;
   out_5893284074989477049[256] = 0;
   out_5893284074989477049[257] = 0;
   out_5893284074989477049[258] = 0;
   out_5893284074989477049[259] = 0;
   out_5893284074989477049[260] = 0;
   out_5893284074989477049[261] = 0;
   out_5893284074989477049[262] = 0;
   out_5893284074989477049[263] = 0;
   out_5893284074989477049[264] = 0;
   out_5893284074989477049[265] = 0;
   out_5893284074989477049[266] = 1;
   out_5893284074989477049[267] = 0;
   out_5893284074989477049[268] = 0;
   out_5893284074989477049[269] = 0;
   out_5893284074989477049[270] = 0;
   out_5893284074989477049[271] = 0;
   out_5893284074989477049[272] = 0;
   out_5893284074989477049[273] = 0;
   out_5893284074989477049[274] = 0;
   out_5893284074989477049[275] = 0;
   out_5893284074989477049[276] = 0;
   out_5893284074989477049[277] = 0;
   out_5893284074989477049[278] = 0;
   out_5893284074989477049[279] = 0;
   out_5893284074989477049[280] = 0;
   out_5893284074989477049[281] = 0;
   out_5893284074989477049[282] = 0;
   out_5893284074989477049[283] = 0;
   out_5893284074989477049[284] = 0;
   out_5893284074989477049[285] = 1;
   out_5893284074989477049[286] = 0;
   out_5893284074989477049[287] = 0;
   out_5893284074989477049[288] = 0;
   out_5893284074989477049[289] = 0;
   out_5893284074989477049[290] = 0;
   out_5893284074989477049[291] = 0;
   out_5893284074989477049[292] = 0;
   out_5893284074989477049[293] = 0;
   out_5893284074989477049[294] = 0;
   out_5893284074989477049[295] = 0;
   out_5893284074989477049[296] = 0;
   out_5893284074989477049[297] = 0;
   out_5893284074989477049[298] = 0;
   out_5893284074989477049[299] = 0;
   out_5893284074989477049[300] = 0;
   out_5893284074989477049[301] = 0;
   out_5893284074989477049[302] = 0;
   out_5893284074989477049[303] = 0;
   out_5893284074989477049[304] = 1;
   out_5893284074989477049[305] = 0;
   out_5893284074989477049[306] = 0;
   out_5893284074989477049[307] = 0;
   out_5893284074989477049[308] = 0;
   out_5893284074989477049[309] = 0;
   out_5893284074989477049[310] = 0;
   out_5893284074989477049[311] = 0;
   out_5893284074989477049[312] = 0;
   out_5893284074989477049[313] = 0;
   out_5893284074989477049[314] = 0;
   out_5893284074989477049[315] = 0;
   out_5893284074989477049[316] = 0;
   out_5893284074989477049[317] = 0;
   out_5893284074989477049[318] = 0;
   out_5893284074989477049[319] = 0;
   out_5893284074989477049[320] = 0;
   out_5893284074989477049[321] = 0;
   out_5893284074989477049[322] = 0;
   out_5893284074989477049[323] = 1;
}
void h_4(double *state, double *unused, double *out_3588243949097045624) {
   out_3588243949097045624[0] = state[6] + state[9];
   out_3588243949097045624[1] = state[7] + state[10];
   out_3588243949097045624[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2472675352896336147) {
   out_2472675352896336147[0] = 0;
   out_2472675352896336147[1] = 0;
   out_2472675352896336147[2] = 0;
   out_2472675352896336147[3] = 0;
   out_2472675352896336147[4] = 0;
   out_2472675352896336147[5] = 0;
   out_2472675352896336147[6] = 1;
   out_2472675352896336147[7] = 0;
   out_2472675352896336147[8] = 0;
   out_2472675352896336147[9] = 1;
   out_2472675352896336147[10] = 0;
   out_2472675352896336147[11] = 0;
   out_2472675352896336147[12] = 0;
   out_2472675352896336147[13] = 0;
   out_2472675352896336147[14] = 0;
   out_2472675352896336147[15] = 0;
   out_2472675352896336147[16] = 0;
   out_2472675352896336147[17] = 0;
   out_2472675352896336147[18] = 0;
   out_2472675352896336147[19] = 0;
   out_2472675352896336147[20] = 0;
   out_2472675352896336147[21] = 0;
   out_2472675352896336147[22] = 0;
   out_2472675352896336147[23] = 0;
   out_2472675352896336147[24] = 0;
   out_2472675352896336147[25] = 1;
   out_2472675352896336147[26] = 0;
   out_2472675352896336147[27] = 0;
   out_2472675352896336147[28] = 1;
   out_2472675352896336147[29] = 0;
   out_2472675352896336147[30] = 0;
   out_2472675352896336147[31] = 0;
   out_2472675352896336147[32] = 0;
   out_2472675352896336147[33] = 0;
   out_2472675352896336147[34] = 0;
   out_2472675352896336147[35] = 0;
   out_2472675352896336147[36] = 0;
   out_2472675352896336147[37] = 0;
   out_2472675352896336147[38] = 0;
   out_2472675352896336147[39] = 0;
   out_2472675352896336147[40] = 0;
   out_2472675352896336147[41] = 0;
   out_2472675352896336147[42] = 0;
   out_2472675352896336147[43] = 0;
   out_2472675352896336147[44] = 1;
   out_2472675352896336147[45] = 0;
   out_2472675352896336147[46] = 0;
   out_2472675352896336147[47] = 1;
   out_2472675352896336147[48] = 0;
   out_2472675352896336147[49] = 0;
   out_2472675352896336147[50] = 0;
   out_2472675352896336147[51] = 0;
   out_2472675352896336147[52] = 0;
   out_2472675352896336147[53] = 0;
}
void h_10(double *state, double *unused, double *out_6829459391907355466) {
   out_6829459391907355466[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6829459391907355466[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6829459391907355466[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5498212507308944349) {
   out_5498212507308944349[0] = 0;
   out_5498212507308944349[1] = 9.8100000000000005*cos(state[1]);
   out_5498212507308944349[2] = 0;
   out_5498212507308944349[3] = 0;
   out_5498212507308944349[4] = -state[8];
   out_5498212507308944349[5] = state[7];
   out_5498212507308944349[6] = 0;
   out_5498212507308944349[7] = state[5];
   out_5498212507308944349[8] = -state[4];
   out_5498212507308944349[9] = 0;
   out_5498212507308944349[10] = 0;
   out_5498212507308944349[11] = 0;
   out_5498212507308944349[12] = 1;
   out_5498212507308944349[13] = 0;
   out_5498212507308944349[14] = 0;
   out_5498212507308944349[15] = 1;
   out_5498212507308944349[16] = 0;
   out_5498212507308944349[17] = 0;
   out_5498212507308944349[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5498212507308944349[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5498212507308944349[20] = 0;
   out_5498212507308944349[21] = state[8];
   out_5498212507308944349[22] = 0;
   out_5498212507308944349[23] = -state[6];
   out_5498212507308944349[24] = -state[5];
   out_5498212507308944349[25] = 0;
   out_5498212507308944349[26] = state[3];
   out_5498212507308944349[27] = 0;
   out_5498212507308944349[28] = 0;
   out_5498212507308944349[29] = 0;
   out_5498212507308944349[30] = 0;
   out_5498212507308944349[31] = 1;
   out_5498212507308944349[32] = 0;
   out_5498212507308944349[33] = 0;
   out_5498212507308944349[34] = 1;
   out_5498212507308944349[35] = 0;
   out_5498212507308944349[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5498212507308944349[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5498212507308944349[38] = 0;
   out_5498212507308944349[39] = -state[7];
   out_5498212507308944349[40] = state[6];
   out_5498212507308944349[41] = 0;
   out_5498212507308944349[42] = state[4];
   out_5498212507308944349[43] = -state[3];
   out_5498212507308944349[44] = 0;
   out_5498212507308944349[45] = 0;
   out_5498212507308944349[46] = 0;
   out_5498212507308944349[47] = 0;
   out_5498212507308944349[48] = 0;
   out_5498212507308944349[49] = 0;
   out_5498212507308944349[50] = 1;
   out_5498212507308944349[51] = 0;
   out_5498212507308944349[52] = 0;
   out_5498212507308944349[53] = 1;
}
void h_13(double *state, double *unused, double *out_8252438701567181479) {
   out_8252438701567181479[0] = state[3];
   out_8252438701567181479[1] = state[4];
   out_8252438701567181479[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6306430816198860171) {
   out_6306430816198860171[0] = 0;
   out_6306430816198860171[1] = 0;
   out_6306430816198860171[2] = 0;
   out_6306430816198860171[3] = 1;
   out_6306430816198860171[4] = 0;
   out_6306430816198860171[5] = 0;
   out_6306430816198860171[6] = 0;
   out_6306430816198860171[7] = 0;
   out_6306430816198860171[8] = 0;
   out_6306430816198860171[9] = 0;
   out_6306430816198860171[10] = 0;
   out_6306430816198860171[11] = 0;
   out_6306430816198860171[12] = 0;
   out_6306430816198860171[13] = 0;
   out_6306430816198860171[14] = 0;
   out_6306430816198860171[15] = 0;
   out_6306430816198860171[16] = 0;
   out_6306430816198860171[17] = 0;
   out_6306430816198860171[18] = 0;
   out_6306430816198860171[19] = 0;
   out_6306430816198860171[20] = 0;
   out_6306430816198860171[21] = 0;
   out_6306430816198860171[22] = 1;
   out_6306430816198860171[23] = 0;
   out_6306430816198860171[24] = 0;
   out_6306430816198860171[25] = 0;
   out_6306430816198860171[26] = 0;
   out_6306430816198860171[27] = 0;
   out_6306430816198860171[28] = 0;
   out_6306430816198860171[29] = 0;
   out_6306430816198860171[30] = 0;
   out_6306430816198860171[31] = 0;
   out_6306430816198860171[32] = 0;
   out_6306430816198860171[33] = 0;
   out_6306430816198860171[34] = 0;
   out_6306430816198860171[35] = 0;
   out_6306430816198860171[36] = 0;
   out_6306430816198860171[37] = 0;
   out_6306430816198860171[38] = 0;
   out_6306430816198860171[39] = 0;
   out_6306430816198860171[40] = 0;
   out_6306430816198860171[41] = 1;
   out_6306430816198860171[42] = 0;
   out_6306430816198860171[43] = 0;
   out_6306430816198860171[44] = 0;
   out_6306430816198860171[45] = 0;
   out_6306430816198860171[46] = 0;
   out_6306430816198860171[47] = 0;
   out_6306430816198860171[48] = 0;
   out_6306430816198860171[49] = 0;
   out_6306430816198860171[50] = 0;
   out_6306430816198860171[51] = 0;
   out_6306430816198860171[52] = 0;
   out_6306430816198860171[53] = 0;
}
void h_14(double *state, double *unused, double *out_7741604625342005453) {
   out_7741604625342005453[0] = state[6];
   out_7741604625342005453[1] = state[7];
   out_7741604625342005453[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5555463785191708443) {
   out_5555463785191708443[0] = 0;
   out_5555463785191708443[1] = 0;
   out_5555463785191708443[2] = 0;
   out_5555463785191708443[3] = 0;
   out_5555463785191708443[4] = 0;
   out_5555463785191708443[5] = 0;
   out_5555463785191708443[6] = 1;
   out_5555463785191708443[7] = 0;
   out_5555463785191708443[8] = 0;
   out_5555463785191708443[9] = 0;
   out_5555463785191708443[10] = 0;
   out_5555463785191708443[11] = 0;
   out_5555463785191708443[12] = 0;
   out_5555463785191708443[13] = 0;
   out_5555463785191708443[14] = 0;
   out_5555463785191708443[15] = 0;
   out_5555463785191708443[16] = 0;
   out_5555463785191708443[17] = 0;
   out_5555463785191708443[18] = 0;
   out_5555463785191708443[19] = 0;
   out_5555463785191708443[20] = 0;
   out_5555463785191708443[21] = 0;
   out_5555463785191708443[22] = 0;
   out_5555463785191708443[23] = 0;
   out_5555463785191708443[24] = 0;
   out_5555463785191708443[25] = 1;
   out_5555463785191708443[26] = 0;
   out_5555463785191708443[27] = 0;
   out_5555463785191708443[28] = 0;
   out_5555463785191708443[29] = 0;
   out_5555463785191708443[30] = 0;
   out_5555463785191708443[31] = 0;
   out_5555463785191708443[32] = 0;
   out_5555463785191708443[33] = 0;
   out_5555463785191708443[34] = 0;
   out_5555463785191708443[35] = 0;
   out_5555463785191708443[36] = 0;
   out_5555463785191708443[37] = 0;
   out_5555463785191708443[38] = 0;
   out_5555463785191708443[39] = 0;
   out_5555463785191708443[40] = 0;
   out_5555463785191708443[41] = 0;
   out_5555463785191708443[42] = 0;
   out_5555463785191708443[43] = 0;
   out_5555463785191708443[44] = 1;
   out_5555463785191708443[45] = 0;
   out_5555463785191708443[46] = 0;
   out_5555463785191708443[47] = 0;
   out_5555463785191708443[48] = 0;
   out_5555463785191708443[49] = 0;
   out_5555463785191708443[50] = 0;
   out_5555463785191708443[51] = 0;
   out_5555463785191708443[52] = 0;
   out_5555463785191708443[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2966425499024310265) {
  err_fun(nom_x, delta_x, out_2966425499024310265);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4677272284422365493) {
  inv_err_fun(nom_x, true_x, out_4677272284422365493);
}
void pose_H_mod_fun(double *state, double *out_6471686313161485195) {
  H_mod_fun(state, out_6471686313161485195);
}
void pose_f_fun(double *state, double dt, double *out_8577303503121138630) {
  f_fun(state,  dt, out_8577303503121138630);
}
void pose_F_fun(double *state, double dt, double *out_5893284074989477049) {
  F_fun(state,  dt, out_5893284074989477049);
}
void pose_h_4(double *state, double *unused, double *out_3588243949097045624) {
  h_4(state, unused, out_3588243949097045624);
}
void pose_H_4(double *state, double *unused, double *out_2472675352896336147) {
  H_4(state, unused, out_2472675352896336147);
}
void pose_h_10(double *state, double *unused, double *out_6829459391907355466) {
  h_10(state, unused, out_6829459391907355466);
}
void pose_H_10(double *state, double *unused, double *out_5498212507308944349) {
  H_10(state, unused, out_5498212507308944349);
}
void pose_h_13(double *state, double *unused, double *out_8252438701567181479) {
  h_13(state, unused, out_8252438701567181479);
}
void pose_H_13(double *state, double *unused, double *out_6306430816198860171) {
  H_13(state, unused, out_6306430816198860171);
}
void pose_h_14(double *state, double *unused, double *out_7741604625342005453) {
  h_14(state, unused, out_7741604625342005453);
}
void pose_H_14(double *state, double *unused, double *out_5555463785191708443) {
  H_14(state, unused, out_5555463785191708443);
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
