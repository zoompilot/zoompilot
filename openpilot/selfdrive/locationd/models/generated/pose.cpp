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
void err_fun(double *nom_x, double *delta_x, double *out_4384279722743490608) {
   out_4384279722743490608[0] = delta_x[0] + nom_x[0];
   out_4384279722743490608[1] = delta_x[1] + nom_x[1];
   out_4384279722743490608[2] = delta_x[2] + nom_x[2];
   out_4384279722743490608[3] = delta_x[3] + nom_x[3];
   out_4384279722743490608[4] = delta_x[4] + nom_x[4];
   out_4384279722743490608[5] = delta_x[5] + nom_x[5];
   out_4384279722743490608[6] = delta_x[6] + nom_x[6];
   out_4384279722743490608[7] = delta_x[7] + nom_x[7];
   out_4384279722743490608[8] = delta_x[8] + nom_x[8];
   out_4384279722743490608[9] = delta_x[9] + nom_x[9];
   out_4384279722743490608[10] = delta_x[10] + nom_x[10];
   out_4384279722743490608[11] = delta_x[11] + nom_x[11];
   out_4384279722743490608[12] = delta_x[12] + nom_x[12];
   out_4384279722743490608[13] = delta_x[13] + nom_x[13];
   out_4384279722743490608[14] = delta_x[14] + nom_x[14];
   out_4384279722743490608[15] = delta_x[15] + nom_x[15];
   out_4384279722743490608[16] = delta_x[16] + nom_x[16];
   out_4384279722743490608[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_107855010463002620) {
   out_107855010463002620[0] = -nom_x[0] + true_x[0];
   out_107855010463002620[1] = -nom_x[1] + true_x[1];
   out_107855010463002620[2] = -nom_x[2] + true_x[2];
   out_107855010463002620[3] = -nom_x[3] + true_x[3];
   out_107855010463002620[4] = -nom_x[4] + true_x[4];
   out_107855010463002620[5] = -nom_x[5] + true_x[5];
   out_107855010463002620[6] = -nom_x[6] + true_x[6];
   out_107855010463002620[7] = -nom_x[7] + true_x[7];
   out_107855010463002620[8] = -nom_x[8] + true_x[8];
   out_107855010463002620[9] = -nom_x[9] + true_x[9];
   out_107855010463002620[10] = -nom_x[10] + true_x[10];
   out_107855010463002620[11] = -nom_x[11] + true_x[11];
   out_107855010463002620[12] = -nom_x[12] + true_x[12];
   out_107855010463002620[13] = -nom_x[13] + true_x[13];
   out_107855010463002620[14] = -nom_x[14] + true_x[14];
   out_107855010463002620[15] = -nom_x[15] + true_x[15];
   out_107855010463002620[16] = -nom_x[16] + true_x[16];
   out_107855010463002620[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_402663571807210546) {
   out_402663571807210546[0] = 1.0;
   out_402663571807210546[1] = 0.0;
   out_402663571807210546[2] = 0.0;
   out_402663571807210546[3] = 0.0;
   out_402663571807210546[4] = 0.0;
   out_402663571807210546[5] = 0.0;
   out_402663571807210546[6] = 0.0;
   out_402663571807210546[7] = 0.0;
   out_402663571807210546[8] = 0.0;
   out_402663571807210546[9] = 0.0;
   out_402663571807210546[10] = 0.0;
   out_402663571807210546[11] = 0.0;
   out_402663571807210546[12] = 0.0;
   out_402663571807210546[13] = 0.0;
   out_402663571807210546[14] = 0.0;
   out_402663571807210546[15] = 0.0;
   out_402663571807210546[16] = 0.0;
   out_402663571807210546[17] = 0.0;
   out_402663571807210546[18] = 0.0;
   out_402663571807210546[19] = 1.0;
   out_402663571807210546[20] = 0.0;
   out_402663571807210546[21] = 0.0;
   out_402663571807210546[22] = 0.0;
   out_402663571807210546[23] = 0.0;
   out_402663571807210546[24] = 0.0;
   out_402663571807210546[25] = 0.0;
   out_402663571807210546[26] = 0.0;
   out_402663571807210546[27] = 0.0;
   out_402663571807210546[28] = 0.0;
   out_402663571807210546[29] = 0.0;
   out_402663571807210546[30] = 0.0;
   out_402663571807210546[31] = 0.0;
   out_402663571807210546[32] = 0.0;
   out_402663571807210546[33] = 0.0;
   out_402663571807210546[34] = 0.0;
   out_402663571807210546[35] = 0.0;
   out_402663571807210546[36] = 0.0;
   out_402663571807210546[37] = 0.0;
   out_402663571807210546[38] = 1.0;
   out_402663571807210546[39] = 0.0;
   out_402663571807210546[40] = 0.0;
   out_402663571807210546[41] = 0.0;
   out_402663571807210546[42] = 0.0;
   out_402663571807210546[43] = 0.0;
   out_402663571807210546[44] = 0.0;
   out_402663571807210546[45] = 0.0;
   out_402663571807210546[46] = 0.0;
   out_402663571807210546[47] = 0.0;
   out_402663571807210546[48] = 0.0;
   out_402663571807210546[49] = 0.0;
   out_402663571807210546[50] = 0.0;
   out_402663571807210546[51] = 0.0;
   out_402663571807210546[52] = 0.0;
   out_402663571807210546[53] = 0.0;
   out_402663571807210546[54] = 0.0;
   out_402663571807210546[55] = 0.0;
   out_402663571807210546[56] = 0.0;
   out_402663571807210546[57] = 1.0;
   out_402663571807210546[58] = 0.0;
   out_402663571807210546[59] = 0.0;
   out_402663571807210546[60] = 0.0;
   out_402663571807210546[61] = 0.0;
   out_402663571807210546[62] = 0.0;
   out_402663571807210546[63] = 0.0;
   out_402663571807210546[64] = 0.0;
   out_402663571807210546[65] = 0.0;
   out_402663571807210546[66] = 0.0;
   out_402663571807210546[67] = 0.0;
   out_402663571807210546[68] = 0.0;
   out_402663571807210546[69] = 0.0;
   out_402663571807210546[70] = 0.0;
   out_402663571807210546[71] = 0.0;
   out_402663571807210546[72] = 0.0;
   out_402663571807210546[73] = 0.0;
   out_402663571807210546[74] = 0.0;
   out_402663571807210546[75] = 0.0;
   out_402663571807210546[76] = 1.0;
   out_402663571807210546[77] = 0.0;
   out_402663571807210546[78] = 0.0;
   out_402663571807210546[79] = 0.0;
   out_402663571807210546[80] = 0.0;
   out_402663571807210546[81] = 0.0;
   out_402663571807210546[82] = 0.0;
   out_402663571807210546[83] = 0.0;
   out_402663571807210546[84] = 0.0;
   out_402663571807210546[85] = 0.0;
   out_402663571807210546[86] = 0.0;
   out_402663571807210546[87] = 0.0;
   out_402663571807210546[88] = 0.0;
   out_402663571807210546[89] = 0.0;
   out_402663571807210546[90] = 0.0;
   out_402663571807210546[91] = 0.0;
   out_402663571807210546[92] = 0.0;
   out_402663571807210546[93] = 0.0;
   out_402663571807210546[94] = 0.0;
   out_402663571807210546[95] = 1.0;
   out_402663571807210546[96] = 0.0;
   out_402663571807210546[97] = 0.0;
   out_402663571807210546[98] = 0.0;
   out_402663571807210546[99] = 0.0;
   out_402663571807210546[100] = 0.0;
   out_402663571807210546[101] = 0.0;
   out_402663571807210546[102] = 0.0;
   out_402663571807210546[103] = 0.0;
   out_402663571807210546[104] = 0.0;
   out_402663571807210546[105] = 0.0;
   out_402663571807210546[106] = 0.0;
   out_402663571807210546[107] = 0.0;
   out_402663571807210546[108] = 0.0;
   out_402663571807210546[109] = 0.0;
   out_402663571807210546[110] = 0.0;
   out_402663571807210546[111] = 0.0;
   out_402663571807210546[112] = 0.0;
   out_402663571807210546[113] = 0.0;
   out_402663571807210546[114] = 1.0;
   out_402663571807210546[115] = 0.0;
   out_402663571807210546[116] = 0.0;
   out_402663571807210546[117] = 0.0;
   out_402663571807210546[118] = 0.0;
   out_402663571807210546[119] = 0.0;
   out_402663571807210546[120] = 0.0;
   out_402663571807210546[121] = 0.0;
   out_402663571807210546[122] = 0.0;
   out_402663571807210546[123] = 0.0;
   out_402663571807210546[124] = 0.0;
   out_402663571807210546[125] = 0.0;
   out_402663571807210546[126] = 0.0;
   out_402663571807210546[127] = 0.0;
   out_402663571807210546[128] = 0.0;
   out_402663571807210546[129] = 0.0;
   out_402663571807210546[130] = 0.0;
   out_402663571807210546[131] = 0.0;
   out_402663571807210546[132] = 0.0;
   out_402663571807210546[133] = 1.0;
   out_402663571807210546[134] = 0.0;
   out_402663571807210546[135] = 0.0;
   out_402663571807210546[136] = 0.0;
   out_402663571807210546[137] = 0.0;
   out_402663571807210546[138] = 0.0;
   out_402663571807210546[139] = 0.0;
   out_402663571807210546[140] = 0.0;
   out_402663571807210546[141] = 0.0;
   out_402663571807210546[142] = 0.0;
   out_402663571807210546[143] = 0.0;
   out_402663571807210546[144] = 0.0;
   out_402663571807210546[145] = 0.0;
   out_402663571807210546[146] = 0.0;
   out_402663571807210546[147] = 0.0;
   out_402663571807210546[148] = 0.0;
   out_402663571807210546[149] = 0.0;
   out_402663571807210546[150] = 0.0;
   out_402663571807210546[151] = 0.0;
   out_402663571807210546[152] = 1.0;
   out_402663571807210546[153] = 0.0;
   out_402663571807210546[154] = 0.0;
   out_402663571807210546[155] = 0.0;
   out_402663571807210546[156] = 0.0;
   out_402663571807210546[157] = 0.0;
   out_402663571807210546[158] = 0.0;
   out_402663571807210546[159] = 0.0;
   out_402663571807210546[160] = 0.0;
   out_402663571807210546[161] = 0.0;
   out_402663571807210546[162] = 0.0;
   out_402663571807210546[163] = 0.0;
   out_402663571807210546[164] = 0.0;
   out_402663571807210546[165] = 0.0;
   out_402663571807210546[166] = 0.0;
   out_402663571807210546[167] = 0.0;
   out_402663571807210546[168] = 0.0;
   out_402663571807210546[169] = 0.0;
   out_402663571807210546[170] = 0.0;
   out_402663571807210546[171] = 1.0;
   out_402663571807210546[172] = 0.0;
   out_402663571807210546[173] = 0.0;
   out_402663571807210546[174] = 0.0;
   out_402663571807210546[175] = 0.0;
   out_402663571807210546[176] = 0.0;
   out_402663571807210546[177] = 0.0;
   out_402663571807210546[178] = 0.0;
   out_402663571807210546[179] = 0.0;
   out_402663571807210546[180] = 0.0;
   out_402663571807210546[181] = 0.0;
   out_402663571807210546[182] = 0.0;
   out_402663571807210546[183] = 0.0;
   out_402663571807210546[184] = 0.0;
   out_402663571807210546[185] = 0.0;
   out_402663571807210546[186] = 0.0;
   out_402663571807210546[187] = 0.0;
   out_402663571807210546[188] = 0.0;
   out_402663571807210546[189] = 0.0;
   out_402663571807210546[190] = 1.0;
   out_402663571807210546[191] = 0.0;
   out_402663571807210546[192] = 0.0;
   out_402663571807210546[193] = 0.0;
   out_402663571807210546[194] = 0.0;
   out_402663571807210546[195] = 0.0;
   out_402663571807210546[196] = 0.0;
   out_402663571807210546[197] = 0.0;
   out_402663571807210546[198] = 0.0;
   out_402663571807210546[199] = 0.0;
   out_402663571807210546[200] = 0.0;
   out_402663571807210546[201] = 0.0;
   out_402663571807210546[202] = 0.0;
   out_402663571807210546[203] = 0.0;
   out_402663571807210546[204] = 0.0;
   out_402663571807210546[205] = 0.0;
   out_402663571807210546[206] = 0.0;
   out_402663571807210546[207] = 0.0;
   out_402663571807210546[208] = 0.0;
   out_402663571807210546[209] = 1.0;
   out_402663571807210546[210] = 0.0;
   out_402663571807210546[211] = 0.0;
   out_402663571807210546[212] = 0.0;
   out_402663571807210546[213] = 0.0;
   out_402663571807210546[214] = 0.0;
   out_402663571807210546[215] = 0.0;
   out_402663571807210546[216] = 0.0;
   out_402663571807210546[217] = 0.0;
   out_402663571807210546[218] = 0.0;
   out_402663571807210546[219] = 0.0;
   out_402663571807210546[220] = 0.0;
   out_402663571807210546[221] = 0.0;
   out_402663571807210546[222] = 0.0;
   out_402663571807210546[223] = 0.0;
   out_402663571807210546[224] = 0.0;
   out_402663571807210546[225] = 0.0;
   out_402663571807210546[226] = 0.0;
   out_402663571807210546[227] = 0.0;
   out_402663571807210546[228] = 1.0;
   out_402663571807210546[229] = 0.0;
   out_402663571807210546[230] = 0.0;
   out_402663571807210546[231] = 0.0;
   out_402663571807210546[232] = 0.0;
   out_402663571807210546[233] = 0.0;
   out_402663571807210546[234] = 0.0;
   out_402663571807210546[235] = 0.0;
   out_402663571807210546[236] = 0.0;
   out_402663571807210546[237] = 0.0;
   out_402663571807210546[238] = 0.0;
   out_402663571807210546[239] = 0.0;
   out_402663571807210546[240] = 0.0;
   out_402663571807210546[241] = 0.0;
   out_402663571807210546[242] = 0.0;
   out_402663571807210546[243] = 0.0;
   out_402663571807210546[244] = 0.0;
   out_402663571807210546[245] = 0.0;
   out_402663571807210546[246] = 0.0;
   out_402663571807210546[247] = 1.0;
   out_402663571807210546[248] = 0.0;
   out_402663571807210546[249] = 0.0;
   out_402663571807210546[250] = 0.0;
   out_402663571807210546[251] = 0.0;
   out_402663571807210546[252] = 0.0;
   out_402663571807210546[253] = 0.0;
   out_402663571807210546[254] = 0.0;
   out_402663571807210546[255] = 0.0;
   out_402663571807210546[256] = 0.0;
   out_402663571807210546[257] = 0.0;
   out_402663571807210546[258] = 0.0;
   out_402663571807210546[259] = 0.0;
   out_402663571807210546[260] = 0.0;
   out_402663571807210546[261] = 0.0;
   out_402663571807210546[262] = 0.0;
   out_402663571807210546[263] = 0.0;
   out_402663571807210546[264] = 0.0;
   out_402663571807210546[265] = 0.0;
   out_402663571807210546[266] = 1.0;
   out_402663571807210546[267] = 0.0;
   out_402663571807210546[268] = 0.0;
   out_402663571807210546[269] = 0.0;
   out_402663571807210546[270] = 0.0;
   out_402663571807210546[271] = 0.0;
   out_402663571807210546[272] = 0.0;
   out_402663571807210546[273] = 0.0;
   out_402663571807210546[274] = 0.0;
   out_402663571807210546[275] = 0.0;
   out_402663571807210546[276] = 0.0;
   out_402663571807210546[277] = 0.0;
   out_402663571807210546[278] = 0.0;
   out_402663571807210546[279] = 0.0;
   out_402663571807210546[280] = 0.0;
   out_402663571807210546[281] = 0.0;
   out_402663571807210546[282] = 0.0;
   out_402663571807210546[283] = 0.0;
   out_402663571807210546[284] = 0.0;
   out_402663571807210546[285] = 1.0;
   out_402663571807210546[286] = 0.0;
   out_402663571807210546[287] = 0.0;
   out_402663571807210546[288] = 0.0;
   out_402663571807210546[289] = 0.0;
   out_402663571807210546[290] = 0.0;
   out_402663571807210546[291] = 0.0;
   out_402663571807210546[292] = 0.0;
   out_402663571807210546[293] = 0.0;
   out_402663571807210546[294] = 0.0;
   out_402663571807210546[295] = 0.0;
   out_402663571807210546[296] = 0.0;
   out_402663571807210546[297] = 0.0;
   out_402663571807210546[298] = 0.0;
   out_402663571807210546[299] = 0.0;
   out_402663571807210546[300] = 0.0;
   out_402663571807210546[301] = 0.0;
   out_402663571807210546[302] = 0.0;
   out_402663571807210546[303] = 0.0;
   out_402663571807210546[304] = 1.0;
   out_402663571807210546[305] = 0.0;
   out_402663571807210546[306] = 0.0;
   out_402663571807210546[307] = 0.0;
   out_402663571807210546[308] = 0.0;
   out_402663571807210546[309] = 0.0;
   out_402663571807210546[310] = 0.0;
   out_402663571807210546[311] = 0.0;
   out_402663571807210546[312] = 0.0;
   out_402663571807210546[313] = 0.0;
   out_402663571807210546[314] = 0.0;
   out_402663571807210546[315] = 0.0;
   out_402663571807210546[316] = 0.0;
   out_402663571807210546[317] = 0.0;
   out_402663571807210546[318] = 0.0;
   out_402663571807210546[319] = 0.0;
   out_402663571807210546[320] = 0.0;
   out_402663571807210546[321] = 0.0;
   out_402663571807210546[322] = 0.0;
   out_402663571807210546[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2303112348537759179) {
   out_2303112348537759179[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2303112348537759179[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2303112348537759179[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2303112348537759179[3] = dt*state[12] + state[3];
   out_2303112348537759179[4] = dt*state[13] + state[4];
   out_2303112348537759179[5] = dt*state[14] + state[5];
   out_2303112348537759179[6] = state[6];
   out_2303112348537759179[7] = state[7];
   out_2303112348537759179[8] = state[8];
   out_2303112348537759179[9] = state[9];
   out_2303112348537759179[10] = state[10];
   out_2303112348537759179[11] = state[11];
   out_2303112348537759179[12] = state[12];
   out_2303112348537759179[13] = state[13];
   out_2303112348537759179[14] = state[14];
   out_2303112348537759179[15] = state[15];
   out_2303112348537759179[16] = state[16];
   out_2303112348537759179[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3747527490433827778) {
   out_3747527490433827778[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3747527490433827778[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3747527490433827778[2] = 0;
   out_3747527490433827778[3] = 0;
   out_3747527490433827778[4] = 0;
   out_3747527490433827778[5] = 0;
   out_3747527490433827778[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3747527490433827778[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3747527490433827778[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3747527490433827778[9] = 0;
   out_3747527490433827778[10] = 0;
   out_3747527490433827778[11] = 0;
   out_3747527490433827778[12] = 0;
   out_3747527490433827778[13] = 0;
   out_3747527490433827778[14] = 0;
   out_3747527490433827778[15] = 0;
   out_3747527490433827778[16] = 0;
   out_3747527490433827778[17] = 0;
   out_3747527490433827778[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3747527490433827778[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3747527490433827778[20] = 0;
   out_3747527490433827778[21] = 0;
   out_3747527490433827778[22] = 0;
   out_3747527490433827778[23] = 0;
   out_3747527490433827778[24] = 0;
   out_3747527490433827778[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3747527490433827778[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3747527490433827778[27] = 0;
   out_3747527490433827778[28] = 0;
   out_3747527490433827778[29] = 0;
   out_3747527490433827778[30] = 0;
   out_3747527490433827778[31] = 0;
   out_3747527490433827778[32] = 0;
   out_3747527490433827778[33] = 0;
   out_3747527490433827778[34] = 0;
   out_3747527490433827778[35] = 0;
   out_3747527490433827778[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3747527490433827778[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3747527490433827778[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3747527490433827778[39] = 0;
   out_3747527490433827778[40] = 0;
   out_3747527490433827778[41] = 0;
   out_3747527490433827778[42] = 0;
   out_3747527490433827778[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3747527490433827778[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3747527490433827778[45] = 0;
   out_3747527490433827778[46] = 0;
   out_3747527490433827778[47] = 0;
   out_3747527490433827778[48] = 0;
   out_3747527490433827778[49] = 0;
   out_3747527490433827778[50] = 0;
   out_3747527490433827778[51] = 0;
   out_3747527490433827778[52] = 0;
   out_3747527490433827778[53] = 0;
   out_3747527490433827778[54] = 0;
   out_3747527490433827778[55] = 0;
   out_3747527490433827778[56] = 0;
   out_3747527490433827778[57] = 1;
   out_3747527490433827778[58] = 0;
   out_3747527490433827778[59] = 0;
   out_3747527490433827778[60] = 0;
   out_3747527490433827778[61] = 0;
   out_3747527490433827778[62] = 0;
   out_3747527490433827778[63] = 0;
   out_3747527490433827778[64] = 0;
   out_3747527490433827778[65] = 0;
   out_3747527490433827778[66] = dt;
   out_3747527490433827778[67] = 0;
   out_3747527490433827778[68] = 0;
   out_3747527490433827778[69] = 0;
   out_3747527490433827778[70] = 0;
   out_3747527490433827778[71] = 0;
   out_3747527490433827778[72] = 0;
   out_3747527490433827778[73] = 0;
   out_3747527490433827778[74] = 0;
   out_3747527490433827778[75] = 0;
   out_3747527490433827778[76] = 1;
   out_3747527490433827778[77] = 0;
   out_3747527490433827778[78] = 0;
   out_3747527490433827778[79] = 0;
   out_3747527490433827778[80] = 0;
   out_3747527490433827778[81] = 0;
   out_3747527490433827778[82] = 0;
   out_3747527490433827778[83] = 0;
   out_3747527490433827778[84] = 0;
   out_3747527490433827778[85] = dt;
   out_3747527490433827778[86] = 0;
   out_3747527490433827778[87] = 0;
   out_3747527490433827778[88] = 0;
   out_3747527490433827778[89] = 0;
   out_3747527490433827778[90] = 0;
   out_3747527490433827778[91] = 0;
   out_3747527490433827778[92] = 0;
   out_3747527490433827778[93] = 0;
   out_3747527490433827778[94] = 0;
   out_3747527490433827778[95] = 1;
   out_3747527490433827778[96] = 0;
   out_3747527490433827778[97] = 0;
   out_3747527490433827778[98] = 0;
   out_3747527490433827778[99] = 0;
   out_3747527490433827778[100] = 0;
   out_3747527490433827778[101] = 0;
   out_3747527490433827778[102] = 0;
   out_3747527490433827778[103] = 0;
   out_3747527490433827778[104] = dt;
   out_3747527490433827778[105] = 0;
   out_3747527490433827778[106] = 0;
   out_3747527490433827778[107] = 0;
   out_3747527490433827778[108] = 0;
   out_3747527490433827778[109] = 0;
   out_3747527490433827778[110] = 0;
   out_3747527490433827778[111] = 0;
   out_3747527490433827778[112] = 0;
   out_3747527490433827778[113] = 0;
   out_3747527490433827778[114] = 1;
   out_3747527490433827778[115] = 0;
   out_3747527490433827778[116] = 0;
   out_3747527490433827778[117] = 0;
   out_3747527490433827778[118] = 0;
   out_3747527490433827778[119] = 0;
   out_3747527490433827778[120] = 0;
   out_3747527490433827778[121] = 0;
   out_3747527490433827778[122] = 0;
   out_3747527490433827778[123] = 0;
   out_3747527490433827778[124] = 0;
   out_3747527490433827778[125] = 0;
   out_3747527490433827778[126] = 0;
   out_3747527490433827778[127] = 0;
   out_3747527490433827778[128] = 0;
   out_3747527490433827778[129] = 0;
   out_3747527490433827778[130] = 0;
   out_3747527490433827778[131] = 0;
   out_3747527490433827778[132] = 0;
   out_3747527490433827778[133] = 1;
   out_3747527490433827778[134] = 0;
   out_3747527490433827778[135] = 0;
   out_3747527490433827778[136] = 0;
   out_3747527490433827778[137] = 0;
   out_3747527490433827778[138] = 0;
   out_3747527490433827778[139] = 0;
   out_3747527490433827778[140] = 0;
   out_3747527490433827778[141] = 0;
   out_3747527490433827778[142] = 0;
   out_3747527490433827778[143] = 0;
   out_3747527490433827778[144] = 0;
   out_3747527490433827778[145] = 0;
   out_3747527490433827778[146] = 0;
   out_3747527490433827778[147] = 0;
   out_3747527490433827778[148] = 0;
   out_3747527490433827778[149] = 0;
   out_3747527490433827778[150] = 0;
   out_3747527490433827778[151] = 0;
   out_3747527490433827778[152] = 1;
   out_3747527490433827778[153] = 0;
   out_3747527490433827778[154] = 0;
   out_3747527490433827778[155] = 0;
   out_3747527490433827778[156] = 0;
   out_3747527490433827778[157] = 0;
   out_3747527490433827778[158] = 0;
   out_3747527490433827778[159] = 0;
   out_3747527490433827778[160] = 0;
   out_3747527490433827778[161] = 0;
   out_3747527490433827778[162] = 0;
   out_3747527490433827778[163] = 0;
   out_3747527490433827778[164] = 0;
   out_3747527490433827778[165] = 0;
   out_3747527490433827778[166] = 0;
   out_3747527490433827778[167] = 0;
   out_3747527490433827778[168] = 0;
   out_3747527490433827778[169] = 0;
   out_3747527490433827778[170] = 0;
   out_3747527490433827778[171] = 1;
   out_3747527490433827778[172] = 0;
   out_3747527490433827778[173] = 0;
   out_3747527490433827778[174] = 0;
   out_3747527490433827778[175] = 0;
   out_3747527490433827778[176] = 0;
   out_3747527490433827778[177] = 0;
   out_3747527490433827778[178] = 0;
   out_3747527490433827778[179] = 0;
   out_3747527490433827778[180] = 0;
   out_3747527490433827778[181] = 0;
   out_3747527490433827778[182] = 0;
   out_3747527490433827778[183] = 0;
   out_3747527490433827778[184] = 0;
   out_3747527490433827778[185] = 0;
   out_3747527490433827778[186] = 0;
   out_3747527490433827778[187] = 0;
   out_3747527490433827778[188] = 0;
   out_3747527490433827778[189] = 0;
   out_3747527490433827778[190] = 1;
   out_3747527490433827778[191] = 0;
   out_3747527490433827778[192] = 0;
   out_3747527490433827778[193] = 0;
   out_3747527490433827778[194] = 0;
   out_3747527490433827778[195] = 0;
   out_3747527490433827778[196] = 0;
   out_3747527490433827778[197] = 0;
   out_3747527490433827778[198] = 0;
   out_3747527490433827778[199] = 0;
   out_3747527490433827778[200] = 0;
   out_3747527490433827778[201] = 0;
   out_3747527490433827778[202] = 0;
   out_3747527490433827778[203] = 0;
   out_3747527490433827778[204] = 0;
   out_3747527490433827778[205] = 0;
   out_3747527490433827778[206] = 0;
   out_3747527490433827778[207] = 0;
   out_3747527490433827778[208] = 0;
   out_3747527490433827778[209] = 1;
   out_3747527490433827778[210] = 0;
   out_3747527490433827778[211] = 0;
   out_3747527490433827778[212] = 0;
   out_3747527490433827778[213] = 0;
   out_3747527490433827778[214] = 0;
   out_3747527490433827778[215] = 0;
   out_3747527490433827778[216] = 0;
   out_3747527490433827778[217] = 0;
   out_3747527490433827778[218] = 0;
   out_3747527490433827778[219] = 0;
   out_3747527490433827778[220] = 0;
   out_3747527490433827778[221] = 0;
   out_3747527490433827778[222] = 0;
   out_3747527490433827778[223] = 0;
   out_3747527490433827778[224] = 0;
   out_3747527490433827778[225] = 0;
   out_3747527490433827778[226] = 0;
   out_3747527490433827778[227] = 0;
   out_3747527490433827778[228] = 1;
   out_3747527490433827778[229] = 0;
   out_3747527490433827778[230] = 0;
   out_3747527490433827778[231] = 0;
   out_3747527490433827778[232] = 0;
   out_3747527490433827778[233] = 0;
   out_3747527490433827778[234] = 0;
   out_3747527490433827778[235] = 0;
   out_3747527490433827778[236] = 0;
   out_3747527490433827778[237] = 0;
   out_3747527490433827778[238] = 0;
   out_3747527490433827778[239] = 0;
   out_3747527490433827778[240] = 0;
   out_3747527490433827778[241] = 0;
   out_3747527490433827778[242] = 0;
   out_3747527490433827778[243] = 0;
   out_3747527490433827778[244] = 0;
   out_3747527490433827778[245] = 0;
   out_3747527490433827778[246] = 0;
   out_3747527490433827778[247] = 1;
   out_3747527490433827778[248] = 0;
   out_3747527490433827778[249] = 0;
   out_3747527490433827778[250] = 0;
   out_3747527490433827778[251] = 0;
   out_3747527490433827778[252] = 0;
   out_3747527490433827778[253] = 0;
   out_3747527490433827778[254] = 0;
   out_3747527490433827778[255] = 0;
   out_3747527490433827778[256] = 0;
   out_3747527490433827778[257] = 0;
   out_3747527490433827778[258] = 0;
   out_3747527490433827778[259] = 0;
   out_3747527490433827778[260] = 0;
   out_3747527490433827778[261] = 0;
   out_3747527490433827778[262] = 0;
   out_3747527490433827778[263] = 0;
   out_3747527490433827778[264] = 0;
   out_3747527490433827778[265] = 0;
   out_3747527490433827778[266] = 1;
   out_3747527490433827778[267] = 0;
   out_3747527490433827778[268] = 0;
   out_3747527490433827778[269] = 0;
   out_3747527490433827778[270] = 0;
   out_3747527490433827778[271] = 0;
   out_3747527490433827778[272] = 0;
   out_3747527490433827778[273] = 0;
   out_3747527490433827778[274] = 0;
   out_3747527490433827778[275] = 0;
   out_3747527490433827778[276] = 0;
   out_3747527490433827778[277] = 0;
   out_3747527490433827778[278] = 0;
   out_3747527490433827778[279] = 0;
   out_3747527490433827778[280] = 0;
   out_3747527490433827778[281] = 0;
   out_3747527490433827778[282] = 0;
   out_3747527490433827778[283] = 0;
   out_3747527490433827778[284] = 0;
   out_3747527490433827778[285] = 1;
   out_3747527490433827778[286] = 0;
   out_3747527490433827778[287] = 0;
   out_3747527490433827778[288] = 0;
   out_3747527490433827778[289] = 0;
   out_3747527490433827778[290] = 0;
   out_3747527490433827778[291] = 0;
   out_3747527490433827778[292] = 0;
   out_3747527490433827778[293] = 0;
   out_3747527490433827778[294] = 0;
   out_3747527490433827778[295] = 0;
   out_3747527490433827778[296] = 0;
   out_3747527490433827778[297] = 0;
   out_3747527490433827778[298] = 0;
   out_3747527490433827778[299] = 0;
   out_3747527490433827778[300] = 0;
   out_3747527490433827778[301] = 0;
   out_3747527490433827778[302] = 0;
   out_3747527490433827778[303] = 0;
   out_3747527490433827778[304] = 1;
   out_3747527490433827778[305] = 0;
   out_3747527490433827778[306] = 0;
   out_3747527490433827778[307] = 0;
   out_3747527490433827778[308] = 0;
   out_3747527490433827778[309] = 0;
   out_3747527490433827778[310] = 0;
   out_3747527490433827778[311] = 0;
   out_3747527490433827778[312] = 0;
   out_3747527490433827778[313] = 0;
   out_3747527490433827778[314] = 0;
   out_3747527490433827778[315] = 0;
   out_3747527490433827778[316] = 0;
   out_3747527490433827778[317] = 0;
   out_3747527490433827778[318] = 0;
   out_3747527490433827778[319] = 0;
   out_3747527490433827778[320] = 0;
   out_3747527490433827778[321] = 0;
   out_3747527490433827778[322] = 0;
   out_3747527490433827778[323] = 1;
}
void h_4(double *state, double *unused, double *out_8983849997392014675) {
   out_8983849997392014675[0] = state[6] + state[9];
   out_8983849997392014675[1] = state[7] + state[10];
   out_8983849997392014675[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2127501724899142132) {
   out_2127501724899142132[0] = 0;
   out_2127501724899142132[1] = 0;
   out_2127501724899142132[2] = 0;
   out_2127501724899142132[3] = 0;
   out_2127501724899142132[4] = 0;
   out_2127501724899142132[5] = 0;
   out_2127501724899142132[6] = 1;
   out_2127501724899142132[7] = 0;
   out_2127501724899142132[8] = 0;
   out_2127501724899142132[9] = 1;
   out_2127501724899142132[10] = 0;
   out_2127501724899142132[11] = 0;
   out_2127501724899142132[12] = 0;
   out_2127501724899142132[13] = 0;
   out_2127501724899142132[14] = 0;
   out_2127501724899142132[15] = 0;
   out_2127501724899142132[16] = 0;
   out_2127501724899142132[17] = 0;
   out_2127501724899142132[18] = 0;
   out_2127501724899142132[19] = 0;
   out_2127501724899142132[20] = 0;
   out_2127501724899142132[21] = 0;
   out_2127501724899142132[22] = 0;
   out_2127501724899142132[23] = 0;
   out_2127501724899142132[24] = 0;
   out_2127501724899142132[25] = 1;
   out_2127501724899142132[26] = 0;
   out_2127501724899142132[27] = 0;
   out_2127501724899142132[28] = 1;
   out_2127501724899142132[29] = 0;
   out_2127501724899142132[30] = 0;
   out_2127501724899142132[31] = 0;
   out_2127501724899142132[32] = 0;
   out_2127501724899142132[33] = 0;
   out_2127501724899142132[34] = 0;
   out_2127501724899142132[35] = 0;
   out_2127501724899142132[36] = 0;
   out_2127501724899142132[37] = 0;
   out_2127501724899142132[38] = 0;
   out_2127501724899142132[39] = 0;
   out_2127501724899142132[40] = 0;
   out_2127501724899142132[41] = 0;
   out_2127501724899142132[42] = 0;
   out_2127501724899142132[43] = 0;
   out_2127501724899142132[44] = 1;
   out_2127501724899142132[45] = 0;
   out_2127501724899142132[46] = 0;
   out_2127501724899142132[47] = 1;
   out_2127501724899142132[48] = 0;
   out_2127501724899142132[49] = 0;
   out_2127501724899142132[50] = 0;
   out_2127501724899142132[51] = 0;
   out_2127501724899142132[52] = 0;
   out_2127501724899142132[53] = 0;
}
void h_10(double *state, double *unused, double *out_7432581086548000889) {
   out_7432581086548000889[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7432581086548000889[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7432581086548000889[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_9051257725730722421) {
   out_9051257725730722421[0] = 0;
   out_9051257725730722421[1] = 9.8100000000000005*cos(state[1]);
   out_9051257725730722421[2] = 0;
   out_9051257725730722421[3] = 0;
   out_9051257725730722421[4] = -state[8];
   out_9051257725730722421[5] = state[7];
   out_9051257725730722421[6] = 0;
   out_9051257725730722421[7] = state[5];
   out_9051257725730722421[8] = -state[4];
   out_9051257725730722421[9] = 0;
   out_9051257725730722421[10] = 0;
   out_9051257725730722421[11] = 0;
   out_9051257725730722421[12] = 1;
   out_9051257725730722421[13] = 0;
   out_9051257725730722421[14] = 0;
   out_9051257725730722421[15] = 1;
   out_9051257725730722421[16] = 0;
   out_9051257725730722421[17] = 0;
   out_9051257725730722421[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_9051257725730722421[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_9051257725730722421[20] = 0;
   out_9051257725730722421[21] = state[8];
   out_9051257725730722421[22] = 0;
   out_9051257725730722421[23] = -state[6];
   out_9051257725730722421[24] = -state[5];
   out_9051257725730722421[25] = 0;
   out_9051257725730722421[26] = state[3];
   out_9051257725730722421[27] = 0;
   out_9051257725730722421[28] = 0;
   out_9051257725730722421[29] = 0;
   out_9051257725730722421[30] = 0;
   out_9051257725730722421[31] = 1;
   out_9051257725730722421[32] = 0;
   out_9051257725730722421[33] = 0;
   out_9051257725730722421[34] = 1;
   out_9051257725730722421[35] = 0;
   out_9051257725730722421[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_9051257725730722421[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_9051257725730722421[38] = 0;
   out_9051257725730722421[39] = -state[7];
   out_9051257725730722421[40] = state[6];
   out_9051257725730722421[41] = 0;
   out_9051257725730722421[42] = state[4];
   out_9051257725730722421[43] = -state[3];
   out_9051257725730722421[44] = 0;
   out_9051257725730722421[45] = 0;
   out_9051257725730722421[46] = 0;
   out_9051257725730722421[47] = 0;
   out_9051257725730722421[48] = 0;
   out_9051257725730722421[49] = 0;
   out_9051257725730722421[50] = 1;
   out_9051257725730722421[51] = 0;
   out_9051257725730722421[52] = 0;
   out_9051257725730722421[53] = 1;
}
void h_13(double *state, double *unused, double *out_537443524052514409) {
   out_537443524052514409[0] = state[3];
   out_537443524052514409[1] = state[4];
   out_537443524052514409[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8708611140493708555) {
   out_8708611140493708555[0] = 0;
   out_8708611140493708555[1] = 0;
   out_8708611140493708555[2] = 0;
   out_8708611140493708555[3] = 1;
   out_8708611140493708555[4] = 0;
   out_8708611140493708555[5] = 0;
   out_8708611140493708555[6] = 0;
   out_8708611140493708555[7] = 0;
   out_8708611140493708555[8] = 0;
   out_8708611140493708555[9] = 0;
   out_8708611140493708555[10] = 0;
   out_8708611140493708555[11] = 0;
   out_8708611140493708555[12] = 0;
   out_8708611140493708555[13] = 0;
   out_8708611140493708555[14] = 0;
   out_8708611140493708555[15] = 0;
   out_8708611140493708555[16] = 0;
   out_8708611140493708555[17] = 0;
   out_8708611140493708555[18] = 0;
   out_8708611140493708555[19] = 0;
   out_8708611140493708555[20] = 0;
   out_8708611140493708555[21] = 0;
   out_8708611140493708555[22] = 1;
   out_8708611140493708555[23] = 0;
   out_8708611140493708555[24] = 0;
   out_8708611140493708555[25] = 0;
   out_8708611140493708555[26] = 0;
   out_8708611140493708555[27] = 0;
   out_8708611140493708555[28] = 0;
   out_8708611140493708555[29] = 0;
   out_8708611140493708555[30] = 0;
   out_8708611140493708555[31] = 0;
   out_8708611140493708555[32] = 0;
   out_8708611140493708555[33] = 0;
   out_8708611140493708555[34] = 0;
   out_8708611140493708555[35] = 0;
   out_8708611140493708555[36] = 0;
   out_8708611140493708555[37] = 0;
   out_8708611140493708555[38] = 0;
   out_8708611140493708555[39] = 0;
   out_8708611140493708555[40] = 0;
   out_8708611140493708555[41] = 1;
   out_8708611140493708555[42] = 0;
   out_8708611140493708555[43] = 0;
   out_8708611140493708555[44] = 0;
   out_8708611140493708555[45] = 0;
   out_8708611140493708555[46] = 0;
   out_8708611140493708555[47] = 0;
   out_8708611140493708555[48] = 0;
   out_8708611140493708555[49] = 0;
   out_8708611140493708555[50] = 0;
   out_8708611140493708555[51] = 0;
   out_8708611140493708555[52] = 0;
   out_8708611140493708555[53] = 0;
}
void h_14(double *state, double *unused, double *out_7069210550597015765) {
   out_7069210550597015765[0] = state[6];
   out_7069210550597015765[1] = state[7];
   out_7069210550597015765[2] = state[8];
}
void H_14(double *state, double *unused, double *out_955286707396230164) {
   out_955286707396230164[0] = 0;
   out_955286707396230164[1] = 0;
   out_955286707396230164[2] = 0;
   out_955286707396230164[3] = 0;
   out_955286707396230164[4] = 0;
   out_955286707396230164[5] = 0;
   out_955286707396230164[6] = 1;
   out_955286707396230164[7] = 0;
   out_955286707396230164[8] = 0;
   out_955286707396230164[9] = 0;
   out_955286707396230164[10] = 0;
   out_955286707396230164[11] = 0;
   out_955286707396230164[12] = 0;
   out_955286707396230164[13] = 0;
   out_955286707396230164[14] = 0;
   out_955286707396230164[15] = 0;
   out_955286707396230164[16] = 0;
   out_955286707396230164[17] = 0;
   out_955286707396230164[18] = 0;
   out_955286707396230164[19] = 0;
   out_955286707396230164[20] = 0;
   out_955286707396230164[21] = 0;
   out_955286707396230164[22] = 0;
   out_955286707396230164[23] = 0;
   out_955286707396230164[24] = 0;
   out_955286707396230164[25] = 1;
   out_955286707396230164[26] = 0;
   out_955286707396230164[27] = 0;
   out_955286707396230164[28] = 0;
   out_955286707396230164[29] = 0;
   out_955286707396230164[30] = 0;
   out_955286707396230164[31] = 0;
   out_955286707396230164[32] = 0;
   out_955286707396230164[33] = 0;
   out_955286707396230164[34] = 0;
   out_955286707396230164[35] = 0;
   out_955286707396230164[36] = 0;
   out_955286707396230164[37] = 0;
   out_955286707396230164[38] = 0;
   out_955286707396230164[39] = 0;
   out_955286707396230164[40] = 0;
   out_955286707396230164[41] = 0;
   out_955286707396230164[42] = 0;
   out_955286707396230164[43] = 0;
   out_955286707396230164[44] = 1;
   out_955286707396230164[45] = 0;
   out_955286707396230164[46] = 0;
   out_955286707396230164[47] = 0;
   out_955286707396230164[48] = 0;
   out_955286707396230164[49] = 0;
   out_955286707396230164[50] = 0;
   out_955286707396230164[51] = 0;
   out_955286707396230164[52] = 0;
   out_955286707396230164[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4384279722743490608) {
  err_fun(nom_x, delta_x, out_4384279722743490608);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_107855010463002620) {
  inv_err_fun(nom_x, true_x, out_107855010463002620);
}
void pose_H_mod_fun(double *state, double *out_402663571807210546) {
  H_mod_fun(state, out_402663571807210546);
}
void pose_f_fun(double *state, double dt, double *out_2303112348537759179) {
  f_fun(state,  dt, out_2303112348537759179);
}
void pose_F_fun(double *state, double dt, double *out_3747527490433827778) {
  F_fun(state,  dt, out_3747527490433827778);
}
void pose_h_4(double *state, double *unused, double *out_8983849997392014675) {
  h_4(state, unused, out_8983849997392014675);
}
void pose_H_4(double *state, double *unused, double *out_2127501724899142132) {
  H_4(state, unused, out_2127501724899142132);
}
void pose_h_10(double *state, double *unused, double *out_7432581086548000889) {
  h_10(state, unused, out_7432581086548000889);
}
void pose_H_10(double *state, double *unused, double *out_9051257725730722421) {
  H_10(state, unused, out_9051257725730722421);
}
void pose_h_13(double *state, double *unused, double *out_537443524052514409) {
  h_13(state, unused, out_537443524052514409);
}
void pose_H_13(double *state, double *unused, double *out_8708611140493708555) {
  H_13(state, unused, out_8708611140493708555);
}
void pose_h_14(double *state, double *unused, double *out_7069210550597015765) {
  h_14(state, unused, out_7069210550597015765);
}
void pose_H_14(double *state, double *unused, double *out_955286707396230164) {
  H_14(state, unused, out_955286707396230164);
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
