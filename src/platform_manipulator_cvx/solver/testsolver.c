/* Produced by CVXGEN, 2022-04-22 18:20:13 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.ref[0] = 0.20319161029830202;
  params.ref[1] = 0.8325912904724193;
  params.ref[2] = -0.8363810443482227;
  params.M[0] = 0.04331042079065206;
  params.M[1] = 1.5717878173906188;
  params.M[2] = 1.5851723557337523;
  params.M[3] = -1.497658758144655;
  params.M[4] = -1.171028487447253;
  params.M[5] = -1.7941311867966805;
  params.M[6] = -0.23676062539745413;
  params.M[7] = -1.8804951564857322;
  params.M[8] = -0.17266710242115568;
  params.M[9] = 0.596576190459043;
  params.M[10] = -0.8860508694080989;
  params.M[11] = 0.7050196079205251;
  params.M[12] = 0.3634512696654033;
  params.M[13] = -1.9040724704913385;
  params.M[14] = 0.23541635196352795;
  params.M[15] = -0.9629902123701384;
  params.M[16] = -0.3395952119597214;
  params.M[17] = -0.865899672914725;
  params.M[18] = 0.7725516732519853;
  params.M[19] = -0.23818512931704205;
  params.M[20] = -1.372529046100147;
  params.M[21] = 0.17859607212737894;
  params.M[22] = 1.1212590580454682;
  params.M[23] = -0.774545870495281;
  params.M[24] = -1.1121684642712744;
  params.M[25] = -0.44811496977740495;
  params.M[26] = 1.7455345994417217;
  params.M[27] = 1.9039816898917352;
  params.M[28] = 0.6895347036512547;
  params.M[29] = 1.6113364341535923;
  params.M[30] = 1.383003485172717;
  params.M[31] = -0.48802383468444344;
  params.M[32] = -1.631131964513103;
  params.M[33] = 0.6136436100941447;
  params.M[34] = 0.2313630495538037;
  params.M[35] = -0.5537409477496875;
  params.M[36] = -1.0997819806406723;
  params.M[37] = -0.3739203344950055;
  params.M[38] = -0.12423900520332376;
  params.M[39] = -0.923057686995755;
  params.M[40] = -0.8328289030982696;
  params.M[41] = -0.16925440270808823;
  params.M[42] = 1.442135651787706;
  params.M[43] = 0.34501161787128565;
  params.M[44] = -0.8660485502711608;
  params.M[45] = -0.8880899735055947;
  params.M[46] = -0.1815116979122129;
  params.M[47] = -1.17835862158005;
  params.M[48] = -1.1944851558277074;
  params.M[49] = 0.05614023926976763;
  params.M[50] = -1.6510825248767813;
  params.M[51] = -0.06565787059365391;
  params.M[52] = -0.5512951504486665;
  params.M[53] = 0.8307464872626844;
  params.m[0] = 0.9869848924080182;
  params.m[1] = 0.7643716874230573;
  params.m[2] = 0.7567216550196565;
  params.m[3] = -0.5055995034042868;
  params.m[4] = 0.6725392189410702;
  params.m[5] = -0.6406053441727284;
  params.m[6] = 0.29117547947550015;
  params.m[7] = -0.6967713677405021;
  params.m[8] = -0.21941980294587182;
  params.m[9] = -1.753884276680243;
  params.m[10] = -1.0292983112626475;
  params.m[11] = 1.8864104246942706;
  params.m[12] = -1.077663182579704;
  params.m[13] = 0.7659100437893209;
  params.m[14] = 0.6019074328549583;
  params.m[15] = 0.8957565577499285;
  params.m[16] = -0.09964555746227477;
  params.m[17] = 0.38665509840745127;
}
