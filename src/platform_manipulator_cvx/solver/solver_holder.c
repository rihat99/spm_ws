/* Produced by CVXGEN, 2022-04-18 05:23:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"


// #define NUMTESTS 0
void solve_for_theta(double ref0, double ref1, double ref2) {
  
  // setup_indexing();
  params.ref[0] = ref0;
  params.ref[1] = ref1;
  params.ref[2] = ref2;
  /* Solve problem instance for the record. */
  settings.verbose = 0;
  solve();
}
void load_default_data(void) {
  float big_M[18][3] = {
	{-0.0023809321350169,0.002380932135017,0.002380932135017},
	{0.002380932135017,0.002380932135017,-0.00238093213501689},
	{0.0023809321350169,-0.00238093213501689,0.0023809321350169},
	{-2.76057425895318E-018,0.0111097396229727,-0.0111097396229727},
	{-0.0111097396229727,6.15458428984808E-018,0.0111097396229727},
	{1.73472347597681E-018,-0.0111097396229727,0.0111097396229727},
	{-1,0,0},
	{0,0,-1},
	{0,-1,-1.11022302462516E-016},
	{-0.0111097396229727,0.0111097396229727,5.55156015548123E-018},
	{-0.0133315559110321,0.00666577795551606,0.00666577795551606},
	{0.0111097396229727,-1.02329049441555E-017,-0.0111097396229727},
	{0.00666577795551606,0.00666577795551606,-0.0133315559110321},
	{0.0111097396229727,-0.0111097396229727,-3.46805474677816E-018},
	{0.00666577795551606,-0.0133315559110321,0.00666577795551606},
	{6.97882215034351E-019,0.00277776706110454,1.21636781046588E-018},
	{1.77893310652328E-018,1.71686466178787E-018,0.00277776706110454},
	{0.00277776706110454,2.17572883806635E-018,-8.04236792298464E-019}
  };
  // int k=0;
  for (int i=0; i<18; i++){
    for (int j=0; j<3; j++){
      params.M[i + j*18] = big_M[i][j];
      // k++;
    };
  };

  float small_M[18] = {0.9999914967071,0.9999914967071,0.9999914967071,
	0.999876566067542,0.999876566067542,0.999876566067542,0,0,0,0.999876566067542,
	0.999866693327409,0.999876566067542,0.999866693327409,0.999876566067542,
	0.999866693327409,0.999996141997635,0.999996141997635,0.999996141997635};

  for (int i=0; i<18; i++){
	params.m[i] = small_M[i];

  };
}