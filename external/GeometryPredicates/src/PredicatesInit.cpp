// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================


#include <stdio.h>

/* FPU control. We MUST have only double precision (not extended precision) */
#include "ApproxMVBB/GeometryPredicates/Rounding.hpp"


int main (int , char * [])
{
  FPU_DECLARE

  double half = 0.5;
  double check = 1.0, lastcheck;
  int every_other = 1;
  /* epsilon = 2^(-p).  Used to estimate roundoff errors. */
  double epsilon = 1.0;
  /* splitter = 2^ceiling(p / 2) + 1.  Used to split floats in half. */
  double splitter = 1.0;
  /* A set of coefficients used to calculate maximum roundoff errors. */
  double resulterrbound;
  double ccwerrboundA, ccwerrboundB, ccwerrboundC;
  double o3derrboundA, o3derrboundB, o3derrboundC;
  double iccerrboundA, iccerrboundB, iccerrboundC;
  double isperrboundA, isperrboundB, isperrboundC;

  FPU_ROUND_DOUBLE;

  epsilon = 1.0;
  splitter = 1.0;
  /* Repeatedly divide `epsilon' by two until it is too small to add to   */
  /* one without causing roundoff.  (Also check if the sum is equal to    */
  /* the previous sum, for machines that round up instead of using exact  */
  /* rounding.  Not that this library will work on such machines anyway). */
  do {
    lastcheck = check;
    epsilon *= half;
    if (every_other) {
      splitter *= 2.0;
    }
    every_other = !every_other;
    check = 1.0 + epsilon;
  } while ((check != 1.0) && (check != lastcheck));
  splitter += 1.0;
  /* Error bounds for orientation and incircle tests. */
  resulterrbound = (3.0 + 8.0 * epsilon) * epsilon;
  ccwerrboundA = (3.0 + 16.0 * epsilon) * epsilon;
  ccwerrboundB = (2.0 + 12.0 * epsilon) * epsilon;
  ccwerrboundC = (9.0 + 64.0 * epsilon) * epsilon * epsilon;
  o3derrboundA = (7.0 + 56.0 * epsilon) * epsilon;
  o3derrboundB = (3.0 + 28.0 * epsilon) * epsilon;
  o3derrboundC = (26.0 + 288.0 * epsilon) * epsilon * epsilon;
  iccerrboundA = (10.0 + 96.0 * epsilon) * epsilon;
  iccerrboundB = (4.0 + 48.0 * epsilon) * epsilon;
  iccerrboundC = (44.0 + 576.0 * epsilon) * epsilon * epsilon;
  isperrboundA = (16.0 + 224.0 * epsilon) * epsilon;
  isperrboundB = (5.0 + 72.0 * epsilon) * epsilon;
  isperrboundC = (71.0 + 1408.0 * epsilon) * epsilon * epsilon;



  FILE * pFile = fopen ("PredicatesInit.hpp","w");

  fputs("/* This file was generated automatically by PredicatsInit.c */\n"
"// ========================================================================================\n"
"//  ApproxMVBB\n"
"//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (d0t) ch>\n"
"//\n"
"//  This Source Code Form is subject to the terms of the Mozilla Public\n"
"//  License, v. 2.0. If a copy of the MPL was not distributed with this\n"
"//  file, You can obtain one at http://mozilla.org/MPL/2.0/.\n"
"// ========================================================================================\n", pFile);
  fprintf(pFile,"static double splitter = %f;\n", splitter);
  fprintf(pFile,"static double resulterrbound = %.16g;\n", resulterrbound);
  fprintf(pFile,"static double ccwerrboundA = %.16g;\n", ccwerrboundA);
  fprintf(pFile,"static double ccwerrboundB = %.16g;\n", ccwerrboundB);
  fprintf(pFile,"static double ccwerrboundC = %.16g;\n", ccwerrboundC);
  fprintf(pFile,"static double o3derrboundA = %.16g;\n", o3derrboundA);
  fprintf(pFile,"static double o3derrboundB = %.16g;\n", o3derrboundB);
  fprintf(pFile,"static double o3derrboundC = %.16g;\n", o3derrboundC);
  fprintf(pFile,"static double iccerrboundA = %.16g;\n", iccerrboundA);
  fprintf(pFile,"static double iccerrboundB = %.16g;\n", iccerrboundB);
  fprintf(pFile,"static double iccerrboundC = %.16g;\n", iccerrboundC);
  fprintf(pFile,"static double isperrboundA = %.16g;\n", isperrboundA);
  fprintf(pFile,"static double isperrboundB = %.16g;\n", isperrboundB);
  fprintf(pFile,"static double isperrboundC = %.16g;\n", isperrboundC);
  fclose(pFile);

  FPU_RESTORE;

  return 0;
}
