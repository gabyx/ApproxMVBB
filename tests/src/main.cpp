// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <iostream>

#include "ComputeApproxMVBBTests.hpp"

int  main( int  argc, char  ** argv ){

    ApproxMVBB::convexHullTest();
    ApproxMVBB::minAreaBoxTest();
    //ApproxMVBB::diameterTest();
    //ApproxMVBB::mvbbTest();

    return 0;
};
