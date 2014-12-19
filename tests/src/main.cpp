// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
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
