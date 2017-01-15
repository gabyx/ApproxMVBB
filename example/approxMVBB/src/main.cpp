// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <iostream>


#include "ApproxMVBB/ComputeApproxMVBB.hpp"

int  main( int  argc, char  ** argv ) {

    unsigned int nPoints = 100000;

    std::cout << "Sample "<<nPoints<<" points in unite cube (coordinates are in world frame I) " << std::endl;
    ApproxMVBB::Matrix3Dyn points(3,nPoints);
    points.setRandom();

    ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,
                                                        0.001,
                                                        500,
                                                        5 /*increasing the grid size decreases speed */
                                                        ,0,5);

    std::cout << "Computed OOBB: " << std::endl
              << "---> lower point in OOBB frame: " << oobb.m_minPoint.transpose() << std::endl
              << "---> upper point in OOBB frame: " << oobb.m_maxPoint.transpose() << std::endl
              << "---> coordinate transformation A_IK matrix from OOBB frame K to world frame I" << std::endl
              << oobb.m_q_KI.matrix() << std::endl
              << "---> this is also the rotation matrix R_KI  which turns the world frame I into the OOBB frame K" <<std::endl << std::endl;


    // To make all points inside the OOBB :
    ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose(); // faster to store the transformation matrix first
    auto size = points.cols();
    for( unsigned int i=0;  i<size; ++i ) {
        oobb.unite(A_KI*points.col(i));
    }
    std::cout << "OOBB with all point included: " << std::endl
              << "---> lower point in OOBB frame: " << oobb.m_minPoint.transpose() << std::endl
              << "---> upper point in OOBB frame: " << oobb.m_maxPoint.transpose() << std::endl;


    return 0;
}
