// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "TestFunctions.hpp"

namespace ApproxMVBB
{
namespace TestFunctions
{
OOBB readOOBBAndCheck(OOBB& oobb, std::string filePath)
{
    Vector3List points(8);
    Vector3 minP;
    Vector3 maxP;
    Matrix33 R_KI;
    readOOBB(filePath, minP, maxP, R_KI, points);

    Vector3List pointsV = oobb.getCornerPoints();

    Matrix3Dyn pV(3, pointsV.size());
    Matrix3Dyn p(3, points.size());

    for(unsigned int i = 0; i < pointsV.size(); ++i)
    {
        pV.col(i) = pointsV[i];
    }
    for(unsigned int i = 0; i < points.size(); ++i)
    {
        p.col(i) = points[i];
    }

    std::cout << "Compare Corners: valid:" << std::endl;
    std::cout << p << std::endl;
    std::cout << "and:" << std::endl;
    std::cout << pV << std::endl;

    EXPECT_TRUE(assertNearArrayColsRows<true>(p, pV))
        << "Not all corner points of OOBB equal to validation OOBB in file: " << filePath;

    std::cout << filePath << std::endl
              << "valid minP: " << std::endl
              << minP.transpose() << " and " << std::endl
              << oobb.m_minPoint.transpose() << std::endl
              << "valid maxP: " << std::endl
              << maxP.transpose() << " and " << std::endl
              << oobb.m_maxPoint.transpose() << std::endl
              << "valid R_IK: " << std::endl
              << R_KI << " and " << std::endl
              << oobb.m_q_KI.matrix() << std::endl;

    // return valid oobb
    return {minP, maxP, R_KI};
}
}
}
