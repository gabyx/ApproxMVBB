// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "ApproxMVBB/MinAreaRectangle.hpp"

namespace ApproxMVBB
{
    void MinAreaRectangle::compute()
    {
        // Compute minimum area rectangle

        // Clear minBox;
        m_minBox.reset();

        // If no points return early!
        if(m_p.cols() == 0)
        {
            return;
        }

        // Generate Convex Hull
        m_convh.compute();
        ApproxMVBB_ASSERTMSG(m_convh.verifyHull(), "Convex hull not ok!")

            // Compute Rectangle
            // computes p,u,v of box (not normalized)
            computeRectangle();

        // Final adjustments (u and v are getting normalized)
        adjustRectangle();
    }

    void MinAreaRectangle::computeRectangle()
    {
        // Copy Points;
        m_hullIdx    = m_convh.getIndices();
        auto nPoints = m_hullIdx.size();

        // std::cout << "Convex hull points:" << nPoints << std::endl;

        // The below code works for points >= 2
        // Anyway catch the cases n=1 and 2 and return early!
        if(nPoints == 0)
        {
            return;
        }
        else if(nPoints == 1)
        {
            m_minBox.m_p = m_p.col(m_hullIdx[0]);
            m_minBox.m_u.setZero();
            m_minBox.m_v.setZero();
            return;
        }
        else if(nPoints == 2)
        {
            m_minBox.m_p = m_p.col(m_hullIdx[0]);
            m_minBox.m_u = m_p.col(m_hullIdx[1]) - m_p.col(m_hullIdx[0]);
            m_minBox.m_v.setZero();
            return;
        }

        // Performing Rotating Calipers method
        // 1. find the vertices with the minimum and maximum x an y coordinates.
        //    These vertices (points) will be denoted by {PI, PJ, PK, PL}
        // 2. construct CALL and CALJ as the first set of calipers parallel to the
        // x-axis,
        //    and CALI and CALK as the second set of calipers parallel to the y-axis
        // 3. create for every CALX a next caliper,
        //    NEXT_CALX, based on the next point in the convex hull and calculate the
        //    tangent of CALX and it's NEXT_CALX
        // 4. get the smallest positive tangent between all CALX and NEXT_CALX and
        // rotate every caliper by that smallest gradient
        // 5. repeat step 3 and 4 untill the calipers have turned more then 90 degrees

        // We implement this algorithm a bit differently:
        // 1. Compute all angles of all edges angle[i] is angle of edge p[i],p[i+1].
        // 2. Rotatate the first caliper A from edge to edge.
        // 3. Determine all new angles for B,C,D such that it forms a box (rotation of
        // pi from angle of caliper A)
        // 4. Determine for each caliper I  the location (the corresponding vertex
        // index) where the
        //    caliper I forms a tangent (starting from the currend index).
        // 5. Determine the box by intersecting the calipers.
        // 6. Compare to area to minimum and return to 2.

        // Calcualte all edge angles

        m_angles.resize(nPoints);
        for(unsigned int i = 0; i < nPoints; ++i)
        {
            m_angles[i] = PointFunctions::getAngle(m_p.col(m_hullIdx[i]), m_p.col(m_hullIdx[(i + 1) % nPoints]));
            // std::cout << "angle: " << m_angles[i] << std::endl;
        }

        // 2. Construct 4 Calipers
        Caliper calipers[4];
        // Set A
        calipers[0].m_idx   = 0;
        calipers[0].m_ptIdx = m_hullIdx[0];
        // Init A,B,C,D
        updateCalipers(m_angles[0], calipers);

        Box2d box;
        // Init minBox
        getBox(calipers, m_minBox);

        // Rotate calipers over all edges and take minimum box
        for(unsigned int i = 1; i < nPoints; ++i)
        {
            updateCalipers(m_angles[i], calipers);
            // Get Box from Calipers
            getBox(calipers, box);

            if(box.m_area < m_minBox.m_area)
            {
                m_minBox = box;
            }
        }
    }

    // determine the vertex v for which the edge angle is greater than c.m_currAngle
    // the find algroithm starts at c.m_idx
    void MinAreaRectangle::findVertex(Caliper& c)
    {
        PREC matchAngle = c.m_currAngle;  // between 0-2pi
        // std::cout << "Match Angle: " << matchAngle << std::endl;

        unsigned int currIdx = c.m_idx;

        PREC currAngle = m_angles[currIdx];  // all between 0-2pi
        PREC nextAngle;

        unsigned int nPoints = m_hullIdx.size();
        bool found           = false;
        unsigned int i       = 0;
        while(!found && i < nPoints)
        {
            currIdx = (currIdx + 1) % nPoints;
            // std::cout << "c: " << currIdx << std::endl;
            nextAngle = m_angles[currIdx];
            // std::cout << "diff: " << std::abs(currAngle-nextAngle) << std::endl;
            //                                         curr       next
            // if we are not at the boundary [ ..., 45 degree, 50 degree , ....]
            // and in
            if(nextAngle > currAngle)
            {
                // std::cout << " greater" <<std::endl;
                found = (currAngle < matchAngle && matchAngle <= nextAngle);
            }
            else
            {
                // std::cout << " boundary" <<std::endl;
                //                                     curr       next
                // if we are at the boundary [ ..., 359 degree] [ 5 degree , .... ]
                // skip this if we have an angle difference |curr -next| < eps
                // this might happen due to floating point nonsense
                found = (std::abs(currAngle - nextAngle) > 1e-10) && (currAngle < matchAngle || matchAngle <= nextAngle);
            }

            currAngle = nextAngle;
            ++i;
        }

        if(found)
        {
            c.m_idx   = currIdx;
            c.m_ptIdx = m_hullIdx[currIdx];
            // std::cout << "caliper idx:" << currIdx << std::endl;
        }
        else
        {
            std::stringstream ss;
            for(auto& a : m_angles)
            {
                ss << a << ",";
            }
            ApproxMVBB_ERRORMSG("Could not find vertex with angle greater than: " << matchAngle
                                                                                  << "in angles: " << ss.str());
        }
    }

    void MinAreaRectangle::getBox(Caliper (&c)[4], Box2d& box)
    {
        using namespace PointFunctions;
        // Intersect Caliper A with B,
        box.m_u = intersectLines(m_p.col(c[0].m_ptIdx), c[0].m_currAngle, m_p.col(c[1].m_ptIdx), c[1].m_currAngle);
        // Intersect Caliper D with C,
        box.m_v = intersectLines(m_p.col(c[3].m_ptIdx), c[3].m_currAngle, m_p.col(c[2].m_ptIdx), c[2].m_currAngle);

        // Intersect Caliper A with D,
        box.m_p = intersectLines(m_p.col(c[0].m_ptIdx), c[0].m_currAngle, m_p.col(c[3].m_ptIdx), c[3].m_currAngle);

        box.m_u -= box.m_p;  //(p1-p0)
        box.m_v -= box.m_p;  //(p2-p0)
        box.m_area = box.m_u.norm() * box.m_v.norm();
    }
}  // namespace ApproxMVBB
