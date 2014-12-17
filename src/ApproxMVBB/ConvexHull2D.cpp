// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================
//#include <iterator> // for ostream_iterator
//#include "TestFunctions.hpp"
#include "ApproxMVBB/ConvexHull2D.hpp"
namespace ApproxMVBB{
void ConvexHull2D::compute() {
    using namespace PointFunctions;
    using namespace ContainerFunctions;

    m_indicesCH.clear();

    // Need at least 1 point! m_p.col(0)
    if(m_p.cols()==0) {
        return;
    }

    // Compute min point p0
    unsigned int position = minPointYX(m_p);
    Vector2 base = m_p.col(position);

    // Indices into m_p
    // first = index into m_p, second =  delete flag!
    using PointData = std::pair<unsigned int, bool>;
    std::vector< PointData > indicesTemp;

    //Save p0 as first point in indices list (temporary)
    indicesTemp.emplace_back(position,false);

    //Add all indices of points not almostEqual to position
    for(unsigned int i = 0; i<m_p.cols(); ++i) {
        if( (i != position) && !almostEqual(base,m_p.col(i)) ) {
            indicesTemp.emplace_back(i,false);
        }
    }

    // Convex hull consists of only 1 or 2 points!
    if(indicesTemp.size() <= 2  ) {
        for(auto & pa : indicesTemp){ m_indicesCH.emplace_back(pa.first);}  return;
    }

    // Sort by angle
    unsigned int deletedPoints = 0;
    CompareByAngle comp(m_p,base,position,deletedPoints);
    std::sort( indicesTemp.begin() + 1, indicesTemp.end(), comp );

    // copy indices into new array which only consists of the indices which have no delete flag (index == position)
    std::vector<unsigned int> indices(indicesTemp.size() - deletedPoints);
    unsigned int k=0;
    auto s = indicesTemp.size();
    for(decltype(s) i = 0; i < s; ++i){
        PointData & pp = indicesTemp[i]; if(!pp.second){indices[k++] = pp.first;}
    }

    // Convex hull consists of only 1,2 or 3 points!
    if(indices.size() <= 3  ) {
        m_indicesCH.assign(indices.begin(),indices.end());
        return;
    }

//    // DEBUG
//    {
//        std::cout << "DUMP Points ConvexHUll DEBUG: points: " <<indices.size()<< std::endl;
//        Matrix2Dyn pp(2,indices.size());
//        unsigned int k = 0;
//        for(auto i : indices){
//            pp.col(k++) = m_p.col(i);
//        }
//        TestFunctions::dumpPointsMatrixBinary("DumpPointsConvexHull.bin",pp);
//    }

    // Perform Graham Scan over indices
    // Iterate over point list starting at 1:
    // Start with triangle p0,p1,p2

    m_indicesCH.push_back( indices[0] );
    m_indicesCH.push_back( indices[1] );
    unsigned int lPtIdx  = indices[0];
    unsigned int mPtIdx  = indices[1];
    unsigned int currIdx;
    unsigned int i = 2; // current point
    auto size = indices.size();
    while( i < size) {
        ApproxMVBB_ASSERTMSG( lPtIdx < m_p.size() && mPtIdx < m_p.size() && currIdx < m_p.size(), "?")

        currIdx = indices[i];

        if( areaSign( m_p.col(lPtIdx), m_p.col(mPtIdx), m_p.col(currIdx)) > 0) { // if we have a left turn!
            //std::cout << "Left turn: c:" << currIdx << " m:" << mPtIdx << " l: "<< lPtIdx << " sign: " << areaSign( m_p.col(lPtIdx), m_p.col(mPtIdx), m_p.col(currIdx)) << std::endl;

            m_indicesCH.push_back(currIdx); //add currIdx to convex hull list
            lPtIdx  = mPtIdx;          // last becomes middle
            mPtIdx  = currIdx;         // middle becomes currIdx
            ++i;                       // next point

        } else { // if we have a right turn
            //std::cout << "Right turn: c:" << currIdx << " m:" << mPtIdx << " l: "<< lPtIdx << " sign: " << areaSign( m_p.col(lPtIdx), m_p.col(mPtIdx), m_p.col(currIdx)) << std::endl;
            m_indicesCH.pop_back();    // remove middle point;

            // move triplet back if possible
            if( m_indicesCH.size() <= 1) { // Degenerate Case if we come back to the beginning
                m_indicesCH.push_back(currIdx);
                mPtIdx  = currIdx;
                ++i;                    // next point
            } else {
                // move triplet back
                mPtIdx = lPtIdx;                      // middle point becomes last
                lPtIdx = *(++m_indicesCH.rbegin());   // previous of .back();
            }
        }

    }

    //std::cout << "indices End: " << m_indicesCH.size() << std::endl;
}

bool ConvexHull2D::verifyHull() {
    using namespace PointFunctions;
    if(m_indicesCH.size()>2) {
        unsigned int i = 2;
        while( i<m_indicesCH.size()) {
            if( ! (areaSign( m_p.col(m_indicesCH[i-2]), m_p.col(m_indicesCH[i-1]), m_p.col(m_indicesCH[i])) >= 0) ) { // if this is not a left turn
                return false;
            }
            ++i;
        }
    }
    return true;
}


};
