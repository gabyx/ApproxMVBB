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
//    std::cout << "min:" << position << std::endl;
    Vector2 base = m_p.col(position);

    // Indices into m_p
    // first = index into m_p, second =  delete flag!
    using PointData = std::pair<unsigned int, bool>; std::vector< PointData > indicesTemp;

    //Save p0 as first point in indices list (temporary)
    indicesTemp.emplace_back(position,false);
    //Add all indices of points not almostEqual to position
    for(unsigned int i = 0; i<m_p.cols(); ++i) {
        if( (i != position) && !almostEqualUlp(base,m_p.col(i))) {
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
    std::sort( indicesTemp.begin()+1, indicesTemp.end(), comp );


    // copy indices into new array which only consists of the indices which have no delete flag (index == position)
    //std::cout << "Deleted points:" << deletedPoints << " of " << indicesTemp.size() << std::endl;
    std::vector<unsigned int> indices(indicesTemp.size() - deletedPoints);
    unsigned int k=0; auto s = indicesTemp.size();
    for(decltype(s) i = 0; i < s; ++i){
        PointData & pp = indicesTemp[i]; if(!pp.second){indices[k++] = pp.first;}
    }

    // Remove almost equal elements
    auto d2 = ContainerFunctions::moveConsecutiveToFrontIf(indices.begin(),indices.end(),
                                                             [&](const unsigned int idx1, const unsigned int idx2){
                                                                return !almostEqualUlp(this->m_p.col(idx1),this->m_p.col(idx2),1e-8);
                                                             }
                                                             );
    //    std::cout << "Deleted almostEqual: " << std::distance(d2,indices.end()) << std::endl;
    indices.resize( std::distance(indices.begin(),d2) );

    // Convex hull consists of only 1 or 2 points!
    if(indicesTemp.size() <= 2  ) {
        for(auto & pa : indicesTemp){ m_indicesCH.emplace_back(pa.first);}  return;
    }

    //    std::cout << "Graham Scan points: " << indices.size() << std::endl;

    unsigned int nPoints  = indices.size();
    unsigned int lastIdx  = indices[0];
    unsigned int firstIdx = indices[1];
    m_indicesCH.push_back( lastIdx  );
    m_indicesCH.push_back( firstIdx );

    unsigned int lPtIdx  = lastIdx;
    unsigned int mPtIdx  = firstIdx;
    unsigned int currIdx;
    unsigned int i = 1; // current point;

    // skip the first non left turns in the sequence!
//    std::cout << "lastIdx point: " <<lastIdx << ","<< m_p.col(lastIdx).transpose() << std::endl;
//    std::cout << "firstIdx point: " <<firstIdx << ","<< m_p.col(firstIdx).transpose() << std::endl;

    do{
            currIdx = indices[++i];
    }
    while (( i < nPoints-1 ) && !leftTurn(m_p.col(lastIdx), m_p.col(firstIdx), m_p.col(currIdx)));

//    std::cout << "currIdx point: " <<currIdx << std::endl;
//    std::cout << ","<< m_p.col(currIdx).transpose() << std::endl << "===="<<std::endl;


    if ( i < nPoints )
    {
      m_indicesCH.push_back( currIdx );
      decltype(m_indicesCH.rbegin()) revIter;
      lPtIdx  = mPtIdx;
      mPtIdx  = currIdx;

      for ( ++i ; i < nPoints; ++i )
      {
          currIdx = indices[i];
          if ( leftTurn(m_p.col(mPtIdx), m_p.col(currIdx), m_p.col(lastIdx)) )
          {

//            std::cout << "lastIdx point: " <<lPtIdx << ","<< m_p.col(lPtIdx).transpose() << std::endl;
//            std::cout << "firstIdx point: " <<mPtIdx << ","<< m_p.col(mPtIdx).transpose() << std::endl;
//            std::cout << "currIdx point: " <<currIdx << ","<< m_p.col(currIdx).transpose() << std::endl;
              while ( !leftTurn(m_p.col(lPtIdx), m_p.col(mPtIdx), m_p.col(currIdx)) )
              {

                  //ApproxMVBB_ASSERTMSG(m_indicesCH.size()>2,"");
                  m_indicesCH.pop_back();

                  if( m_indicesCH.size() <= 1) { // Degenerate Case if we come back to the beginning
                    m_indicesCH.push_back(currIdx);
                    mPtIdx  = currIdx;
                    break;
                  }else{
                      mPtIdx = lPtIdx;                      // middle point becomes last
                      revIter = m_indicesCH.rbegin();
                      lPtIdx = *(++revIter);                // previous of .back();
//                      std::cout << "rem: " << lPtIdx << "," << mPtIdx << "," << currIdx << std::endl;
                  }

              }
              m_indicesCH.push_back( currIdx );
              lPtIdx  = mPtIdx;          // last becomes middle
              mPtIdx  = currIdx;         // middle becomes currIdx
          }
      }

    }
}


bool ConvexHull2D::verifyHull() {
    using namespace PointFunctions;
    if(m_indicesCH.size()>2) {
        unsigned int i = 2;
        while( i<m_indicesCH.size()) {
            if( ! (orient2d( m_p.col(m_indicesCH[i-2]), m_p.col(m_indicesCH[i-1]), m_p.col(m_indicesCH[i])) >= 0) ) { // if this is not a left turn
                return false;
            }
            ++i;
        }
    }
    return true;
}


};
