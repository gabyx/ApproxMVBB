// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_AABB_hpp
#define ApproxMVBB_AABB_hpp

#include <algorithm>

#include "ApproxMVBB/Common/TypeDefs.hpp"
#include "ApproxMVBB/Common/AssertionDebug.hpp"

namespace ApproxMVBB{
class APPROXMVBB_EXPORT AABB {
public:

    DEFINE_MATRIX_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AABB() {
        reset();
    };

    void reset();

    AABB( const Vector3 &p);
    AABB( const Vector3 &l, const Vector3 &u);
    AABB& unite(const Vector3 &p);
    AABB& unite(const AABB & box);

    AABB operator+ (const Vector3 &p);

    AABB operator+ (const AABB & box);

    AABB& operator+= (const AABB & box);

    AABB & transform(const AffineTrafo & M);

    inline bool overlaps(const AABB & box) const {
        bool x = (m_maxPoint(0) >= box. m_minPoint(0)) && ( m_minPoint(0) <= box.m_maxPoint(0));
        bool y = (m_maxPoint(1) >= box. m_minPoint(1)) && ( m_minPoint(1) <= box.m_maxPoint(1));
        bool z = (m_maxPoint(2) >= box. m_minPoint(2)) && ( m_minPoint(2) <= box.m_maxPoint(2));
        return (x && y && z);
    };

    inline bool inside(const Vector3 &p) const {
        return (
                   p(0) >= m_minPoint(0) && p(0) <= m_maxPoint(0) &&
                   p(1) >= m_minPoint(1) && p(1) <= m_maxPoint(1) &&
                   p(2) >= m_minPoint(2) && p(2) <= m_maxPoint(2));
    };

    inline Array3 extent() const{
        return (m_maxPoint - m_minPoint).array();
    };

    inline PREC maxExtent() const{
        return (m_maxPoint - m_minPoint).maxCoeff();
    };

    inline bool isEmpty() const {
        return m_maxPoint(0) <= m_minPoint(0) || m_maxPoint(1) <= m_minPoint(1) || m_maxPoint(2) <= m_minPoint(2);
    }

    inline void expand(PREC d) {
        ASSERTMSG(d>=0,"d>=0")
        m_minPoint -= Vector3(d,d,d);
        m_maxPoint += Vector3(d,d,d);
    };

    inline void expand(Vector3 d) {
        ASSERTMSG(d(0)>=0 && d(1)>=0 && d(2)>=0,"d>=0")
        m_minPoint -= d;
        m_maxPoint += d;
    };

    inline PREC volume() const {
        Vector3 d = m_maxPoint- m_minPoint;
        return d(0) * d(1) * d(2);
    };

    //info about axis aligned bounding box
    Vector3 m_minPoint;
    Vector3 m_maxPoint;
};


class APPROXMVBB_EXPORT AABB2d {
public:

    DEFINE_MATRIX_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AABB2d() {
        reset();
    };

    void reset();

    AABB2d( const Vector2 &p);
    AABB2d( const Vector2 &l, const Vector2 &u);

    template<typename Derived>
    AABB2d& unite(const MatrixBase<Derived> &p) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,2);
        m_maxPoint(0) = std::max(m_maxPoint(0),p(0));
        m_maxPoint(1) = std::max(m_maxPoint(1),p(1));
        m_minPoint(0) = std::min( m_minPoint(0),p(0));
        m_minPoint(1) = std::min( m_minPoint(1),p(1));
        return *this;
    };

    AABB2d& unite(const AABB2d & box);
    AABB2d operator+ (const Vector2 &p);
    AABB2d operator+ (const AABB2d & box);
    AABB2d& operator+= (const AABB2d & box);

    inline Array2 extent() const{
        return (m_maxPoint - m_minPoint).array();
    };

    inline PREC maxExtent() const{
        return (m_maxPoint - m_minPoint).maxCoeff();
    };

    inline AABB2d & transform(const AffineTrafo2d & M);

    inline bool overlaps(const AABB2d & box) const {
        bool x = (m_maxPoint(0) >= box.m_minPoint(0)) && ( m_minPoint(0) <= box.m_maxPoint(0));
        bool y = (m_maxPoint(1) >= box.m_minPoint(1)) && ( m_minPoint(1) <= box.m_maxPoint(1));
        return (x && y);
    };

    inline bool inside(const Vector2 &p) const {
        return (
                   p(0) >= m_minPoint(0) && p(0) <= m_maxPoint(0) &&
                   p(1) >= m_minPoint(1) && p(1) <= m_maxPoint(1));
    };

    inline bool isEmpty() const {
        return m_maxPoint(0) <= m_minPoint(0) || m_maxPoint(1) <= m_minPoint(1);
    }

    inline void expand(PREC d) {
        ASSERTMSG(d>=0,"d>=0")
        m_minPoint -= Vector2(d,d);
        m_maxPoint += Vector2(d,d);
    };

    inline void expand(Vector2 d) {
        ASSERTMSG(d(0)>=0 && d(1)>=0,"d>=0")
        m_minPoint -= d;
        m_maxPoint += d;
    };

    inline PREC area() const {
        Vector2 d = m_maxPoint- m_minPoint;
        return d(0) * d(1);
    };

    //info about axis aligned bounding box
    Vector2 m_minPoint;
    Vector2 m_maxPoint;
};

};

 #endif
