// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================


#include "ApproxMVBB/AABB.hpp"
namespace ApproxMVBB{

void AABB::reset() {
    // Violating the constraint min<max for making a completey empty box!
    m_minPoint(0) = std::numeric_limits<PREC>::max();
    m_maxPoint(0) = std::numeric_limits<PREC>::min();
    m_minPoint(1) = std::numeric_limits<PREC>::max();
    m_maxPoint(1) = std::numeric_limits<PREC>::min();
    m_minPoint(2) = std::numeric_limits<PREC>::max();
    m_maxPoint(2) = std::numeric_limits<PREC>::min();
}


AABB::AABB( const Vector3 &p) {
    m_minPoint = p;
    m_maxPoint = p;
};
AABB::AABB( const Vector3 &l, const Vector3 &u) {
    m_minPoint = Vector3(std::min(l(0),u(0)),std::min(l(1),u(1)),std::min(l(2),u(2)));
    m_maxPoint = Vector3(std::max(l(0),u(0)),std::max(l(1),u(1)),std::max(l(2),u(2)));
};

AABB& AABB::unite(const Vector3 &p) {
    m_maxPoint(0) = std::max(m_maxPoint(0),p(0));
    m_maxPoint(1) = std::max(m_maxPoint(1),p(1));
    m_maxPoint(2) = std::max(m_maxPoint(2),p(2));
    m_minPoint(0) = std::min( m_minPoint(0),p(0));
    m_minPoint(1) = std::min( m_minPoint(1),p(1));
    m_minPoint(2) = std::min( m_minPoint(2),p(2));
    return *this;
};

AABB& AABB::unite(const AABB & box) {
    m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
    m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
    m_maxPoint(2) = std::max(m_maxPoint(2),box.m_maxPoint(2));
    m_minPoint(0) = std::min( m_minPoint(0),box.m_minPoint(0));
    m_minPoint(1) = std::min( m_minPoint(1),box.m_minPoint(1));
    m_minPoint(2) = std::min( m_minPoint(2),box.m_minPoint(2));
    return *this;
};

AABB AABB::operator+ (const Vector3 &p) {
    AABB a;
    a.m_maxPoint(0) = std::max(m_maxPoint(0),p(0));
    a.m_maxPoint(1) = std::max(m_maxPoint(1),p(1));
    a.m_maxPoint(2) = std::max(m_maxPoint(2),p(2));
    a. m_minPoint(0) = std::min( m_minPoint(0),p(0));
    a. m_minPoint(1) = std::min( m_minPoint(1),p(1));
    a. m_minPoint(2) = std::min( m_minPoint(2),p(2));
    return a;
};

AABB AABB::operator+ (const AABB & box) {
    AABB a;
    a.m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
    a.m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
    a.m_maxPoint(2) = std::max(m_maxPoint(2),box.m_maxPoint(2));
    a. m_minPoint(0) = std::min( m_minPoint(0),box. m_minPoint(0));
    a. m_minPoint(1) = std::min( m_minPoint(1),box. m_minPoint(1));
    a. m_minPoint(2) = std::min( m_minPoint(2),box. m_minPoint(2));
    return a;
};

AABB& AABB::operator+= (const AABB & box) {
    m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
    m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
    m_maxPoint(2) = std::max(m_maxPoint(2),box.m_maxPoint(2));
    m_minPoint(0) = std::min( m_minPoint(0),box. m_minPoint(0));
    m_minPoint(1) = std::min( m_minPoint(1),box. m_minPoint(1));
    m_minPoint(2) = std::min( m_minPoint(2),box. m_minPoint(2));
    return *this;
};

AABB&  AABB::transform(const AffineTrafo & M) {

    AABB ret( M*(Vector3( m_minPoint(0), m_minPoint(1), m_minPoint(2))));
    ret.unite(M*(Vector3( m_maxPoint(0), m_minPoint(1), m_minPoint(2))));
    ret.unite(M*(Vector3( m_minPoint(0), m_maxPoint(1), m_minPoint(2))));
    ret.unite(M*(Vector3( m_minPoint(0), m_minPoint(1), m_maxPoint(2))));
    ret.unite(M*(Vector3( m_minPoint(0), m_maxPoint(1), m_maxPoint(2))));
    ret.unite(M*(Vector3( m_maxPoint(0), m_maxPoint(1), m_minPoint(2))));
    ret.unite(M*(Vector3( m_maxPoint(0), m_minPoint(1), m_maxPoint(2))));
    ret.unite(M*(Vector3( m_maxPoint(0), m_maxPoint(1), m_maxPoint(2))));
    *this = ret;
    return *this;
};


void AABB2d::reset() {
    // Violating the constraint min<max for making a completey empty box!
    m_minPoint(0) = std::numeric_limits<PREC>::max();
    m_maxPoint(0) = std::numeric_limits<PREC>::min();
    m_minPoint(1) = std::numeric_limits<PREC>::max();
    m_maxPoint(1) = std::numeric_limits<PREC>::min();
}

AABB2d::AABB2d( const Vector2 &p) {
    m_minPoint = p;
    m_maxPoint = p;
};
AABB2d::AABB2d( const Vector2 &l, const Vector2 &u) {
    m_minPoint = Vector2(std::min(l(0),u(0)),std::min(l(1),u(1)));
    m_maxPoint = Vector2(std::max(l(0),u(0)),std::max(l(1),u(1)));
};

AABB2d& AABB2d::unite(const AABB2d & box) {
    m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
    m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
    m_minPoint(0) = std::min( m_minPoint(0),box.m_minPoint(0));
    m_minPoint(1) = std::min( m_minPoint(1),box.m_minPoint(1));
    return *this;
};

AABB2d AABB2d::operator+ (const Vector2 &p) {
    AABB2d a;
    a.m_maxPoint(0) = std::max(m_maxPoint(0),p(0));
    a.m_maxPoint(1) = std::max(m_maxPoint(1),p(1));
    a. m_minPoint(0) = std::min( m_minPoint(0),p(0));
    a. m_minPoint(1) = std::min( m_minPoint(1),p(1));
    return a;
};

AABB2d AABB2d::operator+ (const AABB2d & box) {
    AABB2d a;
    a.m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
    a.m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
    a. m_minPoint(0) = std::min( m_minPoint(0),box. m_minPoint(0));
    a. m_minPoint(1) = std::min( m_minPoint(1),box. m_minPoint(1));
    return a;
};

AABB2d& AABB2d::operator+= (const AABB2d & box) {
    m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
    m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
    m_minPoint(0) = std::min( m_minPoint(0),box. m_minPoint(0));
    m_minPoint(1) = std::min( m_minPoint(1),box. m_minPoint(1));
    return *this;
};


AABB2d & AABB2d::transform(const AffineTrafo2d & M) {

    AABB2d ret( M*(Vector2( m_minPoint(0), m_minPoint(1))));
    ret.unite(M*(Vector2( m_maxPoint(0), m_minPoint(1))));
    ret.unite(M*(Vector2( m_minPoint(0), m_maxPoint(1))));
    ret.unite(M*(Vector2( m_maxPoint(0), m_maxPoint(1))));
    *this = ret;
    return *this;
};
};
