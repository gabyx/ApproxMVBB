// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_AABB_hpp
#define ApproxMVBB_AABB_hpp

#include <algorithm>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include ApproxMVBB_StaticAssert_INCLUDE_FILE
#include ApproxMVBB_AssertionDebug_INCLUDE_FILE

namespace ApproxMVBB
{
    template<unsigned int Dim>
    class AABB
    {
    public:
        ApproxMVBB_DEFINE_MATRIX_TYPES;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        template<unsigned int D = Dim, bool = true>
        struct unite_impl
        {
            template<typename T, typename P>
            inline static void apply(T* t, const P& p)
            {
                t->m_maxPoint(D - 1) = std::max(t->m_maxPoint(D - 1), p(D - 1));
                t->m_minPoint(D - 1) = std::min(t->m_minPoint(D - 1), p(D - 1));
                unite_impl<D - 1>::apply(t, p);
            }
        };
        template<bool dummy>
        struct unite_impl<0, dummy>
        {
            template<typename T, typename P>
            inline static void apply(T*, const P&)
            {
            }
        };

        template<unsigned int D = Dim, bool = true>
        struct uniteBox_impl
        {
            template<typename T, typename B>
            inline static void apply(T* t, const B& b)
            {
                t->m_maxPoint(D - 1) = std::max(t->m_maxPoint(D - 1), b.m_maxPoint(D - 1));
                t->m_minPoint(D - 1) = std::min(t->m_minPoint(D - 1), b.m_minPoint(D - 1));
                uniteBox_impl<D - 1>::apply(t, b);
            }
        };
        template<bool dummy>
        struct uniteBox_impl<0, dummy>
        {
            template<typename T, typename B>
            inline static void apply(T*, const B&)
            {
            }
        };

        template<unsigned int D = Dim, bool = true>
        struct reset_impl
        {
            template<typename T>
            inline static void apply(T* t)
            {
                t->m_minPoint(D - 1) = std::numeric_limits<PREC>::max();
                t->m_maxPoint(D - 1) = std::numeric_limits<PREC>::lowest();
                reset_impl<D - 1>::apply(t);
            }
        };

        template<bool dummy>
        struct reset_impl<0, dummy>
        {
            template<typename T>
            inline static void apply(T*)
            {
            }
        };

    public:
        AABB()
        {
            reset();
        }
        void reset()
        {
            reset_impl<>::apply(this);
        }

        AABB(const VectorStat<Dim>& p)
            : m_minPoint(p), m_maxPoint(p)
        {
        }

        AABB(const VectorStat<Dim>& l, const VectorStat<Dim>& u)
            : m_minPoint(l), m_maxPoint(u)
        {
            ApproxMVBB_ASSERTMSG(
                (m_maxPoint.array() >= m_minPoint.array()).all(),
                "AABB initialized wrongly! min/max: " << m_minPoint.transpose() << "/" << m_maxPoint.transpose());
        }

        template<typename Derived>
        AABB& unite(const MatrixBase<Derived>& p)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, Dim);
            unite_impl<>::apply(this, p);
            return *this;
        }

        template<typename Derived>
        AABB& operator+=(const MatrixBase<Derived>& p)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, Dim);
            unite_impl<>::apply(this, p);
            return *this;
        }

        void unite(const AABB& box)
        {
            uniteBox_impl<>::apply(this, box);
        }

        AABB& operator+=(const AABB& box)
        {
            uniteBox_impl<>::apply(this, box);
            return *this;
        }

        AABB operator+(const AABB& box)
        {
            AABB r = this;
            uniteBox_impl<>::apply(&r, box);
            return r;
        }

        AABB& transform(const AffineTrafo& M)
        {
            ApproxMVBB_STATIC_ASSERTM(Dim == 3, "So far AABB transform is only implemented in 3d");
            AABB ret(M * (Vector3(m_minPoint(0), m_minPoint(1), m_minPoint(2))));
            ret.unite(M * (Vector3(m_maxPoint(0), m_minPoint(1), m_minPoint(2))));
            ret.unite(M * (Vector3(m_minPoint(0), m_maxPoint(1), m_minPoint(2))));
            ret.unite(M * (Vector3(m_minPoint(0), m_minPoint(1), m_maxPoint(2))));
            ret.unite(M * (Vector3(m_minPoint(0), m_maxPoint(1), m_maxPoint(2))));
            ret.unite(M * (Vector3(m_maxPoint(0), m_maxPoint(1), m_minPoint(2))));
            ret.unite(M * (Vector3(m_maxPoint(0), m_minPoint(1), m_maxPoint(2))));
            ret.unite(M * (Vector3(m_maxPoint(0), m_maxPoint(1), m_maxPoint(2))));
            *this = ret;
            return *this;
        }

        inline VectorStat<Dim> center()
        {
            return 0.5 * (m_maxPoint + m_minPoint);
        }

        inline bool overlaps(const AABB& box) const
        {
            return ((m_maxPoint.array() >= box.m_minPoint.array()) && (m_minPoint.array() <= box.m_maxPoint.array())).all();
        }

        template<typename Derived>
        inline bool overlaps(const MatrixBase<Derived>& p) const
        {
            return ((p.array() >= m_minPoint.array()) && (p.array() <= m_maxPoint.array())).all();
        }

        inline bool overlapsSubSpace(const AABB& box, unsigned int fixedAxis) const
        {
            MyMatrix::ArrayStat<bool, Dim> t =
                ((m_maxPoint.array() >= box.m_minPoint.array()) && (m_minPoint.array() <= box.m_maxPoint.array()));
            t(fixedAxis) = true;
            return t.all();
        }

        inline ArrayStat<Dim> extent() const
        {
            // since min <= max, extent can not be smaller than zero
            // , except if AABB contains no points/uninitialized (reset())
            return (m_maxPoint - m_minPoint).array();
        }

        inline PREC maxExtent() const
        {
            return (m_maxPoint - m_minPoint).maxCoeff();
        }

        inline bool isEmpty() const
        {
            return (m_maxPoint.array() <= m_minPoint.array()).any();
        }

        inline void expand(PREC d)
        {
            ApproxMVBB_ASSERTMSG(d >= 0, "d>=0") m_minPoint.array() -= d;
            m_maxPoint.array() += d;
        }

        inline void expand(VectorStat<Dim> d)
        {
            ApproxMVBB_ASSERTMSG((d.array() >= 0).all(), "d<0") m_minPoint -= d;
            m_maxPoint += d;
        }

        /** Adjust box that all axes have at least a minimal extent of maxExtent*p, if
     * maxExtent*p < eps then all axes to default extent */
        void expandToMinExtentRelative(PREC p, PREC defaultExtent, PREC eps)
        {
            ArrayStat<Dim> e  = extent();
            VectorStat<Dim> c = center();
            typename ArrayStat<Dim>::Index idx;
            PREC ext = std::abs(e.maxCoeff(&idx)) * p;

            if(ext < eps)
            {  // extent of max axis almost zero, set all axis to
                // defaultExtent --> cube
                ext        = defaultExtent;
                m_minPoint = c - 0.5 * ext;
                m_maxPoint = c + 0.5 * ext;
            }
            else
            {
                for(int i = 0; i < Dim; ++i)
                {
                    if(i != idx && std::abs(e(i)) < ext)
                    {
                        m_minPoint(i) = c(i) - 0.5 * ext;
                        m_maxPoint(i) = c(i) + 0.5 * ext;
                    }
                }
            }
        }

        /** Adjust box that all axes have at least a minimal extent  minExtent*/
        void expandToMinExtentAbsolute(PREC minExtent)
        {
            Array3 e  = extent();
            Vector3 c = center();

            PREC l = 0.5 * minExtent;
            for(int i = 0; i < Dim; ++i)
            {
                if(std::abs(e(i)) < minExtent)
                {
                    m_minPoint(i) = c(i) - l;
                    m_maxPoint(i) = c(i) + l;
                }
            }
        }

        /** Adjust box that all axes have at least a minimal extent  minExtent for
     * each axis*/
        void expandToMinExtentAbsolute(ArrayStat<Dim> minExtent)
        {
            Array3 e  = extent();
            Vector3 c = center();

            for(unsigned int i = 0; i < Dim; ++i)
            {
                PREC l = minExtent(i);
                if(std::abs(e(i)) < l)
                {
                    m_minPoint(i) = c(i) - 0.5 * l;
                    m_maxPoint(i) = c(i) + 0.5 * l;
                }
            }
        }

        /** Expands the selected axes \p axis to maximal value,
     *  which simulates a box with infinite extent in this direction
     *  \tparam MoveMax If true, the maximum value is moved to max value, otherwise
     * the minimum value is moved to lowest.
     */
        template<bool MoveMax>
        void expandToMaxExtent(const unsigned int& axis)
        {
            ApproxMVBB_ASSERTMSG(axis < Dim, "axis >= Dim !");
            if(!MoveMax)
            {
                m_minPoint(axis) = std::numeric_limits<PREC>::lowest();
            }
            else
            {
                m_maxPoint(axis) = std::numeric_limits<PREC>::max();
            }
        }

        void expandToMaxExtent()
        {
            m_minPoint.setConstant(std::numeric_limits<PREC>::lowest());
            m_maxPoint.setConstant(std::numeric_limits<PREC>::max());
        }

        inline PREC volume() const
        {
            return (m_maxPoint - m_minPoint).prod();
        }

        // info about axis aligned bounding box
        VectorStat<Dim> m_minPoint;
        VectorStat<Dim> m_maxPoint;
    };

    using AABB3d = AABB<3>;
    using AABB2d = AABB<2>;
}  // namespace ApproxMVBB

#endif
