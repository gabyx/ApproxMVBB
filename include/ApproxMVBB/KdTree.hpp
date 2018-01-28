// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_KdTree_hpp
#define ApproxMVBB_KdTree_hpp

#include <algorithm>
#include <array>
#include <deque>
#include <fstream>
#include <initializer_list>
#include <list>
#include <memory>
#include <meta/meta.hpp>
#include <queue>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include ApproxMVBB_AssertionDebug_INCLUDE_FILE
#include "ApproxMVBB/Common/ContainerTag.hpp"
#include "ApproxMVBB/Common/SfinaeMacros.hpp"
#include "ApproxMVBB/Common/StaticAssert.hpp"

#include "ApproxMVBB/AABB.hpp"

namespace ApproxMVBB
{
    namespace KdTree
    {
        ApproxMVBB_DEFINE_MATRIX_TYPES;
        ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

        namespace details
        {
            struct TakeDefault;

            template<typename T>
            using isDefault = meta::or_<meta::_t<std::is_same<T, TakeDefault>>, meta::_t<std::is_same<T, void>>>;
        }  // namespace details

#define DEFINE_KDTREE_BASETYPES(__Traits__)                                    \
    /* NodeDataType, Dimension and NodeType */                                 \
    using NodeDataType                  = typename __Traits__::NodeDataType;   \
    static const unsigned int Dimension = __Traits__::NodeDataType::Dimension; \
    using NodeType                      = typename __Traits__::NodeType;       \
    /*Containers*/                                                             \
    using NodeContainerType    = std::vector<NodeType*>;                       \
    using LeafContainerType    = NodeContainerType;                            \
    using LeafNeighbourMapType = std::unordered_map<std::size_t, std::unordered_set<std::size_t>>;

        struct EuclideanDistSq
        {
            template<typename Derived>
            static typename Derived::Scalar apply(const MatrixBase<Derived>& p)
            {
                return p.squaredNorm();
            }
        };

        template<typename TPoint, typename TPointGetter, typename DistSq = EuclideanDistSq>
        struct DistanceComp
        {
            DistanceComp()
            {
                m_ref.setZero();
            }
            template<typename T>
            DistanceComp(const T& ref)
                : m_ref(ref)
            {
            }

            template<typename T>
            inline bool operator()(const T& p1, const T& p2)
            {
                return DistSq::apply(m_ref - TPointGetter::get(p1)) < DistSq::apply(m_ref - TPointGetter::get(p2));
            }

            template<typename T>
            inline PREC operator()(const T& p1)
            {
                return DistSq::apply(m_ref - TPointGetter::get(p1));
            }

            TPoint m_ref;
        };

        /** The default distance comperator traits*/
        template<typename TPoint, typename TPointGetter>
        struct DefaultDistanceCompTraits
        {
            template<typename TDistSq>
            using DistanceComp = DistanceComp<TPoint, TPointGetter>;
        };

        /** The default point data traits*/
        template<unsigned int Dim             = 3,
                 typename TPoint              = Vector3,
                 typename TValue              = Vector3*,
                 typename TPointGetter        = details::TakeDefault, /* decides on TValue type */
                 typename TDistanceCompTraits = details::TakeDefault  /* decides on TPoint,TPointGetter */
                 >
        class DefaultPointDataTraits
        {
        public:
            static const unsigned int Dimension = Dim;
            using PointType                     = TPoint;
            using PointListType                 = StdVecAligned<TValue>;

            using iterator       = typename PointListType::iterator;
            using const_iterator = typename PointListType::const_iterator;

        private:
            /* Standart Point Getter (T = value_type no ptr) */
            template<typename T>
            struct PointGetterImpl
            {
                inline static PointType& get(T& t)
                {
                    return t;
                }
                inline static const PointType& get(const T& t)
                {
                    return t;
                }
            };

            template<typename TT>
            struct PointGetterImpl<TT*>
            {
                inline static PointType& get(TT* t)
                {
                    return *t;
                }
                inline static const PointType& get(const TT* t)
                {
                    return *t;
                }
            };

        public:
            /** The getter which turns a value_type into the PointType */
            using PointGetter =
                meta::if_<details::isDefault<TPointGetter>, PointGetterImpl<typename PointListType::value_type>, TPointGetter>;

            /** The distance comperator traits */
            using DistCompTraits = meta::if_<details::isDefault<TDistanceCompTraits>,
                                             DefaultDistanceCompTraits<PointType, PointGetter>,
                                             TDistanceCompTraits>;
        };

        /** Standart class for the node data type in the Tree */
        template<typename TTraits = DefaultPointDataTraits<>>
        class PointData
        {
        public:
            using Traits                        = TTraits;
            static const unsigned int Dimension = Traits::Dimension;
            using PointType                     = typename Traits::PointType;
            using PointListType                 = typename Traits::PointListType; /** linear in memory!*/
            using iterator                      = typename Traits::iterator;
            using const_iterator                = typename Traits::const_iterator;
            using PointGetter                   = typename Traits::PointGetter;

            template<typename TDistSq>
            using DistanceComp = typename Traits::DistCompTraits::template DistanceComp<TDistSq>;

            /** Constructor for the root node, which owns the pointer list
     *  All child nodes don't set m_points internally
     *  \p points can be nullptr, such that kdTree is not responsible for the
     * points!
     */
            PointData(iterator begin, iterator end, std::unique_ptr<PointListType> points = nullptr)
                : m_begin(begin), m_end(end), m_points(points.release())
            {
            }

            ~PointData()
            {
                if(m_points)
                {
                    delete m_points;
                }
            }
            /** Splits the data into two new node datas if the split heuristics wants a
     * split */
            std::pair<PointData*, PointData*> split(iterator splitRightIt) const
            {
                // make left
                PointData* left = new PointData(m_begin, splitRightIt);
                // make right
                PointData* right = new PointData(splitRightIt, m_end);
                return std::make_pair(left, right);
            }

            PREC getGeometricMean(unsigned int axis)
            {
                PREC ret = 0.0;

                for(auto& pPoint : *this)
                {
                    ret += PointGetter::get(pPoint)(axis);
                }
                ret /= size();
                return ret;
            }

            iterator begin()
            {
                return m_begin;
            }

            iterator end()
            {
                return m_end;
            }

            const_iterator begin() const
            {
                return m_begin;
            }

            const_iterator end() const
            {
                return m_end;
            }

            std::size_t size() const
            {
                return std::distance(m_begin, m_end);
            }

            std::string getPointString()
            {
                std::stringstream ss;
                for(auto& pPoint : *this)
                {
                    ss << PointGetter::get(pPoint).transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
                }
                return ss.str();
            }

            friend class XML;

        private:
            iterator m_begin, m_end;  ///< The actual range of m_points which this node contains

            /** Owned pointer to the points which is deleted in Ctor (if not nullptr) */
            PointListType* m_points = nullptr;
        };

        /** Quality evaluator for the split heuristic
 *   s = split ratio is between (0,0.5]
 *   p = point ratio is between [0,0.5]
 *   e = extent ratio is between (0,1]
 *   ws,wp,we are weightings between [0,1] for the following linear criteria:
 *   J(s,p,e) = ws*2*s + wp*2*p + we*e which is the return value of compute()
 *   and maximized by the kd-Tree building procedure!
 */
        class LinearQualityEvaluator
        {
        public:
            /**
     *   By default ws=0, wp=0, we=1.0 which maximizes the extent ratio which
     *   is sensfull for midpoint splitting!
     */
            LinearQualityEvaluator(PREC ws = 0.0, PREC wp = 0.0, PREC we = 1.0)
                : m_weightSplitRatio(ws), m_weightPointRatio(wp), m_weightMinMaxExtentRatio(we)
            {
            }

            inline void setLevel(unsigned int level)
            {
                /* set parameters for tree level dependent (here not used)*/
            }

            inline PREC compute(const PREC& splitRatio, const PREC& pointRatio, const PREC& minMaxExtentRatio)
            {
                return m_weightSplitRatio * 2.0 * splitRatio + m_weightPointRatio * 2.0 * pointRatio +
                       m_weightMinMaxExtentRatio * minMaxExtentRatio;
            }

        private:
            /** Quality weights */
            PREC m_weightSplitRatio        = 0.0;
            PREC m_weightPointRatio        = 0.0;
            PREC m_weightMinMaxExtentRatio = 1.0;
        };

        template<typename Traits>
        class SplitHeuristic;

        template<typename TQualityEvaluator, typename Traits>
        class SplitHeuristicPointData
        {
        public:
            DEFINE_KDTREE_BASETYPES(Traits)

            using PointDataType = NodeDataType;
            using PointListType = typename NodeDataType::PointListType;
            using PointType     = typename PointDataType::PointType;
            using iterator      = typename PointDataType::iterator;
            using PointGetter   = typename PointDataType::PointGetter;

            enum class Method : char
            {
                MIDPOINT,
                MEDIAN,
                GEOMETRIC_MEAN
            };

            /** Search Criterias how to find the best split
            * axis which fullfills the constraints, minExtent etc.
            FIND_BEST has not been implemented so far, because we shuffle the point
            array constantly while searching!
            */
            enum class SearchCriteria : char
            {
                FIND_FIRST,
                FIND_BEST
            };

            using QualityEvaluator = TQualityEvaluator;

            using SplitAxisType = typename NodeType::SplitAxisType;

            SplitHeuristicPointData()
                : m_methods({Method::MIDPOINT}), m_searchCriteria(SearchCriteria::FIND_BEST)
            {
                for(SplitAxisType i = 0; i < static_cast<SplitAxisType>(Dimension); i++)
                {
                    m_splitAxes.push_back(i);
                }
                resetStatistics();
            }

            void init(const std::initializer_list<Method>& m,
                      unsigned int allowSplitAboveNPoints = 100,
                      PREC minExtent                      = 0.0,
                      SearchCriteria searchCriteria       = SearchCriteria::FIND_BEST,
                      const QualityEvaluator& qualityEval = QualityEvaluator(),
                      PREC minSplitRatio                  = 0.0,
                      PREC minPointRatio                  = 0.0,
                      PREC minExtentRatio                 = 0.0)
            {
                m_methods = m;

                if(m_methods.size() == 0)
                {
                    ApproxMVBB_ERRORMSG("No methods for splitting given!")
                }
                m_allowSplitAboveNPoints = allowSplitAboveNPoints;
                m_minExtent              = minExtent;
                if(m_minExtent < 0)
                {
                    ApproxMVBB_ERRORMSG("Minimal extent has wrong value!")
                }
                m_searchCriteria = searchCriteria;

                m_qualityEval = qualityEval;

                m_minSplitRatio  = minSplitRatio;
                m_minPointRatio  = minPointRatio;
                m_minExtentRatio = minExtentRatio;

                if(m_minSplitRatio < 0 || m_minPointRatio < 0 || m_minExtentRatio < 0)
                {
                    ApproxMVBB_ERRORMSG("Minimal split ratio, point ratio or extent ratio have <0 values!");
                }

                resetStatistics();
            }

            void resetStatistics()
            {
                m_splitCalls = 0;
                m_tries      = 0;
                m_splits     = 0;

                m_avgSplitRatio  = 0.0;
                m_avgPointRatio  = 0.0;
                m_avgExtentRatio = 0.0;
            }

            std::string getStatisticsString()
            {
                std::stringstream s;
                s << "\t splits            : " << m_splits << "\n\t avg. split ratio  (0,0.5] : " << m_avgSplitRatio / m_splits
                  << "\n\t avg. point ratio  [0,0.5] : " << m_avgPointRatio / m_splits
                  << "\n\t avg. extent ratio (0,1]   : " << m_avgExtentRatio / m_splits
                  << "\n\t tries / calls     : " << m_tries << "/" << m_splitCalls << " = " << (PREC)m_tries / m_splitCalls;
                return s.str();
            }

            /** Compute the split with the set heuristic and return the two new
             *   node data types if a split happened otherwise (nullptr)
             */
            std::pair<NodeDataType*, NodeDataType*> doSplit(NodeType* node, SplitAxisType& splitAxis, PREC& splitPosition)
            {
                ++m_splitCalls;

                auto* data = node->data();

                // No split for to little points or extent to small!
                if(data->size() <= m_allowSplitAboveNPoints)
                {
                    return std::make_pair(nullptr, nullptr);
                }

                // Sort split axes according to biggest extent
                m_extent = node->aabb().extent();

                auto biggest = [&](const SplitAxisType& a, const SplitAxisType& b) { return m_extent(a) > m_extent(b); };
                std::sort(m_splitAxes.begin(), m_splitAxes.end(), biggest);

                // Cycle through each method and check each axis
                unsigned int tries = 0;

                std::size_t nMethods  = m_methods.size();
                std::size_t methodIdx = 0;
                std::size_t axisIdx   = 0;
                m_found               = false;
                m_bestQuality         = std::numeric_limits<PREC>::lowest();
                // std::cout << "start: " << std::endl;
                while(((m_searchCriteria == SearchCriteria::FIND_FIRST && !m_found) ||
                       m_searchCriteria == SearchCriteria::FIND_BEST) &&
                      methodIdx < nMethods)
                {
                    // std::cout << "method " << methodIdx<<std::endl;
                    m_wasLastTry = false;
                    m_method     = m_methods[methodIdx];
                    m_splitAxis  = m_splitAxes[axisIdx];

                    // skip all axes which are smaller or equal than 2* min Extent
                    // -> if we would split, left or right would be smaller or equal to
                    // min_extent!
                    if(m_extent(m_splitAxis) > 2.0 * m_minExtent)
                    {
                        // Check split
                        if(computeSplitPosition(node))
                        {
                            // check all min ratio values (hard cutoff)
                            if(m_splitRatio >= m_minSplitRatio && m_pointRatio >= m_minPointRatio &&
                               m_extentRatio >= m_minExtentRatio)
                            {
                                //  if quality is better, update
                                updateSolution();
                                // std::cout << "Axis: " << axisIdx << "," << int(m_splitAxis)<< "
                                // q: " << m_quality << std::endl;
                                m_found = true;
                            }
                        }

                        ++tries;
                    }
                    else
                    {
                        // std::cout << "cannot split -> extent " << std::endl;
                    }

                    // next axis
                    if(++axisIdx == Dimension)
                    {
                        axisIdx = 0;
                        ++methodIdx;
                    }
                }
                m_tries += tries;

                if(m_found)
                {
                    // take best values
                    // finalize sorting (for all except (lastTry true and method was median) )
                    if(!(m_wasLastTry && m_bestMethod == Method::MEDIAN))
                    {
                        switch(m_bestMethod)
                        {
                            case Method::GEOMETRIC_MEAN:
                            case Method::MIDPOINT:
                            {
                                auto leftPredicate = [&](const typename PointListType::value_type& a) {
                                    return PointGetter::get(a)(m_bestSplitAxis) < m_bestSplitPosition;
                                };
                                m_bestSplitRightIt = std::partition(data->begin(), data->end(), leftPredicate);
                                break;
                            }
                            case Method::MEDIAN:
                            {
                                auto less = [&](const typename PointListType::value_type& a,
                                                const typename PointListType::value_type& b) {
                                    return PointGetter::get(a)(m_bestSplitAxis) < PointGetter::get(b)(m_bestSplitAxis);
                                };
                                std::nth_element(data->begin(), m_bestSplitRightIt, data->end(), less);
                                m_bestSplitPosition = PointGetter::get(*(m_bestSplitRightIt))(m_bestSplitAxis);
                                auto leftPredicate  = [&](const typename PointListType::value_type& a) {
                                    return PointGetter::get(a)(m_bestSplitAxis) < m_bestSplitPosition;
                                };
                                m_bestSplitRightIt = std::partition(data->begin(), m_bestSplitRightIt, leftPredicate);
                                break;
                            }
                        }
                    }

                    computeStatistics();

                    // Finally set the position and axis and return
                    splitAxis     = m_bestSplitAxis;
                    splitPosition = m_bestSplitPosition;
                    return data->split(m_bestSplitRightIt);
                }

                return std::make_pair(nullptr, nullptr);
            };

        private:
            /** Compute the split position and maybe the splitAxis */
            bool computeSplitPosition(NodeType* node, unsigned int tries = 1)
            {
                auto* data = node->data();
                ApproxMVBB_ASSERTMSG(data, "Node @  " << node << " has no data!")

                    auto& aabb = node->aabb();
                switch(m_method)
                {
                    case Method::MIDPOINT:
                    {
                        m_splitPosition = 0.5 * (aabb.m_minPoint(m_splitAxis) + aabb.m_maxPoint(m_splitAxis));

                        if(!checkPosition(aabb))
                        {
                            return false;
                        }
                        // Compute bodyRatio/splitRatio and quality
                        m_splitRatio  = 0.5;
                        m_extentRatio = computeExtentRatio(aabb);
                        m_pointRatio  = computePointRatio(data);
                        m_quality     = m_qualityEval.compute(m_splitRatio, m_pointRatio, m_extentRatio);

                        break;
                    }
                    case Method::MEDIAN:
                    {
                        auto beg       = data->begin();
                        m_splitRightIt = beg + data->size() / 2;  // split position is the median!

                        auto less = [&](const typename PointListType::value_type& a,
                                        const typename PointListType::value_type& b) {
                            return PointGetter::get(a)(m_splitAxis) < PointGetter::get(b)(m_splitAxis);
                        };
                        // [5, 5, 1, 5, 6, 7, 9, 5] example for [beg ... end]
                        // partition such that:  left points(m_splitAxis) <= nth element
                        // (splitRightIt) <= right points(m_splitAxis)
                        std::nth_element(beg, m_splitRightIt, data->end(), less);
                        m_splitPosition =
                            PointGetter::get(*(m_splitRightIt))(m_splitAxis);  // TODO make transform iterator to avoid
                                                                               // PointGetter::geterence here!

                        if(!checkPosition(aabb))
                        {
                            return false;
                        }

                        // e.g [ 5 5 5 1 5 5 6 9 7 ]
                        //               ^median, splitPosition = 5;

                        // move left points which are equal to nth element to the right!

                        auto leftPredicate = [&](const typename PointListType::value_type& a) {
                            return PointGetter::get(a)(m_splitAxis) < m_splitPosition;
                        };
                        m_splitRightIt = std::partition(beg, m_splitRightIt, leftPredicate);
                        // it could happen that the list now looks [1 5 5 5 5 5 6 9 7]
                        //                                            ^splitRightIt

                        // Compute bodyRatio/splitRatio and quality
                        m_splitRatio  = computeSplitRatio(aabb);
                        m_extentRatio = computeExtentRatio(aabb);
                        m_pointRatio  = (PREC)std::distance(beg, m_splitRightIt) / std::distance(beg, data->end());
                        // normally 0.5 but we shift all equal points to the right (so this
                        // changes!)
                        m_quality = m_qualityEval.compute(m_splitRatio, m_pointRatio, m_extentRatio);

                        // to avoid this check if in tolerance (additional points < 5%) if not
                        // choose other axis and try again!
                        //            bool notInTolerance = (data->size()/2 -
                        //            std::distance(beg,splitRightIt))  >= 0.05 * data->size();

                        //            if( notInTolerance ) {
                        //                if(tries < NodeDataType::Dimension) {
                        //                    ++tries;
                        //                    m_splitAxis = (m_splitAxis + 1) %
                        //                    NodeDataType::Dimension;
                        //                    return
                        //                    computeSplitPosition(splitRightIt,node,tries);
                        //                }
                        //            }

                        break;
                    }
                    case Method::GEOMETRIC_MEAN:
                    {
                        m_splitPosition = data->getGeometricMean(m_splitAxis);
                        if(!checkPosition(aabb))
                        {
                            return false;
                        }
                        // Compute bodyRatio/splitRatio and quality
                        m_splitRatio  = computeSplitRatio(aabb);
                        m_extentRatio = computeExtentRatio(aabb);
                        m_pointRatio  = computePointRatio(data);
                        m_quality     = m_qualityEval.compute(m_splitRatio, m_pointRatio, m_extentRatio);
                        break;
                    }
                }
                return true;
            }

            inline bool checkPosition(AABB<Dimension>& aabb)
            {
                ApproxMVBB_ASSERTMSG(
                    m_splitPosition >= aabb.m_minPoint(m_splitAxis) && m_splitPosition <= aabb.m_maxPoint(m_splitAxis),
                    "split position wrong: " << m_splitPosition << " min: " << aabb.m_minPoint.transpose()
                                             << " max: " << aabb.m_maxPoint.transpose())

                    if((m_splitPosition - aabb.m_minPoint(m_splitAxis)) <= m_minExtent ||
                       (aabb.m_maxPoint(m_splitAxis) - m_splitPosition) <= m_minExtent)
                {
                    return false;
                }
                return true;
            }

            inline void updateSolution()
            {
                if(m_quality > m_bestQuality)
                {
                    m_wasLastTry = true;

                    m_bestMethod        = m_method;
                    m_bestQuality       = m_quality;
                    m_bestSplitRightIt  = m_splitRightIt;
                    m_bestSplitAxis     = m_splitAxis;
                    m_bestSplitPosition = m_splitPosition;

                    m_bestSplitRatio  = m_splitRatio;
                    m_bestPointRatio  = m_pointRatio;
                    m_bestExtentRatio = m_extentRatio;
                }
            }

            inline PREC computePointRatio(NodeDataType* data)
            {
                PREC n = 0.0;
                for(auto& p : *data)
                {
                    if(PointGetter::get(p)(m_splitAxis) < m_splitPosition)
                    {
                        n += 1.0;
                    }
                }
                n /= data->size();
                return (n > 0.5) ? 1.0 - n : n;
            }

            inline PREC computeSplitRatio(AABB<Dimension>& aabb)
            {
                PREC n = (m_splitPosition - aabb.m_minPoint(m_splitAxis)) /
                         (aabb.m_maxPoint(m_splitAxis) - aabb.m_minPoint(m_splitAxis));
                ApproxMVBB_ASSERTMSG(n > 0.0 && n <= 1.0, " split ratio negative!, somthing wrong with splitPosition: " << n);
                return (n > 0.5) ? 1.0 - n : n;
            }

            inline PREC computeExtentRatio(AABB<Dimension>& aabb)
            {
                // take the lowest min/max extent ratio
                static ArrayStat<Dimension> t;
                t       = m_extent;
                PREC tt = t(m_splitAxis);

                t(m_splitAxis) = m_splitPosition - aabb.m_minPoint(m_splitAxis);
                PREC r         = t.minCoeff() / t.maxCoeff();  // gives NaN if max == 0, cannot
                                                               // happen since initial aabb is
                                                               // feasible!

                t(m_splitAxis) = tt;  // restore
                t(m_splitAxis) = aabb.m_maxPoint(m_splitAxis) - m_splitPosition;
                r              = std::min(r, t.minCoeff() / t.maxCoeff());

                ApproxMVBB_ASSERTMSG(r > 0, "extent ratio <= 0!");
                return r;
            }

            inline void computeStatistics()
            {
                ++m_splits;
                m_avgSplitRatio += m_bestSplitRatio;
                m_avgPointRatio += m_bestPointRatio;
                m_avgExtentRatio += m_bestExtentRatio;
            }

            /** Statistics */
            unsigned int m_splitCalls = 0;
            unsigned int m_tries      = 0;
            unsigned int m_splits     = 0;
            PREC m_avgSplitRatio      = 0;
            PREC m_avgPointRatio      = 0;
            PREC m_avgExtentRatio     = 0;

            QualityEvaluator m_qualityEval;

            /** Temp. values */
            ArrayStat<Dimension> m_extent;  // current extent of the aabb
            iterator m_splitRightIt;
            PREC m_quality = 0.0;

            PREC m_splitRatio  = 0;  ///< ratio of the splitting plane in range (0,0.5)
            PREC m_pointRatio  = 0;  ///< ratio of points in left and right splittet box, in range [0,0.5]
            PREC m_extentRatio = 0;  ///< minimal ratio of min/max extent of the left and
                                     /// right splitted box, in range (0,1] ;

            /** Min values to allow a split*/
            PREC m_minSplitRatio  = 0.0;
            PREC m_minPointRatio  = 0.0;
            PREC m_minExtentRatio = 0.0;

            Method m_method;
            PREC m_splitPosition = 0.0;
            SplitAxisType m_splitAxis;

            /** Best values */
            bool m_wasLastTry = false;  ///< Flag which tells if the best values have just
                                        /// been updated when we finished the opt. loop
            bool m_found = false;
            iterator m_bestSplitRightIt;
            PREC m_bestQuality    = std::numeric_limits<PREC>::lowest();
            PREC m_bestSplitRatio = 0, m_bestPointRatio = 0, m_bestExtentRatio = 0;
            Method m_bestMethod;
            PREC m_bestSplitPosition = 0.0;
            SplitAxisType m_bestSplitAxis;

            /** Fixed values */
            std::vector<SplitAxisType> m_splitAxes;
            std::vector<Method> m_methods;
            SearchCriteria m_searchCriteria;

            std::size_t m_allowSplitAboveNPoints = 100;
            PREC m_minExtent                     = 0.0;
        };

        /** Forward declar all tree classes */
        template<typename T>
        class TreeBase;
        template<typename T>
        class TreeSimple;
        template<typename T>
        class Tree;

#define DEFINE_KDTREE_BASENODETYPES(_Base_)                  \
    using SplitAxisType    = typename _Base_::SplitAxisType; \
    using BoundaryInfoType = typename _Base_::BoundaryInfoType;

        /** Node stuff
          * =======================================================================================*/

        template<typename TDerivedNode, unsigned int Dimension>
        class NodeBase
        {
        private:
            ApproxMVBB_STATIC_ASSERT(Dimension <= std::numeric_limits<char>::max());

            template<typename D, unsigned int Dim>
            friend class NodeBase;  ///< all classes are friends (even the ones with
                                    /// other dimension, which results in compilation
            /// error

        public:
            using DerivedNode   = TDerivedNode;
            using SplitAxisType = char;

        private:
            class BoundaryInformation
            {
            public:
                static const unsigned int size = Dimension * 2;

                inline DerivedNode*& at(unsigned int idx)
                {
                    ApproxMVBB_ASSERTMSG(idx < size, "Index " << idx << "out of bound!");
                    return m_nodes[idx];
                }
                inline DerivedNode* const& at(unsigned int idx) const
                {
                    ApproxMVBB_ASSERTMSG(idx < size, "Index " << idx << "out of bound!");
                    return m_nodes[idx];
                }

                inline DerivedNode*& at(unsigned int axis, char minMax)
                {
                    ApproxMVBB_ASSERTMSG(minMax * Dimension + axis < size,
                                         "Index " << minMax * Dimension + axis << "out of bound!");
                    return m_nodes[minMax * Dimension + axis];
                }
                inline DerivedNode* const& at(unsigned int axis, char minMax) const
                {
                    ApproxMVBB_ASSERTMSG(minMax * Dimension + axis < size,
                                         "Index " << minMax * Dimension + axis << "out of bound!");
                    return m_nodes[minMax * Dimension + axis];
                }

                DerivedNode** begin()
                {
                    return m_nodes;
                }
                DerivedNode** end()
                {
                    return m_nodes + size;
                }

                DerivedNode* const* begin() const
                {
                    return m_nodes;
                }
                DerivedNode* const* end() const
                {
                    return m_nodes + size;
                }

            private:
                DerivedNode* m_nodes[size] = {nullptr};  ///< min/max pointers
            };

        public:
            using BoundaryInfoType = BoundaryInformation;

            NodeBase()
            {
            }

            NodeBase(std::size_t idx, const AABB<Dimension>& aabb, unsigned int treeLevel = 0)
                : m_idx(idx), m_treeLevel(treeLevel), m_aabb(aabb)
            {
            }

            ~NodeBase()
            {
            }

            /** Copy from node
             *   Childs are not deep copied (since the node does not own the childs)
             *   Values of the child pointers \p n are left uninitialized.
             *   The tree class is responsible for copying the childs accordingly.
             */
            template<typename Derived>
            NodeBase(const NodeBase<Derived, Dimension>& n)
                : m_idx(n.m_idx)
                , m_treeLevel(n.m_treeLevel)
                , m_aabb(n.m_aabb)
                , m_splitAxis(n.m_splitAxis)
                , m_splitPosition(n.m_splitPosition)
                , m_child{{nullptr, nullptr}}
                , m_parent{nullptr}
            {
            }
            /** Move from node */
            template<typename Derived>
            NodeBase(NodeBase<Derived, Dimension>&& n)
                : m_idx(n.m_idx)
                , m_treeLevel(n.m_treeLevel)
                , m_aabb(std::move(n.m_aabb))
                , m_splitAxis(n.m_splitAxis)
                , m_splitPosition(n.m_splitPosition)
                , m_child(n.child)
                , m_parent(n.parent)
            {
                n.m_parent = nullptr;
                n.m_child  = {{nullptr, nullptr}};
            }

            inline void setChilds(DerivedNode* r, DerivedNode* l)
            {
                m_child[0] = r;
                m_child[1] = l;
            }

            inline DerivedNode* parent()
            {
                return m_parent;
            }
            inline const DerivedNode* parent() const
            {
                return m_parent;
            }

            inline DerivedNode* leftNode()
            {
                return m_child[0];
            }
            inline const DerivedNode* leftNode() const
            {
                return m_child[0];
            }

            inline DerivedNode* rightNode()
            {
                return m_child[1];
            }
            inline const DerivedNode* rightNode() const
            {
                return m_child[1];
            }

            inline AABB<Dimension>& aabb()
            {
                return m_aabb;
            }
            inline const AABB<Dimension>& aabb() const
            {
                return m_aabb;
            }

            inline bool hasLeftChildren() const
            {
                return m_child[0];
            }
            inline bool hasChildren() const
            {
                return m_child[0] && m_child[1];
            }

            inline bool isLeaf() const
            {
                return (m_splitAxis == -1);
            }

            inline void setIdx(std::size_t i)
            {
                m_idx = i;
            }

            inline std::size_t getIdx() const
            {
                return m_idx;
            }

            inline void setSplitAxis(SplitAxisType splitAxis)
            {
                m_splitAxis = splitAxis;
            }
            inline void setSplitPosition(PREC splitPos)
            {
                m_splitPosition = splitPos;
            }

            inline SplitAxisType getSplitAxis() const
            {
                return m_splitAxis;
            }
            inline PREC getSplitPosition() const
            {
                return m_splitPosition;
            }
            inline PREC getSplitRatio() const
            {
                return (m_splitPosition - m_aabb.m_minPoint(m_splitAxis)) /
                       (m_aabb.m_maxPoint(m_splitAxis) - m_aabb.m_minPoint(m_splitAxis));
            }

            inline unsigned int getLevel() const
            {
                return m_treeLevel;
            }

            inline void setLevel(unsigned int l) const
            {
                m_treeLevel = l;
            }

        protected:
            NodeBase(std::size_t idx, const AABB<Dimension>& aabb, SplitAxisType axis, PREC splitPos)
                : m_idx(idx), m_aabb(aabb), m_splitAxis(axis), m_splitPosition(splitPos)
            {
            }

            std::size_t m_idx        = std::numeric_limits<std::size_t>::max();  ///< node index
            unsigned int m_treeLevel = 0;
            AABB<Dimension> m_aabb;
            SplitAxisType m_splitAxis = -1;  ///< -1 indicates leaf node
            PREC m_splitPosition      = 0.0;

            /** Child Nodes
     * The child nodes, these objects are not owned by this node
     * and if it is not a leaf, both pointers are valid!
     */
            std::array<DerivedNode*, 2> m_child{{nullptr, nullptr}};
            DerivedNode* m_parent = nullptr;
        };

        /** Forward declaration */
        template<typename Traits>
        class Node;

        namespace details
        {
            // Helper to select the correct Derived type
            template<typename PD, typename T>
            struct select
            {
                using type = PD;
            };

            template<typename T>
            struct select<void, T>
            {
                using type = T;
            };
        }  // namespace details

        template<typename TTraits, typename PD = void> /*PD = PossibleDerived */
        class NodeSimple : public NodeBase<typename details::select<PD, NodeSimple<TTraits, PD>>::type, TTraits::Dimension>
        {
        public:
            using Traits = TTraits;

            using Base        = NodeBase<typename details::select<PD, NodeSimple<TTraits, PD>>::type, TTraits::Dimension>;
            using DerivedType = typename details::select<PD, NodeSimple<TTraits, PD>>::type;

        protected:
            using Base::m_aabb;
            using Base::m_child;
            using Base::m_idx;
            using Base::m_splitAxis;
            using Base::m_splitPosition;

            template<typename T>
            friend class TreeBase;

            /** Boundary information which is empty for non-leaf nodes
             *   Pointer which point to the subtrees min/max for each dimension
             */
            typename Base::BoundaryInfoType m_bound;

        public:
            DEFINE_KDTREE_BASETYPES(Traits)
            DEFINE_KDTREE_BASENODETYPES(Base)

            NodeSimple()
            {
            }
            NodeSimple(std::size_t idx, const AABB<Dimension>& aabb)
                : Base(idx, aabb)
            {
            }
            NodeSimple(NodeSimple&& t)
                : Base(std::move(t))
            {
            }
            NodeSimple(const NodeSimple& t)
                : Base(t)
            {
            }

            /** Copy values from TreeNode<T>, only Base class does copy */
            template<typename T>
            explicit NodeSimple(const Node<T>& t)
                : Base(t)
            {
            }

            /**
             *   Setup node from some other node  \p t, with a node pointer list \p nodes
             * (continous index ordered)!
             */
            template<typename T, typename NodeVector>
            void setup(const Node<T>* t, const NodeVector& nodes)
            {
                ApproxMVBB_STATIC_ASSERT(Dimension == Node<T>::Dimension);

                // link left child
                Node<T>* c = t->m_child[0];
                if(c)
                {
                    m_child[0]           = nodes[c->getIdx()];
                    m_child[0]->m_parent = static_cast<DerivedType*>(this);
                }

                // link right child
                c = t->m_child[1];
                if(c)
                {
                    m_child[1]           = nodes[c->getIdx()];
                    m_child[1]->m_parent = static_cast<DerivedType*>(this);
                }

                // link boundary information if this (and of course t) is a leaf
                if(t->isLeaf())
                {
                    auto it = m_bound.begin();
                    for(const auto* bNode : t->m_bound)
                    {
                        if(bNode)
                        {  // if boundary node is valid
                            // find node idx in node list (access by index)
                            auto* p = nodes[bNode->getIdx()];
                            if(p == nullptr)
                            {
                                ApproxMVBB_ERRORMSG("Setup in node: " << this->getIdx() << " failed!");
                            }
                            *it = p;  // set boundary pointer to the found node pointer.
                        }
                        else
                        {
                            *it = nullptr;
                        }

                        ++it;
                    }
                }
            }

            const BoundaryInfoType& getBoundaries() const
            {
                return m_bound;
            }
            BoundaryInfoType& getBoundaries()
            {
                return m_bound;
            }
        };

        /** Default class used for NodeSimple and TreeSimpleTraits*/
        template<unsigned int Dim = 3>
        struct NoData
        {
            static const unsigned int Dimension = Dim;
        };

        template<typename TTraits>
        class Node : public NodeBase<Node<TTraits>, TTraits::Dimension>
        {
        public:
            using Traits = TTraits;
            using Base   = NodeBase<Node<TTraits>, TTraits::Dimension>;

        private:
            using Base::m_aabb;
            using Base::m_child;
            using Base::m_idx;
            using Base::m_splitAxis;
            using Base::m_splitPosition;

            /** Tree Access */
            template<typename T>
            friend class Tree;
            template<typename T>
            friend class TreeBase;

            /** Node Friends */
            template<typename T>
            friend class Node;
            template<typename T, typename PD>
            friend class NodeSimple;

        public:
            DEFINE_KDTREE_BASETYPES(Traits)
            DEFINE_KDTREE_BASENODETYPES(Base)

            Node(std::size_t idx, const AABB<Dimension>& aabb, NodeDataType* data, unsigned int treeLevel = 0)
                : Base(idx, aabb, treeLevel), m_data(data)
            {
            }

            ~Node()
            {
                cleanUp();
            }

            /** Copy from node
             *   childs are not deep copied (since the node does not own the childs)
             *   the child pointers have the same values as the node \p n.
             *   The tree class is responsible for copying the childs accordingly.
             */
            template<typename Traits>
            Node(const Node<Traits>& n)
                : Base(n)
            {
                if(n.m_data)
                {
                    m_data = new NodeDataType(*n.m_data);
                }
            }

            /** Move from node */
            Node(Node&& n)
                : Base(std::move(n)), m_bound(n.m_bound), m_data(n.m_data)
            {
                n.m_data = nullptr;
            }

            const BoundaryInfoType& getBoundaries() const
            {
                return m_bound;
            }
            BoundaryInfoType& getBoundaries()
            {
                return m_bound;
            }

            void setBoundaryInfo(const BoundaryInfoType& b)
            {
                m_bound = b;
            }

            inline NodeDataType* data()
            {
                return m_data;
            }
            inline const NodeDataType* data() const
            {
                return m_data;
            }

            /** Splits the node into two new nodes by the splitting position
             * The ownership of the left and right nodes is the caller of this function!
             * If \p startIdx, which specifies the start numbering for the left child ,
             * right child is startIdx+1,
             * is zero then we number according to a complete binary tree, left = 2*m_idx
             * +1 and  right 2*m_idx + 2
             */
            template<typename TSplitHeuristic>
            bool split(TSplitHeuristic& s, std::size_t startIdx = 0)
            {
                auto pLR = s.doSplit(this, m_splitAxis, m_splitPosition);

                if(pLR.first == nullptr)
                {  // split has failed!
                    return false;
                }

                // if startIdx for numbering is zero then number according to complete
                // binary tree
                startIdx = (startIdx == 0) ? 2 * m_idx + 1 : startIdx;

                // Split aabb and make left and right
                // left (idx for next child if tree is complete binary tree, then idx =
                // 2*idx+1 for left child)
                AABB<Dimension> t(m_aabb);
                PREC v                    = t.m_maxPoint(m_splitAxis);
                t.m_maxPoint(m_splitAxis) = m_splitPosition;
                m_child[0]                = new Node(startIdx, t, pLR.first, this->m_treeLevel + 1);
                m_child[0]->m_parent      = this;

                // right
                t.m_maxPoint(m_splitAxis) = v;  // restore
                t.m_minPoint(m_splitAxis) = m_splitPosition;
                m_child[1]                = new Node(startIdx + 1, t, pLR.second, this->m_treeLevel + 1);
                m_child[1]->m_parent      = this;

                // Set Boundary Information
                BoundaryInfoType b   = m_bound;  // copy
                Node* tn             = b.at(m_splitAxis, 1);
                b.at(m_splitAxis, 1) = m_child[1];  // left changes pointer at max value
                m_child[0]->setBoundaryInfo(b);

                b.at(m_splitAxis, 1) = tn;          // restore
                b.at(m_splitAxis, 0) = m_child[0];  // right changes pointer at min value
                m_child[1]->setBoundaryInfo(b);

                // clean up own node if it is not the root node on level 0
                if(this->m_treeLevel != 0)
                {
                    cleanUp();
                }

                return true;
            }

            template<typename NeighbourIdxMap>
            void getNeighbourLeafsIdx(NeighbourIdxMap& neigbourIdx, PREC minExtent)
            {
                // we determine in each direction of the kd-Tree in R^n (if 3D, 3
                // directions)
                // for "min" and "max" the corresponding leafs in the subtrees given by
                // the boundary information in the leaf node:
                // e.g. for the "max" direction in x for one leaf, we traverse the  subtree
                // of the boundary
                // information in "max" x direction
                // for "max" we always take the left node  (is the one which is closer to
                // our max x boundary)
                // for "min" we always take the right node (is the one which is closer to
                // our min x boundary)
                // in this way we get all candidate leaf nodes (which share the same split
                // axis with split position s)
                // which need still to be checked if they are neighbours
                // this is done by checking if the boundary subspace with the corresponding
                // axis set to the split position s
                // (space without the axis which is checked, e.g y-z space, with x = s)
                // against the current leaf nodes boundary subspace
                // (which is thickened up by the amount of the minimal leaf node extent
                // size,
                // important because its not clear what should count as a neighbour or what
                // not)
                // if this neighbour n subspace overlaps the thickened leaf node subspace
                // then this node is
                // considered as a neighbour for leaf l, (and also neighbour n has l as
                // neighbour obviously)
                // the tree should be build with a slightly bigger min_extent size than the
                // thickening in the step here!
                // to avoid to many nodes to be classified as neighbours (trivial example,
                // min_extent grid)
                // if we have no boundary information

                std::deque<Node*> nodes;                // Breath First Search
                auto& neighbours = neigbourIdx[m_idx];  // Get this neighbour map
                Node* f;

                AABB<Dimension> aabb(m_aabb);
                aabb.expand(minExtent);  // expand this nodes aabb such that we find all the
                                         // neighbours which overlap this aabb;

                // For each axis traverse subtree
                for(SplitAxisType d = 0; d < static_cast<SplitAxisType>(Dimension); ++d)
                {
                    // for min and max
                    for(unsigned int m = 0; m < 2; ++m)
                    {
                        // push first -> Breath First Search (of boundary subtree)
                        Node* subTreeRoot = m_bound.at(d, m);
                        if(!subTreeRoot)
                        {
                            continue;
                        }
                        else
                        {
                            nodes.emplace_back(subTreeRoot);
                        }

                        while(!nodes.empty())
                        {
                            // std::cout << nodes.size() << std::endl;
                            f = nodes.front();
                            ApproxMVBB_ASSERTMSG(f, "Node f is nullptr")

                                auto axis = f->m_splitAxis;
                            if(f->isLeaf())
                            {
                                // std::cerr << "is leaf" << std::endl;
                                // determine if f is a neighbour to this node
                                // if leaf is not already in neighbour map for this node (skip)
                                if(neighbours.find(f->m_idx) == neighbours.end())
                                {
                                    // check if the subspace (fixedAxis = m) overlaps
                                    if(aabb.overlapsSubSpace(f->m_aabb, d))
                                    {
                                        // std::cerr << m_idx << " overlaps" << f->getIdx() <<
                                        // std::endl;
                                        neighbours.emplace(f->m_idx);
                                        neigbourIdx[f->m_idx].emplace(m_idx);
                                    }
                                }
                            }
                            else
                            {
                                // if the node f currently processed has the same split axis as the
                                // boundary direction
                                // add only the nodes closer to the cell of this node
                                if(axis == d)
                                {
                                    if(m == 0)
                                    {
                                        // for minmal boundary only visit right nodes (closer to this)
                                        nodes.emplace_back(f->rightNode());
                                    }
                                    else
                                    {
                                        // for maximal boundary only visit left nodes (closer to this)
                                        nodes.emplace_back(f->leftNode());
                                    }
                                }
                                else
                                {
                                    // otherwise add all
                                    nodes.emplace_back(f->leftNode());
                                    nodes.emplace_back(f->rightNode());
                                }
                            }
                            nodes.pop_front();
                        }
                    }
                }
            }

            void cleanUp(bool data = true /*,bool bounds = true*/)
            {
                if(data && m_data)
                {
                    delete m_data;
                    m_data = nullptr;
                }
            }

            std::size_t size()
            {
                return (m_data) ? m_data->size() : 0;
            }

        private:
            NodeDataType* m_data = nullptr;

            /** Boundary information which is empty for non-leaf nodes
             *   Pointer which point to the subtrees min/max for each dimension
             */
            BoundaryInfoType m_bound;
        };
        /**
         * =======================================================================================*/

        /** Tree stuff
         * ============================================================================*/
        template<typename Traits>
        class TreeBase
        {
        private:
            template<typename T>
            friend class TreeBase;

        public:
            DEFINE_KDTREE_BASETYPES(Traits)

            TreeBase()
            {
            }

            TreeBase(TreeBase&& t)
                : m_nodes(std::move(t.m_nodes)), m_leafs(std::move(t.m_leafs)), m_root(t.m_root)
            {
                // Make other tree empty
                t.m_root = nullptr;
                t.m_nodes.clear();
                t.m_leafs.clear();
            }

            TreeBase(const TreeBase& t)
            {
                copyFrom(t);
            }

            template<typename T>
            explicit TreeBase(const TreeBase<T>& t)
            {
                copyFrom(t);
            }

        protected:
            /** Deep copy, copies all nodes, and leafs , and links childs together
             *   Special links added to the Node classes are not preserved!
             *   For example: BoundaryInformation
             *   Therefore a newNode->setup() routine is called for each new copied node at
             * the end of this function
             *   such that every newNode can setup these special links by accesing a
             * (index,newNode) map and the oldNode.
             */
            template<typename T>
            void copyFrom(const TreeBase<T>& tree)
            {
                // using CNodeType = typename TreeBase<T>::NodeType;

                if(!tree.m_root)
                {
                    return;
                }

                this->m_nodes.reserve(tree.m_nodes.size());
                this->m_leafs.reserve(tree.m_leafs.size());

                // copy all nodes
                for(auto* n : tree.m_nodes)
                {
                    ApproxMVBB_ASSERTMSG(n,
                                         "Node of tree to copy from is nullptr!") this->m_nodes.emplace_back(new NodeType(*n));
                }

                // setup all nodes
                auto s = tree.m_nodes.size();
                for(std::size_t i = 0; i < s; ++i)
                {
                    this->m_nodes[i]->setup(tree.m_nodes[i], this->m_nodes);
                }

                // setup leaf list
                for(auto* n : tree.m_leafs)
                {
                    // std::cerr << "Approx:: n->getIdx() " << n->getIdx() << std::endl;
                    ApproxMVBB_ASSERTMSG((n->getIdx() < this->m_nodes.size()),
                                         "Leaf node from source is out of range: "
                                             << n->getIdx() << ","
                                             << this->m_nodes.size()) this->m_leafs.emplace_back(this->m_nodes[n->getIdx()]);
                }

                // setup root
                ApproxMVBB_ASSERTMSG(tree.m_root->getIdx() < this->m_nodes.size(),
                                     "Root idx out of range: " << tree.m_root->getIdx() << "," << this->m_nodes.size()) m_root =
                    m_nodes[tree.m_root->getIdx()];
            }

            ~TreeBase()
            {
                resetTree();
            }  ///< Prohibit the use of this base polymophically

        public:
            /** Built a tree from a node map and links
             * \p c   is a associative container of nodes with type \tp NodeType where the
             * key type is std::size_t and
             * value type is a pointe to type NodeType. The tree owns the pointers
             * afterwards!
             * \p links is an associative container with type \tp NodeToChildMap
             * where the key is std::size_t and specifies the parent and the value type is
             * a std::pair<std::size_t,std::size_t>
             * for left and right child node indices in the map \p c.
             */
            template<typename NodeMap, typename NodeToChildMap>
            void build(NodeType* root, NodeMap& c, NodeToChildMap& links)
            {
                resetTree();
                m_leafs.clear();
                m_nodes.clear();
                m_nodes.reserve(c.size());
                m_nodes.assign(c.begin(), c.end());

                for(auto* n : m_nodes)
                {
                    if(n->isLeaf())
                    {
                        m_leafs.push_back(n);
                    }
                }

                if(c.find(root->getIdx()) == c.end())
                {
                    ApproxMVBB_ERRORMSG("Root node not in NodeMap!")
                }

                std::unordered_set<std::size_t> hasParent;
                // first link all nodes together
                auto itE = c.end();
                for(auto& l : links)
                {  // first idx, second pair<idxL,idxR>
                    auto it  = c.find(l.first);
                    auto itL = c.find(l.second.first);
                    auto itR = c.find(l.second.second);

                    if(it == itE || itL == itE || itR == itE)
                    {
                        ApproxMVBB_ERRORMSG("Link at node idx: " << l.first << " wrong!")
                    }

                    if(!hasParent.emplace(l.second.first).second)
                    {
                        ApproxMVBB_ERRORMSG("Node idx: " << l.second.first << "has already a parent!")
                    };
                    if(!hasParent.emplace(l.second.second).second)
                    {
                        ApproxMVBB_ERRORMSG("Node idx: " << l.second.first << "has already a parent!")
                    };
                    if(!it->second || !itL->second || !itR->second)
                    {
                        ApproxMVBB_ERRORMSG("Ptr for link zero")
                    }
                    it->second->m_child[0] = itL->second;  // link left
                    it->second->m_child[1] = itR->second;  // link right
                }

                if(hasParent.size() != c.size() - 1)
                {
                    ApproxMVBB_ERRORMSG("Tree needs to have N nodes, with one root, which gives N-1 parents!")
                }

                // Save root as it is a valid binary tree
                m_root = root;

                // Move over all nodes again an do some setup
            }

            void resetTree()
            {
                for(auto* n : this->m_nodes)
                {
                    delete n;
                }
                // root node is also in node list!
                this->m_root = nullptr;

                m_leafs.clear();
                m_nodes.clear();
            }

            /** Get cell index of the leaf which owns point \p point
             * \p point is the d-dimensional point in the frame of reference the kd-Tree
             * was built!
             * Points outside the roots AABB box, are naturally project to the most outer
             * leaf automatically.
             */
            template<typename Derived>
            const NodeType* getLeaf(const MatrixBase<Derived>& point) const
            {
                EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, Dimension);
                // Recursively traverse tree to find the leaf which contains the point
                ApproxMVBB_ASSERTMSG(m_root, "Tree is not built!") const NodeType* currentNode = m_root;

                while(!currentNode->isLeaf())
                {
                    // all points greater or equal to the splitPosition belong to the right
                    // node
                    if(point(currentNode->getSplitAxis()) >= currentNode->getSplitPosition())
                    {
                        currentNode = currentNode->rightNode();
                    }
                    else
                    {
                        currentNode = currentNode->leftNode();
                    }
                }
                return currentNode;
            }

            /** Get common ancestor of two nodes
             *  Complexity: O(h) algorithm
             */
            const NodeType* getLowestCommonAncestor(const NodeType* a, const NodeType* b)
            {
                // build list of parents to root

                ApproxMVBB_ASSERTMSG(a && b, "Input nodes are nullptr!")

                    static std::vector<NodeType*>
                        aL;                        // reserve treelevel nodes
                static std::vector<NodeType*> bL;  // reserve treelevel nodes
                aL.clear();
                bL.clear();

                // std::cerr << " List A: ";
                NodeType* r = a->m_parent;
                while(r != nullptr)
                {  // root has nullptr which stops
                    aL.emplace_back(r);
                    // std::cerr << r->getIdx() << ",";
                    r = r->m_parent;
                }
                // std::cerr << std::endl;

                // std::cerr << " List B: ";
                r = b->m_parent;
                while(r != nullptr)
                {
                    bL.emplace_back(r);
                    // std::cerr << r->getIdx() << ",";
                    r = r->m_parent;
                }
                // std::cerr << std::endl;

                // list a = [5,4,3,1]
                // list b = [9,13,11,2,3,1]
                // lowest common ancestor = 3
                // get the node from root (from back of both lists)
                // where it differs

                std::size_t sizeA = aL.size();
                std::size_t sizeB = bL.size();
                NodeType* ret     = nullptr;
                std::size_t s     = std::min(sizeA, sizeB);

                for(std::size_t i = 1; i <= s; ++i)
                {
                    if(aL[sizeA - i] == bL[sizeB - i])
                    {
                        ret = aL[sizeA - i];
                    }
                    else
                    {
                        break;
                    }
                }
                return ret;
            }

            inline const NodeType* getLeaf(const std::size_t& leafIndex) const
            {
                if(leafIndex < m_leafs.size())
                {
                    return m_leafs[leafIndex];
                }
                return nullptr;
            }

            inline const LeafContainerType& getLeafs()
            {
                return m_leafs;
            }

            inline const NodeType* getNode(const std::size_t& globalIndex) const
            {
                if(globalIndex < m_nodes.size())
                {
                    return m_nodes[globalIndex];
                }
                return nullptr;
            }

            inline const NodeContainerType& getNodes()
            {
                return m_nodes;
            }

            inline const NodeType* getRootNode()
            {
                return m_root;
            }

            /** Clean up the nodes.
             * Parameter are perfectly forwarded to NodeType::cleanUp(...)
             */
            template<typename... T>
            void cleanUp(T&&... t)
            {
                for(auto* p : m_nodes)
                {
                    p->cleanUp(std::forward<T>(t)...);
                }
            }

            std::tuple<std::size_t, std::size_t> getStatistics()
            {
                return std::make_tuple(m_nodes.size(), m_leafs.size());
            }

            /** Enumerate nodes (continously, leafs first, then non-leafs
             *  This is reflected in the list m_nodes too, which needs to correspond to
             * getIdx()!
             */
            void enumerateNodes()
            {
                std::size_t leafIdx = 0;

                this->m_leafs.clear();  // rebuild leafs list

                // reenumerate leaf nodes and isnert in leaf list
                for(auto* n : this->m_nodes)
                {
                    if(n->isLeaf())
                    {
                        n->m_idx = leafIdx++;
                        this->m_leafs.emplace_back(n);
                    }
                }
                std::size_t nonleafIdx = this->m_leafs.size();

                // reenumerate other nodes
                for(auto* n : this->m_nodes)
                {
                    if(!n->isLeaf())
                    {
                        n->m_idx = nonleafIdx++;
                    }
                }

                // Sort m_nodes (IMPORTANT such that m_nodes[getIdx()] gives the correct
                // node
                std::sort(m_nodes.begin(), m_nodes.end(), [](NodeType* a, NodeType* b) { return a->getIdx() < b->getIdx(); });
            }

            friend class XML;

        protected:
            NodeContainerType m_leafs;  ///< Only leaf nodes , continously index ordered:
                                        /// leafs[idx]->getIdx() < leafs[idx+1]->getIdx();
            NodeContainerType m_nodes;  ///< All nodes, continously index ordered, with
                                        /// first element = m_root

            NodeType* m_root = nullptr;  ///< Root node, has index 0!
        };
        /**
         *  =======================================================================================*/

        class TreeStatistics
        {
        public:
            TreeStatistics()
            {
                reset();
            }
            TreeStatistics(const TreeStatistics& s) = default;

            bool m_computedTreeStats;
            unsigned int m_treeDepth;
            PREC m_avgSplitPercentage;
            /** Min/Max Extent for Leafs */
            PREC m_minLeafExtent;
            PREC m_maxLeafExtent;
            /** Data Sizes for Leafs*/
            PREC m_avgLeafSize;
            std::size_t m_minLeafDataSize;
            std::size_t m_maxLeafDataSize;

            /** Leaf Neighbour stats */
            bool m_computedNeighbourStats;
            std::size_t m_minNeighbours;
            std::size_t m_maxNeighbours;
            PREC m_avgNeighbours;

            void reset()
            {
                m_computedTreeStats  = false;
                m_treeDepth          = 0;
                m_avgSplitPercentage = 0.0;
                m_minLeafExtent      = std::numeric_limits<PREC>::max();
                m_maxLeafExtent      = 0.0;
                m_avgLeafSize        = 0;
                m_minLeafDataSize    = std::numeric_limits<std::size_t>::max();
                m_maxLeafDataSize    = 0;

                m_computedNeighbourStats = false;
                m_minNeighbours          = std::numeric_limits<std::size_t>::max();
                m_maxNeighbours          = 0;
                m_avgNeighbours          = 0;
            }

            void average(std::size_t nodes, std::size_t leafs)
            {
                if(nodes)
                {
                    m_avgLeafSize /= nodes;
                }
                if(leafs)
                {
                    m_avgSplitPercentage /= nodes - leafs;
                }
            }

            template<typename TNode>
            void addNode(TNode* n)
            {
                if(n->isLeaf())
                {
                    auto s = n->data()->size();
                    m_avgLeafSize += s;
                    m_minLeafDataSize = std::min(m_minLeafDataSize, s);
                    m_maxLeafDataSize = std::max(m_maxLeafDataSize, s);
                    m_minLeafExtent   = std::min(m_minLeafExtent, n->aabb().extent().minCoeff());
                    m_maxLeafExtent   = std::max(m_maxLeafExtent, n->aabb().extent().maxCoeff());
                }
            }

            friend class XML;
        };

        /** Tree simple stuff
         * ============================================================================*/

        template<typename TNodeData = NoData<>, template<typename...> class TNode = NodeSimple>
        struct TreeSimpleTraits
        {
            struct BaseTraits
            {
                using NodeDataType                  = TNodeData;
                static const unsigned int Dimension = NodeDataType::Dimension;
                using NodeType                      = TNode<BaseTraits>;
            };
        };

        ///** Standart Class to build a kd-tree */
        template<typename TTraits = TreeSimpleTraits<>>
        class TreeSimple : public TreeBase<typename TTraits::BaseTraits>
        {
        public:
            using Traits     = TTraits;
            using BaseTraits = typename TTraits::BaseTraits;
            using Base       = TreeBase<typename TTraits::BaseTraits>;

            DEFINE_KDTREE_BASETYPES(BaseTraits)

            /** Standart constructor */
            TreeSimple()
            {
            }

            /** Move constructor */
            template<typename Traits>
            TreeSimple(TreeSimple<Traits>&& tree)
                : Base(std::move(tree)), m_statistics(std::move(tree.m_statistics))
            {
            }
            /** Copy the tree */
            template<typename Traits>
            TreeSimple(const TreeSimple<Traits>& tree)
                : Base(tree), m_statistics(tree.m_statistics)
            {
            }

            /** Copy from a Tree<Traits> with any kind of traits if possible
             * The underlying Traits::NodeType has a copy constructor for T::NodeType!
             */
            template<typename Traits>
            explicit TreeSimple(const Tree<Traits>& tree)
                : Base(tree), m_statistics(tree.m_statistics)
            {
            }

            ~TreeSimple()
            {
            }

            /** Returns tuple with values
             * (number of leafs, avg. leaf data size, min. leaf data size, max. leaf data
             * size)
             */
            std::tuple<std::size_t,
                       std::size_t,
                       std::size_t,
                       PREC,
                       std::size_t,
                       std::size_t,
                       PREC,
                       PREC,
                       std::size_t,
                       std::size_t,
                       PREC>
            getStatistics()
            {
                return std::tuple_cat(Base::getStatistics(),
                                      std::make_tuple(m_statistics.m_treeDepth,
                                                      m_statistics.m_avgLeafSize,
                                                      m_statistics.m_minLeafDataSize,
                                                      m_statistics.m_maxLeafDataSize,
                                                      m_statistics.m_minLeafExtent,
                                                      m_statistics.m_maxLeafExtent,
                                                      m_statistics.m_minNeighbours,
                                                      m_statistics.m_maxNeighbours,
                                                      m_statistics.m_avgNeighbours));
            }

            std::string getStatisticsString()
            {
                std::stringstream s;
                auto t = getStatistics();
                s << "Tree Stats: "
                  << "\n\t nodes      : " << std::get<0>(t) << "\n\t leafs      : " << std::get<1>(t)
                  << "\n\t tree level : " << std::get<2>(t) << "\n\t avg. leaf data size : " << std::get<3>(t)
                  << "\n\t min. leaf data size : " << std::get<4>(t) << "\n\t max. leaf data size : " << std::get<5>(t)
                  << "\n\t min. leaf extent    : " << std::get<6>(t) << "\n\t max. leaf extent    : " << std::get<7>(t)
                  << "\nNeighbour Stats (if computed): \n"
                  << "\n\t min. leaf neighbours    : " << std::get<8>(t) << "\n\t max. leaf neighbours    : " << std::get<9>(t)
                  << "\n\t avg. leaf neighbours    : " << std::get<10>(t) << std::endl;

                return s.str();
            }

            friend class XML;

        protected:
            using Base::m_leafs;
            using Base::m_nodes;
            using Base::m_root;

            TreeStatistics m_statistics;
        };

        /**
         * =======================================================================================*/

        /** Tree stuff
         * ============================================================================*/
        template<typename Traits>
        using SplitHeuristicPointDataDefault =
            meta::invoke<meta::bind_front<meta::quote<SplitHeuristicPointData>, LinearQualityEvaluator>, Traits>;

        template<typename TNodeData                          = PointData<>,
                 template<typename...> class TSplitHeuristic = SplitHeuristicPointDataDefault,
                 template<typename...> class TNode           = Node>
        struct TreeTraits
        {
            struct BaseTraits
            {
                using NodeDataType                  = TNodeData;
                static const unsigned int Dimension = NodeDataType::Dimension;
                using NodeType                      = TNode<BaseTraits>;
            };

            using SplitHeuristicType = TSplitHeuristic<BaseTraits>;
        };

        /** Standart Class to build a kd-tree */
        template<typename TTraits = TreeTraits<>>
        class Tree : public TreeBase<typename TTraits::BaseTraits>
        {
        private:
            template<typename T>
            friend class Tree;

            template<typename T>
            friend class TreeSimple;

        public:
            using Traits     = TTraits;
            using BaseTraits = typename TTraits::BaseTraits;
            using Base       = TreeBase<typename TTraits::BaseTraits>;
            DEFINE_KDTREE_BASETYPES(BaseTraits)

            using SplitHeuristicType = typename Traits::SplitHeuristicType;

            Tree()
            {
            }
            ~Tree()
            {
            }

            /** Move constructor */
            Tree(Tree&& tree)
                : Base(std::move(tree))
                , m_heuristic(std::move(tree.m_heuristic))
                , m_statistics(std::move(tree.m_statistics))
                , m_maxLeafs(tree.m_maxLeafs)
                , m_maxTreeDepth(tree.m_maxTreeDepth)
            {
                tree.resetStatistics();
            }

            /** Copies the tree */
            Tree(const Tree& tree)
                : Base(tree)
                , m_heuristic(tree.m_heuristic)
                , m_statistics(tree.m_statistics)
                , m_maxLeafs(tree.m_maxLeafs)
                , m_maxTreeDepth(tree.m_maxTreeDepth)
            {
            }
            /** Copies the tree with different traits */
            template<typename T>
            explicit Tree(const Tree<T>& tree)
                : Base(tree), m_statistics(tree.m_statistics), m_maxLeafs(tree.m_maxLeafs), m_maxTreeDepth(tree.m_maxTreeDepth)
            {
            }

            /** Copies the tree if the underlying NodeType has a function NodeType(const
             * TTree::NodeType & n)
             *  This tree needs to be a friend of TTree::NodeType to successfully copy the
             * nodes!
             */
            template<typename TTree>
            Tree(const TTree& tree)
                : Base(tree)
            {
            }

            Tree& operator=(const Tree& t) = delete;

            void resetTree()
            {
                resetStatistics();
                Base::resetTree();
            }

            /** Builds a new Tree with the SplitHeurstic
             *   First node in m_nodes is always root!
             *   All following nodes are in breath first order, and continuously numbered
             *   and m_nodes[node->getIdx()] == node (index in sync with the list)
             */
            template<bool computeStatistics = true>
            void build(const AABB<Dimension>& aabb,
                       std::unique_ptr<NodeDataType> data,
                       unsigned int maxTreeDepth = 50,
                       unsigned int maxLeafs     = std::numeric_limits<unsigned int>::max())
            {
                resetTree();

                m_statistics.m_computedTreeStats = computeStatistics;
                m_maxTreeDepth                   = maxTreeDepth;
                m_maxLeafs                       = maxLeafs;

                if((aabb.extent() <= 0.0).any())
                {
                    ApproxMVBB_ERRORMSG("AABB given has wrong extent!");
                }
                this->m_root = new NodeType(0, aabb, data.release());

                std::deque<NodeType*> splitList;  // Breath first splitting
                splitList.push_back(this->m_root);
                this->m_nodes.push_back(this->m_root);

                bool nodeSplitted;

                m_statistics.m_treeDepth     = 0;
                unsigned int nNodesLevelCurr = 1;  // number of nodes in list of current level
                unsigned int nNodesLevelNext = 0;  // number of nodes in list (after the current nodes with next level)

                unsigned int nLeafs = 1;
                //        unsigned int nodeIdx = 0; // root node has idx = 0;

                //            auto greaterData = [](const NodeType * a, const NodeType * b)
                //            {
                //                return a->data()->size() > b->data()->size() ;
                //            };

                while(!splitList.empty())
                {
                    auto* f = splitList.front();

                    // first check if number of nodes at curr level is zero
                    if(nNodesLevelCurr == 0)
                    {
                        // std::cout << "Tree Level: " << m_statistics.m_treeDepth << " done,
                        // added childs: " << nNodesLevelNext << std::endl;
                        ++m_statistics.m_treeDepth;

                        std::swap(nNodesLevelCurr, nNodesLevelNext);

                        // may be sort the child nodes in splitList according to their data size
                        // (slow for big trees)
                        // std::sort(splitList.begin(),splitList.end(),greaterData);

                        f = splitList.front();
                        // std::cout << "biggest leaf size: " <<
                        // splitList.front()->data()->size() << std::endl;
                    }

                    if(m_statistics.m_treeDepth + 1 <= m_maxTreeDepth && nLeafs < m_maxLeafs)
                    {
                        // try to split the nodes in the  list (number continuously!)
                        nodeSplitted = f->split(m_heuristic, this->m_nodes.size());
                        if(nodeSplitted)
                        {
                            auto* l = f->leftNode();
                            auto* r = f->rightNode();
                            splitList.emplace_back(l);  // add to front
                            splitList.emplace_back(r);  // add to front

                            // Push to total list
                            this->m_nodes.emplace_back(l);
                            this->m_nodes.emplace_back(r);

                            nNodesLevelNext += 2;
                            ++nLeafs;  // each split adds one leaf
                        }
                        else
                        {
                            // this is a leaf
                            this->m_leafs.emplace_back(f);
                        }

                        // add to node statistic:
                        if(computeStatistics)
                        {
                            m_statistics.addNode(f);
                        }
                    }
                    else
                    {
                        // depth level reached, add to leafs

                        this->m_leafs.emplace_back(f);

                        // add to node statistics
                        if(computeStatistics)
                        {
                            m_statistics.addNode(f);
                        }
                    }
                    --nNodesLevelCurr;
                    // pop node at the front
                    splitList.pop_front();
                }

                // enumerate nodes new (firsts leafs then all other nodes)
                this->enumerateNodes();

                if(computeStatistics)
                {
                    averageStatistics();
                }
            }

            template<typename... T>
            void initSplitHeuristic(T&&... t)
            {
                m_heuristic.init(std::forward<T>(t)...);
            }

            template<bool computeStatistics = true, bool safetyCheck = true>
            LeafNeighbourMapType buildLeafNeighboursAutomatic()
            {
                if(!m_statistics.m_computedTreeStats)
                {
                    ApproxMVBB_ERRORMSG("You did not compute statistics for this tree while constructing it!")
                }
                /* Here the minLeafExtent ratio is made slightly smaller to make the
                   neighbour classification sensfull 
                   Multiple cells can have m_minLeafExtent */
                return buildLeafNeighbours<computeStatistics, safetyCheck>(0.99 * m_statistics.m_minLeafExtent);
            }

            template<bool computeStatistics = true, bool safetyCheck = true>
            LeafNeighbourMapType buildLeafNeighbours(PREC minExtent)
            {
                if(!this->m_root)
                {
                    ApproxMVBB_ERRORMSG("There is not root node! KdTree not built!")
                }

                m_statistics.m_computedNeighbourStats = computeStatistics;

                // Get all leaf neighbour indices for each leaf node!
                // To do this, we traverse the leaf list and for each leaf l
                // we determine in each direction of the kd-Tree in R^n (if 3D, 3
                // directions)
                // for "min" and "max" the corresponding leafs in the subtrees given by
                // the boundary information in the leaf node:
                // e.g. for the "max" direction in x for one leaf, we traverse the  subtree
                // of the boundary
                // information in "max" x direction
                // for "max" we always take the left node  (is the one which is closer to
                // our max x boundary)
                // for "min" we always take the right node (is the one which is closer to
                // our min x boundary)
                // in this way we get all candidate leaf nodes (which share the same split
                // axis with split position s)
                // which need still to be checked if they are neighbours
                // this is done by checking if the boundary subspace with the corresponding
                // axis set to the split position s
                // (space without the axis which is checked, e.g y-z space, with x = s)
                // against the current leaf nodes boundary subspace
                // (which is thickened up by the amount of the minimal leaf node extent
                // size,
                // important because its not clear what should count as a neighbout or what
                // not)
                // if this neighbour n subspace overlaps the thickened leaf node subspace
                // then this node is
                // considered as a neighbour for leaf l, (and also neighbour n has l as
                // neighbour obviously)
                // If the tree is build with the same min_extent size as the thickening in
                // this step here, then it should work
                // since the tree has all extents striclty greater then min_extent, and here
                // to avoid to many nodes to be classified as neighbours (trivial example,
                // min_extent grid)

                // each leaf gets a
                std::unordered_map<std::size_t, std::unordered_set<std::size_t>> leafToNeighbourIdx;

                // iterate over all leafs
                for(auto* node : this->m_leafs)
                {
                    node->getNeighbourLeafsIdx(leafToNeighbourIdx, minExtent);
                }

                // Do safety check in debug mode
                if(safetyCheck)
                {
                    safetyCheckNeighbours(leafToNeighbourIdx, minExtent);
                }

                // Compute statistics
                if(computeStatistics)
                {
                    m_statistics.m_minNeighbours = std::numeric_limits<std::size_t>::max();
                    m_statistics.m_maxNeighbours = 0;
                    m_statistics.m_avgNeighbours = 0;

                    for(auto& n : leafToNeighbourIdx)
                    {
                        m_statistics.m_avgNeighbours += n.second.size();
                        m_statistics.m_minNeighbours = std::min(m_statistics.m_minNeighbours, n.second.size());
                        m_statistics.m_maxNeighbours = std::max(m_statistics.m_maxNeighbours, n.second.size());
                    }
                    m_statistics.m_avgNeighbours /= this->m_leafs.size();
                }

                return leafToNeighbourIdx;
            }

            /** K-Nearst neighbour search
             * ===================================================*/
            struct ParentInfo
            {
                ParentInfo(NodeType* p, bool l = false, bool r = false)
                    : m_parent(p), childVisited{l, r}
                {
                }
                NodeType* m_parent;
                bool childVisited[2];
            };

        private:
            /** Priority queue adapter, to let the comperator be changed on the fly!
             *   This is usefull if we call getKNearestNeighbours lots of times.
             *   and want to update the comperator in between.
             */
            template<typename Container, typename Compare>
            class KNearestPrioQueue
                : public std::priority_queue<typename NodeDataType::PointListType::value_type, Container, Compare>
            {
            public:
                using value_type = typename Container::value_type;

                ApproxMVBB_STATIC_ASSERT((std::is_same<value_type, typename NodeDataType::PointListType::value_type>::value));

                using Base = std::priority_queue<typename NodeDataType::PointListType::value_type, Container, Compare>;

                using iterator         = typename Container::iterator;
                using reverse_iterator = typename Container::reverse_iterator;

                Container& getContainer()
                {
                    return this->c;
                }
                Compare& getComperator()
                {
                    return this->comp;
                }

                iterator begin()
                {
                    return this->c.begin();
                }
                iterator end()
                {
                    return this->c.end();
                }

                reverse_iterator rbegin()
                {
                    return this->c.rbegin();
                }
                reverse_iterator rend()
                {
                    return this->c.rend();
                }

                KNearestPrioQueue(std::size_t maxSize)
                    : m_maxSize(maxSize)
                {
                    this->c.reserve(m_maxSize);
                }

                inline void clear()
                {
                    this->c.clear();
                }

                inline bool full()
                {
                    return this->size() == m_maxSize;
                }

                inline void push(const typename Base::value_type& v)
                {
                    if(this->size() < m_maxSize)
                    {
                        Base::push(v);
                    }
                    else if(this->comp(v, this->top()))
                    {
                        Base::pop();
                        Base::push(v);
                    }
                }

                template<typename It>
                inline void push(It beg, It end)
                {
                    It it  = beg;
                    auto s = this->size();
                    while(s < m_maxSize && it != end)
                    {
                        Base::push(*it);
                        ++it;
                        ++s;
                    }
                    while(it != end)
                    {
                        // if *it < top -> insert
                        if(this->comp(*it, this->top()))
                        {
                            Base::pop();
                            Base::push(*it);
                        }
                        ++it;
                    }
                }

                /** Replace total container with new elements */
                template<typename Iterator>
                void replace(Iterator begin, Iterator end)
                {
                    this->c.clear();
                    this->c.insert(this->c.begin(), begin, end);
                    std::make_heap(this->c.begin(), this->c.end(), this->comp);
                }

                std::size_t maxSize()
                {
                    return m_maxSize;
                }

            private:
                std::size_t m_maxSize;
            };

        public:
            template<typename TDistSq    = EuclideanDistSq,
                     typename TContainer = StdVecAligned<typename NodeDataType::PointListType::value_type>>
            struct KNNTraits
            {
                using DistSqType    = EuclideanDistSq;
                using ContainerType = TContainer;
                using DistCompType  = typename NodeDataType::template DistanceComp<DistSqType>;
                using PrioQueue     = KNearestPrioQueue<ContainerType, DistCompType>;
            };

        private:
            template<typename T>
            struct isKNNTraits;
            template<typename N, typename C>
            struct isKNNTraits<KNNTraits<N, C>>
            {
                static const bool value = true;
            };

        public:
            template<typename TKNNTraits>
            void getKNearestNeighbours(typename TKNNTraits::PrioQueue& kNearest) const
            {
                ApproxMVBB_STATIC_ASSERT(isKNNTraits<TKNNTraits>::value);

                kNearest.clear();

                if(!this->m_root || kNearest.maxSize() == 0)
                {
                    return;
                }

                // distance comperator
                // using DistSqType = typename TKNNTraits::DistSqType;
                typename TKNNTraits::DistCompType& distComp = kNearest.getComperator();
                // reference point is distComp.m_ref

                // Debug set, will be optimized away in release
                // std::set<NodeType*> visitedLeafs;

                // Get leaf node and parent stack by traversing down the tree
                std::vector<ParentInfo> parents;
                parents.reserve(m_statistics.m_treeDepth);

                parents.emplace_back(nullptr, false, false);  // emplace
                NodeType* currNode = this->m_root;

                while(!currNode->isLeaf())
                {
                    // all points greater or equal to the splitPosition belong to the right
                    // node
                    if(distComp.m_ref(currNode->m_splitAxis) >= currNode->m_splitPosition)
                    {
                        parents.emplace_back(currNode, false, true);
                        currNode = currNode->rightNode();
                    }
                    else
                    {
                        parents.emplace_back(currNode, true, false);
                        currNode = currNode->leftNode();
                    }
                }
                ApproxMVBB_ASSERTMSG(currNode && currNode->isLeaf(), "currNode is nullptr!")
                    // currNode is a leaf !

                    PREC d                 = 0.0;
                PREC maxDistSq             = 0.0;
                ParentInfo* currParentInfo = nullptr;

                // Move up the tree, always visiting the all childs which overlap the norm
                // ball
                while(currNode != nullptr)
                {
                    currParentInfo = &parents.back();
                    ApproxMVBB_ASSERTMSG(currNode, "currNode is nullptr!")

                        if(!currNode->isLeaf())
                    {
                        // this is no leaf
                        if(!currParentInfo->childVisited[0])
                        {
                            // left not visited
                            // we processed this child
                            currParentInfo->childVisited[0] = true;  // set parents flag

                            if(kNearest.full())
                            {
                                // compute distance to split Axis
                                d = currParentInfo->m_parent->m_splitPosition -
                                    distComp.m_ref(currParentInfo->m_parent->m_splitAxis);
                                if(d <= 0.0)
                                {
                                    // ref point is right of split axis
                                    if(d * d >= maxDistSq)
                                    {
                                        continue;  // no overlap
                                    }
                                }
                                // ref point is left of split axis
                            }
                            // maxNorm ball overlaps left side or to little points
                            // visit left side!
                            currNode = currNode->leftNode();  // cannot be nullptr, since currNode is not leaf
                            ApproxMVBB_ASSERTMSG(currNode, "cannot be nullptr, since a leaf") if(!currNode->isLeaf())
                            {
                                // add the parent if no leaf
                                parents.emplace_back(currNode);
                            }

                            continue;
                        }
                        else if(!currParentInfo->childVisited[1])
                        {
                            // right not visited
                            // we processed this child
                            currParentInfo->childVisited[1] = true;  // set parents flag
                            if(kNearest.full())
                            {
                                d = currParentInfo->m_parent->m_splitPosition -
                                    distComp.m_ref(currParentInfo->m_parent->m_splitAxis);
                                if(d > 0)
                                {
                                    // ref point is left of split axis
                                    if(d * d > maxDistSq)
                                    {
                                        continue;  // no overlap
                                    }
                                }
                            }

                            // maxNorm ball overlaps right side or to little points!
                            // visit right side!
                            currNode = currNode->rightNode();
                            ApproxMVBB_ASSERTMSG(currNode, "cannot be nullptr, since a leaf") if(!currNode->isLeaf())
                            {
                                // add to parent
                                parents.emplace_back(currNode);
                            }
                            continue;
                        }

                        // we have have visited both,
                        // we pop parent and move a level up!
                        parents.pop_back();  // last one is this nonleaf, pop it
                        // the first parent contains a nullptr, such that we break when we move
                        // up from the root
                        currNode = parents.back().m_parent;
                    }
                    else
                    {
                        // this is a leaf
                        // if(visitedLeafs.insert(currNode).second==false){
                        //  ApproxMVBB_ERRORMSG("leaf has already been visited!")
                        // }

                        // get at least k nearst  points in this leaf and merge with kNearest
                        // list
                        if(currNode->size() > 0)
                        {
                            kNearest.push(currNode->data()->begin(), currNode->data()->end());
                            // update max norm
                            maxDistSq = distComp(kNearest.top());
                        }
                        // finished with this leaf, got to parent!
                        currNode = currParentInfo->m_parent;
                        continue;
                    }
                }
            }

            /**
             * =============================================================================*/

            /** Returns tuple with values
             * (number of leafs, avg. leaf data size, min. leaf data size, max. leaf data
             * size)
             */
            std::tuple<std::size_t,
                       std::size_t,
                       std::size_t,
                       PREC,
                       std::size_t,
                       std::size_t,
                       PREC,
                       PREC,
                       std::size_t,
                       std::size_t,
                       PREC>
            getStatistics()
            {
                return std::tuple_cat(Base::getStatistics(),
                                      std::make_tuple(m_statistics.m_treeDepth,
                                                      m_statistics.m_avgLeafSize,
                                                      m_statistics.m_minLeafDataSize,
                                                      m_statistics.m_maxLeafDataSize,
                                                      m_statistics.m_minLeafExtent,
                                                      m_statistics.m_maxLeafExtent,
                                                      m_statistics.m_minNeighbours,
                                                      m_statistics.m_maxNeighbours,
                                                      m_statistics.m_avgNeighbours));
            }

            std::string getStatisticsString()
            {
                std::stringstream s;
                auto t        = getStatistics();
                std::string h = m_heuristic.getStatisticsString();
                s << "Tree Stats: "
                  << "\n\t nodes      : " << std::get<0>(t) << "\n\t leafs      : " << std::get<1>(t)
                  << "\n\t tree level : " << std::get<2>(t) << "\n\t avg. leaf data size : " << std::get<3>(t)
                  << "\n\t min. leaf data size : " << std::get<4>(t) << "\n\t max. leaf data size : " << std::get<5>(t)
                  << "\n\t min. leaf extent    : " << std::get<6>(t) << "\n\t max. leaf extent    : " << std::get<7>(t)
                  << "\nSplitHeuristics Stats: \n"
                  << h << "\nNeighbour Stats (if computed): \n"
                  << "\n\t min. leaf neighbours    : " << std::get<8>(t) << "\n\t max. leaf neighbours    : " << std::get<9>(t)
                  << "\n\t avg. leaf neighbours    : " << std::get<10>(t) << std::endl;

                return s.str();
            }

            friend class XML;

        private:
            SplitHeuristicType m_heuristic;

            unsigned int m_maxTreeDepth = 50;
            unsigned int m_maxLeafs     = std::numeric_limits<unsigned int>::max();

            /** Statistics ========================*/
            TreeStatistics m_statistics;

            void resetStatistics()
            {
                m_statistics.reset();
                m_heuristic.resetStatistics();
            }
            void averageStatistics()
            {
                m_statistics.average(this->m_nodes.size(), this->m_leafs.size());
            }

            /** Safety check for neighbour list */
            template<typename NeighbourMap>
            void safetyCheckNeighbours(const NeighbourMap& n, PREC minExtent)
            {
                if(n.size() != this->m_leafs.size())
                {
                    ApproxMVBB_ERRORMSG("Safety check for neighbours failed!: size:" << n.size() << "," << this->m_leafs.size())
                }

                for(auto* l : this->m_leafs)
                {
                    // Check leaf l
                    AABB<Dimension> t = l->aabb();
                    t.expand(minExtent);

                    // check against neighbours ( if all neighbours really overlap )
                    auto it = n.find(l->getIdx());  // get neighbours for this leaf
                    if(it == n.end())
                    {
                        ApproxMVBB_ERRORMSG("Safety check: Leaf idx" << l->getIdx() << " not in neighbour map!")
                    }

                    for(const auto& idx : it->second)
                    {
                        if(this->getLeaf(idx) == nullptr)
                        {
                            ApproxMVBB_ERRORMSG("Safety check: Neighbour idx" << idx << " not found!")
                        }
                        // check if this neighbour overlaps
                        if(!t.overlaps(this->m_leafs[idx]->aabb()))
                        {
                            ApproxMVBB_ERRORMSG("Safety check: Leaf idx: " << idx << " does not overlap " << l->getIdx())
                        }
                        // check if this neighbours also has this leaf as neighbour
                        auto nIt = n.find(idx);
                        if(nIt == n.end())
                        {
                            ApproxMVBB_ERRORMSG("Safety check: Neighbour idx" << idx << " not in neighbour map!")
                        }
                        if(nIt->second.find(l->getIdx()) == nIt->second.end())
                        {
                            ApproxMVBB_ERRORMSG("Safety check: Neighbour idx"
                                                << idx << " does not have leaf idx: " << l->getIdx() << " as neighbour")
                        }
                    }

                    // built brute force list with AABB t
                    std::unordered_set<std::size_t> ns;
                    for(auto* ll : this->m_leafs)
                    {
                        if(ll->getIdx() != l->getIdx() && t.overlaps(ll->aabb()))
                        {
                            ns.emplace(ll->getIdx());
                        }
                    }
                    // check brute force list with computed list
                    for(auto& i : ns)
                    {
                        if(it->second.find(i) == it->second.end())
                        {
                            ApproxMVBB_ERRORMSG("Safety check: Bruteforce list has neighbour idx: "
                                                << i << " for leaf idx: " << l->getIdx() << " but not computed list!")
                        }
                    }
                    if(ns.size() != it->second.size())
                    {
                        ApproxMVBB_ERRORMSG(
                            "Safety check: Bruteforce list and computed list "
                            "are not the same size!"
                            << ns.size() << "," << it->second.size())
                    }
                }
            }
        };

        /**
         * =======================================================================================*/

        template<typename TTraits = KdTree::DefaultPointDataTraits<>>
        class NearestNeighbourFilter
        {
        public:
            using PointDataTraits         = TTraits;
            static const unsigned int Dim = PointDataTraits::Dimension;
            using PointListType           = typename PointDataTraits::PointListType;
            using PointType               = typename PointDataTraits::PointType;
            using PointGetter             = typename PointDataTraits::PointGetter;

            using Tree               = KdTree::Tree<KdTree::TreeTraits<KdTree::PointData<PointDataTraits>>>;
            using SplitHeuristicType = typename Tree::SplitHeuristicType;
            using NodeDataType       = typename Tree::NodeDataType;

            /** CTor */
            NearestNeighbourFilter(std::size_t kNeighboursMean        = 20,
                                   PREC stdDevMult                    = 1.0,
                                   std::size_t allowSplitAboveNPoints = 10)
                : m_kNeighboursMean(kNeighboursMean), m_stdDevMult(stdDevMult), m_allowSplitAboveNPoints(allowSplitAboveNPoints)
            {
                if(m_kNeighboursMean == 0)
                {
                    ApproxMVBB_ERRORMSG("kNeighboursMean is zero! (needs to be >=1)")
                }
            }

            inline std::tuple<std::size_t, PREC, std::size_t> getSettings()
            {
                return std::make_tuple(m_kNeighboursMean, m_stdDevMult, m_allowSplitAboveNPoints);
            }

            inline void setSettings(std::size_t kNeighboursMean, PREC stdDevMult, std::size_t allowSplitAboveNPoints)
            {
                m_kNeighboursMean        = kNeighboursMean;
                m_stdDevMult             = stdDevMult;
                m_allowSplitAboveNPoints = allowSplitAboveNPoints;
            }

            /**
             *   This filter function computes the nearest distance distribution (mean,
             * standart deviation) of the points \p points
             *   and classifies all points which have mean nearest distance <  mean +
             * stdDevMult * standart deviation
             *   as outliers.
             *   The AABB \p aabb is for building the kd-Tree.
             *   The function modifies the container \p points (shuffling, sorting etc.)
             * and saves the
             *   remaining points in \p output. If \p invert is on the outliers are saved
             * in \p output.
             */
            template<typename Container,
                     typename DistSq = EuclideanDistSq,
                     typename        = typename std::enable_if<ContainerTags::has_randomAccessIterator<Container>::value>::type>
            void filter(Container& points, const AABB<Dim>& aabb, Container& output, bool invert = false)
            {
                ApproxMVBB_STATIC_ASSERTM(
                    (std::is_same<typename Container::value_type, typename PointListType::value_type>::value),
                    "Container value_type needs to be the same as value_type of "
                    "PointDataTraits!");

                // Make kdTree;
                Tree tree;

                typename SplitHeuristicType::QualityEvaluator qual(0.0,  /* splitratio (maximized by MidPoint) */
                                                                   2.0,  /* pointratio (maximized by MEDIAN)*/
                                                                   1.0); /* extentratio (maximized by MidPoint)*/

                PREC minExtent = 0.0;  // box extents are bigger than this!
                tree.initSplitHeuristic(
                    std::initializer_list<typename SplitHeuristicType::Method>{/*SplitHeuristicType::Method::MEDIAN,*/
                                                                               /*SplitHeuristicType::Method::GEOMETRIC_MEAN,*/
                                                                               SplitHeuristicType::Method::MIDPOINT},
                    m_allowSplitAboveNPoints,
                    minExtent,
                    SplitHeuristicType::SearchCriteria::FIND_FIRST,
                    qual,
                    0.0,
                    0.0,
                    0.1);

                auto rootData    = std::unique_ptr<NodeDataType>(new NodeDataType(points.begin(), points.end()));
                unsigned int inf = std::numeric_limits<unsigned int>::max();
                tree.build(aabb, std::move(rootData), inf /*max tree depth*/, inf /*max leafs*/);

                // Start filtering =======================================
                using KNNTraits = typename Tree::template KNNTraits<DistSq>;
                typename KNNTraits::PrioQueue kNearest(m_kNeighboursMean + 1);
                typename KNNTraits::DistCompType& compDist = kNearest.getComperator();

                // reserve space for all nearest distances (we basically analyse the
                // histogram of the nearst distances)

                m_nearestDists.assign(points.size(), 0.0);

                std::size_t validPoints = 0;
                typename KNNTraits::PrioQueue::iterator it;
                typename KNNTraits::PrioQueue::iterator e;
                PREC sum     = 0.0;
                auto nPoints = points.size();
                for(std::size_t i = 0; i < nPoints; ++i)
                {
                    // std::cout << "i: " << i << std::endl;
                    //  Get the kNearest neighbours
                    kNearest.getComperator().m_ref = PointGetter::get(points[i]);
                    tree.template getKNearestNeighbours<KNNTraits>(kNearest);

                    // compute sample mean and standart deviation of the sample

                    // we dont include our own point (which is also a result)
                    // we should always have some kNearst neighbours, if we have a non-empty
                    // point cloud
                    // otherwise something is fishy!, anyway check for this

                    if(kNearest.size() > 1)
                    {
                        e   = kNearest.end();
                        sum = 0.0;
                        for(it = ++kNearest.begin(); it != e; ++it)
                        {
                            sum += std::sqrt(compDist(*it));
                        }
                        // save mean nearest distance
                        m_nearestDists[i] = sum / (kNearest.size() - 1);
                        ++validPoints;
                    }
                }

                // compute mean and standart deviation
                sum        = 0.0;
                PREC sumSq = 0.0;
                for(std::size_t i = 0; i < nPoints; ++i)
                {
                    sum += m_nearestDists[i];
                    sumSq += m_nearestDists[i] * m_nearestDists[i];
                }
                m_mean   = sum / validPoints;
                m_stdDev = std::sqrt((sumSq - sum * sum / validPoints) / validPoints);

                PREC distanceThreshold =
                    m_mean + m_stdDevMult * m_stdDev;  // a distance that is bigger than this signals an outlier

                // move over all points and build new list (without outliers)
                // all points which where invalid are left untouched since m_nearestDists[i]
                // == 0
                output.resize(points.size());
                std::size_t nPointsOut = 0;
                if(!invert)
                {
                    for(std::size_t i = 0; i < nPoints; ++i)
                    {
                        if(m_nearestDists[i] < distanceThreshold)
                        {
                            // no outlier add to list
                            output[nPointsOut] = points[i];
                            ++nPointsOut;
                        }
                    }
                }
                else
                {
                    for(std::size_t i = 0; i < nPoints; ++i)
                    {
                        if(m_nearestDists[i] >= distanceThreshold)
                        {
                            // outlier add to list
                            output[nPointsOut] = points[i];
                            ++nPointsOut;
                        }
                    }
                }
                output.resize(nPointsOut);

                // =======================================================
            }

            /** Get the nearestDists for all points from the filter.
             *   Take care, these values at index i do really correspond to index in \p
             * points in filter().
             *   because filter() function may have changed the order which is also
             *   reflected in the return value of this function.
             */

            using NearestDistancesType = std::vector<PREC>;
            std::vector<PREC>& getNearestDists()
            {
                return m_nearestDists;
            }

            /** Get the computed mean/standart deviation of the nearest distance
             *   (the mean of the returned array of getNearestDists())
             */
            std::pair<PREC, PREC> getMoments()
            {
                return std::make_pair(m_mean, m_stdDev);
            }

        private:
            NearestDistancesType m_nearestDists;
            PREC m_mean   = 0;
            PREC m_stdDev = 0;

            /** How many neighbours points are search for one point to classify to build
             * the mean neighbour distance */
            std::size_t m_kNeighboursMean;
            /** The multiplier for the standart deviation,
             * if the distance of the point to classify is > \p stdDevMult * stdDevDist +
             * meanDist,
             * then the point is classfied as an outlier.
             */
            PREC m_stdDevMult;

            std::size_t m_allowSplitAboveNPoints;
        };
    }  // namespace KdTree
}  // namespace ApproxMVBB

#endif
