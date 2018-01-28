// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_KdTreeXml_hpp
#define ApproxMVBB_KdTreeXml_hpp

#ifndef ApproxMVBB_XML_SUPPORT
#    warning "Your using the KdTreeXml header, which needs to be linked with the Xml library!"
#endif

#include <pugixml.hpp>

#include "KdTree.hpp"

namespace ApproxMVBB
{
    namespace KdTree
    {
        class XML
        {
        public:
            using XMLNodeType = pugi::xml_node;

            /** Append all points in \p obj to the XML node \p root */
            template<typename TTraits>
            static void appendToXML(PointData<TTraits> const& obj, XMLNodeType& root)
            {
                using PointGetter            = typename PointData<TTraits>::PointGetter;
                static const auto nodePCData = pugi::node_pcdata;
                XMLNodeType node             = root.append_child("Points");
                std::stringstream ss;
                for(auto& p : obj)
                {
                    ss << PointGetter::get(p).transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
                }
                node.append_child(nodePCData).set_value(ss.str().c_str());
            }

            /** Append the data of the base kdTree class \p obj to the XML node \p kdNode
     */
            template<typename Traits>
            static void appendToXML(TreeBase<Traits> const& obj, XMLNodeType kdNode)
            {
                using NodeType               = typename TreeBase<Traits>::NodeType;
                static const auto nodePCData = pugi::node_pcdata;
                std::stringstream ss;
                XMLNodeType r    = kdNode.append_child("Root");
                XMLNodeType aabb = r.append_child("AABB");
                ss.str("");
                ss << obj.m_root->aabb().m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << " "
                   << obj.m_root->aabb().m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << "\n";
                aabb.append_child(nodePCData).set_value(ss.str().c_str());

                // Save leafs
                XMLNodeType leafs = kdNode.append_child("Leafs");

                for(auto* l : obj.m_leafs)
                {
                    XMLNodeType node = leafs.append_child("Leaf");
                    node.append_attribute("level").set_value(l->getLevel());
                    node.append_attribute("idx").set_value(std::to_string(l->getIdx()).c_str());
                    aabb = node.append_child("AABB");
                    ss.str("");
                    ss << l->aabb().m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << " "
                       << l->aabb().m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << "\n";
                    aabb.append_child(nodePCData).set_value(ss.str().c_str());
                }

                // Save AABB tree (breath first)
                XMLNodeType aabbTree = kdNode.append_child("AABBTree");
                std::deque<NodeType*> q;  // Breath first queue

                q.push_back(obj.m_root);
                unsigned int currLevel = obj.m_root->getLevel();
                ss.str("");
                while(q.size() > 0)
                {
                    // Write stuff of f if not leaf
                    auto* f = q.front();

                    if(f->getLevel() > currLevel)
                    {
                        // write new string
                        aabb = aabbTree.append_child("AABBSubTree");
                        aabb.append_attribute("level").set_value(currLevel);
                        aabb.append_child(nodePCData).set_value(ss.str().c_str());
                        // update to next level
                        currLevel = f->getLevel();
                        ss.str("");
                    }

                    if(!f->isLeaf())
                    {
                        ss << f->aabb().m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << " "
                           << f->aabb().m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << "\n";
                    }

                    // push the left/right
                    auto* n = f->leftNode();
                    if(n)
                    {
                        q.push_back(n);
                    }
                    n = f->rightNode();
                    if(n)
                    {
                        q.push_back(n);
                    }

                    q.pop_front();
                }

                // write last string
                auto s = ss.str();
                if(!s.empty())
                {
                    aabb = aabbTree.append_child("AABBSubTree");
                    aabb.append_attribute("level").set_value(currLevel);
                    aabb.append_child(nodePCData).set_value(s.c_str());
                }
            }
            /** Append the kdTree statistics \p obj to the XML node \p kdNode */
            static void appendToXML(const TreeStatistics& obj, XMLNodeType& kdNode)
            {
                auto stat = kdNode.append_child("Statistics");

                stat.append_attribute("m_computedTreeStats").set_value(obj.m_computedTreeStats);
                stat.append_attribute("m_treeDepth").set_value(obj.m_treeDepth);
                stat.append_attribute("m_avgSplitPercentage").set_value(obj.m_avgSplitPercentage);
                stat.append_attribute("m_minLeafExtent").set_value(obj.m_minLeafExtent);
                stat.append_attribute("m_maxLeafExtent").set_value(obj.m_maxLeafExtent);
                stat.append_attribute("m_avgLeafSize").set_value(obj.m_avgLeafSize);
                stat.append_attribute("m_minLeafDataSize").set_value((long long unsigned int)obj.m_minLeafDataSize);
                stat.append_attribute("m_maxLeafDataSize").set_value((long long unsigned int)obj.m_maxLeafDataSize);
                stat.append_attribute("m_computedNeighbourStats")
                    .set_value((long long unsigned int)obj.m_computedNeighbourStats);
                stat.append_attribute("m_minNeighbours").set_value((long long unsigned int)obj.m_minNeighbours);
                stat.append_attribute("m_maxNeighbours").set_value((long long unsigned int)obj.m_maxNeighbours);
                stat.append_attribute("m_avgNeighbours").set_value(obj.m_avgNeighbours);
            }

            /** Main function to append the whole data of the kdTree \p obj to the XML
     * node \p root */
            template<typename TTraits>
            static void appendToXML(const Tree<TTraits>& obj,
                                    XMLNodeType& root,
                                    bool aligned         = true,
                                    const Matrix33& A_IK = Matrix33::Identity()
                                    /*,bool exportPoints = false*/)
            {
                using Base = typename Tree<TTraits>::Base;

                static const auto nodePCData = pugi::node_pcdata;

                std::stringstream ss;
                XMLNodeType node;
                XMLNodeType kdNode = root.append_child("KdTree");

                kdNode.append_attribute("aligned").set_value(aligned);

                XMLNodeType a = kdNode.append_child("A_IK");
                ss << A_IK.format(MyMatrixIOFormat::SpaceSep);
                a.append_child(nodePCData).set_value(ss.str().c_str());

                appendToXML(static_cast<const Base&>(obj), kdNode);

                appendToXML(obj.m_statistics, kdNode);
            }

            /** Main function to append the whole data of a simple kdTree \p obj to the
     * XML node \p root */
            template<typename TTraits>
            static void appendToXML(const TreeSimple<TTraits>& obj,
                                    XMLNodeType root,
                                    bool aligned         = true,
                                    const Matrix33& A_IK = Matrix33::Identity())
            {
                using Base                   = typename TreeSimple<TTraits>::Base;
                static const auto nodePCData = pugi::node_pcdata;

                std::stringstream ss;
                XMLNodeType node;
                XMLNodeType kdNode = root.append_child("KdTree");

                kdNode.append_attribute("aligned").set_value(aligned);

                XMLNodeType a = kdNode.append_child("A_IK");
                ss << A_IK.format(MyMatrixIOFormat::SpaceSep);
                a.append_child(nodePCData).set_value(ss.str().c_str());

                appendToXML(static_cast<const Base&>(obj), kdNode);
                appendToXML(obj.m_statistics, kdNode);
            }
        };
    }  // namespace KdTree
}  // namespace ApproxMVBB
#endif
