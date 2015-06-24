import xml.etree.ElementTree as ET
import numpy as np
from transforms3d.quaternions import *

from .ParseNDArray import *
from .AABB import *

class KdTreeLeaf:
      def __init__(self,level,idx,aabb, points):
        self.level = level;
        self.idx = idx;
        self.points = points;
        self.aabb = aabb;  

class KdTree:
    def __init__(self, rootAABB, aligned, A_IK, leafs, aabbTree):
        self.aabb = rootAABB;
        self.aligned = aligned;
        self.leafs = leafs;
        self.aabbTree = aabbTree;
        self.A_IK = A_IK;
        self.q_KI = mat2quat(A_IK); # A_IK = R_KI
        
    @staticmethod    
    def parseFromXML(g):
        dtAABB = np.dtype([('min', (float,3) ), ('max',(float,3) )]);
        dtPoints = float;
        
        aligned = g.attrib["aligned"] in ["true","True"]
        print("aligned:",aligned)
        
        # parse root
        s = g.find("./Root/AABB");
        if(s.text):
            rootAABB = parseNDArray(s.text,dtAABB)
            rootAABB = AABB(rootAABB['min'],rootAABB['max'])
            print("Root AABB: " ,rootAABB);
        else:
            raise NameError("No Root AABB found!");
        
        s = g.find("./A_IK").text
        dt = np.dtype( float  );
        A_IK = np.reshape(parseNDArray(s,dt),(3,3));
        print("A_IK: \n", A_IK)
        
        # parse all leafs
        leafs = g.find("./Leafs");
        leafsDict ={};
        for leaf in leafs.iter("Leaf"):
            
            level = int(leaf.attrib['level']);
            idx = int(leaf.attrib['idx']);
            
            s = leaf.find("AABB");
            if(s.text):
                leafAABB = parseNDArray(s.text,dtAABB)
            else:
                raise NameError("No Leaf AABB found!");
            
            points = None
            s = leaf.find("Points");
            if s:
                points = parseNDArray(s.text,dtPoints);
            
            if level not in leafsDict.keys():
                l = leafsDict[level] = [];
            else:
                l = leafsDict[level];
                   
            l.append( KdTreeLeaf(level,idx, AABB(leafAABB['min'],leafAABB['max']) ,points) )
        
        # parse all AABBSubTrees
        aabbSubTreeDict ={};
        for subtree in g.find("AABBTree").iter("AABBSubTree"):
            level = int(subtree.attrib['level']);
            aabbs = parseNDArray(subtree.text,dtAABB);
            aabbSubTreeDict[level]= aabbs;
        
        
        
        return KdTree(rootAABB,aligned,A_IK,leafsDict,aabbSubTreeDict);
        