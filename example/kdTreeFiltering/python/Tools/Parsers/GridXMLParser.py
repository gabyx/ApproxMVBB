import xml.etree.ElementTree as ET;
import numpy as np;
from transforms3d.quaternions import *

from .ParseNDArray import *
from .AABB import *

class Grid:
    def __init__(self,processDim, aligned, aabb, A_IK):
        self.processDim = processDim;
        self.aabb = aabb;
        self.aligned = aligned;
        self.A_IK = A_IK;
        self.q_KI = mat2quat(A_IK); # A_IK = R_KI
        
    @staticmethod
    def parseFromXML(g):
        aligned = g.attrib["aligned"] in ["true","True"]
        print("aligned:",aligned)
        
        #Parse processDIm min/max A_IK
        s = g.find("./Dimension").text
        dt = np.dtype( np.uint  );
        processDim = parseNDArray(s,dt);
        print("processDim:" , processDim)
        
        s = g.find("./MinPoint").text
        dt = np.dtype( float );
        minP = parseNDArray(s,dt);
        print("min:" , minP)
        
        s = g.find("./MaxPoint").text
        maxP = parseNDArray(s,dt);
        print("max:" , maxP)
        
        s = g.find("./A_IK").text
        dt = np.dtype( float  );
        A_IK = np.reshape(parseNDArray(s,dt),(3,3));
        print("A_IK: \n", A_IK)
        
        # Parse points
        pointNode = root.find("./Points")
        dt = float;
        points = parseNDArray(pointNode.text,dt)
        print("Points: " ,np.shape(points))
            
        aabb = AABB(minP,maxP);
        return Grid(processDim,aligned,aabb,A_IK);
