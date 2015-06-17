import numpy as np;

class AABB:
    def __init__(self,min,max):
        self.min =  min.flatten();
        self.max =  max.flatten();
    
    def center(self):
        return 0.5*(self.max + self.min);
