==========
ApproxMVBB
==========

----------------------------------------
Fast algorithms to compute an approximation of the minimal volume oriented bounding box of a point cloud in 3D.
----------------------------------------

Computing the minimal volume oriented bounding box for a given point cloud in 3D is a hard problem in computer science.
Exact algorithms are known and of cubic order in the number of points in 3D. A faster exact algorithm is currently not know. However, for lots of applications an approximation of the minimum volume oriented bounding box is acceptable and already accurate enough.  
This small library can either be built into a shared object library 
or directly be included in an existing C++ project. It includes code for computing the ConvexHull in 2D an the minimal area rectangle of a 2D point cloud as well.

![alt text](https://github.com/gabyx/ApproxMVBB/wiki/images/Bunny.png "Bunny") ![alt text](https://github.com/gabyx/ApproxMVBB/wiki/images/Cube.png "Cube")

---------------------------
Installation & Dependencies
---------------------------
To build the library, the tests and the example you need the built tool [cmake](
http://www.cmake.org).
This library only depends on the matrix library [Eigen](http://eigen.tuxfamily.org) at least version 3. Download it and install it on your system.

Download the latest ApproxMVBB code::

    $ git clone https://github.com/gabyx/ApproxMVBB.git ApproxMVBB  
  
Make a build directory and navigate to it::

    $ mkdir Build
    $ cd Build

Invoke cmake in the Build directory::

    $ cmake ../ApproxMVBB

The cmake script will find Eigen if you installed it in a system wide folder (e.g ``/usr/local/``)
In the `CMakeCache.txt` file you can specify what you want to build 
( ``ApproxMVBB_BUILD_EXAMPLE, ApproxMVBB_BUILD_LIBRARY, ApproxMVBB_BUILD_TESTS`` )

To install the library and the header files at a specific location `/usr/local/include/` run cmake with::

    $ cmake -DCMAKE_INSTALL_PREFIX="/usr/local/include/" ../ApproxMVBB

Finally, build and install the project::

    $ make all   /* can be ApproxMVBB for the library or ApproxMVBBExample or ApproxMVBBTests */
    $ make install
 
 To build in parallel use the ``-jN`` flag in the `make` commmand, where ``N``denotes the number of parallel threads to use.
 
 
---------------------------
Example Usage
---------------------------
Please see the ``example/main.cpp`` in the source directory.
Given a point cloud with ``n=10000`` points sampled in the unit cube in 3D 
we compute an approximation of the minimum volume bounding volume by the following calls::
```C++
    #include <iostream>
    #include "ApproxMVBB/ComputeApproxMVBB.hpp"

    int  main( int  argc, char  ** argv ){
    
          ApproxMVBB::Matrix3Dyn points(3,10000);
          points.setRandom();
          ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,0.001,500,5,0,5);
          
          return 0;
          
    }
```
The returned object oriented bounding box ``oobb`` contains the lower ``oobb.m_minPoint`` and upper point ``oobb.m_maxPoint``
in expressed in the coordinate frame K of the bounding box. The bounding box also stores the rotation matrix from the world frame to the object frame K 
in form of a quaternion  ``oobb.m_q_KI`` . The rotation matrix ``R_KI`` from frame I to frame K  can be obtained by ``oobb.m_q_KI.matrix()`` (see ``Eigen::Quaternion``). This rotation matrix ``R_KI`` corresponds to a coordinate transformation A_IK which transforms coordinates from frame K to coordinates in frame I. Thereforce, to get the lower point expressed in the coordinate frame I this yields::

```C++
    ApproxMVBB::Vector3 p = oobb.m_q_IK * oobb.m_minPoint  // A_IK * oobb.m_minPoint 
```

---------------------------
Function Parameters & How It Works
---------------------------
The most important function::
```C++
    ApproxMVBB::approximateMVBB(pts, 
                                epsilon, 
                                pointSamples, 
                                gridSize,
                                mvbbDiamOptLoops, 
                                gridSearchOptLoops)
```
computes an approximation of the minimal volume bounding box in the following steps:

1. **An approximation of the diameter** (direction which realizes the diameter: ``z`` ) of the points ``pts`` is computed. 
   The value ``epsilon`` is the absolut tolarance for 
   the approximation of the diameter and has the same units as the points ``pts`` (in the example 0.001 meter)
2. The points are projected into the plane perpendicular to the direction ``z``
3. An approximation of the diameter of the projected points in 2D is computed (direction ``x`` )
4. **The initial approximate bounding box** ``A`` is computed in the orthogonal frame ``[x,y,z]``
5. **An optional optimization loop** is performed (value ``mvbbDiamOptLoops`` specifies how many loops) 
   by computing the minimal volume bounding box over a direction ``t`` where the direction ``t`` 
   is choosen sequentially from the current optimal bounding box solution.  
5. **The initial bounding box** ``A`` is used as a tight fit around the points ``pts`` 
   to compute a **representative sample** of the point cloud. The value ``pointSamples`` 
   defines how many points are used for the exhaustive grid search procedure in the next step
6. **An exhaustive grid search** (value ``gridSize`` specifies the x,y,z dimension of the grid defined by the bounding box ``A``) is performed.
   This search is a trivial loop over all grid directions (see Gill Barequet, and Sariel Har-Peled [1]) to find a even smaller bounding box.
   For each grid direction ``g`` the minimal bounding box in this direction is computed. This consists 
   of finding the minimal rectangle of the projected 2D point cloud in the plane perpendicular to direction ``g``.
   For each grid direction another **optional optimization loop** (same as in step 5, value ``gridSearchOptLoops`` ) can be 
   specified which again optimizes the volume starting from direction ``g``.
7. The final approximation for the mininmal volume bounding box is returned. :poop:

---------------------------
Building and Visualizing the Tests
---------------------------
Building and installing the basic tests is done by ::

    $ cd Build
    $ make ApproxMVBBTests
    
**Note:**
To run the test in high-performance mode (needs lots of ram), which tests also points clouds of 
140 million points and some polygonal statue ``lucy.txt`` succesfully you need 
to set the cmake variable ``ApproxMVBB_TESTS_HIGH_PERFORMANCE`` to ``ON``
and additionally initialize the submodule ``AdditionalFiles``
and copy the file ``Lucy.txt`` (~500mb) to the build folder of the tests ``BUILD/tests/``


Executing the test application ``cd tests; ./ApproxMVBBTests`` will then run the following tests:

1. Testing the ConvexHull2D for several point clouds in 2D
2. Minimal area rectangle tests for several point clouds in 2D
3. Testing the diameter computation and calculation of the initial bounding box ``A`` (see `Function Parameters & How It Works`_)
   for point clouds in 3D
4. Testing the full optimization pipeline to generate an approximation of the minimal volume bounding box

The output can be visualized with the ``ipython notebook`` ``PlotTestResults.ipynb``::

    $ cd Build/tests
    $ ipython noteboook

<p align="center">
<img src="https://github.com/gabyx/ApproxMVBB/wiki/images/ConvexHull.png"/>
</p>

--------------------------
Licensing
--------------------------

This source code is released under GPL License Version 3.0. (I might change the License to Boost/MIT depending on the owners of the depending source coder, yet to discuss)

---------------------------
Author and Acknowledgements
---------------------------

ApproxMVBB was written by Gabriel Nützi, with source code from Grégoire Malandain & Jean-Daniel Boissonnat 
(for the approximation of the diameter of a point cloud)
and Gill Barequet & Sariel Har-Peled (for the inspiration of the algorithms to compute a minimal volume bounding box)
(reference to come!)

