==========
ApproxMVBB [![Build Status](https://travis-ci.org/gabyx/ApproxMVBB.svg?branch=master)](https://travis-ci.org/gabyx/ApproxMVBB) ![C++](https://img.shields.io/badge/c%2B%2B-11/14-green.svg) ![Deps](https://img.shields.io/badge/dependencies-eigen3,pugixml,meta,python3-blue.svg)
==========

[Homepage](http://gabyx.github.io/ApproxMVBB/)


----------------------------------------
Fast algorithms to compute an approximation of the minimal volume oriented bounding box of a point cloud in 3D.
----------------------------------------

Computing the minimal volume oriented bounding box for a given point cloud in 3D is a hard problem in computer science.
Exact algorithms are known and of cubic order in the number of points in 3D. A faster exact algorithm is currently not know. However, for lots of applications an approximation of the minimum volume oriented bounding box is acceptable and already accurate enough. This project was developped for research in [Granular Rigidbody Dynamics](http://www.zfm.ethz.ch/~nuetzig/?page=research).
This small standart compliant C++11 library can either be built into a shared object library 
or directly be included in an existing C++ project. 
It includes code for :
    
- computing an approximation of an oriented minimal volume box,
- computing the convex hull of a point cloud in 2d,
- computing the minimal area rectangle of a 2d point cloud,
- 2d projections of point clouds,
- fast building a kD-Tree (n-dimensional, templated) with sophisticated splitting techniques which optimizes a 
  quality criteria during the splitting process,
- computing the k-nearest neighbours to a given point (kNN search) via kd-Tree.
- fast statistical outlier filtering of point clouds via (nearest neighbour search, kD-Tree).


<p align="center">
<a href="https://github.com/gabyx/ApproxMVBB/wiki/images/Bunny.png" tag="Bunny"  target="_blank"> <img src="https://github.com/gabyx/ApproxMVBB/wiki/images/Bunny.png"   height="300px" border="10px"/></a>
<a href="https://github.com/gabyx/ApproxMVBB/wiki/images/Cube.png" tag="Cube"  target="_blank"><img src="https://github.com/gabyx/ApproxMVBB/wiki/images/Cube.png"  height="300px" border="10px"/></a>
</p>

---------------------------
Installation & Dependencies
---------------------------
To build the library, the tests and the example you need the built tool [cmake](
http://www.cmake.org).
This library has these light-weight dependencies:

- [Eigen](http://eigen.tuxfamily.org) at least version 3, 
- [meta](https://github.com/ericniebler/meta), 
- [pugixml](https://github.com/zeux/pugixml) (install with ``#define PUGIXML_HAS_LONG_LONG`` enabled in pugiconfig.hpp),
- [python3](https://www.python.org/downloads/)(only for visualization)

Download these and install it on your system.

Download the latest ApproxMVBB code:
```bash
    $ git clone https://github.com/gabyx/ApproxMVBB.git ApproxMVBB  
```
Make a build directory and navigate to it:
```bash
    $ mkdir Build
    $ cd Build
```
Invoke cmake in the Build directory:
```bash
    $ cmake ../ApproxMVBB
```
The cmake script tries to find  [Eigen](http://eigen.tuxfamily.org),[meta](https://github.com/ericniebler/meta) and [pugixml](https://github.com/zeux/pugixml). If you installed these in a system wide folder (e.g ``/usr/local/``) this should succed without any problems.
In the `CMakeCache.txt` file you can specify what you want to build 
( ``ApproxMVBB_BUILD_EXAMPLE, ApproxMVBB_BUILD_LIBRARY, ApproxMVBB_BUILD_TESTS`` )

To install the library and the header files at a specific location `/usr/local/include/` run cmake with::
```bash
    $ cmake -DCMAKE_INSTALL_PREFIX="/usr/local/include/" ../ApproxMVBB
```
Finally, build and install the project:
```bash
    $ make all   /* can be ApproxMVBB for the library or ApproxMVBBExample or ApproxMVBBTests */
    $ make install
``` 
 To build in parallel use the ``-jN`` flag in the `make` commmand, where ``N``denotes the number of parallel threads to use.

**Cmake Find Scripts**   
The installation installs also scripts ``approxmvbb-config.cmake`` and ``approxmvbb-config-version.cmake`` into the ``lib/cmake`` folder. To include the library in another project the only thing you need to add in your cmake script is
```cmake
    find_package(ApproxMVBB [version] [Required] )
```
which defines the following variables if ApproxMVBB has been found successfully:
```cmake
    ApproxMVBB_CXX_FLAGS    #- extra flags for compilation
    ApproxMVBB_INCLUDE_DIRS #- include directories
    ApproxMVBB_LIBRARY_REL  #- Release library
    ApproxMVBB_LIBRARY_DGB  #- Debug library
    ApproxMVBB_LIBRARIES    #- libraries to link with
```    
If you installed the library into non-system generic location you can set the cmake variable ``$ApproxMVBB_DIR`` before invoking the ``find_library`` command:
```cmake
    set(ApproxMVBB_DIR "path/to/installation/lib/cmake")
    find_package(ApproxMVBB [version] [Required] )
```

--------------------------
Supported Platforms
--------------------------
The code has been tested on Linux and OS X with compilers ``clang`` and ``gcc``. 
It should work for Windows as well, but has not been tested.

---------------------------
Example Usage: Approximation MVBB
---------------------------
Please see the ``example/approxMVBB/main.cpp`` in the source directory.
Given a point cloud with ``n=10000`` points sampled in the unit cube in 3D 
we compute an approximation of the minimum volume bounding volume by the following calls:
```C++
    #include <iostream>
    #include "ApproxMVBB/ComputeApproxMVBB.hpp"

    int  main( int  argc, char  ** argv ){
    
          ApproxMVBB::Matrix3Dyn points(3,10000);
          points.setRandom();
          ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,0.001,500,5,0,5);
          oobb.expandZeroExtent(0.1);
          return 0;
          
    }
```
The returned object oriented bounding box ``oobb`` contains the lower ``oobb.m_minPoint`` and upper point ``oobb.m_maxPoint`` expressed in the coordinate frame K of the bounding box. The bounding box also stores the rotation matrix from the world frame to the object frame K as a quaternion  ``oobb.m_q_KI`` . The rotation matrix ``R_KI`` from frame I to frame K  can be obtained by ``oobb.m_q_KI.matrix()`` (see ``Eigen::Quaternion``). This rotation matrix ``R_KI`` corresponds to a coordinate transformation A_IK which transforms coordinates from frame K to coordinates in frame I. Thereforce, to get the lower point expressed in the coordinate frame I this yields:

```C++
    ApproxMVBB::Vector3 p = oobb.m_q_KI * oobb.m_minPoint  // A_IK * oobb.m_minPoint 
```
**Degenerate OOBB:**    
The returned bounding box might have a degenerated extent in some axis directions depending on the input points (e.g. 3 points defines a plane which is the minimal oriented bounding box with zero volume). The function ``expandZeroExtent`` is a post processing function to enlarge the bounding box by a certain percentage of the largest extent (if exisiting, otherwise a default value is used).

**Points Outside of the final OOBB:**    
Because the algorithm  works internally with a sample of the point cloud, the resulting OOBB might not contain all points of the original point cloud! To compensate for this an additional loop is required:

```C++
    ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose();
    auto size = points.cols();
    for( unsigned int i=0;  i<size; ++i ) {
        oobb.unite(A_KI*points.col(i));
    }
```

**Function Parameters & How It Works:**    
The most important function:
```C++
    ApproxMVBB::approximateMVBB(pts, 
                                epsilon, 
                                pointSamples, 
                                gridSize,
                                mvbbDiamOptLoops, 
                                mvbbGridSearchOptLoops)
```
computes an approximation of the minimal volume bounding box in the following steps:

1. **An approximation of the diameter** (direction which realizes the diameter: ``z`` ) of the points ``pts`` is computed. 
   The value ``epsilon`` is the absolut tolarance for 
   the approximation of the diameter and has the same units as the points ``pts`` (in the example 0.001 meter)
2. The points are projected into the plane perpendicular to the direction ``z``
3. An approximation of the diameter of the projected points in 2D is computed (direction ``x`` )
4. **The initial approximate bounding box** ``A`` is computed in the orthogonal frame ``[x,y,z]``
5. **A first optional optimization loop** is performed (parameter ``mvbbDiamOptLoops`` specifies how many loops) 
   by computing the minimal volume bounding box over a direction ``t`` where the direction ``t`` 
   is choosen sequentially from the current optimal bounding box solution. The algorithm starts with the directions of the box ``A``. *This optimzation works with all points in ``pts`` and might use a lot of time*
5. **The initial bounding box** ``A`` is used as a tight fit around the points ``pts`` 
   to compute a **representative sample** ``RS`` of the point cloud. The value ``pointSamples`` 
   defines how many points are used for the exhaustive grid search procedure in the next step
6. **An exhaustive grid search** (value ``gridSize`` specifies the x,y,z dimension of the grid defined by the bounding box ``A``) is performed.
   This search is a trivial loop over all grid directions (see Gill Barequet, and Sariel Har-Peled [1]) to find a even smaller bounding box.
   For each grid direction ``g`` the minimal bounding box of the projected points in direction ``g`` is computed. This consists 
   of finding the minimal rectangle (axis ``u`` and ``v`` in world frame) of the projected point cloud in the plane perpendicular to direction ``g``. The minimal bounding box ``G`` in direction ``g`` can be computed from the basis ``(u,v,g)`` and is a candidate for the overall minimzation problem.
   Each found minimal bounding box candidate ``G`` and its directions ``(u,v,g)`` can be used as a starting point for a **second optional optimization loop** (parameter ``mvbbGridSearchOptLoops``, same algorithm as in step 5 but with less points namely ``RS`` ).
7. The final approximation for the mininmal volume bounding box (minimal volume over all computed candiadates) is returned. :poop:



---------------------------
Example Usage: Generating a KdTree and Outlier Filtering
---------------------------

The library includes a fast KdTree implementation (which is not claimed to be ultimatively fast and absolutely memory efficient, 
but was written to fullfill this aspects to a certain level, real benchmarks still need to be done, the implementation 
can really well compete with famous implementations such as PCL(FLANN),ANN, and CGAL )
The KdTree splitting heuristic implements an extendable sophisticated splitting optimization 
which in the most elaborate, performance worst case consists of 
searching for the best split between the splitting heuristics ``MIDPOINT`` , ``MEDIAN`` and ``GEOMETRIC_MEAN``
by evaluating a user-provided quality evaluator. The simple standart quality evaluator is the ``LinearQualityEvaluator`` which computes the split quality by a weighted linear combination of the quantities ``splitRatio`` , ``pointRatio``, ``minMaxExtentRatio``.

Outlier filtering is done with the k-nearest neighbour search algorithm (similar to the PCL library but faster, and with user defined precision) and works roughly as the following:
The algorithm finds for each point ``p`` in the point cloud ``k``  nearest neighbours and averages their distance (distance functor) to the point ``p`` 
to obtain a mean distance ``distance`` for this particular point.
All nearest mean distances for all points give a histogram with a sample mean ``mean`` and sample standart deviation ``stdDev``.
All points which have a mean nearest neighbour distance greater or equal to ``mean + stdDevMult * stdDev`` 
are classified as outlier points.

Look at the examples in ``examples/kdTreeFiltering`` which produced the following pictures with the provided visualization notebook
``examples/kdTreeFiltering/python/VisualizeKdTree.ipynb``.

<p align="center">
<a href="https://github.com/gabyx/ApproxMVBB/wiki/images/BunnyKdTree1.png" tag="Bunny Kd-Tree Special Split Optimization"  target="_blank"> <img src="https://github.com/gabyx/ApproxMVBB/wiki/images/BunnyKdTree1.png"   width="300px"/></a>
<a href="https://github.com/gabyx/ApproxMVBB/wiki/images/BunnyKdTree2.png" tag="Bunny Kd-Tree, simple midpoint split"  target="_blank"><img src="https://github.com/gabyx/ApproxMVBB/wiki/images/BunnyKdTree1.png"  width="300px"/></a>
</p>

**Function Parameters & How It Works**    
To come

---------------------------
Building and Visualizing the Tests
---------------------------
Building and installing the basic tests is done by:

    $ cd Build
    $ make build_and_test
    
** Note the tests validation will fail currently. However the results can still be visualized and should be correct. I am currently working on making the tests meaningfull and correct on any platform which is not an easy task due to random stuff happening during the optimization loops.**

  
**Note:**
> To run the test in high-performance mode (needs lots of ram), which tests also points clouds of 
> 140 million points and some polygonal statue ``lucy.txt`` succesfully you need 
> to set the cmake variable ``ApproxMVBB_TESTS_HIGH_PERFORMANCE`` to ``ON``
> and additionally initialize the submodule ``additional`` and unzip the files:

>     $ cd ApproxMVBB
>     $ git submodule init
>     $ git submodule update
>     $ cd addtional/tests/files; cat Lucy* | tar xz 

> and rebuild the tests. (this will copy the additional files next to the executable)


Executing the test application ``cd tests; ./ApproxMVBBTests`` will then run the following tests:

1. Testing the ConvexHull2D for several point clouds in 2D
2. Minimal area rectangle tests for several point clouds in 2D
3. Testing the diameter computation and calculation of the initial bounding box ``A`` 
   (see [section](Function Parameters & How It Works))
   for point clouds in 3D
4. Testing the full optimization pipeline to generate an approximation of the minimal volume bounding box

The output can be visualized with the ``ipython notebook`` ``/tests/python/PlotTestResults.ipynb``:

    $ cd Build/tests
    $ ipython noteboook

<p align="center">
<img src="https://github.com/gabyx/ApproxMVBB/wiki/images/ConvexHull.png"/>
</p>

--------------------------
Benchmark
--------------------------
Here are some short benchmarks from the tests folder:   

| Point Cloud  | # Points | ~ CPU Time ``approximateMVBB``|
| ------------ | --------:| --------:|
| Standford Bunny | 35'945          |   0.91 s | 
| Standford Lucy  | 14'027'872      |   1.19 s |   
| Unit Cube       | 140'000'000     |    7.0 s |   

``approximateMVBB`` runs ``approximateMVBBDiam`` and performs a grid search afterwards (here 5x5x5=25 directions with  5 optimization runs for each)
It seems to take a long time for 140 million points. The most ineffiecient task is to get a good initial bounding box. This takes the most time as diameter computations are performed in 3d and then all points are projected in the found diameter direction in 3d and another diameter in the projected plane in 2d is computed. Afterwards the point cloud is sampled (not just random points, its done with a grid) and convex hull, minimal rectangle computations are performed over the grid directions. These algorithms could be made faster by exploiting the following things:
* Use an axis aligned bounding box as the initial bounding box for the grid search (not implemented yet)
* Parllelism for the projection -> (CUDA, threads)
    
    
--------------------------
Licensing
--------------------------

This source code is released under MPL 2.0. 

---------------------------
Author and Acknowledgements
---------------------------

ApproxMVBB was written by Gabriel Nützi, with source code from [Grégoire Malandain & Jean-Daniel Boissonnat](http://www-sop.inria.fr/members/Gregoire.Malandain/diameter/) 
for the approximation of the diameter of a point cloud.
I was inspired by the work and algorithms of [Gill Barequet & Sariel Har-Peled](http://sarielhp.org/p/98/bbox/) for computing a minimal volume bounding box.
Additionally,  the geometric predicates (orient2d) used in the convex hull algorithm (graham scan) have been taken from the fine work of [Jonathan Richard Shewchuk](http://www.cs.cmu.edu/~quake/robust.html).
Special thanks go to my significant other which always had an ear during breakfast for this little project :kissing_heart:
