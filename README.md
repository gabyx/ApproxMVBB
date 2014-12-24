==========
ApproxMVBB
==========

----------------------------------------
Fast algorithms to compute an approximation of the minimal volume oriented bounding box of a point cloud in 3D.
----------------------------------------

Computing the minimal volume oriented bounding box for a given point cloud in 3D is a hard problem in computer science.
Exact algorithms are known and of cubic order in the number of points in 3D. A faster exact algorithm is currently not know. However, for lots of applications an approximation of the minimum volume oriented bounding box is acceptable and already accurate enough.  
This small standart compliant C++11 library can either be built into a shared object library 
or directly be included in an existing C++ project. It includes code for computing the ConvexHull in 2D and the minimal area rectangle of a 2D point cloud as well.

![alt text](https://github.com/gabyx/ApproxMVBB/wiki/images/Bunny.png "Bunny") ![alt text](https://github.com/gabyx/ApproxMVBB/wiki/images/Cube.png "Cube")

---------------------------
Installation & Dependencies
---------------------------
To build the library, the tests and the example you need the built tool [cmake](
http://www.cmake.org).
This library only depends on the matrix library [Eigen](http://eigen.tuxfamily.org) at least version 3. Download it and install it on your system.

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
The cmake script will find Eigen if you installed it in a system wide folder (e.g ``/usr/local/``)
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
Example Usage
---------------------------
Please see the ``example/main.cpp`` in the source directory.
Given a point cloud with ``n=10000`` points sampled in the unit cube in 3D 
we compute an approximation of the minimum volume bounding volume by the following calls:
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
in form of a quaternion  ``oobb.m_q_KI`` . The rotation matrix ``R_KI`` from frame I to frame K  can be obtained by ``oobb.m_q_KI.matrix()`` (see ``Eigen::Quaternion``). This rotation matrix ``R_KI`` corresponds to a coordinate transformation A_IK which transforms coordinates from frame K to coordinates in frame I. Thereforce, to get the lower point expressed in the coordinate frame I this yields:

```C++
    ApproxMVBB::Vector3 p = oobb.m_q_IK * oobb.m_minPoint  // A_IK * oobb.m_minPoint 
```

---------------------------
Function Parameters & How It Works
---------------------------
The most important function:
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
Building and installing the basic tests is done by:

    $ cd Build
    $ make ApproxMVBBTests
    
  
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
I was inspired by the work and algorithms of [Gill Barequet & Sariel Har-Peled](http://sarielhp.org/papers/00/diameter/) for computing a minimal volume bounding box.
Additionally,  the geometric predicates (orient2d) used in the convex hull algorithm (graham scan) have been taken from the fine work of [Jonathan Richard Shewchuk](http://www.cs.cmu.edu/~quake/robust.html).
Special thanks go to my significant other which always had an ear during breakfast for this little project :kissing_heart:
