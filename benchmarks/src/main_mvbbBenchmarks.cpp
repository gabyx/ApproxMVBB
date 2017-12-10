// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "ApproxMVBB/ComputeApproxMVBB.hpp"

#include "CommonFunctions.hpp"
#include "benchmark/benchmark.h"

#define MY_BENCHMARK(name) void benchmark_##name(benchmark::State& state)
#define MY_BENCHMARK_REGISTER(name) BENCHMARK(benchmark_##name)

#define MY_BENCHMARK_RANDOM_STUFF(name)                                               \
    std::string testName = #name;                                                     \
    auto seed            = hashString(#name);                                         \
    std::cout << "Seed for this test: " << seed << std::endl;                         \
    ApproxMVBB::RandomGenerators::DefaultRandomGen rng(seed);                         \
    ApproxMVBB::RandomGenerators::DefaultUniformRealDistribution<PREC> uni(0.0, 1.0); \
    auto f = [&](PREC) { return uni(rng); };

namespace ApproxMVBB
{
namespace MVBBBenchmarks
{
template <typename TMatrix>
void mvbbTest(const TMatrix& v,
              PREC eps                        = 0.001,
              unsigned int nPoints            = 400,
              unsigned int gridSize           = 5,
              unsigned int mvbbDiamOptLoops   = 2,
              unsigned int gridSearchOptLoops = 10)
{
    auto oobb = ApproxMVBB::approximateMVBB(v, eps, nPoints, gridSize, mvbbDiamOptLoops, gridSearchOptLoops);
}
}
}

using namespace ApproxMVBB;
using namespace TestFunctions;
using namespace PointFunctions;
using namespace ApproxMVBB::MVBBBenchmarks;

MY_BENCHMARK(bunny)
{
    MY_BENCHMARK_RANDOM_STUFF(bunny);
    auto v = getPointsFromFile3D(getFileInPath("Bunny.txt"));
    Matrix3Dyn t(3, v.size());
    for(unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    applyRandomRotTrans(t, f);
    std::cout << "Start..." << std::endl;
    while(state.KeepRunning())
    {
        mvbbTest(t, 0.1);
    }
}

MY_BENCHMARK(random140M)
{
    MY_BENCHMARK_RANDOM_STUFF(random140M);
    Matrix3Dyn t(3, 140000000);
    t = t.unaryExpr(f);
    std::cout << "Start..." << std::endl;
    while(state.KeepRunning())
    {
        mvbbTest(t, 0.01, 400, 5, 0, 5);
    }
}

MY_BENCHMARK(lucy)
{
    MY_BENCHMARK_RANDOM_STUFF(lucy);
    auto v = getPointsFromFile3D(getFileInAddPath("Lucy.txt"));
    Matrix3Dyn t(3, v.size());
    for(unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    applyRandomRotTrans(t, f);
    std::cout << "Start..." << std::endl;
    while(state.KeepRunning())
    {
        mvbbTest(t, 100, 400, 5, 0, 5);
    }
}

MY_BENCHMARK_REGISTER(bunny)->Unit(benchmark::kMillisecond);
MY_BENCHMARK_REGISTER(random140M)->Unit(benchmark::kMillisecond)->MinTime(7000);
MY_BENCHMARK_REGISTER(lucy)->Unit(benchmark::kMillisecond)->MinTime(7000);

BENCHMARK_MAIN();
