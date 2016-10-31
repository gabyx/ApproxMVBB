// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef ApproxMVBB_RandomGenerator_hpp
#define ApproxMVBB_RandomGenerator_hpp

#include "ApproxMVBB/Config/Config.hpp"
#include "ApproxMVBB/Common/StaticAssert.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#include <type_traits>
#include <cstdint>
#include <random>
#include <limits>
#include <ctime>

namespace ApproxMVBB{
namespace RandomGenerators{

   /** This is a fixed-increment version of Java 8's SplittableRandom generator
   See http://dx.doi.org/10.1145/2714064.2660195 and
   http://docs.oracle.com/javase/8/docs/api/java/util/SplittableRandom.html

   It is a very fast generator passing BigCrush, and it can be useful if
   for some reason you absolutely want 64 bits of state; otherwise, we
   rather suggest to use a xorshift128+ (for moderately parallel
   computations) or xorshift1024* (for massively parallel computations)
   generator. */
   class APPROXMVBB_EXPORT SplitMix64{
    private:
        uint64_t x; //< The state can be seeded with any value.
    public:

        SplitMix64(const SplitMix64 & gen) = default;
        SplitMix64(SplitMix64 && gen) = default;

        SplitMix64(uint64_t seed);
        void seed(uint64_t seed);
        uint64_t operator()();

         // for standard random
        using result_type = uint64_t;
        static constexpr result_type min(){ return std::numeric_limits<result_type>::min(); }
        static constexpr result_type max(){ return std::numeric_limits<result_type>::max(); }
    };


    /** This is the fastest generator passing BigCrush without
           systematic failures, but due to the relatively short period it is
           acceptable only for applications with a mild amount of parallelism;
           otherwise, use a xorshift1024* generator.

           The state must be seeded so that it is not everywhere zero. If you have
           a 64-bit seed, we suggest to seed a splitmix64 generator and use its
           output to fill s.
    */
    class APPROXMVBB_EXPORT XorShift128Plus{
        private:
        uint64_t s[2];

        public:
        XorShift128Plus(const XorShift128Plus & gen) = default;
        XorShift128Plus(XorShift128Plus && gen) = default;

        XorShift128Plus(uint64_t seed);
        /** Take time() to seed this generator */
        XorShift128Plus();

        void seed(uint64_t seed);
        /** Generate random number */
        uint64_t operator()();

        /** This is the jump function for the generator. It is equivalent
           to 2^64 calls to next(); it can be used to generate 2^64
           non-overlapping subsequences for parallel computations. */
        void jump();

          // for standard random
        using result_type = uint64_t;
        static constexpr result_type min(){ return std::numeric_limits<result_type>::min(); }
        static constexpr result_type max(){ return std::numeric_limits<result_type>::max(); }
    };


    /* This is a fast, top-quality generator. If 1024 bits of state are too
       much, try a xorshift128+ generator.

       The state must be seeded so that it is not everywhere zero. If you have
       a 64-bit seed, we suggest to seed a splitmix64 generator and use its
       output to fill s. */
    class APPROXMVBB_EXPORT XorShift1024Star{

        private:
        uint64_t s[16];
        int p = 0;

        public:
        XorShift1024Star(const XorShift1024Star & gen) = default;
        XorShift1024Star(XorShift1024Star && gen) = default;


        XorShift1024Star(uint64_t seed);
        /** Take time() to seed this generator */
        XorShift1024Star();

        void seed(uint64_t seed);
        /** Generate random number */
        uint64_t operator()();

        /** This is the jump function for the generator. It is equivalent
        to 2^512 calls to next(); it can be used to generate 2^512
        non-overlapping subsequences for parallel computations. */
        void jump();

          // for standard random
        using result_type = uint64_t;
        static constexpr result_type min(){ return std::numeric_limits<result_type>::min(); }
        static constexpr result_type max(){ return std::numeric_limits<result_type>::max(); }
    };

    /** A fast portable, non-truly uniform integer distribution */
    template<typename T>
    class AlmostUniformUIntDistribution{
        public:
            ApproxMVBB_STATIC_ASSERT( std::is_unsigned<T>::value )

            AlmostUniformUIntDistribution(T min,T max): m_min(min), m_max(max) {
                m_nRange = m_max - m_min + 1;
            }

            template<typename G>
            T operator()(G & g){
                return ((double)g() / ((double)(G::max()-G::min()) + 1.0)) * m_nRange + m_min;
            }

        private:
        T m_min,m_max,m_nRange;
    };

    /** A fast portable, non-truly uniform real distribution */
    template<typename T>
    class AlmostUniformRealDistribution{
        public:
            ApproxMVBB_STATIC_ASSERT( std::is_floating_point<T>::value )

            AlmostUniformRealDistribution(T min,T max): m_min(min), m_max(max) {
                m_nRange = m_max - m_min + 1;
            }

            template<typename G>
            T operator()(G & g){
                return  ((T)g() / (T)(G::max()-G::min())) * m_nRange + m_min;
            }

        private:
        T m_min,m_max,m_nRange;
    };

    /** Default random generator definitions */
    static const uint64_t defaultSeed = 314159;
    using DefaultRandomGen =  XorShift128Plus;

    /** Define the Uniform distributions for the library and for the tests */
    #ifdef ApproxMVBB_BUILD_TESTS
        #warning "ApproxMVBB: Using non-standart uniform distributions for testing!"
        template<typename T>
        using DefaultUniformUIntDistribution = AlmostUniformUIntDistribution<T>;
        template<typename T>
        using DefaultUniformRealDistribution = AlmostUniformRealDistribution<T>;
    #else
        template<typename T>
        using DefaultUniformUIntDistribution = std::uniform_int_distribution<T>;
        template<typename T>
        using DefaultUniformRealDistribution = std::uniform_real_distribution<T>;
    #endif


}
}



namespace ApproxMVBB{
namespace RandomGenerators{


inline SplitMix64::SplitMix64(uint64_t sd) :x(sd) {}

inline void SplitMix64::seed(uint64_t sd){ x = sd;}

inline uint64_t SplitMix64::operator()() {
    uint64_t z = (x += UINT64_C(0x9E3779B97F4A7C15));
    z = (z ^ (z >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)) * UINT64_C(0x94D049BB133111EB);
    return z ^ (z >> 31);
}


inline XorShift128Plus::XorShift128Plus(uint64_t sd){
    seed(sd);
}

/** Take time() to seed this generator */
inline XorShift128Plus::XorShift128Plus(){
    seed(time(NULL));
}

inline void XorShift128Plus::seed(uint64_t sd){
    s[0] = SplitMix64{sd}();
    s[1] = s[0] ;
}

/** Generate random number */
inline uint64_t XorShift128Plus::operator()(){
    uint64_t s1 = s[0];
    const uint64_t s0 = s[1];
    s[0] = s0;
    s1 ^= s1 << 23; // a
    s[1] = s1 ^ s0 ^ (s1 >> 18) ^ (s0 >> 5); // b, c
    return s[1] + s0;
}



inline void XorShift128Plus::jump() {
    static const uint64_t JUMP[] = { 0x8a5cd789635d2dff, 0x121fd2155c472f96 };

    uint64_t s0 = 0;
    uint64_t s1 = 0;
    for(unsigned int i = 0; i < sizeof(JUMP) / sizeof (*JUMP); i++)
        for(int b = 0; b < 64; b++) {
            if (JUMP[i] & 1ULL << b) {
                s0 ^= s[0];
                s1 ^= s[1];
            }
            this->operator()();
        }

    s[0] = s0;
    s[1] = s1;
}



inline XorShift1024Star::XorShift1024Star(uint64_t sd){
    seed(sd);
}

/** Take time() to seed this generator */
inline XorShift1024Star::XorShift1024Star(){
    seed(time(NULL));
}

inline void XorShift1024Star::seed(uint64_t sd){
    uint64_t ss = SplitMix64{sd}();
    for(auto & v : s){
        v = ss;
    }
}

/** Generate random number */
inline uint64_t XorShift1024Star::operator()() {
   const uint64_t s0 = s[p];
    uint64_t s1 = s[p = (p + 1) & 15];
    s1 ^= s1 << 31; // a
    s[p] = s1 ^ s0 ^ (s1 >> 11) ^ (s0 >> 30); // b,c
    return s[p] * UINT64_C(1181783497276652981);
}



/** This is the jump function for the generator. It is equivalent
to 2^512 calls to next(); it can be used to generate 2^512
non-overlapping subsequences for parallel computations. */
inline void XorShift1024Star::jump() {
    static const uint64_t JUMP[] = { 0x84242f96eca9c41d,
        0xa3c65b8776f96855, 0x5b34a39f070b5837, 0x4489affce4f31a1e,
        0x2ffeeb0a48316f40, 0xdc2d9891fe68c022, 0x3659132bb12fea70,
        0xaac17d8efa43cab8, 0xc4cb815590989b13, 0x5ee975283d71c93b,
        0x691548c86c1bd540, 0x7910c41d10a1e6a5, 0x0b5fc64563b3e2a8,
        0x047f7684e9fc949d, 0xb99181f2d8f685ca, 0x284600e3f30e38c3
    };

    uint64_t t[16] = { 0 };
    for(unsigned int i = 0; i < sizeof(JUMP) / sizeof (*JUMP); i++)
        for(int b = 0; b < 64; b++) {
            if (JUMP[i] & 1ULL << b)
                for(int j = 0; j < 16; j++)
                    t[j] ^= s[(j + p) & 15];
            this->operator()();
        }

    for(int j = 0; j < 16; j++)
        s[(j + p) & 15] = t[j];
}

}
}

#endif
