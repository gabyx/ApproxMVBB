// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef CYGWINPATCH_H_
#define CYGWINPATCH_H_
#include <sstream>
#include <string>

// This is a patch file for Cygwin-based compilers to overcome a bug present in
// the compilers. Code adapted from:
// http://stackoverflow.com/questions/12975341/to-string-is-not-a-member-of-std-says-so-g

namespace std
{
    template<typename T>
    std::string to_string(const T& n)
    {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
}  // namespace std

#endif