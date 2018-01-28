// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "ApproxMVBB/Common/MyMatrixTypeDefs.hpp"
#include "ApproxMVBB/Config/Config.hpp"

#ifdef __clang__
#    pragma clang diagnostic push
#    pragma clang diagnostic ignored "-Wglobal-constructors"
#    pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

namespace ApproxMVBB
{
    const Eigen::IOFormat MyMatrixIOFormat::Matlab   = Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
    const Eigen::IOFormat MyMatrixIOFormat::CommaSep = Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
    const Eigen::IOFormat MyMatrixIOFormat::SpaceSep = Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");
}  // namespace ApproxMVBB

#ifdef __clang__
#    pragma clang diagnostic pop
#endif
