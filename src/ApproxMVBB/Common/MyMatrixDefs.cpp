// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE

namespace ApproxMVBB{
    Eigen::IOFormat MyMatrixIOFormat::Matlab(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
    Eigen::IOFormat MyMatrixIOFormat::CommaSep(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
    Eigen::IOFormat MyMatrixIOFormat::SpaceSep(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");
};
