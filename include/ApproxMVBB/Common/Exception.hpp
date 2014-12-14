// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef MVBB_Common_Exception_hpp
#define MVBB_Common_Exception_hpp

#include <stdexcept>
#include <exception>
#include <string>
#include <sstream>

namespace ApproxMVBB{
    class Exception : public std::runtime_error {
    public:
        Exception(const std::stringstream & ss): std::runtime_error(ss.str()){};
    private:

    };
};

#define ApproxMVBB_THROWEXCEPTION( message ) {std::stringstream ___s___ ; ___s___ << message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; throw Exception(___s___);}



#endif // Exception_hpp
