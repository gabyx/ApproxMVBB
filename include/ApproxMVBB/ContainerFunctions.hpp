// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_ContainerFunctions_hpp
#define ApproxMVBB_ContainerFunctions_hpp

namespace ApproxMVBB{
namespace ContainerFunctions {

/** Move elements in the range [b,e) to front of the container if Func returns true
* This function is especially efficient if we have little items which need to move to the front
* This function respects the order of the elements
* @param b iterator to the first item
* @param e iterator to the last item
* @return Iterator r  where the range [r,e] is the back part of the vector where Func returned true
*/
template<typename Iterator, typename Func>
Iterator moveElementsToFrontIf(Iterator b, Iterator  e, Func f) {

    Iterator w = b;  // write pointer
    while  ( b != e ) {
        if( !f(*b) ) { // if not move to front increment read pointer
            b++;
            continue;
        }

        if(w!=b) {   // copy only if not at same position!
            *w = *b; // copy  value to front (test is not true)
        }
        ++w;
        ++b;
    }
    return w;
}

/** Move elements in the range [b,e] to the back of the container if Func returns true
* This function is especially efficient if we have little items which need to move to the front
* Caution: This function does not respect the order of the items (so do not use if container should stay sorted)!
* @param b begin iterator
* @param e end iterator (no past the end iterator of a container!)
* @return Iterator r  where the range [r,e] is the back part of the vector where Func returned true
*/
template<typename Iterator,typename Func>
Iterator moveElementsToBackIf(Iterator b, Iterator  e, Func f) {

    if( b == e  ) {
        return b;
    }
    while(b != e){

        if(f(*b)){
            // Move end to next swappable item
            while(f(*e)){
            	--e;
                if( e == b){
                    return e;
                };
            }

            // swap with back
            if(b!=e){
            	std::swap(*b,*e);
            }
        }
        ++b;
    }
    return f(*e)? e : ++e;
}


/** Move element b of consecutive elements a,b to front of the container if Func(a,b) returns true
* This function is especially efficient if we have little items which need to move to the front
* This function respects the order of the elements
* @return Iterator r  where the range [r,e] is the back part of the vector where Func returned true
*/
template<typename Iterator, typename Func>
Iterator moveConsecutiveToFrontIf(Iterator b, Iterator  e, Func f) {


    if( std::distance(b,e)<2 ) {
        return e;
    }

    Iterator comp = b;
    Iterator dest = ++b;
    while  ( b != e ) {
        if( f(*std::prev(dest), *b ) ) { // *b == std::next(b)  for example
            ++b;
            continue;
        }

        if(dest!=b) {   // copy only if not at same position!
            *dest = *b; // copy  value to front (test is not true)
        }
        //std::cout << *dest << std::endl;
        ++dest;
        ++b;
    }

    return dest;
}
};
};
#endif // ContainerFunctions_hpp
