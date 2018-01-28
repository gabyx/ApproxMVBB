// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_ContainerFunctions_hpp
#define ApproxMVBB_ContainerFunctions_hpp

namespace ApproxMVBB
{
    namespace ContainerFunctions
    {
        /** Move elements in the range [b,e) to front of the container if Func returns
 * true
 * This function is especially efficient if we have little items which need to
 * move to the front
 * This function respects the order of the elements
 * @param b iterator to the first item
 * @param e iterator to the last item
 * @return Iterator r  where the range [r,e] is the back part of the vector where
 * Func returned true
 */
        template<typename Iterator, typename Func>
        Iterator moveElementsToFrontIf(Iterator b, Iterator e, Func f)
        {
            Iterator w = b;  // write pointer
            while(b != e)
            {
                if(!f(*b))
                {  // if not move to front increment read pointer
                    b++;
                    continue;
                }

                if(w != b)
                {             // copy only if not at same position!
                    *w = *b;  // copy  value to front (test is not true)
                }
                ++w;
                ++b;
            }
            return w;
        }

        /** Move elements in the range [b,e] to the back of the container if Func
 * returns true
 * This function is especially efficient if we have little items which need to
 * move to the front
 * Caution: This function does not respect the order of the items (so do not use
 * if container should stay sorted)!
 * @param b begin iterator
 * @param e end iterator (no past the end iterator of a container!)
 * @return Iterator r  where the range [r,e] is the back part of the vector where
 * Func returned true
 */
        template<typename Iterator, typename Func>
        Iterator moveElementsToBackIf(Iterator b, Iterator e, Func f)
        {
            if(b == e)
            {
                return b;
            }
            while(b != e)
            {
                if(f(*b))
                {
                    // Move end to next swappable item
                    while(f(*e))
                    {
                        --e;
                        if(e == b)
                        {
                            return e;
                        };
                    }

                    // swap with back
                    if(b != e)
                    {
                        std::swap(*b, *e);
                    }
                }
                ++b;
            }
            return f(*e) ? e : ++e;
        }

        /** Move all sequences S in [b,e) to the front where each element i
 * of the subsequence S =[start,end] fullfils c(start,i).
 * Example: 1 2 2 2 3 3 3 4 5 5 6  -> 1 2 3 4 5 6 (with Comp c=AlmostEqual)
 * This function is especially efficient if we have little items which need to
 * move to the front
 * This function respects the order of the elements
 * @return Iterator r  where the range [r,e] is the back part of the vector where
 * Comp c returned true
 */
        template<typename Iterator, typename Comp>
        Iterator moveConsecutiveToFrontIf(Iterator b, Iterator e, Comp c)
        {
            if(std::distance(b, e) < 2)
            {
                return e;
            }

            Iterator comp  = b;
            Iterator write = ++b;
            while(b != e)
            {
                if(!c(*comp, *b))
                {  // if we can skip element or if
                    ++b;
                    continue;
                }

                if(write != b)
                {                 // copy only if not at same position!
                    *write = *b;  // copy  value to front (test is not true)
                }
                // std::cout << *dest << std::endl;
                comp = write++;
                ++b;
            }

            return write;
        }
        /** Move all sequences S in [b,e) to the front where each element i
 * of the subsequence S =[start,end] fullfils Func(start,i).
 * This function skips elements for which s(i) is true
 * Example: 1 2 2 2 3 3 3 4 5 5 6  -> 1 2 3 4 5 6 (with Func=AlmostEqual)
 * This function is especially efficient if we have little items which need to
 * move to the front
 * This function respects the order of the elements
 * @return Iterator r  where the range [r,e] is the back part of the vector where
 * Func returned true
 */
        template<typename Iterator, typename Func, typename Skip>
        Iterator moveConsecutiveToFrontIf(Iterator b, Iterator e, Func f, Skip s)
        {
            if(std::distance(b, e) < 2)
            {
                return e;
            }
            Iterator comp;
            Iterator write = b;
            // skip all at beginning
            while(b != e && s(*b))
            {
                ++b;
            }
            if(b != e)
            {
                // write first element to front
                if(write != b)
                {
                    *write = *b;
                }
                comp = write++;  // shift write pointer (comp is previous)

                // Start loop over all elements
                while(b != e)
                {
                    if(s(*b) || !f(*comp, *b))
                    {
                        ++b;
                        continue;
                    }

                    if(write != b)
                    {                 // copy only if not at same position!
                        *write = *b;  // copy  value to front (test is not true)
                    }
                    // std::cout << *dest << std::endl;
                    comp = write++;
                    ++b;
                }
            }

            return write;
        }
    }  // namespace ContainerFunctions
}  // namespace ApproxMVBB
#endif  // ContainerFunctions_hpp
