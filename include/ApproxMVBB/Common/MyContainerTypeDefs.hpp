#ifndef ApproxMVBB_Common_MyContainerTypeDefs_hpp
#define ApproxMVBB_Common_MyContainerTypeDefs_hpp

//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <map>
#include <unordered_map>
#include <vector>

/** @brief
 *	These are some container definitions
 */
namespace ApproxMVBB
{
    namespace MyContainers
    {
        // Sepcial STL map where the type is 16byte aligned
        template<typename Key, typename Type, typename Comp = std::less<Key>>
        using StdMapAligned = std::map<Key, Type, Comp, Eigen::aligned_allocator<std::pair<const Key, Type>>>;

        // Sepcial STL map where the type is 16byte aligned
        template<typename Key, typename Type, typename Hash = std::hash<Key>, typename Pred = std::equal_to<Key>>
        using StdUMapAligned = std::unordered_map<Key, Type, Hash, Pred, Eigen::aligned_allocator<std::pair<const Key, Type>>>;

        // Special STL vectors where the type is 16byte aligned
        template<typename Type>
        using StdVecAligned = std::vector<Type, Eigen::aligned_allocator<Type>>;
    }  // namespace MyContainers
}  // namespace ApproxMVBB

    /**
     * @brief This macro is used to typedef all custom container types.
     */
#    define ApproxMVBB_DEFINE_CONTAINER_TYPES                                                   \
                                                                                                \
        template<typename Key, typename Type, typename Comp>                                    \
        using StdMapAligned = ApproxMVBB::MyContainers::StdMapAligned<Key, Type, Comp>;         \
                                                                                                \
        template<typename Key, typename Type, typename Hash, typename Pred>                     \
        using StdUMapAligned = ApproxMVBB::MyContainers::StdUMapAligned<Key, Type, Hash, Pred>; \
                                                                                                \
        template<typename Type>                                                                 \
        using StdVecAligned = ApproxMVBB::MyContainers::StdVecAligned<Type>

#endif
