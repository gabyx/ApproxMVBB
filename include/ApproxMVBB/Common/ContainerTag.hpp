#ifndef GRSF_Common_ContainerTag_hpp
#define GRSF_Common_ContainerTag_hpp

#include <type_traits>

namespace ContainerTags{

namespace details{
    inline constexpr auto is_container_impl(...) -> std::false_type {
        return std::false_type{};
    }

    template <typename C>
    constexpr auto is_container_impl(C const* c) ->
        decltype(begin(*c), end(*c), std::true_type{})
    {
        return std::true_type{};
    }


    inline constexpr auto is_associative_container_impl(...)
        -> std::false_type
    { return std::false_type{}; }

    template <typename C, typename = typename C::key_type>
    constexpr auto is_associative_container_impl(C const*) -> std::true_type {
        return std::true_type{};
    }
};


    template <typename C>
    constexpr auto is_container(C const& c) -> decltype(details::is_container_impl(&c)) {
        return details::is_container_impl(&c);
    }

    template <typename C>
    constexpr auto is_associative(C const& c)
        -> decltype(details::is_associative_container_impl(&c))
    {
        return details::is_associative_container_impl(&c);
    }

    template<typename C>
    using IteratorCategoryOf = typename std::iterator_traits<typename C::iterator>::iterator_category;

    template<typename C>
    using has_randomAccessIterator = std::is_base_of<std::random_access_iterator_tag, IteratorCategoryOf<C> >;
    
    template<typename C>
    using has_bidirectionalIterator = std::is_base_of<std::bidirectional_iterator_tag, IteratorCategoryOf<C> >;
};


#endif // AssociativeContainer_hpp

