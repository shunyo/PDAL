// (C) Copyright Jeremy Siek 2002.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_ITERATOR_CONCEPTS_HPP
#define BOOST_ITERATOR_CONCEPTS_HPP

#include <boost/concept_check.hpp>
#include <boost/iterator/iterator_categories.hpp>

// Use pdalboost::detail::iterator_traits to work around some MSVC/Dinkumware problems.
#include <boost/detail/iterator.hpp>

#include <boost/type_traits/is_same.hpp>
#include <boost/type_traits/is_integral.hpp>

#include <boost/mpl/bool.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/and.hpp>
#include <boost/mpl/or.hpp>

#include <boost/static_assert.hpp>

// Use boost/limits to work around missing limits headers on some compilers
#include <boost/limits.hpp>
#include <boost/config.hpp>

#include <algorithm>

#include <boost/concept/detail/concept_def.hpp>

namespace pdalboost_concepts
{
  // Used a different namespace here (instead of "boost") so that the
  // concept descriptions do not take for granted the names in
  // namespace pdalboost.

  //===========================================================================
  // Iterator Access Concepts

  BOOST_concept(ReadableIterator,(Iterator))
    : pdalboost::Assignable<Iterator>
    , pdalboost::CopyConstructible<Iterator>

  {
      typedef BOOST_DEDUCED_TYPENAME pdalboost::detail::iterator_traits<Iterator>::value_type value_type;
      typedef BOOST_DEDUCED_TYPENAME pdalboost::detail::iterator_traits<Iterator>::reference reference;

      BOOST_CONCEPT_USAGE(ReadableIterator)
      {

          value_type v = *i;
          pdalboost::ignore_unused_variable_warning(v);
      }
  private:
      Iterator i;
  };
  
  template <
      typename Iterator
    , typename ValueType = BOOST_DEDUCED_TYPENAME pdalboost::detail::iterator_traits<Iterator>::value_type
  >
  struct WritableIterator
    : pdalboost::CopyConstructible<Iterator>
  {
      BOOST_CONCEPT_USAGE(WritableIterator)
      {
          *i = v;
      }
  private:
      ValueType v;
      Iterator i;
  };

  template <
      typename Iterator
    , typename ValueType = BOOST_DEDUCED_TYPENAME pdalboost::detail::iterator_traits<Iterator>::value_type
  >
  struct WritableIteratorConcept : WritableIterator<Iterator,ValueType> {};
  
  BOOST_concept(SwappableIterator,(Iterator))
  {
      BOOST_CONCEPT_USAGE(SwappableIterator)
      {
          std::iter_swap(i1, i2);
      }
  private:
      Iterator i1;
      Iterator i2;
  };

  BOOST_concept(LvalueIterator,(Iterator))
  {
      typedef typename pdalboost::detail::iterator_traits<Iterator>::value_type value_type;
      
      BOOST_CONCEPT_USAGE(LvalueIterator)
      {
        value_type& r = const_cast<value_type&>(*i);
        pdalboost::ignore_unused_variable_warning(r);
      }
  private:
      Iterator i;
  };

  
  //===========================================================================
  // Iterator Traversal Concepts

  BOOST_concept(IncrementableIterator,(Iterator))
    : pdalboost::Assignable<Iterator>
    , pdalboost::CopyConstructible<Iterator>
  {
      typedef typename pdalboost::iterator_traversal<Iterator>::type traversal_category;

      BOOST_CONCEPT_ASSERT((
        pdalboost::Convertible<
            traversal_category
          , pdalboost::incrementable_traversal_tag
        >));

      BOOST_CONCEPT_USAGE(IncrementableIterator)
      {
          ++i;
          (void)i++;
      }
  private:
      Iterator i;
  };

  BOOST_concept(SinglePassIterator,(Iterator))
    : IncrementableIterator<Iterator>
    , pdalboost::EqualityComparable<Iterator>

  {
      BOOST_CONCEPT_ASSERT((
          pdalboost::Convertible<
             BOOST_DEDUCED_TYPENAME SinglePassIterator::traversal_category
           , pdalboost::single_pass_traversal_tag
          > ));
  };

  BOOST_concept(ForwardTraversal,(Iterator))
    : SinglePassIterator<Iterator>
    , pdalboost::DefaultConstructible<Iterator>
  {
      typedef typename pdalboost::detail::iterator_traits<Iterator>::difference_type difference_type;
      
      BOOST_MPL_ASSERT((pdalboost::is_integral<difference_type>));
      BOOST_MPL_ASSERT_RELATION(std::numeric_limits<difference_type>::is_signed, ==, true);

      BOOST_CONCEPT_ASSERT((
          pdalboost::Convertible<
             BOOST_DEDUCED_TYPENAME ForwardTraversal::traversal_category
           , pdalboost::forward_traversal_tag
          > ));
  };
  
  BOOST_concept(BidirectionalTraversal,(Iterator))
    : ForwardTraversal<Iterator>
  {
      BOOST_CONCEPT_ASSERT((
          pdalboost::Convertible<
             BOOST_DEDUCED_TYPENAME BidirectionalTraversal::traversal_category
           , pdalboost::bidirectional_traversal_tag
          > ));

      BOOST_CONCEPT_USAGE(BidirectionalTraversal)
      {
          --i;
          (void)i--;
      }
   private:
      Iterator i;
  };

  BOOST_concept(RandomAccessTraversal,(Iterator))
    : BidirectionalTraversal<Iterator>
  {
      BOOST_CONCEPT_ASSERT((
          pdalboost::Convertible<
             BOOST_DEDUCED_TYPENAME RandomAccessTraversal::traversal_category
           , pdalboost::random_access_traversal_tag
          > ));

      BOOST_CONCEPT_USAGE(RandomAccessTraversal)
      {
          i += n;
          i = i + n;
          i = n + i;
          i -= n;
          i = i - n;
          n = i - j;
      }
      
   private:
      typename BidirectionalTraversal<Iterator>::difference_type n;
      Iterator i, j;
  };

  //===========================================================================
  // Iterator Interoperability 

  namespace detail
  {
    template <typename Iterator1, typename Iterator2>
    void interop_single_pass_constraints(Iterator1 const& i1, Iterator2 const& i2)
    {
        bool b;
        b = i1 == i2;
        b = i1 != i2;

        b = i2 == i1;
        b = i2 != i1;
        pdalboost::ignore_unused_variable_warning(b);
    }

    template <typename Iterator1, typename Iterator2>
    void interop_rand_access_constraints(
        Iterator1 const& i1, Iterator2 const& i2,
        pdalboost::random_access_traversal_tag, pdalboost::random_access_traversal_tag)
    {
        bool b;
        typename pdalboost::detail::iterator_traits<Iterator2>::difference_type n;
        b = i1 <  i2;
        b = i1 <= i2;
        b = i1 >  i2;
        b = i1 >= i2;
        n = i1 -  i2;

        b = i2 <  i1;
        b = i2 <= i1;
        b = i2 >  i1;
        b = i2 >= i1;
        n = i2 -  i1;
        pdalboost::ignore_unused_variable_warning(b);
        pdalboost::ignore_unused_variable_warning(n);
    }

    template <typename Iterator1, typename Iterator2>
    void interop_rand_access_constraints(
        Iterator1 const&, Iterator2 const&,
        pdalboost::single_pass_traversal_tag, pdalboost::single_pass_traversal_tag)
    { }

  } // namespace detail

  BOOST_concept(InteroperableIterator,(Iterator)(ConstIterator))
  {
   private:
      typedef typename pdalboost::detail::pure_traversal_tag<
          typename pdalboost::iterator_traversal<
              Iterator
          >::type
      >::type traversal_category;

      typedef typename pdalboost::detail::pure_traversal_tag<
          typename pdalboost::iterator_traversal<
              ConstIterator
          >::type
      >::type const_traversal_category;
      
  public:
      BOOST_CONCEPT_ASSERT((SinglePassIterator<Iterator>));
      BOOST_CONCEPT_ASSERT((SinglePassIterator<ConstIterator>));

      BOOST_CONCEPT_USAGE(InteroperableIterator)
      {
          detail::interop_single_pass_constraints(i, ci);
          detail::interop_rand_access_constraints(i, ci, traversal_category(), const_traversal_category());

          ci = i;
      }
      
   private:
      Iterator      i;
      ConstIterator ci;
  };

} // namespace pdalboost_concepts

#include <boost/concept/detail/concept_undef.hpp>

#endif // BOOST_ITERATOR_CONCEPTS_HPP
