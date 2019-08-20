/* -*- c++ -*- */
/*
 * Copyright 2019 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <gnuradio/fft/fft_shift.h>
#include <boost/test/unit_test.hpp>
#include <vector>

namespace gr {
namespace fft {

BOOST_AUTO_TEST_CASE(t1)
{
    fft::fft_shift<int> s(1023);

    std::vector<int> x_even{ 0, 1, 2, 3, -4, -3, -2, -1 };
    std::vector<int> y_even{ -4, -3, -2, -1, 0, 1, 2, 3 }; // expected result

    s.shift(x_even);
    BOOST_TEST(x_even == y_even, boost::test_tools::per_element());

    // two shifts should not change the result
    s.shift(x_even);
    s.shift(x_even);
    BOOST_TEST(x_even == y_even, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(t2)
{
    fft::fft_shift<int> s(7);

    std::vector<int> x_odd{ 0, 1, 2, 3, -3, -2, -1 };
    std::vector<int> y_odd{ -3, -2, -1, 0, 1, 2, 3 }; // expected result

    s.shift(x_odd);
    BOOST_TEST(x_odd == y_odd, boost::test_tools::per_element());
}

} /* namespace fft */
} /* namespace gr */
