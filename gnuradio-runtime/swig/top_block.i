/* -*- c++ -*- */
/*
 * Copyright 2007,2008,2010,2013 Free Software Foundation, Inc.
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

namespace gr {

  gr::top_block_sptr make_top_block(const std::string name)
    throw (std::logic_error);

  class top_block : public gr::hier_block
  {
  private:
    top_block(const std::string &name);

  public:
    ~top_block();

    void lock();
    void dump();

    std::string dot_graph();

    int max_noutput_items();
    void set_max_noutput_items(int nmax);

  };

}

%template(top_block_sptr) boost::shared_ptr<gr::top_block>;


#ifdef SWIGPYTHON

%inline %{
void top_block_run_unlocked(gr::top_block_sptr r) noexcept(false)
{
    GR_PYTHON_BLOCKING_CODE
    (
        r->run();
    )
}

void top_block_start_unlocked(gr::top_block_sptr r, int max_noutput_items) noexcept(false)
{
    GR_PYTHON_BLOCKING_CODE
    (
        r->start(max_noutput_items);
    )
}

void top_block_wait_unlocked(gr::top_block_sptr r) noexcept(false)
{
    GR_PYTHON_BLOCKING_CODE
    (
        r->wait();
    )
}

void top_block_stop_unlocked(gr::top_block_sptr r) noexcept(false)
{
    GR_PYTHON_BLOCKING_CODE
    (
        r->stop();
    )
}

void top_block_unlock_unlocked(gr::top_block_sptr r) noexcept(false)
{
    GR_PYTHON_BLOCKING_CODE
    (
        r->unlock();
    )
}

%}

#endif
