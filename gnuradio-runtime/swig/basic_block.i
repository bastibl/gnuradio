/* -*- c++ -*- */
/*
 * Copyright 2006,2010,2013 Free Software Foundation, Inc.
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

%import "pmt_swig.i"

%begin %{
#define SWIG_PYTHON_2_UNICODE
%}

namespace gr {

  class gr::basic_block
  {
  protected:
    basic_block();

  public:
    typedef boost::shared_ptr<gr::basic_block> sptr;

    virtual ~basic_block();
    std::string name() const;
    long unique_id() const;
    std::string unique_name() const;
    gr::io_signature::sptr input_signature() const;
    gr::io_signature::sptr output_signature() const;
    gr::basic_block::sptr to_basic_block();
    virtual bool check_topology(int ninputs, int noutputs);
    std::string alias();
    void set_block_alias(std::string name);
    virtual void post(pmt::pmt_t which_port, pmt::pmt_t msg);
    pmt::pmt_t message_ports_in();
    pmt::pmt_t message_ports_out();
    pmt::pmt_t message_subscribers(pmt::pmt_t which_port);
  };
}

%template(basic_block_sptr) boost::shared_ptr<gr::basic_block>;

#ifdef SWIGPYTHON
%import py3compat.i

%pythoncode %{
basic_block_sptr.__repr__ = lambda self: "<basic_block %s (%d)>" % (self.name(), self.unique_id ())
%}
#endif
