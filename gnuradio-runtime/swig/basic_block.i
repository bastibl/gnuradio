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

  class msg_endpoint;

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
    std::string alias() const;
    void set_block_alias(const std::string name);
    virtual void post(const std::string, pmt::pmt_t msg);
    virtual std::vector<std::string> message_ports_in() const;
    std::vector<std::string> message_ports_out() const;
    std::vector<msg_endpoint> message_subscribers(const std::string which_port);
  };

  class msg_endpoint
  {
  private:
      basic_block::sptr d_basic_block;
      std::string d_port;

  public:
      msg_endpoint() : d_basic_block(nullptr), d_port("") {}
      msg_endpoint(basic_block::sptr block, std::string port)
      {
          d_basic_block = block;
          d_port = port;
      }
      basic_block::sptr block() const { return d_basic_block; }
      std::string port() const { return d_port; }
      std::string identifier() const
      {
          return d_basic_block->alias() + ":" + d_port;
      }

      bool operator==(const msg_endpoint& other) const;
  };

  inline bool msg_endpoint::operator==(const msg_endpoint& other) const
  {
      return (d_basic_block == other.d_basic_block && d_port == other.d_port);
  }

  // Hold vectors of gr::endpoint objects
  typedef std::vector<endpoint> endpoint_vector_t;
  typedef std::vector<endpoint>::iterator endpoint_viter_t;

}

%template(msg_endpoint_vector_t) std::vector<gr::msg_endpoint>;

%template(basic_block_sptr) boost::shared_ptr<gr::basic_block>;

#ifdef SWIGPYTHON
%import py3compat.i

%pythoncode %{
basic_block_sptr.__repr__ = lambda self: "<basic_block %s (%d)>" % (self.name(), self.unique_id ())
%}
#endif
