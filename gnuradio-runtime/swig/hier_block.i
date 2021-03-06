/* -*- c++ -*- */
/*
 * Copyright 2005-2007,2013 Free Software Foundation, Inc.
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

%include <basic_block.i>

namespace gr {

  gr::hier_block_sptr
  make_hier_block(const std::string name,
                  gr::io_signature::sptr input_signature,
                  gr::io_signature::sptr output_signature) noexcept(false);


  // Rename connect and disconnect so that we can more easily build a
  // better interface in scripting land.
  %rename(primitive_connect) hier_block::connect;
  %rename(primitive_disconnect) hier_block::disconnect;
  %rename(primitive_msg_connect) hier_block::msg_connect;
  %rename(primitive_msg_disconnect) hier_block::msg_disconnect;
  %rename(primitive_message_port_register_in) hier_block::message_port_register_in;
  %rename(primitive_message_port_register_out) hier_block::message_port_register_out;

  class hier_block : public gr::basic_block
  {
  private:
    hier_block(const std::string name,
                gr::io_signature::sptr input_signature,
                gr::io_signature::sptr output_signature);

  public:

    ~hier_block ();

    void connect(gr::basic_block_sptr block) noexcept(false);
    void connect(gr::basic_block_sptr src, int src_port,
                 gr::basic_block_sptr dst, int dst_port) noexcept(false);
    void msg_connect(gr::basic_block_sptr src, std::string srcport,
                     gr::basic_block_sptr dst,  std::string dstport) noexcept(false);
    void msg_disconnect(gr::basic_block_sptr src, std::string srcport,
                        gr::basic_block_sptr dst, std::string dstport) noexcept(false);

    void disconnect(gr::basic_block_sptr block) noexcept(false);
    void disconnect(gr::basic_block_sptr src, int src_port,
                    gr::basic_block_sptr dst, int dst_port) noexcept(false);
    void disconnect_all();

    void message_port_register_in(std::string port_id);
    void message_port_register_out(std::string port_id);

    void set_processor_affinity(const std::vector<int> &mask);
    void unset_processor_affinity();
    std::vector<int> processor_affinity();

    void set_log_level(std::string level);
    std::string log_level();

    // Methods to manage block's min/max buffer sizes.
    size_t max_output_buffer(int i);
    void set_max_output_buffer(size_t max_output_buffer);
    void set_max_output_buffer(int port, size_t max_output_buffer);
    size_t min_output_buffer(int i);
    void set_min_output_buffer(size_t min_output_buffer);
    void set_min_output_buffer(int port, size_t min_output_buffer);
  };
}

%template(hier_block_sptr) boost::shared_ptr<gr::hier_block>;
