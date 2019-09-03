/* -*- c++ -*- */
/*
 * Copyright 2006,2007,2013 Free Software Foundation, Inc.
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

#ifndef INCLUDED_GR_RUNTIME_FLAT_FLOWGRAPH_H
#define INCLUDED_GR_RUNTIME_FLAT_FLOWGRAPH_H

#include <gnuradio/api.h>
#include <gnuradio/block.h>
#include <gnuradio/flowgraph.h>
#include <gnuradio/logger.h>

namespace gr {

// Create a shared pointer to a heap allocated gr::flat_flowgraph
// (types defined in gr_runtime_types.h)
GR_RUNTIME_API flat_flowgraph_sptr make_flat_flowgraph();

/*!
 *\brief Class specializing gr_flat_flowgraph that has all nodes
 * as blocks, with no hierarchy
 * \ingroup internal
 */
class GR_RUNTIME_API flat_flowgraph : public flowgraph
{
public:
    friend GR_RUNTIME_API flat_flowgraph_sptr make_flat_flowgraph();

    // Destruct an arbitrary gr::flat_flowgraph
    virtual ~flat_flowgraph();

    // Wire list of gr::block together in new flat_flowgraph
    void setup_connections(int max_noutput_items);

    // Return a string list of edges
    std::string edge_list();

    // Return a string list of msg edges
    std::string msg_edge_list();

    void dump();

    /*!
     * Make a vector of gr::block from a vector of gr::basic_block
     */
    static block_vector_t make_block_vector(basic_block_vector_t& blocks);

    /*!
     * replace hierarchical message connections with internal primitive ones
     */
    void replace_endpoint(const msg_endpoint& e, const msg_endpoint& r, bool is_src);

    /*!
     * remove a specific hier message connection after replacement
     */
    void clear_endpoint(const msg_endpoint& e, bool is_src);

    /*!
     * Enables export of perf. counters to ControlPort on all blocks in
     * the flowgraph.
     */
    void enable_pc_rpc();

    std::string dot_graph();

private:
    flat_flowgraph();

    block_executor_uptr allocate_block_executor(block_sptr block, int max_noutput_items);
    buffer_sptr allocate_buffer(block_sptr block, int port);
    void connect_block_inputs(block_sptr block);

    gr::logger_ptr d_logger;
    gr::logger_ptr d_debug_logger;
};

} /* namespace gr */

#endif /* INCLUDED_GR_RUNTIME_FLAT_FLOWGRAPH_H */
