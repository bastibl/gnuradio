/* -*- c++ -*- */
/*
 * Copyright 2015 Free Software Foundation, Inc.
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

#include <gnuradio/block_executor.h>
#include <gnuradio/buffer.h>
#include <gnuradio/flat_flowgraph.h>
#include <gnuradio/logger.h>
#include <gnuradio/prefs.h>
#include <volk/volk.h>
#include <boost/format.hpp>
#include <iostream>
#include <map>

namespace gr {

// 32Kbyte buffer size between blocks
static const unsigned int s_fixed_buffer_size = 32 * (1L << 10);

flat_flowgraph_sptr make_flat_flowgraph()
{
    return flat_flowgraph_sptr(new flat_flowgraph());
}

flat_flowgraph::flat_flowgraph()
{
    configure_default_loggers(d_logger, d_debug_logger, "flat_flowgraph");
}

flat_flowgraph::~flat_flowgraph() {

    basic_block_vector_t used_blocks = calc_used_blocks();
    block_vector_t blocks = flat_flowgraph::make_block_vector(used_blocks);

    for (size_t i = 0; i < blocks.size(); i++) {
        blocks[i]->set_executor(nullptr);
    }
}

void flat_flowgraph::setup_connections(int max_noutput_items)
{
    basic_block_vector_t blocks = calc_used_blocks();

    // Assign block executor to blocks
    for (basic_block_viter_t p = blocks.begin(); p != blocks.end(); p++) {

        block_sptr b = cast_to_block_sptr(*p);

        // If set, use internal value instead of global value
        int block_max_noutput_items;
        if (b->is_set_max_noutput_items()) {
            block_max_noutput_items = b->max_noutput_items();
        } else {
            block_max_noutput_items = max_noutput_items;
        }

        b->set_executor(allocate_block_executor(b, block_max_noutput_items));
    }

    // Connect inputs to outputs for each block
    for (basic_block_viter_t p = blocks.begin(); p != blocks.end(); p++) {

        block_sptr block = cast_to_block_sptr(*p);

        connect_block_inputs(block);
        block->set_unaligned(0);
        block->set_is_unaligned(false);
    }

    // Connect message ports connetions
    for (msg_edge_viter_t i = d_msg_edges.begin(); i != d_msg_edges.end(); i++) {
        GR_LOG_DEBUG(
            d_debug_logger,
            boost::format("flat_fg connecting msg primitives: (%s, %s)->(%s, %s)\n") %
                i->src().block() % i->src().port() % i->dst().block() % i->dst().port());
        i->src().block()->message_port_sub(
            i->src().port(), pmt::cons(i->dst().block()->alias_pmt(), i->dst().port()));
    }
}

block_executor_sptr flat_flowgraph::allocate_block_executor(block_sptr block,
                                                          int max_noutput_items)
{
    int ninputs = calc_used_ports(block, true).size();
    int noutputs = calc_used_ports(block, false).size();
    block_executor_sptr executor =
        make_block_executor(block, ninputs, noutputs, max_noutput_items);

    block_sptr grblock = cast_to_block_sptr(block);
    if (!grblock)
        throw std::runtime_error(
            (boost::format("allocate_block_executor found non-gr::block (%s)") %
             block->alias())
                .str());

    GR_LOG_DEBUG(d_debug_logger, "Creating block executor for " + block->alias());

    for (int i = 0; i < noutputs; i++) {
        grblock->expand_minmax_buffer(i);

        buffer_sptr buffer = allocate_buffer(block, i);
        GR_LOG_DEBUG(d_debug_logger,
                     "Allocated buffer for output " + block->alias() + " " +
                         std::to_string(i));
        executor->set_output(i, buffer);

        // Update the block's max_output_buffer based on what was actually allocated.
        if ((grblock->max_output_buffer(i) != buffer->bufsize()) &&
            (grblock->max_output_buffer(i) != -1))
            GR_LOG_WARN(d_logger,
                        boost::format("Block (%1%) max output buffer set to %2%"
                                      " instead of requested %3%") %
                            grblock->alias() % buffer->bufsize() %
                            grblock->max_output_buffer(i));
        grblock->set_max_output_buffer(i, buffer->bufsize());
    }

    return executor;
}

buffer_sptr flat_flowgraph::allocate_buffer(block_sptr grblock, int port)
{
    int item_size = grblock->output_signature()->sizeof_stream_item(port);

    // *2 because we're now only filling them 1/2 way in order to
    // increase the available parallelism when using the TPB scheduler.
    // (We're double buffering, where we used to single buffer)
    int nitems = s_fixed_buffer_size * 2 / item_size;

    // Make sure there are at least twice the output_multiple no. of items
    if (nitems < 2 * grblock->output_multiple()) // Note: this means output_multiple()
        nitems = 2 * grblock->output_multiple(); // can't be changed by block dynamically

    // If any downstream blocks are decimators and/or have a large output_multiple,
    // ensure we have a buffer at least twice their decimation factor*output_multiple
    basic_block_vector_t blocks = calc_downstream_blocks(grblock, port);

    // limit buffer size if indicated
    if (grblock->max_output_buffer(port) > 0) {
        // std::cout << "constraining output items to " << block->max_output_buffer(port)
        // << "\n";
        nitems = std::min((long)nitems, (long)grblock->max_output_buffer(port));
        nitems -= nitems % grblock->output_multiple();
        if (nitems < 1)
            throw std::runtime_error("problems allocating a buffer with the given max "
                                     "output buffer constraint!");
    } else if (grblock->min_output_buffer(port) > 0) {
        nitems = std::max((long)nitems, (long)grblock->min_output_buffer(port));
        nitems -= nitems % grblock->output_multiple();
        if (nitems < 1)
            throw std::runtime_error("problems allocating a buffer with the given min "
                                     "output buffer constraint!");
    }

    for (basic_block_viter_t p = blocks.begin(); p != blocks.end(); p++) {
        block_sptr dgrblock = cast_to_block_sptr(*p);
        if (!dgrblock)
            throw std::runtime_error("allocate_buffer found non-gr::block");

        double decimation = (1.0 / dgrblock->relative_rate());
        int multiple = dgrblock->output_multiple();
        int history = dgrblock->history();
        nitems =
            std::max(nitems, static_cast<int>(2 * (decimation * multiple + history)));
    }

    //  std::cout << "make_buffer(" << nitems << ", " << item_size << ", " << grblock <<
    //  "\n";
    // We're going to let this fail once and retry. If that fails,
    // throw and exit.
    buffer_sptr b;
    try {
        b = make_buffer(nitems, item_size, grblock);
    } catch (std::bad_alloc&) {
        b = make_buffer(nitems, item_size, grblock);
    }

    // Set the max noutput items size here to make sure it's always
    // set in the block and available in the start() method.
    // But don't overwrite if the user has set this externally.
    if (!grblock->is_set_max_noutput_items())
        grblock->set_max_noutput_items(nitems);

    return b;
}

void flat_flowgraph::connect_block_inputs(block_sptr grblock)
{
    // Get its executor and edges that feed into it
    block_executor_sptr executor = grblock->executor();
    edge_vector_t in_edges = calc_upstream_edges(grblock);

    // For each edge that feeds into it
    for (edge_viter_t e = in_edges.begin(); e != in_edges.end(); e++) {
        // Set the buffer reader on the destination port to the output
        // buffer on the source port
        int dst_port = e->dst().port();
        int src_port = e->src().port();
        basic_block::sptr src_block = e->src().block();
        block_sptr src_grblock = cast_to_block_sptr(src_block);
        if (!src_grblock)
            throw std::runtime_error("connect_block_inputs found non-gr::block");
        buffer_sptr src_buffer = src_grblock->executor()->output(src_port);


        GR_LOG_DEBUG(d_debug_logger,
                     "Setting input " + std::to_string(dst_port) + " from edge " +
                         (*e).identifier());

        executor->set_input(dst_port,
                          buffer_add_reader(src_buffer,
                                            grblock->history() - 1,
                                            grblock,
                                            grblock->sample_delay(src_port)));
    }
}

std::string flat_flowgraph::edge_list()
{
    std::stringstream s;
    for (edge_viter_t e = d_edges.begin(); e != d_edges.end(); e++)
        s << (*e) << std::endl;
    return s.str();
}

std::string flat_flowgraph::msg_edge_list()
{
    std::stringstream s;
    for (msg_edge_viter_t e = d_msg_edges.begin(); e != d_msg_edges.end(); e++)
        s << (*e) << std::endl;
    return s.str();
}

void flat_flowgraph::dump()
{
    for (edge_viter_t e = d_edges.begin(); e != d_edges.end(); e++)
        std::cout << " edge: " << (*e) << std::endl;

    for (basic_block_viter_t p = d_blocks.begin(); p != d_blocks.end(); p++) {
        std::cout << " block: " << (*p) << std::endl;
        block_executor_sptr executor = cast_to_block_sptr(*p)->executor();
        std::cout << "  executor @" << executor << ":" << std::endl;

        int ni = executor->ninputs();
        int no = executor->noutputs();
        for (int i = 0; i < no; i++) {
            buffer_sptr buffer = executor->output(i);
            std::cout << "   output " << i << ": " << buffer << std::endl;
        }

        for (int i = 0; i < ni; i++) {
            buffer_reader_sptr reader = executor->input(i);
            std::cout << "   reader " << i << ": " << reader
                      << " reading from buffer=" << reader->buffer() << std::endl;
        }
    }
}

block_vector_t flat_flowgraph::make_block_vector(basic_block_vector_t& blocks)
{
    block_vector_t result;
    for (basic_block_viter_t p = blocks.begin(); p != blocks.end(); p++) {
        result.push_back(cast_to_block_sptr(*p));
    }

    return result;
}

void flat_flowgraph::clear_endpoint(const msg_endpoint& e, bool is_src)
{
    for (size_t i = 0; i < d_msg_edges.size(); i++) {
        if (is_src) {
            if (d_msg_edges[i].src() == e) {
                d_msg_edges.erase(d_msg_edges.begin() + i);
                i--;
            }
        } else {
            if (d_msg_edges[i].dst() == e) {
                d_msg_edges.erase(d_msg_edges.begin() + i);
                i--;
            }
        }
    }
}

void flat_flowgraph::replace_endpoint(const msg_endpoint& e,
                                      const msg_endpoint& r,
                                      bool is_src)
{
    size_t n_replr(0);
    GR_LOG_DEBUG(d_debug_logger,
                 boost::format("flat_flowgraph::replace_endpoint( %s, %s, %d )\n") %
                     e.block() % r.block() % is_src);
    for (size_t i = 0; i < d_msg_edges.size(); i++) {
        if (is_src) {
            if (d_msg_edges[i].src() == e) {
                GR_LOG_DEBUG(
                    d_debug_logger,
                    boost::format(
                        "flat_flowgraph::replace_endpoint() flattening to ( %s, %s )\n") %
                        r % d_msg_edges[i].dst())
                d_msg_edges.push_back(msg_edge(r, d_msg_edges[i].dst()));
                n_replr++;
            }
        } else {
            if (d_msg_edges[i].dst() == e) {
                GR_LOG_DEBUG(
                    d_debug_logger,
                    boost::format(
                        "flat_flowgraph::replace_endpoint() flattening to ( %s, %s )\n") %
                        r % d_msg_edges[i].src());
                d_msg_edges.push_back(msg_edge(d_msg_edges[i].src(), r));
                n_replr++;
            }
        }
    }
}

std::string flat_flowgraph::dot_graph()
{
    basic_block_vector_t blocks = calc_used_blocks();
    edge_vector_t all_edges = edges();
    msg_edge_vector_t all_msg_edges = msg_edges();

    std::stringstream out;

    out << "digraph flowgraph {" << std::endl;

    // Define nodes and set labels
    for (basic_block_viter_t block = blocks.begin(); block != blocks.end(); ++block) {
        out << (*block)->unique_id() << " [ label=\"" << (*block)->alias() << "\" ]"
            << std::endl;
    }

    // Define edges
    for (edge_viter_t edge = all_edges.begin(); edge != all_edges.end(); ++edge) {
        out << edge->src().block()->unique_id() << " -> "
            << edge->dst().block()->unique_id() << std::endl;
    }

    for (msg_edge_viter_t edge = all_msg_edges.begin(); edge != all_msg_edges.end();
         edge++) {
        out << edge->src().block()->unique_id() << " -> "
            << edge->dst().block()->unique_id() << " [color=blue]" << std::endl;
    }

    out << "}" << std::endl;

    return out.str();
}


void flat_flowgraph::enable_pc_rpc()
{
#ifdef GR_PERFORMANCE_COUNTERS
    if (prefs::singleton()->get_bool("PerfCounters", "on", false)) {
        basic_block_viter_t p;
        for (p = d_blocks.begin(); p != d_blocks.end(); p++) {
            block_sptr block = cast_to_block_sptr(*p);
            if (!block->is_pc_rpc_set())
                block->setup_pc_rpc();
        }
    }
#endif /* GR_PERFORMANCE_COUNTERS */
}

} /* namespace gr */
