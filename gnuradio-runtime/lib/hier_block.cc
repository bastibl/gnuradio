/* -*- c++ -*- */
/*
 * Copyright 2006-2008,2013 Free Software Foundation, Inc.
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

#include <gnuradio/flat_flowgraph.h>
#include <gnuradio/flowgraph.h>
#include <gnuradio/hier_block.h>
#include <gnuradio/io_signature.h>
#include <iostream>

#define HIER_BLOCK_DEBUG 1

namespace gr {

hier_block::sptr hier_block::make(const std::string& name,
                                  gr::io_signature::sptr input_signature,
                                  gr::io_signature::sptr output_signature)
{
    return gnuradio::get_initial_sptr(
        new hier_block(name, input_signature, output_signature));
}


hier_block::hier_block(const std::string& name,
                       gr::io_signature::sptr input_signature,
                       gr::io_signature::sptr output_signature)
    : basic_block(name, input_signature, output_signature), d_fg(make_flowgraph())
{
    int min_inputs = input_signature->min_streams();
    int max_inputs = input_signature->max_streams();
    int min_outputs = output_signature->min_streams();
    int max_outputs = output_signature->max_streams();

    if (max_inputs == io_signature::IO_INFINITE ||
        max_outputs == io_signature::IO_INFINITE || (min_inputs != max_inputs) ||
        (min_outputs != max_outputs)) {
        std::stringstream msg;
        msg << "Hierarchical blocks do not yet support arbitrary or"
            << " variable numbers of inputs or outputs (" << name << ")";
        throw std::runtime_error(msg.str());
    }

    d_inputs = std::vector<endpoint_vector_t>(max_inputs);
    d_outputs = endpoint_vector_t(max_outputs);

    d_max_output_buffer = std::vector<size_t>(std::max(max_outputs, 1), 0);
    d_min_output_buffer = std::vector<size_t>(std::max(max_outputs, 1), 0);
}

hier_block::~hier_block() { disconnect_all(); }

void hier_block::connect(basic_block::sptr block)
{
    std::stringstream msg;

    // Check if duplicate
    if (std::find(d_blocks.begin(), d_blocks.end(), block) != d_blocks.end()) {
        msg << "Block " << block << " already connected.";
        throw std::invalid_argument(msg.str());
    }

    // Check if has inputs or outputs
    if (block->input_signature()->max_streams() != 0 ||
        block->output_signature()->max_streams() != 0) {
        msg << "Block " << block << " must not have any input or output ports";
        throw std::invalid_argument(msg.str());
    }

    d_blocks.push_back(block);
}

void hier_block::connect(basic_block::sptr src,
                         int src_port,
                         basic_block::sptr dst,
                         int dst_port)
{
    if (HIER_BLOCK_DEBUG)
        std::cout << "connecting: " << endpoint(src, src_port) << " -> "
                  << endpoint(dst, dst_port) << std::endl;

    if (src.get() == dst.get())
        throw std::invalid_argument(
            "connect: src and destination blocks cannot be the same");

    // Connections to block inputs or outputs
    int max_port;
    if (src.get() == this) {
        max_port = src->input_signature()->max_streams();
        if ((max_port != -1 && (src_port >= max_port)) || src_port < 0) {
            throw std::invalid_argument("source port out of range");
        }

        return connect_input(src_port, dst_port, dst);
    }

    if (dst.get() == this) {
        max_port = dst->output_signature()->max_streams();
        if ((max_port != -1 && (dst_port >= max_port)) || dst_port < 0) {
            throw std::invalid_argument("destination port out of range");
        }

        return connect_output(dst_port, src_port, src);
    }

    // Internal connections
    d_fg->connect(src, src_port, dst, dst_port);
}

void hier_block::msg_connect(basic_block::sptr src,
                             std::string srcport,
                             basic_block::sptr dst,
                             std::string dstport)
{
    if (HIER_BLOCK_DEBUG)
        std::cout << "connecting message port..." << std::endl;

    // add block uniquely to list to internal blocks
    if (std::find(d_blocks.begin(), d_blocks.end(), src) != d_blocks.end()) {
        d_blocks.push_back(src);
    }
    if (std::find(d_blocks.begin(), d_blocks.end(), dst) != d_blocks.end()) {
        d_blocks.push_back(dst);
    }

    // add edge for this message connection
    if (HIER_BLOCK_DEBUG)
        std::cout << boost::format("msg_connect( (%s, %s), (%s, %s) )\n") % src %
                         srcport % dst % dstport;
    d_fg->connect(msg_endpoint(src, srcport), msg_endpoint(dst, dstport));
}

void hier_block::msg_disconnect(basic_block::sptr src,
                                std::string srcport,
                                basic_block::sptr dst,
                                std::string dstport)
{
    if (HIER_BLOCK_DEBUG)
        std::cout << "disconnecting message port..." << std::endl;

    d_fg->disconnect(msg_endpoint(src, srcport), msg_endpoint(dst, dstport));

    hier_block_sptr src_block(cast_to_hier_block_sptr(src));
    hier_block_sptr dst_block(cast_to_hier_block_sptr(dst));

    if (src_block && src.get() != this) {
        // if the source is hier, we need to resolve the endpoint before calling unsub
        msg_edge_vector_t edges = src_block->d_fg->msg_edges();
        for (msg_edge_viter_t it = edges.begin(); it != edges.end(); ++it) {
            if ((*it).dst().block() == src) {
                src = (*it).src().block();
                srcport = (*it).src().port();
            }
        }
    }

    if (dst_block && dst.get() != this) {
        // if the destination is hier, we need to resolve the endpoint before calling
        // unsub
        msg_edge_vector_t edges = dst_block->d_fg->msg_edges();
        for (msg_edge_viter_t it = edges.begin(); it != edges.end(); ++it) {
            if ((*it).src().block() == dst) {
                dst = (*it).dst().block();
                dstport = (*it).dst().port();
            }
        }
    }

    // unregister the subscription - if already subscribed
    src->message_port_unsub(srcport, dst, dstport);
}

void hier_block::message_port_register_in(const std::string port_id)
{

    if (std::find(d_message_ports_in.begin(), d_message_ports_in.end(), port_id) !=
        d_message_ports_in.end()) {
        throw std::invalid_argument("hier msg in port by this name already registered");
    }

    d_message_ports_in.push_back(port_id);
}

std::vector<std::string> hier_block::message_ports_in() const
{
    return d_message_ports_in;
}

void hier_block::post(std::string which_port, pmt::pmt_t msg)
{
    throw std::runtime_error("not implemented yet");
}

void hier_block::disconnect(basic_block::sptr block)
{
    // Check on singleton list
    for (basic_block_viter_t p = d_blocks.begin(); p != d_blocks.end(); p++) {
        if (*p == block) {
            d_blocks.erase(p);

            return;
        }
    }

    // Otherwise find all edges containing block
    edge_vector_t edges, tmp = d_fg->edges();
    edge_vector_t::iterator p;
    for (p = tmp.begin(); p != tmp.end(); p++) {
        if ((*p).src().block() == block || (*p).dst().block() == block) {
            edges.push_back(*p);

            if (HIER_BLOCK_DEBUG)
                std::cout << "disconnect: block found in edge " << (*p) << std::endl;
        }
    }

    if (edges.empty()) {
        std::stringstream msg;
        msg << "cannot disconnect block " << block << ", not found";
        throw std::invalid_argument(msg.str());
    }

    for (p = edges.begin(); p != edges.end(); p++) {
        disconnect(
            (*p).src().block(), (*p).src().port(), (*p).dst().block(), (*p).dst().port());
    }
}

void hier_block::disconnect(basic_block::sptr src,
                            int src_port,
                            basic_block::sptr dst,
                            int dst_port)
{
    if (HIER_BLOCK_DEBUG)
        std::cout << "disconnecting: " << endpoint(src, src_port) << " -> "
                  << endpoint(dst, dst_port) << std::endl;

    if (src.get() == dst.get())
        throw std::invalid_argument(
            "disconnect: source and destination blocks cannot be the same");

    if (src.get() == this)
        return disconnect_input(src_port, dst_port, dst);

    if (dst.get() == this)
        return disconnect_output(dst_port, src_port, src);

    // Internal connections
    d_fg->disconnect(src, src_port, dst, dst_port);
}

void hier_block::refresh_io_signature()
{
    int min_inputs = input_signature()->min_streams();
    int max_inputs = input_signature()->max_streams();
    int min_outputs = output_signature()->min_streams();
    int max_outputs = output_signature()->max_streams();

    if (max_inputs == io_signature::IO_INFINITE ||
        max_outputs == io_signature::IO_INFINITE || (min_inputs != max_inputs) ||
        (min_outputs != max_outputs)) {
        std::stringstream msg;
        msg << "Hierarchical blocks do not yet support arbitrary or"
            << " variable numbers of inputs or outputs (" << name() << ")";
        throw std::runtime_error(msg.str());
    }

    // Check for # input change
    if ((signed)d_inputs.size() != max_inputs) {
        d_inputs.resize(max_inputs);
    }

    // Check for # output change
    if ((signed)d_outputs.size() != max_outputs) {
        d_outputs.resize(max_outputs);
        d_min_output_buffer.resize(max_outputs, 0);
        d_max_output_buffer.resize(max_outputs, 0);
    }
}

void hier_block::connect_input(int my_port, int port, basic_block::sptr block)
{
    std::stringstream msg;

    refresh_io_signature();

    if (my_port < 0 || my_port >= (signed)d_inputs.size()) {
        msg << "input port " << my_port << " out of range for " << block;
        throw std::invalid_argument(msg.str());
    }

    endpoint_vector_t& endps = d_inputs[my_port];
    endpoint endp(block, port);

    endpoint_viter_t p = std::find(endps.begin(), endps.end(), endp);
    if (p != endps.end()) {
        msg << "external input port " << my_port << " already wired to " << endp;
        throw std::invalid_argument(msg.str());
    }

    endps.push_back(endp);
}

void hier_block::connect_output(int my_port, int port, basic_block::sptr block)
{
    std::stringstream msg;

    refresh_io_signature();

    if (my_port < 0 || my_port >= (signed)d_outputs.size()) {
        msg << "output port " << my_port << " out of range for " << block;
        throw std::invalid_argument(msg.str());
    }

    if (d_outputs[my_port].block()) {
        msg << "external output port " << my_port << " already connected from "
            << d_outputs[my_port];
        throw std::invalid_argument(msg.str());
    }

    d_outputs[my_port] = endpoint(block, port);
}

void hier_block::disconnect_input(int my_port, int port, basic_block::sptr block)
{
    std::stringstream msg;

    refresh_io_signature();

    if (my_port < 0 || my_port >= (signed)d_inputs.size()) {
        msg << "input port number " << my_port << " out of range for " << block;
        throw std::invalid_argument(msg.str());
    }

    endpoint_vector_t& endps = d_inputs[my_port];
    endpoint endp(block, port);

    endpoint_viter_t p = std::find(endps.begin(), endps.end(), endp);
    if (p == endps.end()) {
        msg << "external input port " << my_port << " not connected to " << endp;
        throw std::invalid_argument(msg.str());
    }

    endps.erase(p);
}

void hier_block::disconnect_output(int my_port, int port, basic_block::sptr block)
{
    std::stringstream msg;

    refresh_io_signature();

    if (my_port < 0 || my_port >= (signed)d_outputs.size()) {
        msg << "output port number " << my_port << " out of range for " << block;
        throw std::invalid_argument(msg.str());
    }

    if (d_outputs[my_port].block() != block) {
        msg << "block " << block << " not assigned to output " << my_port
            << ", can't disconnect";
        throw std::invalid_argument(msg.str());
    }

    d_outputs[my_port] = endpoint();
}

endpoint_vector_t hier_block::resolve_port(int port, bool is_input)
{
    std::stringstream msg;

    if (HIER_BLOCK_DEBUG)
        std::cout << "Resolving port " << port << " as an "
                  << (is_input ? "input" : "output") << " of " << name() << std::endl;

    endpoint_vector_t result;

    if (is_input) {
        if (port < 0 || port >= (signed)d_inputs.size()) {
            msg << "resolve_port: hierarchical block '" << name() << "': input " << port
                << " is out of range";
            throw std::runtime_error(msg.str());
        }

        if (d_inputs[port].empty()) {
            msg << "resolve_port: hierarchical block '" << name() << "': input " << port
                << " is not connected internally";
            throw std::runtime_error(msg.str());
        }

        endpoint_vector_t& endps = d_inputs[port];
        endpoint_viter_t p;
        for (p = endps.begin(); p != endps.end(); p++) {
            endpoint_vector_t tmp = resolve_endpoint(*p, true);
            std::copy(tmp.begin(), tmp.end(), back_inserter(result));
        }
    } else {
        if (port < 0 || port >= (signed)d_outputs.size()) {
            msg << "resolve_port: hierarchical block '" << name() << "': output " << port
                << " is out of range";
            throw std::runtime_error(msg.str());
        }

        if (d_outputs[port] == endpoint()) {
            msg << "resolve_port: hierarchical block '" << name() << "': output " << port
                << " is not connected internally";
            throw std::runtime_error(msg.str());
        }

        result = resolve_endpoint(d_outputs[port], false);
    }

    if (result.empty()) {
        msg << "resolve_port: hierarchical block '" << name() << "': unable to resolve "
            << (is_input ? "input port " : "output port ") << port;
        throw std::runtime_error(msg.str());
    }

    return result;
}

void hier_block::disconnect_all()
{
    d_fg->clear();
    d_blocks.clear();

    int max_inputs = input_signature()->max_streams();
    int max_outputs = output_signature()->max_streams();
    d_inputs = std::vector<endpoint_vector_t>(max_inputs);
    d_outputs = endpoint_vector_t(max_outputs);
}

endpoint_vector_t hier_block::resolve_endpoint(const endpoint& endp, bool is_input) const
{
    std::stringstream msg;
    endpoint_vector_t result;

    // Check if endpoint is a leaf node
    if (cast_to_block_sptr(endp.block())) {
        if (HIER_BLOCK_DEBUG)
            std::cout << "Block " << endp.block() << " is a leaf node, returning."
                      << std::endl;
        result.push_back(endp);
        return result;
    }

    // Check if endpoint is a hierarchical block
    hier_block_sptr hier_block(cast_to_hier_block_sptr(endp.block()));
    if (hier_block) {
        if (HIER_BLOCK_DEBUG)
            std::cout << "Resolving endpoint " << endp << " as an "
                      << (is_input ? "input" : "output") << ", recursing" << std::endl;
        return hier_block->resolve_port(endp.port(), is_input);
    }

    msg << "unable to resolve" << (is_input ? " input " : " output ") << "endpoint "
        << endp;
    throw std::runtime_error(msg.str());
}

void hier_block::flatten_aux(flat_flowgraph_sptr sfg) const
{
    if (HIER_BLOCK_DEBUG)
        std::cout << " ** Flattening " << name() << std::endl;

    // Add my edges to the flow graph, resolving references to actual endpoints
    edge_vector_t edges = d_fg->edges();
    msg_edge_vector_t msg_edges = d_fg->msg_edges();
    edge_viter_t p;
    msg_edge_viter_t q;

    int min_buff = 0;
    int max_buff = 0;
    // Determine how the buffers should be set
    bool set_all_min_buff = all_min_output_buffer_p();
    bool set_all_max_buff = all_max_output_buffer_p();
    // Get the min and max buffer length
    if (set_all_min_buff) {
        if (HIER_BLOCK_DEBUG)
            std::cout << "Getting (" << (alias()).c_str() << ") min buffer" << std::endl;
        min_buff = min_output_buffer();
    }
    if (set_all_max_buff) {
        if (HIER_BLOCK_DEBUG)
            std::cout << "Getting (" << (alias()).c_str() << ") max buffer" << std::endl;
        max_buff = max_output_buffer();
    }

    // For every block (gr::block and gr::hier_block), set up the RPC
    // interface.
    for (p = edges.begin(); p != edges.end(); p++) {
        basic_block::sptr b;
        b = p->src().block();

        if (set_all_min_buff) {
            // sets the min buff for every block within hier_block
            if (min_buff != 0) {
                block_sptr bb = boost::dynamic_pointer_cast<block>(b);
                if (bb != 0) {
                    if (bb->min_output_buffer(0) != min_buff) {
                        if (HIER_BLOCK_DEBUG)
                            std::cout << "Block (" << (bb->alias()).c_str()
                                      << ") min_buff (" << min_buff << ")" << std::endl;
                        bb->set_min_output_buffer(min_buff);
                    }
                } else {
                    hier_block_sptr hh = boost::dynamic_pointer_cast<hier_block>(b);
                    if (hh != 0) {
                        if (hh->min_output_buffer(0) != min_buff) {
                            if (HIER_BLOCK_DEBUG)
                                std::cout << "HBlock (" << (hh->alias()).c_str()
                                          << ") min_buff (" << min_buff << ")"
                                          << std::endl;
                            hh->set_min_output_buffer(min_buff);
                        }
                    }
                }
            }
        }
        if (set_all_max_buff) {
            // sets the max buff for every block within hier_block
            if (max_buff != 0) {
                block_sptr bb = boost::dynamic_pointer_cast<block>(b);
                if (bb != 0) {
                    if (bb->max_output_buffer(0) != max_buff) {
                        if (HIER_BLOCK_DEBUG)
                            std::cout << "Block (" << (bb->alias()).c_str()
                                      << ") max_buff (" << max_buff << ")" << std::endl;
                        bb->set_max_output_buffer(max_buff);
                    }
                } else {
                    hier_block_sptr hh = boost::dynamic_pointer_cast<hier_block>(b);
                    if (hh != 0) {
                        if (hh->max_output_buffer(0) != max_buff) {
                            if (HIER_BLOCK_DEBUG)
                                std::cout << "HBlock (" << (hh->alias()).c_str()
                                          << ") max_buff (" << max_buff << ")"
                                          << std::endl;
                            hh->set_max_output_buffer(max_buff);
                        }
                    }
                }
            }
        }

        b = p->dst().block();
        if (set_all_min_buff) {
            // sets the min buff for every block within hier_block
            if (min_buff != 0) {
                block_sptr bb = boost::dynamic_pointer_cast<block>(b);
                if (bb != 0) {
                    if (bb->min_output_buffer(0) != min_buff) {
                        if (HIER_BLOCK_DEBUG)
                            std::cout << "Block (" << (bb->alias()).c_str()
                                      << ") min_buff (" << min_buff << ")" << std::endl;
                        bb->set_min_output_buffer(min_buff);
                    }
                } else {
                    hier_block_sptr hh = boost::dynamic_pointer_cast<hier_block>(b);
                    if (hh != 0) {
                        if (hh->min_output_buffer(0) != min_buff) {
                            if (HIER_BLOCK_DEBUG)
                                std::cout << "HBlock (" << (hh->alias()).c_str()
                                          << ") min_buff (" << min_buff << ")"
                                          << std::endl;
                            hh->set_min_output_buffer(min_buff);
                        }
                    }
                }
            }
        }
        if (set_all_max_buff) {
            // sets the max buff for every block within hier_block
            if (max_buff != 0) {
                block_sptr bb = boost::dynamic_pointer_cast<block>(b);
                if (bb != 0) {
                    if (bb->max_output_buffer(0) != max_buff) {
                        if (HIER_BLOCK_DEBUG)
                            std::cout << "Block (" << (bb->alias()).c_str()
                                      << ") max_buff (" << max_buff << ")" << std::endl;
                        bb->set_max_output_buffer(max_buff);
                    }
                } else {
                    hier_block_sptr hh = boost::dynamic_pointer_cast<hier_block>(b);
                    if (hh != 0) {
                        if (hh->max_output_buffer(0) != max_buff) {
                            if (HIER_BLOCK_DEBUG)
                                std::cout << "HBlock (" << (hh->alias()).c_str()
                                          << ") max_buff (" << max_buff << ")"
                                          << std::endl;
                            hh->set_max_output_buffer(max_buff);
                        }
                    }
                }
            }
        }
    }

    if (HIER_BLOCK_DEBUG)
        std::cout << "Flattening stream connections: " << std::endl;

    for (p = edges.begin(); p != edges.end(); p++) {
        if (HIER_BLOCK_DEBUG)
            std::cout << "Flattening edge " << (*p) << std::endl;

        endpoint_vector_t src_endps = resolve_endpoint(p->src(), false);
        endpoint_vector_t dst_endps = resolve_endpoint(p->dst(), true);

        endpoint_viter_t s, d;
        for (s = src_endps.begin(); s != src_endps.end(); s++) {
            for (d = dst_endps.begin(); d != dst_endps.end(); d++) {
                if (HIER_BLOCK_DEBUG)
                    std::cout << (*s) << "->" << (*d) << std::endl;
                sfg->connect(*s, *d);
            }
        }
    }

    // loop through flattening hierarchical connections
    if (HIER_BLOCK_DEBUG)
        std::cout << "Flattening msg connections: " << std::endl;

    std::vector<std::pair<msg_endpoint, bool>> resolved_endpoints;
    for (q = msg_edges.begin(); q != msg_edges.end(); q++) {
        if (HIER_BLOCK_DEBUG)
            std::cout << boost::format(" flattening edge ( %s, %s) -> ( %s, %s)\n") %
                             q->src().block() % q->src().port() % q->dst().block() %
                             q->dst().port();


        if (q->src().block().get() == this) {
            // connection into this block ..
            if (HIER_BLOCK_DEBUG)
                std::cout << "hier incoming port: " << q->src() << std::endl;
            sfg->replace_endpoint(q->src(), q->dst(), false);
            resolved_endpoints.push_back(std::pair<msg_endpoint, bool>(q->src(), false));
        } else if (q->dst().block().get() == this) {
            // connection out of this block
            if (HIER_BLOCK_DEBUG)
                std::cout << "hier outgoing port: " << q->dst() << std::endl;
            sfg->replace_endpoint(q->dst(), q->src(), true);
            resolved_endpoints.push_back(std::pair<msg_endpoint, bool>(q->dst(), true));
        } else {
            // internal connection only
            if (HIER_BLOCK_DEBUG)
                std::cout << "internal msg connection: " << q->src() << "-->" << q->dst()
                          << std::endl;
            sfg->connect(q->src(), q->dst());
        }
    }

    for (std::vector<std::pair<msg_endpoint, bool>>::iterator it =
             resolved_endpoints.begin();
         it != resolved_endpoints.end();
         it++) {
        if (HIER_BLOCK_DEBUG)
            std::cout << "sfg->clear_endpoint(" << (*it).first << ", " << (*it).second
                      << ") " << std::endl;
        sfg->clear_endpoint((*it).first, (*it).second);
    }

    // Construct unique list of blocks used either in edges, inputs,
    // outputs, or by themselves.  I still hate STL.
    basic_block_vector_t blocks; // unique list of used blocks
    basic_block_vector_t tmp = d_fg->calc_used_blocks();

    // First add the list of singleton blocks
    std::vector<basic_block::sptr>::const_iterator b; // Because flatten_aux is const
    for (b = d_blocks.begin(); b != d_blocks.end(); b++) {
        tmp.push_back(*b);
    }

    // Now add the list of connected input blocks
    std::stringstream msg;
    for (unsigned int i = 0; i < d_inputs.size(); i++) {
        if (d_inputs[i].empty()) {
            msg << "In hierarchical block " << name() << ", input " << i
                << " is not connected internally";
            throw std::runtime_error(msg.str());
        }

        for (unsigned int j = 0; j < d_inputs[i].size(); j++)
            tmp.push_back(d_inputs[i][j].block());
    }

    for (unsigned int i = 0; i < d_outputs.size(); i++) {
        basic_block::sptr blk = d_outputs[i].block();
        if (!blk) {
            msg << "In hierarchical block " << name() << ", output " << i
                << " is not connected internally";
            throw std::runtime_error(msg.str());
        }
        // Set the buffers of only the blocks connected to the hier output
        if (!set_all_min_buff) {
            min_buff = min_output_buffer(i);
            if (min_buff != 0) {
                block_sptr bb = boost::dynamic_pointer_cast<block>(blk);
                if (bb != 0) {
                    int bb_src_port = d_outputs[i].port();
                    if (HIER_BLOCK_DEBUG)
                        std::cout << "Block (" << (bb->alias()).c_str() << ") Port ("
                                  << bb_src_port << ") min_buff (" << min_buff << ")"
                                  << std::endl;
                    bb->set_min_output_buffer(bb_src_port, min_buff);
                } else {
                    hier_block_sptr hh = boost::dynamic_pointer_cast<hier_block>(blk);
                    if (hh != 0) {
                        int hh_src_port = d_outputs[i].port();
                        if (HIER_BLOCK_DEBUG)
                            std::cout << "HBlock (" << (hh->alias()).c_str() << ") Port ("
                                      << hh_src_port << ") min_buff (" << min_buff << ")"
                                      << std::endl;
                        hh->set_min_output_buffer(hh_src_port, min_buff);
                    }
                }
            }
        }
        if (!set_all_max_buff) {
            max_buff = max_output_buffer(i);
            if (max_buff != 0) {
                block_sptr bb = boost::dynamic_pointer_cast<block>(blk);
                if (bb != 0) {
                    int bb_src_port = d_outputs[i].port();
                    if (HIER_BLOCK_DEBUG)
                        std::cout << "Block (" << (bb->alias()).c_str() << ") Port ("
                                  << bb_src_port << ") max_buff (" << max_buff << ")"
                                  << std::endl;
                    bb->set_max_output_buffer(bb_src_port, max_buff);
                } else {
                    hier_block_sptr hh = boost::dynamic_pointer_cast<hier_block>(blk);
                    if (hh != 0) {
                        int hh_src_port = d_outputs[i].port();
                        if (HIER_BLOCK_DEBUG)
                            std::cout << "HBlock (" << (hh->alias()).c_str() << ") Port ("
                                      << hh_src_port << ") max_buff (" << max_buff << ")"
                                      << std::endl;
                        hh->set_max_output_buffer(hh_src_port, max_buff);
                    }
                }
            }
        }
        tmp.push_back(blk);
    }
    sort(tmp.begin(), tmp.end());

    std::insert_iterator<basic_block_vector_t> inserter(blocks, blocks.begin());
    unique_copy(tmp.begin(), tmp.end(), inserter);

    // Recurse hierarchical children
    for (basic_block_viter_t p = blocks.begin(); p != blocks.end(); p++) {
        hier_block_sptr hier_block(cast_to_hier_block_sptr(*p));
        if (hier_block && (hier_block.get() != this)) {
            if (HIER_BLOCK_DEBUG)
                std::cout << "flatten_aux: recursing into hierarchical block "
                          << hier_block->alias() << std::endl;
            hier_block->flatten_aux(sfg);
        }
    }
}

int hier_block::max_output_buffer(size_t port) const
{
    if (port >= d_max_output_buffer.size())
        throw std::invalid_argument(
            "hier_block::max_output_buffer(int): port out of range.");
    return d_max_output_buffer[port];
}

void hier_block::set_max_output_buffer(int max_output_buffer)
{
    if (output_signature()->max_streams() > 0) {
        if (d_max_output_buffer.empty()) {
            throw std::length_error("hier_block::set_max_output_buffer(int): out_sig "
                                    "greater than zero, buff_vect isn't");
        }
        for (int idx = 0; idx < output_signature()->max_streams(); idx++) {
            d_max_output_buffer[idx] = max_output_buffer;
        }
    }
}

void hier_block::set_max_output_buffer(size_t port, int max_output_buffer)
{
    if (port >= d_max_output_buffer.size()) {
        throw std::invalid_argument(
            "hier_block::set_max_output_buffer(size_t,int): port out of range.");
    } else {
        d_max_output_buffer[port] = max_output_buffer;
    }
}

int hier_block::min_output_buffer(size_t port) const
{
    if (port >= d_min_output_buffer.size())
        throw std::invalid_argument(
            "hier_block::min_output_buffer(size_t): port out of range.");
    return d_min_output_buffer[port];
}

void hier_block::set_min_output_buffer(int min_output_buffer)
{
    if (output_signature()->max_streams() > 0) {
        if (d_min_output_buffer.empty()) {
            throw std::length_error("hier_block::set_min_output_buffer(int): out_sig "
                                    "greater than zero, buff_vect isn't");
        }
        for (int idx = 0; idx < output_signature()->max_streams(); idx++) {
            d_min_output_buffer[idx] = min_output_buffer;
        }
    }
}

void hier_block::set_min_output_buffer(size_t port, int min_output_buffer)
{
    if (port >= d_min_output_buffer.size())
        throw std::invalid_argument(
            "hier_block::set_min_output_buffer(size_t,int): port out of range.");
    else {
        d_min_output_buffer[port] = min_output_buffer;
    }
}

bool hier_block::all_min_output_buffer_p() const
{
    if (d_min_output_buffer.empty()) {
        return false;
    }
    for (size_t idx = 1; idx < d_min_output_buffer.size(); idx++) {
        if (d_min_output_buffer[0] != d_min_output_buffer[idx])
            return false;
    }
    return true;
}

bool hier_block::all_max_output_buffer_p() const
{
    if (d_max_output_buffer.empty())
        return false;
    for (size_t idx = 1; idx < d_max_output_buffer.size(); idx++) {
        if (d_max_output_buffer[0] != d_max_output_buffer[idx])
            return false;
    }
    return true;
}

void hier_block::set_processor_affinity(const std::vector<int>& mask)
{
    basic_block_vector_t tmp = d_fg->calc_used_blocks();
    for (basic_block_viter_t p = tmp.begin(); p != tmp.end(); p++) {
        (*p)->set_processor_affinity(mask);
    }
}

void hier_block::unset_processor_affinity()
{
    basic_block_vector_t tmp = d_fg->calc_used_blocks();
    for (basic_block_viter_t p = tmp.begin(); p != tmp.end(); p++) {
        (*p)->unset_processor_affinity();
    }
}

std::vector<int> hier_block::processor_affinity()
{
    basic_block_vector_t tmp = d_fg->calc_used_blocks();
    return tmp[0]->processor_affinity();
}

void hier_block::set_log_level(std::string level)
{
    basic_block_vector_t tmp = d_fg->calc_used_blocks();
    for (basic_block_viter_t p = tmp.begin(); p != tmp.end(); p++) {
        (*p)->set_log_level(level);
    }
}

std::string hier_block::log_level()
{
    // Assume that log_level was set for all hier_block blocks
    basic_block_vector_t tmp = d_fg->calc_used_blocks();
    return tmp[0]->log_level();
}

} /* namespace gr */
