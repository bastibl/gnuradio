/* -*- c++ -*- */
/*
 * Copyright 2007,2011,2013 Free Software Foundation, Inc.
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

#include <gnuradio/flowgraph.h>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace gr {

#define FLOWGRAPH_DEBUG 0

edge::~edge() {}

flowgraph_sptr make_flowgraph() { return flowgraph_sptr(new flowgraph()); }

flowgraph::flowgraph() {}

flowgraph::~flowgraph() {}

template <class T>
static std::vector<T> unique_vector(std::vector<T> v)
{
    std::vector<T> result;
    std::insert_iterator<std::vector<T>> inserter(result, result.begin());

    sort(v.begin(), v.end());
    unique_copy(v.begin(), v.end(), inserter);
    return result;
}

void flowgraph::connect(const endpoint& src, const endpoint& dst)
{
    check_valid_port(src.block()->output_signature(), src.port());
    check_valid_port(dst.block()->input_signature(), dst.port());
    check_dst_not_used(dst);
    check_type_match(src, dst);

    // Alles klar, Herr Kommissar
    d_edges.push_back(edge(src, dst));
}

void flowgraph::disconnect(const endpoint& src, const endpoint& dst)
{
    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++) {
        if (src == p->src() && dst == p->dst()) {
            d_edges.erase(p);
            return;
        }
    }

    std::stringstream msg;
    msg << "cannot disconnect edge " << edge(src, dst) << ", not found";
    throw std::invalid_argument(msg.str());
}

void flowgraph::validate()
{
    d_blocks = calc_used_blocks();

    for (basic_block_viter_t p = d_blocks.begin(); p != d_blocks.end(); p++) {
        std::vector<int> used_ports;
        int ninputs, noutputs;

        if (FLOWGRAPH_DEBUG)
            std::cout << "Validating block: " << (*p) << std::endl;

        used_ports = calc_used_ports(*p, true); // inputs
        ninputs = used_ports.size();
        check_contiguity(*p, used_ports, true); // inputs

        used_ports = calc_used_ports(*p, false); // outputs
        noutputs = used_ports.size();
        check_contiguity(*p, used_ports, false); // outputs

        if (!((*p)->check_topology(ninputs, noutputs))) {
            std::stringstream msg;
            msg << "check topology failed on " << (*p) << " using ninputs=" << ninputs
                << ", noutputs=" << noutputs;
            throw std::runtime_error(msg.str());
        }
    }
}

void flowgraph::clear()
{
    // Boost shared pointers will deallocate as needed
    d_blocks.clear();
    d_edges.clear();
}

void flowgraph::check_valid_port(gr::io_signature::sptr sig, int port)
{
    std::stringstream msg;

    if (port < 0) {
        msg << "negative port number " << port << " is invalid";
        throw std::invalid_argument(msg.str());
    }

    int max = sig->max_streams();
    if (max != io_signature::IO_INFINITE && port >= max) {
        msg << "port number " << port << " exceeds max of ";
        if (max == 0)
            msg << "(none)";
        else
            msg << max - 1;
        throw std::invalid_argument(msg.str());
    }
}

void flowgraph::check_dst_not_used(const endpoint& dst)
{
    // A destination is in use if it is already on the edge list
    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++)
        if (p->dst() == dst) {
            std::stringstream msg;
            msg << "destination already in use by edge " << (*p);
            throw std::invalid_argument(msg.str());
        }
}

void flowgraph::check_type_match(const endpoint& src, const endpoint& dst)
{
    int src_size = src.block()->output_signature()->sizeof_stream_item(src.port());
    int dst_size = dst.block()->input_signature()->sizeof_stream_item(dst.port());

    if (src_size != dst_size) {
        std::stringstream msg;
        msg << "itemsize mismatch: " << src << " using " << src_size << ", " << dst
            << " using " << dst_size;
        throw std::invalid_argument(msg.str());
    }
}

basic_block_vector_t flowgraph::calc_used_blocks()
{
    basic_block_vector_t tmp;

    // make sure free standing message blocks are included
    for (msg_edge_viter_t p = d_msg_edges.begin(); p != d_msg_edges.end(); p++) {
        // all msg blocks need a thread context - otherwise start() will never be called!
        // even if it is a sender that never does anything
        tmp.push_back(p->src().block());
        tmp.push_back(p->dst().block());
    }

    // Collect all blocks in the edge list
    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++) {
        tmp.push_back(p->src().block());
        tmp.push_back(p->dst().block());
    }

    return unique_vector<basic_block::sptr>(tmp);
}

std::vector<int> flowgraph::calc_used_ports(basic_block::sptr block, bool check_inputs)
{
    std::vector<int> tmp;

    // Collect all seen ports
    edge_vector_t edges = calc_connections(block, check_inputs);
    for (edge_viter_t p = edges.begin(); p != edges.end(); p++) {
        if (check_inputs == true)
            tmp.push_back(p->dst().port());
        else
            tmp.push_back(p->src().port());
    }

    return unique_vector<int>(tmp);
}

edge_vector_t flowgraph::calc_connections(basic_block::sptr block, bool check_inputs)
{
    edge_vector_t result;

    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++) {
        if (check_inputs) {
            if (p->dst().block() == block)
                result.push_back(*p);
        } else {
            if (p->src().block() == block)
                result.push_back(*p);
        }
    }

    return result; // assumes no duplicates
}

void flowgraph::check_contiguity(basic_block::sptr block,
                                 const std::vector<int>& used_ports,
                                 bool check_inputs)
{
    std::stringstream msg;

    gr::io_signature::sptr sig =
        check_inputs ? block->input_signature() : block->output_signature();

    int nports = used_ports.size();
    int min_ports = sig->min_streams();
    int max_ports = sig->max_streams();

    if (nports == 0 && min_ports == 0)
        return;

    if (nports < min_ports) {
        msg << block << ": insufficient connected "
            << (check_inputs ? "input ports " : "output ports ") << "(" << min_ports
            << " needed, " << nports << " connected)";
        throw std::runtime_error(msg.str());
    }

    if (nports > max_ports && max_ports != io_signature::IO_INFINITE) {
        msg << block << ": too many connected "
            << (check_inputs ? "input ports " : "output ports ") << "(" << max_ports
            << " allowed, " << nports << " connected)";
        throw std::runtime_error(msg.str());
    }

    if (used_ports[nports - 1] + 1 != nports) {
        for (int i = 0; i < nports; i++) {
            if (used_ports[i] != i) {
                msg << block << ": missing connection "
                    << (check_inputs ? "to input port " : "from output port ") << i;
                throw std::runtime_error(msg.str());
            }
        }
    }
}

basic_block_vector_t flowgraph::calc_downstream_blocks(basic_block::sptr block, int port)
{
    basic_block_vector_t tmp;

    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++)
        if (p->src() == endpoint(block, port))
            tmp.push_back(p->dst().block());

    return unique_vector<basic_block::sptr>(tmp);
}

basic_block_vector_t flowgraph::calc_downstream_blocks(basic_block::sptr block)
{
    basic_block_vector_t tmp;

    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++)
        if (p->src().block() == block)
            tmp.push_back(p->dst().block());

    return unique_vector<basic_block::sptr>(tmp);
}

edge_vector_t flowgraph::calc_upstream_edges(basic_block::sptr block)
{
    edge_vector_t result;

    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++)
        if (p->dst().block() == block)
            result.push_back(*p);

    return result; // Assume no duplicates
}

bool flowgraph::has_block_p(basic_block::sptr block)
{
    basic_block_viter_t result;
    result = std::find(d_blocks.begin(), d_blocks.end(), block);
    return (result != d_blocks.end());
}

edge flowgraph::calc_upstream_edge(basic_block::sptr block, int port)
{
    edge result;

    for (edge_viter_t p = d_edges.begin(); p != d_edges.end(); p++) {
        if (p->dst() == endpoint(block, port)) {
            result = (*p);
            break;
        }
    }

    return result;
}

void flowgraph::connect(const msg_endpoint& src, const msg_endpoint& dst)
{
    for (msg_edge_viter_t p = d_msg_edges.begin(); p != d_msg_edges.end(); p++) {
        if (p->src() == src && p->dst() == dst) {
            throw std::runtime_error("connect called on already connected edge!");
        }
    }
    d_msg_edges.push_back(msg_edge(src, dst));
}

void flowgraph::disconnect(const msg_endpoint& src, const msg_endpoint& dst)
{
    for (msg_edge_viter_t p = d_msg_edges.begin(); p != d_msg_edges.end(); p++) {
        if (p->src() == src && p->dst() == dst) {
            d_msg_edges.erase(p);
            return;
        }
    }
    throw std::runtime_error("disconnect called on non-connected edge!");
}

} /* namespace gr */
