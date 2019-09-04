/* -*- c++ -*- */
/*
 * Copyright 2006-2009,2013 Free Software Foundation, Inc.
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

#ifndef INCLUDED_GR_RUNTIME_HIER_BLOCK_H
#define INCLUDED_GR_RUNTIME_HIER_BLOCK_H

#include <gnuradio/api.h>
#include <gnuradio/basic_block.h>
#include <gnuradio/block.h>
#include <gnuradio/flowgraph.h>

namespace gr {

/*!
 * \brief Hierarchical container class for gr::block's and gr::hier_block's
 * \ingroup container_blk
 * \ingroup base_blk
 */
class GR_RUNTIME_API hier_block : public basic_block
{
public:
    typedef boost::shared_ptr<hier_block> sptr;
    static sptr make(const std::string& name,
                     gr::io_signature::sptr input_signature,
                     gr::io_signature::sptr output_signature);

    virtual ~hier_block();

    /*!
     * \brief Add a stand-alone (possibly hierarchical) block to
     * internal graph
     *
     * This adds a gr-block or hierarchical block to the internal
     * graph without wiring it to anything else.
     */
    void connect(basic_block::sptr block);

    /*!
     * \brief Add gr-blocks or hierarchical blocks to internal graph
     * and wire together
     *
     * This adds (if not done earlier by another connect) a pair of
     * gr-blocks or hierarchical blocks to the internal flowgraph, and
     * wires the specified output port to the specified input port.
     */
    void
    connect(basic_block::sptr src, int src_port, basic_block::sptr dst, int dst_port);


    void message_port_register_in(const std::string& port_id) override;
    std::vector<std::string> message_ports_in() const override;

    void post(const std::string& which_port, const pmt::pmt_t& msg) override;

    /*!
     * \brief Add gr-blocks or hierarchical blocks to internal graph
     * and wire together
     *
     * This adds (if not done earlier by another connect) a pair of
     * gr-blocks or hierarchical blocks to the internal message port
     * subscription
     */
    void msg_connect(basic_block::sptr src,
                     std::string srcport,
                     basic_block::sptr dst,
                     std::string dstport);
    void msg_disconnect(basic_block::sptr src,
                        std::string srcport,
                        basic_block::sptr dst,
                        std::string dstport = 0);

    /*!
     * \brief Remove a gr-block or hierarchical block from the
     * internal flowgraph.
     *
     * This removes a gr-block or hierarchical block from the internal
     * flowgraph, disconnecting it from other blocks as needed.
     */
    void disconnect(basic_block::sptr block);

    /*!
     * \brief Disconnect a pair of gr-blocks or hierarchical blocks in
     *        internal flowgraph.
     *
     * This disconnects the specified input port from the specified
     * output port of a pair of gr-blocks or hierarchical blocks.
     */
    void
    disconnect(basic_block::sptr src, int src_port, basic_block::sptr dst, int dst_port);

    /*!
     * \brief Disconnect all connections in the internal flowgraph.
     *
     * This call removes all output port to input port connections in
     * the internal flowgraph.
     */
    void disconnect_all();

    void connect_input(int my_port, int port, basic_block::sptr block);
    void connect_output(int my_port, int port, basic_block::sptr block);
    void disconnect_input(int my_port, int port, basic_block::sptr block);
    void disconnect_output(int my_port, int port, basic_block::sptr block);

    /*!
     * \brief Returns max buffer size (itemcount) on output port \p i.
     */
    int max_output_buffer(size_t port = 0) const;

    /*!
     * \brief Sets max buffer size (itemcount) on all output ports.
     */
    void set_max_output_buffer(int max_output_buffer);

    /*!
     * \brief Sets max buffer size (itemcount) on output port \p port.
     */
    void set_max_output_buffer(size_t port, int max_output_buffer);

    /*!
     * \brief Returns min buffer size (itemcount) on output port \p i.
     */
    int min_output_buffer(size_t port = 0) const;

    /*!
     * \brief Sets min buffer size (itemcount) on all output ports.
     */
    void set_min_output_buffer(int min_output_buffer);

    /*!
     * \brief Sets min buffer size (itemcount) on output port \p port.
     */
    void set_min_output_buffer(size_t port, int min_output_buffer);

    /*!
     * \brief Set the affinity of all blocks in hier_block to processor core \p n.
     *
     * \param mask a vector of ints of the core numbers available to this block.
     */
    void set_processor_affinity(const std::vector<int>& mask) override;

    /*!
     * \brief Remove processor affinity for all blocks in hier_block.
     */
    void unset_processor_affinity() override;

    /*!
     * \brief Get the current processor affinity.
     *
     * \details This returns the processor affinity value for the first
     * block in the hier_block's list of blocks with the assumption
     * that they have always only been set through the hier_block's
     * interface. If any block has been individually set, then this
     * call could be misleading.
     */
    std::vector<int> processor_affinity() override;

    /*!
     * \brief Set the logger's output level.
     *
     * Sets the level of the logger for all connected blocks. This takes
     * a string that is translated to the standard levels and can be
     * (case insensitive):
     *
     * \li off , notset
     * \li debug
     * \li info
     * \li notice
     * \li warn
     * \li error
     * \li crit
     * \li alert
     * \li fatal
     * \li emerg
     */
    void set_log_level(std::string level) override;

    /*!
     * \brief Get the logger's output level
     */
    std::string log_level() override;

    /*!
     * \brief Get if all block min buffers should be set.
     *
     * \details this returns whether all the block min output buffers
     * should be set or just the block ports connected to the hier ports.
     */
    bool all_min_output_buffer_p() const;

    /*!
     * \brief Get if all block max buffers should be set.
     *
     * \details this returns whether all the block max output buffers
     * should be set or just the block ports connected to the hier ports.
     */
    bool all_max_output_buffer_p() const;

protected:
    hier_block(const std::string& name,
               gr::io_signature::sptr input_signature,
               gr::io_signature::sptr output_signature);

    void flatten_aux(flat_flowgraph_sptr sfg) const;

private:
    // Track output buffer min/max settings
    std::vector<size_t> d_max_output_buffer;
    std::vector<size_t> d_min_output_buffer;

    // Private implementation data
    flowgraph_sptr d_fg;
    std::vector<endpoint_vector_t>
        d_inputs;                // Multiple internal endpoints per external input
    endpoint_vector_t d_outputs; // Single internal endpoint per external output
    basic_block_vector_t d_blocks;

    std::vector<std::string> d_message_ports_in;

    void refresh_io_signature();

    endpoint_vector_t resolve_port(int port, bool is_input);
    endpoint_vector_t resolve_endpoint(const endpoint& endp, bool is_input) const;
};

inline hier_block_sptr cast_to_hier_block_sptr(basic_block::sptr block)
{
    return boost::dynamic_pointer_cast<hier_block, basic_block>(block);
}

} /* namespace gr */

#endif /* INCLUDED_GR_RUNTIME_HIER_BLOCK_H */
