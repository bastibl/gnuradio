/* -*- c++ -*- */
/*
 * Copyright 2006,2012-2013 Free Software Foundation, Inc.
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

#include <gnuradio/basic_block.h>
#include <gnuradio/block_registry.h>
#include <gnuradio/logger.h>

namespace gr {

basic_block::basic_block(const std::string& name,
                         io_signature::sptr input_signature,
                         io_signature::sptr output_signature)
    : d_input_signature(input_signature),
      d_output_signature(output_signature),
      d_name(name),
      d_unique_id(global_block_registry.block_register(this)),
      d_unique_name((boost::format("%1%%2%") % d_name % d_unique_id).str()),
      d_rpc_set(false)
{
}

basic_block::~basic_block() { global_block_registry.block_unregister(this); }

basic_block_sptr basic_block::to_basic_block() { return shared_from_this(); }

void basic_block::set_block_alias(const std::string& name)
{
    global_block_registry.update_alias(this, name);

    d_alias = name;
    update_logger_alias(unique_name(), d_alias);
}

// ** Message passing interface **

//  - register a new output message port
void basic_block::message_port_register_out(const std::string& port_id)
{
    if (d_message_subscribers.count(port_id)) {
        throw std::runtime_error("message_port_register_out: port already registered");
    }
    d_message_subscribers[port_id] = std::vector<msg_endpoint>();
}

std::vector<std::string> basic_block::message_ports_out() const
{
    std::vector<std::string> port_names;
    for (auto const& it : d_message_subscribers) {
        port_names.push_back(it.first);
    }

    return port_names;
}

//  - subscribe to a message port
void basic_block::message_port_sub(const std::string& port_id, basic_block_sptr target, const std::string& target_port)
{
    // check if port exists
    if (!d_message_subscribers.count(port_id)) {
        std::stringstream ss;
        ss << "Port does not exist: \"" << port_id << "\" on block id: " << target
           << std::endl;
        throw std::runtime_error(ss.str());
    }

    // subscribe target if not already subscribed
    msg_endpoint ep(target, target_port);
    std::vector<msg_endpoint>& subs = d_message_subscribers[port_id];
    if (std::find(subs.begin(), subs.end(), ep) == subs.end()) {
        subs.push_back(ep);
    }
}

void basic_block::message_port_unsub(const std::string& port_id, basic_block_sptr target, const std::string& target_port)
{
    // check if port exists
    if (!d_message_subscribers.count(port_id)) {
        std::stringstream ss;
        ss << "Port does not exist: \"" << port_id << "\" on block: " << target
           << std::endl;
        throw std::runtime_error(ss.str());
    }

    // remove subscription if possible
    msg_endpoint ep(target, target_port);
    std::vector<msg_endpoint>& subs = d_message_subscribers[port_id];
    subs.erase(std::remove(subs.begin(), subs.end(), ep), subs.end());
}

std::vector<msg_endpoint> basic_block::message_subscribers(const std::string& port_id)
{
    // check if port exists
    if (!d_message_subscribers.count(port_id)) {
        std::stringstream ss;
        ss << "Port does not exist: \"" << port_id << "\" on block: " << alias()
           << std::endl;
        throw std::runtime_error(ss.str());
    }

    return d_message_subscribers[port_id];
}

} /* namespace gr */
