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
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace gr {

basic_block::basic_block(const std::string& name,
                         io_signature::sptr input_signature,
                         io_signature::sptr output_signature)
    : d_input_signature(input_signature),
      d_output_signature(output_signature),
      d_name(name),
      d_unique_id(global_block_registry.block_register(this)),
      d_unique_name((boost::format("%1%%2%") % d_name % d_unique_id).str()),
      d_rpc_set(false),
      d_message_subscribers(pmt::make_dict())
{
}

basic_block::~basic_block() { global_block_registry.block_unregister(this); }

basic_block::sptr basic_block::to_basic_block() { return shared_from_this(); }

void basic_block::set_block_alias(std::string name)
{
    global_block_registry.update_alias(this, name);

    d_alias = name;
    update_logger_alias(unique_name(), d_alias);
}

// ** Message passing interface **

//  - register a new output message port
void basic_block::message_port_register_out(pmt::pmt_t port_id)
{
    if (!pmt::is_symbol(port_id)) {
        throw std::runtime_error("message_port_register_out: bad port id");
    }
    if (pmt::dict_has_key(d_message_subscribers, port_id)) {
        throw std::runtime_error("message_port_register_out: port already in use");
    }
    d_message_subscribers = pmt::dict_add(d_message_subscribers, port_id, pmt::PMT_NIL);
}

pmt::pmt_t basic_block::message_ports_out()
{
    size_t len = pmt::length(d_message_subscribers);
    pmt::pmt_t port_names = pmt::make_vector(len, pmt::PMT_NIL);
    pmt::pmt_t keys = pmt::dict_keys(d_message_subscribers);
    for (size_t i = 0; i < len; i++) {
        pmt::vector_set(port_names, i, pmt::nth(i, keys));
    }
    return port_names;
}

//  - subscribe to a message port
void basic_block::message_port_sub(pmt::pmt_t port_id, pmt::pmt_t target)
{
    if (!pmt::dict_has_key(d_message_subscribers, port_id)) {
        std::stringstream ss;
        ss << "Port does not exist: \"" << pmt::write_string(port_id)
           << "\" on block: " << pmt::write_string(target) << std::endl;
        throw std::runtime_error(ss.str());
    }
    pmt::pmt_t currlist = pmt::dict_ref(d_message_subscribers, port_id, pmt::PMT_NIL);

    // ignore re-adds of the same target
    if (!pmt::list_has(currlist, target))
        d_message_subscribers = pmt::dict_add(
            d_message_subscribers, port_id, pmt::list_add(currlist, target));
}

void basic_block::message_port_unsub(pmt::pmt_t port_id, pmt::pmt_t target)
{
    if (!pmt::dict_has_key(d_message_subscribers, port_id)) {
        std::stringstream ss;
        ss << "Port does not exist: \"" << pmt::write_string(port_id)
           << "\" on block: " << pmt::write_string(target) << std::endl;
        throw std::runtime_error(ss.str());
    }

    // ignore unsubs of unknown targets
    pmt::pmt_t currlist = pmt::dict_ref(d_message_subscribers, port_id, pmt::PMT_NIL);
    d_message_subscribers =
        pmt::dict_add(d_message_subscribers, port_id, pmt::list_rm(currlist, target));
}

pmt::pmt_t basic_block::message_subscribers(pmt::pmt_t port)
{
    return pmt::dict_ref(d_message_subscribers, port, pmt::PMT_NIL);
}

} /* namespace gr */
