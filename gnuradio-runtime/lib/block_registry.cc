/* -*- c++ -*- */
/*
 * Copyright 2012-2013 Free Software Foundation, Inc.
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
#include <gnuradio/block.h>
#include <gnuradio/block_executor.h>
#include <gnuradio/block_registry.h>
#include <stdio.h>

gr::block_registry global_block_registry;

namespace gr {

block_registry::block_registry() : d_seq_nr(1) {}

uint64_t block_registry::block_register(basic_block* block)
{
    gr::thread::scoped_lock guard(d_mutex);

    uint64_t id = d_seq_nr++;

    d_id_map[id] = block;
    d_name_map[(boost::format("%1%%2%") % block->name() % id).str()] = block;

    return id;
}

void block_registry::block_unregister(basic_block* block)
{
    gr::thread::scoped_lock guard(d_mutex);

    d_id_map.erase(block->unique_id());
    d_name_map.erase(block->unique_name());

    if (block->alias_set()) {
        d_alias_map.erase(block->alias());
    }
}

void block_registry::update_alias(basic_block* block, std::string name)
{
    gr::thread::scoped_lock guard(d_mutex);

    for (auto it = d_alias_map.begin(); it != d_alias_map.end(); it++) {
        if (it->second == block) {
            d_alias_map.erase(it->first);
            break;
        }
    }

    for (auto it = d_alias_map.begin(); it != d_alias_map.end(); it++) {
        if (it->first == name) {
            throw std::runtime_error("symbol already exists, can not re-use!");
        }
    }

    d_alias_map[name] = block;
}

basic_block::sptr block_registry::block_lookup(pmt::pmt_t symbol)
{
    gr::thread::scoped_lock guard(d_mutex);

    std::string name = pmt::symbol_to_string(symbol);

    if (d_name_map.find(name) != d_name_map.end()) {
        return d_name_map[name]->shared_from_this();
    }

    if (d_alias_map.find(name) == d_alias_map.end()) {
        throw std::runtime_error("block lookup failed! block not found!");
    }

    return d_alias_map[name]->shared_from_this();
}

void block_registry::register_primitive(uint64_t id, block* ref)
{
    gr::thread::scoped_lock guard(d_mutex);

    primitive_map[id] = ref;
}

void block_registry::unregister_primitive(uint64_t id)
{
    gr::thread::scoped_lock guard(d_mutex);

    primitive_map.erase(primitive_map.find(id));
}

void block_registry::notify_blk(uint64_t id)
{
    gr::thread::scoped_lock guard(d_mutex);

    if (primitive_map.find(id) == primitive_map.end()) {
        return;
    }
    if (primitive_map[id]->executor())
        primitive_map[id]->executor()->notify_msg();
}

} /* namespace gr */
