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

#ifndef GR_RUNTIME_BLOCK_REGISTRY_H
#define GR_RUNTIME_BLOCK_REGISTRY_H

#include <gnuradio/api.h>
#include <gnuradio/runtime_types.h>
#include <gnuradio/thread/thread.h>
#include <pmt/pmt.h>
#include <map>

namespace gr {

class GR_RUNTIME_API block_registry
{
public:
    block_registry();

    uint64_t block_register(basic_block* block);
    void block_unregister(basic_block* block);

    void update_alias(basic_block* block, std::string name);
    basic_block_sptr block_lookup(pmt::pmt_t symbol);

    void register_primitive(uint64_t id, gr::block* ref);
    void unregister_primitive(uint64_t id);
    void notify_blk(uint64_t id);

private:
    std::map<uint64_t, basic_block*> d_id_map;
    std::map<std::string, basic_block*> d_name_map;
    std::map<std::string, basic_block*> d_alias_map;

    uint64_t d_seq_nr;

    std::map<uint64_t, block*> primitive_map;
    gr::thread::mutex d_mutex;
};

} /* namespace gr */

GR_RUNTIME_API extern gr::block_registry global_block_registry;

#endif /* GR_RUNTIME_BLOCK_REGISTRY_H */
