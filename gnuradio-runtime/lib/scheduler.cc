/* -*- c++ -*- */
/*
 * Copyright 2008,2013 Free Software Foundation, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "thread_body.h"
#include <gnuradio/scheduler.h>

namespace gr {

scheduler_sptr make_scheduler(flat_flowgraph_sptr ffg)
{
    return scheduler_sptr(new scheduler(ffg));
}

scheduler::scheduler(flat_flowgraph_sptr ffg)
{
    basic_block_vector_t used_blocks = ffg->calc_used_blocks();
    block_vector_t blocks = flat_flowgraph::make_block_vector(used_blocks);

    // Ensure that the done flag is clear on all blocks
    for (size_t i = 0; i < blocks.size(); i++) {
        blocks[i]->executor()->set_done(false);
    }

    thread::barrier_sptr start_sync =
        boost::make_shared<thread::barrier>(blocks.size() + 1);

    // Fire off a thead for each block
    for (size_t i = 0; i < blocks.size(); i++) {
        std::stringstream name;
        name << "thread-per-block[" << i << "]: " << blocks[i];

        auto f = boost::bind(
            &thread_body::run_thread, blocks[i], start_sync);
        d_threads.create_thread(f);
    }
    start_sync->wait();
}

scheduler::~scheduler() { stop(); }

void scheduler::stop() { d_threads.interrupt_all(); }

void scheduler::wait() { d_threads.join_all(); }

} /* namespace gr */
