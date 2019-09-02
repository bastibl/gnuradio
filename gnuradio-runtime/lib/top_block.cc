/* -*- c++ -*- */
/*
 * Copyright 2007,2008,2013 Free Software Foundation, Inc.
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
#include <gnuradio/top_block.h>
#include <gnuradio/prefs.h>
#include <gnuradio/top_block.h>
#ifdef GR_CTRLPORT
#include <gnuradio/rpcregisterhelpers.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>

namespace gr {


top_block::sptr top_block::make(const std::string& name)
{
    return gnuradio::get_initial_sptr(new top_block(name));
}

top_block::top_block(const std::string& name)
    : hier_block(name, io_signature::make(0, 0, 0), io_signature::make(0, 0, 0)),
      d_ffg(),
      d_state(IDLE),
      d_lock_count(0),
      d_retry_wait(false)
{
}

top_block::~top_block()
{
    stop();
    wait();

    if (d_lock_count) {
        std::cerr << "error: destroying locked block." << std::endl;
    }
}

void top_block::start(int max_noutput_items)
{
    gr::thread::scoped_lock l(d_mutex);

#ifdef GNURADIO_HRT_USE_CLOCK_GETTIME
    std::string initial_clock =
        prefs::singleton()->get_string("PerfCounters", "clock", "thread");
    if (initial_clock.compare("thread") == 0) {
        gr::high_res_timer_source = CLOCK_THREAD_CPUTIME_ID;
    } else if (initial_clock.compare("monotonic") == 0) {
        gr::high_res_timer_source = CLOCK_MONOTONIC;
    } else {
        throw std::runtime_error("bad argument for PerfCounters.clock!");
    }
#endif

    d_max_noutput_items = max_noutput_items;

    if (d_state != IDLE)
        throw std::runtime_error("top_block::start: top block already running or wait() "
                                 "not called after previous stop()");

    if (d_lock_count > 0)
        throw std::runtime_error("top_block::start: can't start with flow graph locked");

    // Create new flat flow graph by flattening hierarchy
    d_ffg = flatten();

    // Validate new simple flow graph and wire it up
    d_ffg->validate();
    d_ffg->setup_connections(max_noutput_items);

    // Only export perf. counters if ControlPort config param is
    // enabled and if the PerfCounter option 'export' is turned on.
    prefs* p = prefs::singleton();
    if (p->get_bool("ControlPort", "on", false) &&
        p->get_bool("PerfCounters", "export", false))
        d_ffg->enable_pc_rpc();

    d_scheduler = scheduler::make(d_ffg);
    d_state = RUNNING;

    if (prefs::singleton()->get_bool("ControlPort", "on", false)) {
        setup_rpc();
    }
}

void top_block::stop()
{
    gr::thread::scoped_lock lock(d_mutex);

    if (d_scheduler)
        d_scheduler->stop();

    d_state = IDLE;
}

void top_block::wait()
{
    do {
        wait_for_jobs();
        {
            gr::thread::scoped_lock lock(d_mutex);
            if (!d_lock_count) {
                if (d_retry_wait) {
                    d_retry_wait = false;
                    continue;
                }
                d_state = IDLE;
                break;
            }
            d_lock_cond.wait(lock);
        }
    } while (true);
}

void top_block::run(int max_noutput_items)
{
    start(max_noutput_items);
    wait();
}


void top_block::wait_for_jobs()
{
    if (d_scheduler)
        d_scheduler->wait();
}

// N.B. lock() and unlock() cannot be called from a flow graph
// thread or deadlock will occur when reconfiguration happens
void top_block::lock()
{
    gr::thread::scoped_lock lock(d_mutex);
    if (d_scheduler)
        d_scheduler->stop();
    d_lock_count++;
}

void top_block::unlock()
{
    gr::thread::scoped_lock lock(d_mutex);

    if (d_lock_count <= 0) {
        d_lock_count = 0; // fix it, then complain
        throw std::runtime_error("unpaired unlock() call");
    }

    d_lock_count--;
    if (d_lock_count > 0 || d_state == IDLE) // nothing to do
        return;

    restart();
    d_lock_cond.notify_all();
}


flat_flowgraph_sptr top_block::flatten() const
{
    flat_flowgraph_sptr new_ffg = make_flat_flowgraph();
    flatten_aux(new_ffg);

    // print all primitive connections at exit
    // std::cout << "flatten_aux finished in top_block" << std::endl;
    // new_ffg->dump();

    return new_ffg;
}


std::string top_block::dot_graph()
{
    return flatten()->dot_graph();
}

/*
 * restart is called with d_mutex held
 */
void top_block::restart()
{
    wait_for_jobs();

    // Create new simple flow graph
    d_ffg = flatten();
    d_ffg->validate();
    d_ffg->setup_connections(d_max_noutput_items);

    // Create a new scheduler to execute it
    d_scheduler = scheduler::make(d_ffg);
    d_retry_wait = true;
}

std::string top_block::edge_list()
{
    if (d_ffg)
        return d_ffg->edge_list();
    else
        return "";
}

std::string top_block::msg_edge_list()
{
    if (d_ffg)
        return d_ffg->msg_edge_list();
    else
        return "";
}

void top_block::dump()
{
    if (d_ffg)
        d_ffg->dump();
}

int top_block::max_noutput_items() { return d_max_noutput_items; }

void top_block::set_max_noutput_items(int nmax) { d_max_noutput_items = nmax; }

void top_block::setup_rpc()
{
#ifdef GR_CTRLPORT
    if (is_rpc_set())
        return;

    // Triggers
    d_rpc_vars.emplace_back(new rpcbasic_register_trigger<top_block>(
        alias(), "stop", &top_block::stop, "Stop the flowgraph", RPC_PRIVLVL_MIN));

    d_rpc_vars.emplace_back(new rpcbasic_register_trigger<top_block>(
        alias(), "lock", &top_block::lock, "Lock the flowgraph", RPC_PRIVLVL_MIN));

    d_rpc_vars.emplace_back(new rpcbasic_register_trigger<top_block>(
        alias(), "unlock", &top_block::unlock, "Unock the flowgraph", RPC_PRIVLVL_MIN));

    // Getters
    add_rpc_variable(rpcbasic_sptr(
        new rpcbasic_register_get<top_block, int>(alias(),
                                                  "max noutput_items",
                                                  &top_block::max_noutput_items,
                                                  pmt::mp(0),
                                                  pmt::mp(8192),
                                                  pmt::mp(8192),
                                                  "items",
                                                  "Max number of output items",
                                                  RPC_PRIVLVL_MIN,
                                                  DISPNULL)));

    if (prefs::singleton()->get_bool("ControlPort", "edges_list", false)) {
        add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<top_block, std::string>(
            alias(),
            "edge list",
            &top_block::edge_list,
            pmt::mp(""),
            pmt::mp(""),
            pmt::mp(""),
            "edges",
            "List of edges in the graph",
            RPC_PRIVLVL_MIN,
            DISPNULL)));
    }

    if (prefs::singleton()->get_bool("ControlPort", "edges_list", false)) {
        add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<top_block, std::string>(
            alias(),
            "msg edges list",
            &top_block::msg_edge_list,
            pmt::mp(""),
            pmt::mp(""),
            pmt::mp(""),
            "msg_edges",
            "List of msg edges in the graph",
            RPC_PRIVLVL_MIN,
            DISPNULL)));
    }

#ifdef GNURADIO_HRT_USE_CLOCK_GETTIME
    add_rpc_variable(rpcbasic_sptr(
        new rpcbasic_register_variable_rw<int>(alias(),
                                               "perfcounter_clock",
                                               (int*)&gr::high_res_timer_source,
                                               pmt::mp(0),
                                               pmt::mp(2),
                                               pmt::mp(2),
                                               "clock",
                                               "Performance Counters Realtime Clock Type",
                                               RPC_PRIVLVL_MIN,
                                               DISPNULL)));
#endif

    // Setters
    add_rpc_variable(rpcbasic_sptr(
        new rpcbasic_register_set<top_block, int>(alias(),
                                                  "max noutput_items",
                                                  &top_block::set_max_noutput_items,
                                                  pmt::mp(0),
                                                  pmt::mp(8192),
                                                  pmt::mp(8192),
                                                  "items",
                                                  "Max number of output items",
                                                  RPC_PRIVLVL_MIN,
                                                  DISPNULL)));
    rpc_set();
#endif /* GR_CTRLPORT */
}

} /* namespace gr */
