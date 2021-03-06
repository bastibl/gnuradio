/* -*- c++ -*- */
/*
 * Copyright 2008,2009,2011,2013 Free Software Foundation, Inc.
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
#include <gnuradio/prefs.h>
#include <pmt/pmt.h>
#include <boost/foreach.hpp>

#ifdef HAVE_SIGNAL_H
#include <signal.h>
#endif

namespace gr {


void thread_body::run_thread(block_sptr block, gr::thread::barrier_sptr start_sync)
{
    mask_signals();
    std::string name(
        boost::str(boost::format("%s%d") % block->name() % block->unique_id()));

    try {
        thread_body::execute_block(block, start_sync);
    } catch (boost::thread_interrupted const&) {
    } catch (std::exception const& e) {
        std::cerr << "thread[" << name << "]: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "thread[" << name << "]: "
                  << "caught unrecognized exception\n";
    }
}

void thread_body::execute_block(block_sptr block, gr::thread::barrier_sptr start_sync)
{
    block_executor* executor = block->executor();
    executor->start();

#if defined(_MSC_VER) || defined(__MINGW32__)
#include <windows.h>
    thread::set_thread_name(
        GetCurrentThread(),
        boost::str(boost::format("%s%d") % block->name() % block->unique_id()));
#else
    thread::set_thread_name(
        pthread_self(),
        boost::str(boost::format("%s%d") % block->name() % block->unique_id()));
#endif

    block_executor::state s;
    pmt::pmt_t msg;

    executor->d_threaded = true;
    executor->d_thread = gr::thread::get_current_thread_id();

    prefs* p = prefs::singleton();
    size_t max_nmsgs = static_cast<size_t>(p->get_long("DEFAULT", "max_messages", 100));

// Setup the logger for the scheduler
#undef LOG
    std::string config_file = p->get_string("LOG", "log_config", "");
    std::string log_level = p->get_string("LOG", "log_level", "off");
    std::string log_file = p->get_string("LOG", "log_file", "");
    GR_LOG_GETLOGGER(LOG, "gr_log.thread_body");
    GR_LOG_SET_LEVEL(LOG, log_level);
    GR_CONFIG_LOGGER(config_file);
    if (!log_file.empty()) {
        if (log_file == "stdout") {
            GR_LOG_SET_CONSOLE_APPENDER(LOG, "stdout", "gr::log :%p: %c{1} - %m%n");
        } else if (log_file == "stderr") {
            GR_LOG_SET_CONSOLE_APPENDER(LOG, "stderr", "gr::log :%p: %c{1} - %m%n");
        } else {
            GR_LOG_SET_FILE_APPENDER(LOG, log_file, true, "%r :%p: %c{1} - %m%n");
        }
    }

    // Set thread affinity if it was set before fg was started.
    if (!block->processor_affinity().empty()) {
        gr::thread::thread_bind_to_processor(executor->d_thread,
                                             block->processor_affinity());
    }

    // Set thread priority if it was set before fg was started
    if (block->thread_priority() > 0) {
        gr::thread::set_thread_priority(executor->d_thread, block->thread_priority());
    }

    // make sure our block isnt finished
    block->clear_finished();

    start_sync->wait();
    while (1) {
        boost::this_thread::interruption_point();

        executor->clear_changed();

        // handle any queued up messages
        BOOST_FOREACH (block::msg_queue_map_t::value_type& i, block->d_msg_queue) {
            // Check if we have a message handler attached before getting
            // any messages. This is mostly a protection for the unknown
            // startup sequence of the threads.
            if (block->has_msg_handler(i.first)) {
                while ((msg = block->delete_head_nowait(i.first))) {
                    block->dispatch_msg(i.first, msg);
                }
            } else {
                // If we don't have a handler but are building up messages,
                // prune the queue from the front to keep memory in check.
                if (block->nmsgs(i.first) > max_nmsgs) {
                    GR_LOG_WARN(
                        LOG, "asynchronous message buffer overflowing, dropping message");
                    msg = block->delete_head_nowait(i.first);
                }
            }
        }

        // run one iteration if we are a connected stream block
        if (executor->noutputs() > 0 || executor->ninputs() > 0) {
            s = executor->run_one_iteration();
        } else {
            if (block->finished()) {
                // a msg port only block wants to shutdown
                s = block_executor::DONE;
            } else {
                s = block_executor::BLKD_IN;
            }
        }

        if (block->finished() && s == block_executor::READY_NO_OUTPUT) {
            s = block_executor::DONE;
            executor->set_done(true);
        }

        if (!executor->ninputs() && s == block_executor::READY_NO_OUTPUT) {
            s = block_executor::BLKD_IN;
        }

        switch (s) {
        case block_executor::READY: // Tell neighbors we made progress.
            executor->notify_neighbors();
            break;

        case block_executor::READY_NO_OUTPUT: // Notify upstream only
            executor->notify_upstream();
            break;

        case block_executor::DONE: // Game over.
            block->shutdown_msg_neighbors();
            executor->notify_neighbors();
            return;

        case block_executor::BLKD_IN: // Wait for input.
        {
            gr::thread::scoped_lock guard(executor->d_mutex);

            if (!executor->d_input_changed) {
                executor->d_input_cond.wait(guard);
            }
        } break;

        case block_executor::BLKD_OUT: // Wait for output buffer space.
        {
            gr::thread::scoped_lock guard(executor->d_mutex);
            while (!executor->d_output_changed) {
                executor->d_output_cond.wait(guard);
            }
        } break;

        default:
            throw std::runtime_error("possible memory corruption in scheduler");
        }
    }
}

#if defined(HAVE_PTHREAD_SIGMASK) && defined(HAVE_SIGNAL_H) && !defined(__MINGW32__)

void thread_body::mask_signals()
{
    sigset_t new_set;
    int r;

    sigemptyset(&new_set);
    sigaddset(&new_set, SIGHUP); // block these...
    sigaddset(&new_set, SIGINT);
    sigaddset(&new_set, SIGPIPE);
    sigaddset(&new_set, SIGALRM);
    sigaddset(&new_set, SIGTERM);
    sigaddset(&new_set, SIGUSR1);
    sigaddset(&new_set, SIGCHLD);
#ifdef SIGPOLL
    sigaddset(&new_set, SIGPOLL);
#endif
#ifdef SIGPROF
    sigaddset(&new_set, SIGPROF);
#endif
#ifdef SIGSYS
    sigaddset(&new_set, SIGSYS);
#endif
#ifdef SIGTRAP
    sigaddset(&new_set, SIGTRAP);
#endif
#ifdef SIGURG
    sigaddset(&new_set, SIGURG);
#endif
#ifdef SIGVTALRM
    sigaddset(&new_set, SIGVTALRM);
#endif
#ifdef SIGXCPU
    sigaddset(&new_set, SIGXCPU);
#endif
#ifdef SIGXFSZ
    sigaddset(&new_set, SIGXFSZ);
#endif
    r = pthread_sigmask(SIG_BLOCK, &new_set, 0);
    if (r != 0)
        perror("pthread_sigmask");
}

#else

void thread_body::mask_signals() {}

#endif

} /* namespace gr */
