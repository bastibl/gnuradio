/* -*- c++ -*- */
/*
 * Copyright 2004,2008-2010,2013,2017 Free Software Foundation, Inc.
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

#include <gnuradio/block_executor.h>
#include <gnuradio/buffer.h>
#include <gnuradio/prefs.h>
#include <assert.h>
#include <boost/format.hpp>

namespace gr {

// must be defined to either 0 or 1
#define ENABLE_LOGGING 0

#if (ENABLE_LOGGING)
#define LOG(x) \
    do {       \
        x;     \
    } while (0)
#else
#define LOG(x) \
    do {       \
        ;      \
    } while (0)
#endif

block_executor_uptr
make_block_executor(block_sptr block, unsigned int ninputs, unsigned int noutputs)
{
    return block_executor_uptr(new block_executor(block, ninputs, noutputs));
}

inline static unsigned int round_up(unsigned int n, unsigned int multiple)
{
    return ((n + multiple - 1) / multiple) * multiple;
}

inline static unsigned int round_down(unsigned int n, unsigned int multiple)
{
    return (n / multiple) * multiple;
}

//
// Return minimum available write space in all our downstream
// buffers or -1 if we're output blocked and the output we're
// blocked on is done.
//
int block_executor::min_available_space(int output_multiple, int min_noutput_items)
{
    int min_space = std::numeric_limits<int>::max();
    if (min_noutput_items == 0)
        min_noutput_items = 1;
    for (int i = 0; i < noutputs(); i++) {
        buffer* out_buf = output(i);
        gr::thread::scoped_lock guard(*out_buf->mutex());
        int avail_n = round_down(out_buf->space_available(), output_multiple);
        int best_n = round_down(out_buf->bufsize() / 2, output_multiple);
        if (best_n < min_noutput_items)
            throw std::runtime_error("Buffer too small for min_noutput_items");
        int n = std::min(avail_n, best_n);
        if (n < min_noutput_items) { // We're blocked on output.
            if (out_buf->done()) {   // Downstream is done, therefore we're done.
                return -1;
            }
            return 0;
        }
        min_space = std::min(min_space, n);
    }
    return min_space;
}

bool block_executor::propagate_tags(block::tag_propagation_policy_t policy,
                                    const std::vector<uint64_t>& start_nitems_read,
                                    double rrate,
                                    mpq_class& mp_rrate,
                                    bool use_fp_rrate,
                                    std::vector<tag_t>& rtags,
                                    long block_id)
{
    static const mpq_class one_half(1, 2);

    // Move tags downstream
    // if a sink, we don't need to move downstream
    if (sink_p()) {
        return true;
    }

    switch (policy) {
    case block::TPP_DONT:
    case block::TPP_CUSTOM:
        return true;
    case block::TPP_ALL_TO_ALL: {
        // every tag on every input propagates to everyone downstream
        std::vector<buffer*> out_buf;

        for (int i = 0; i < ninputs(); i++) {
            get_tags_in_range(rtags, i, start_nitems_read[i], nitems_read(i), block_id);

            if (rtags.empty()) {
                continue;
            }

            if (out_buf.empty()) {
                out_buf.reserve(noutputs());
                for (int o = 0; o < noutputs(); o++)
                    out_buf.push_back(output(o));
            }

            std::vector<tag_t>::iterator t;
            if (rrate == 1.0) {
                for (t = rtags.begin(); t != rtags.end(); t++) {
                    for (int o = 0; o < noutputs(); o++)
                        out_buf[o]->add_item_tag(*t);
                }
            } else if (use_fp_rrate) {
                for (t = rtags.begin(); t != rtags.end(); t++) {
                    tag_t new_tag = *t;
                    new_tag.offset = ((double)new_tag.offset * rrate) + 0.5;
                    for (int o = 0; o < noutputs(); o++)
                        out_buf[o]->add_item_tag(new_tag);
                }
            } else {
                mpz_class offset;
                for (t = rtags.begin(); t != rtags.end(); t++) {
                    tag_t new_tag = *t;
                    mpz_import(offset.get_mpz_t(),
                               1,
                               1,
                               sizeof(new_tag.offset),
                               0,
                               0,
                               &new_tag.offset);
                    offset = offset * mp_rrate + one_half;
                    new_tag.offset = offset.get_ui();
                    for (int o = 0; o < noutputs(); o++)
                        out_buf[o]->add_item_tag(new_tag);
                }
            }
        }
    } break;
    case block::TPP_ONE_TO_ONE:
        // tags from input i only go to output i
        // this requires ninputs() == noutputs; this is checked when this
        // type of tag-propagation system is selected in block_executor
        if (ninputs() == noutputs()) {
            buffer* out_buf;

            for (int i = 0; i < ninputs(); i++) {
                get_tags_in_range(
                    rtags, i, start_nitems_read[i], nitems_read(i), block_id);

                if (rtags.empty()) {
                    continue;
                }

                out_buf = output(i);

                std::vector<tag_t>::iterator t;
                if (rrate == 1.0) {
                    for (t = rtags.begin(); t != rtags.end(); t++) {
                        out_buf->add_item_tag(*t);
                    }
                } else if (use_fp_rrate) {
                    for (t = rtags.begin(); t != rtags.end(); t++) {
                        tag_t new_tag = *t;
                        new_tag.offset = ((double)new_tag.offset * rrate) + 0.5;
                        out_buf->add_item_tag(new_tag);
                    }
                } else {
                    mpz_class offset;
                    for (t = rtags.begin(); t != rtags.end(); t++) {
                        tag_t new_tag = *t;
                        mpz_import(offset.get_mpz_t(),
                                   1,
                                   1,
                                   sizeof(new_tag.offset),
                                   0,
                                   0,
                                   &new_tag.offset);
                        offset = offset * mp_rrate + one_half;
                        new_tag.offset = offset.get_ui();
                        out_buf->add_item_tag(new_tag);
                    }
                }
            }
        } else {
            std::cerr << "Error: block_executor: propagation_policy 'ONE-TO-ONE' "
                         "requires ninputs == noutputs"
                      << std::endl;
            return false;
        }
        break;
    default:
        return true;
    }
    return true;
}

block_executor::block_executor(block_sptr block,
                               unsigned int ninputs,
                               unsigned int noutputs)
    : d_block(block),
      d_log(0),
      d_max_noutput_items(0),
      d_produce_or(0),
      d_input_changed(false),
      d_output_changed(false),
      d_ninputs(ninputs),
      d_noutputs(noutputs),
      d_input(ninputs),
      d_output(noutputs),
      d_done(false),
      d_ins_noutput_items(0),
      d_avg_noutput_items(0),
      d_var_noutput_items(0),
      d_total_noutput_items(0),
      d_ins_nproduced(0),
      d_avg_nproduced(0),
      d_var_nproduced(0),
      d_ins_input_buffers_full(ninputs, 0),
      d_avg_input_buffers_full(ninputs, 0),
      d_var_input_buffers_full(ninputs, 0),
      d_ins_output_buffers_full(noutputs, 0),
      d_avg_output_buffers_full(noutputs, 0),
      d_var_output_buffers_full(noutputs, 0),
      d_ins_work_time(0),
      d_avg_work_time(0),
      d_var_work_time(0),
      d_avg_throughput(0),
      d_pc_counter(0)
{
}

void block_executor::start()
{
    d_pc_start_time = gr::high_res_timer_now();

    if (ENABLE_LOGGING) {
        std::string name = str(boost::format("scheduler.log"));
        d_log = new std::ofstream(name.c_str());
        std::unitbuf(*d_log); // make it unbuffered...
        *d_log << "block_executor: " << d_block << std::endl;
    }

    // Only run setup_rpc if ControlPort config param is enabled.
    bool ctrlport_on = prefs::singleton()->get_bool("ControlPort", "on", false);

    // if ctrlport is enabled, call setup RPC for all blocks in the flowgraph
    if (ctrlport_on) {
        if (!d_block->is_rpc_set()) {
            d_block->setup_rpc();
            d_block->rpc_set();
        }
    }

#ifdef GR_PERFORMANCE_COUNTERS
    prefs* prefs = prefs::singleton();
    d_use_pc = prefs->get_bool("PerfCounters", "on", false);
#endif /* GR_PERFORMANCE_COUNTERS */

    d_block->start(); // enable any drivers, etc.
}

block_executor::~block_executor()
{
    if (ENABLE_LOGGING)
        delete d_log;

    d_block->stop(); // stop any drivers, etc.
}

block_executor::state block_executor::run_one_iteration()
{
    int noutput_items;
    int max_items_avail;
    int max_noutput_items;
    int new_alignment = 0;
    int alignment_state = -1;

    block* m = d_block.get();

    LOG(*d_log << std::endl << m);

    max_noutput_items = round_down(d_max_noutput_items, m->output_multiple());

    if (done()) {
        assert(0);
        return DONE;
    }

    if (source_p()) {
        d_ninput_items_required.resize(0);
        d_ninput_items.resize(0);
        d_input_items.resize(0);
        d_input_done.resize(0);
        d_output_items.resize(noutputs());
        d_start_nitems_read.resize(0);

        // determine the minimum available output space
        noutput_items = min_available_space(m->output_multiple(), m->min_noutput_items());
        noutput_items = std::min(noutput_items, max_noutput_items);
        LOG(*d_log << " source\n  noutput_items = " << noutput_items << std::endl);
        if (noutput_items == -1) // we're done
            goto were_done;

        if (noutput_items == 0) { // we're output blocked
            LOG(*d_log << "  BLKD_OUT\n");
            return BLKD_OUT;
        }

        goto setup_call_to_work; // jump to common code
    }

    else if (sink_p()) {
        d_ninput_items_required.resize(ninputs());
        d_ninput_items.resize(ninputs());
        d_input_items.resize(ninputs());
        d_input_done.resize(ninputs());
        d_output_items.resize(0);
        d_start_nitems_read.resize(ninputs());
        LOG(*d_log << " sink\n");

        max_items_avail = 0;
        for (int i = 0; i < ninputs(); i++) {
            {
                /*
                 * Acquire the mutex and grab local copies of items_available and done.
                 */
                buffer_reader* in_buf = input(i);
                gr::thread::scoped_lock guard(*in_buf->mutex());
                d_ninput_items[i] = in_buf->items_available();
                d_input_done[i] = in_buf->done();
            }

            LOG(*d_log << "  d_ninput_items[" << i << "] = " << d_ninput_items[i]
                       << std::endl);
            LOG(*d_log << "  d_input_done[" << i << "] = " << d_input_done[i]
                       << std::endl);

            if (d_ninput_items[i] < m->output_multiple() && d_input_done[i])
                goto were_done;

            max_items_avail = std::max(max_items_avail, d_ninput_items[i]);
        }

        // take a swag at how much output we can sink
        noutput_items = (int)(max_items_avail * m->relative_rate());
        noutput_items = round_down(noutput_items, m->output_multiple());
        noutput_items = std::min(noutput_items, max_noutput_items);
        LOG(*d_log << "  max_items_avail = " << max_items_avail << std::endl);
        LOG(*d_log << "  noutput_items = " << noutput_items << std::endl);

        if (noutput_items == 0) { // we're blocked on input
            LOG(*d_log << "  BLKD_IN\n");
            return BLKD_IN;
        }

        goto try_again; // Jump to code shared with regular case.
    }

    else {
        // do the regular thing
        d_ninput_items_required.resize(ninputs());
        d_ninput_items.resize(ninputs());
        d_input_items.resize(ninputs());
        d_input_done.resize(ninputs());
        d_output_items.resize(noutputs());
        d_start_nitems_read.resize(ninputs());

        max_items_avail = 0;
        for (int i = 0; i < ninputs(); i++) {
            {
                /*
                 * Acquire the mutex and grab local copies of items_available and done.
                 */
                buffer_reader* in_buf = input(i);
                gr::thread::scoped_lock guard(*in_buf->mutex());
                d_ninput_items[i] = in_buf->items_available();
                d_input_done[i] = in_buf->done();
            }
            max_items_avail = std::max(max_items_avail, d_ninput_items[i]);
        }

        // determine the minimum available output space
        noutput_items = min_available_space(m->output_multiple(), m->min_noutput_items());
        if (ENABLE_LOGGING) {
            *d_log << " regular ";
            *d_log << m->relative_rate_i() << ":" << m->relative_rate_d() << std::endl;
            *d_log << "  max_items_avail = " << max_items_avail << std::endl;
            *d_log << "  noutput_items = " << noutput_items << std::endl;
        }
        if (noutput_items == -1) // we're done
            goto were_done;

        if (noutput_items == 0) { // we're output blocked
            LOG(*d_log << "  BLKD_OUT\n");
            return BLKD_OUT;
        }

    try_again:
        if (m->fixed_rate()) {
            // try to work it forward starting with max_items_avail.
            // We want to try to consume all the input we've got.
            int reqd_noutput_items = m->fixed_rate_ninput_to_noutput(max_items_avail);

            // only test this if we specifically set the output_multiple
            if (m->output_multiple_set())
                reqd_noutput_items = round_down(reqd_noutput_items, m->output_multiple());

            if (reqd_noutput_items > 0 && reqd_noutput_items <= noutput_items)
                noutput_items = reqd_noutput_items;

            // if we need this many outputs, overrule the max_noutput_items setting
            max_noutput_items = std::max(m->output_multiple(), max_noutput_items);
        }
        noutput_items = std::min(noutput_items, max_noutput_items);

        // Check if we're still unaligned; use up items until we're
        // aligned again. Otherwise, make sure we set the alignment
        // requirement.
        if (!m->output_multiple_set()) {
            if (m->is_unaligned()) {
                // When unaligned, don't just set noutput_items to the remaining
                // samples to meet alignment; this causes too much overhead in
                // requiring a premature call back here. Set the maximum amount
                // of samples to handle unalignment and get us back aligned.
                if (noutput_items >= m->unaligned()) {
                    noutput_items = round_up(noutput_items, m->alignment()) -
                                    (m->alignment() - m->unaligned());
                    new_alignment = 0;
                } else {
                    new_alignment = m->unaligned() - noutput_items;
                }
                alignment_state = 0;
            } else if (noutput_items < m->alignment()) {
                // if we don't have enough for an aligned call, keep track of
                // misalignment, set unaligned flag, and proceed.
                new_alignment = m->alignment() - noutput_items;
                m->set_unaligned(new_alignment);
                m->set_is_unaligned(true);
                alignment_state = 1;
            } else {
                // enough to round down to the nearest alignment and process.
                noutput_items = round_down(noutput_items, m->alignment());
                m->set_is_unaligned(false);
                alignment_state = 2;
            }
        }

        // ask the block how much input they need to produce noutput_items
        m->forecast(noutput_items, d_ninput_items_required);

        // See if we've got sufficient input available and make sure we
        // didn't overflow on the input.
        int i;
        for (i = 0; i < ninputs(); i++) {
            if (d_ninput_items_required[i] > d_ninput_items[i]) // not enough
                break;

            if (d_ninput_items_required[i] < 0) {
                std::cerr << "\nsched: <block " << m->name() << " (" << m->unique_id()
                          << ")>"
                          << " thinks its ninput_items required is "
                          << d_ninput_items_required[i] << " and cannot be negative.\n"
                          << "Some parameterization is wrong. "
                          << "Too large a decimation value?\n\n";
                goto were_done;
            }
        }

        if (i < ninputs()) { // not enough input on input[i]
            // if we can, try reducing the size of our output request
            if (noutput_items > m->output_multiple()) {
                noutput_items /= 2;
                noutput_items = round_up(noutput_items, m->output_multiple());
                goto try_again;
            }

            // We're blocked on input
            LOG(*d_log << "  BLKD_IN\n");
            if (d_input_done[i]) // If the upstream block is done, we're done
                goto were_done;

            // Is it possible to ever fulfill this request?
            buffer_reader* in_buf = input(i);
            if (d_ninput_items_required[i] > in_buf->max_possible_items_available()) {
                // Nope, never going to happen...
                std::cerr
                    << "\nsched: <block " << m->name() << " (" << m->unique_id() << ")>"
                    << " is requesting more input data\n"
                    << "  than we can provide.\n"
                    << "  ninput_items_required = " << d_ninput_items_required[i] << "\n"
                    << "  max_possible_items_available = "
                    << in_buf->max_possible_items_available() << "\n"
                    << "  If this is a filter, consider reducing the number of taps.\n";
                goto were_done;
            }

            // If we were made unaligned in this round but return here without
            // processing; reset the unalignment claim before next entry.
            if (alignment_state == 1) {
                m->set_unaligned(0);
                m->set_is_unaligned(false);
            }
            return BLKD_IN;
        }

        // We've got enough data on each input to produce noutput_items.
        // Finish setting up the call to work.
        for (int i = 0; i < ninputs(); i++)
            d_input_items[i] = input(i)->read_pointer();

    setup_call_to_work:

        d_produce_or = 0;
        for (int i = 0; i < noutputs(); i++)
            d_output_items[i] = output(i)->write_pointer();

        // determine where to start looking for new tags
        for (int i = 0; i < ninputs(); i++)
            d_start_nitems_read[i] = nitems_read(i);

#ifdef GR_PERFORMANCE_COUNTERS
        if (d_use_pc)
            start_perf_counters();
#endif /* GR_PERFORMANCE_COUNTERS */

        // Do the actual work of the block
        int n =
            m->general_work(noutput_items, d_ninput_items, d_input_items, d_output_items);

#ifdef GR_PERFORMANCE_COUNTERS
        if (d_use_pc)
            stop_perf_counters(noutput_items, n);
#endif /* GR_PERFORMANCE_COUNTERS */

        LOG(*d_log << "  general_work: noutput_items = " << noutput_items
                   << " result = " << n << std::endl);

        // Adjust number of unaligned items left to process
        if (m->is_unaligned()) {
            m->set_unaligned(new_alignment);
            m->set_is_unaligned(m->unaligned() != 0);
        }

        // Now propagate the tags based on the new relative rate
        if (!propagate_tags(m->tag_propagation_policy(),
                            d_start_nitems_read,
                            m->relative_rate(),
                            m->mp_relative_rate(),
                            m->update_rate(),
                            d_returned_tags,
                            m->unique_id()))
            goto were_done;

        if (n == block::WORK_DONE)
            goto were_done;

        if (n != block::WORK_CALLED_PRODUCE)
            produce_each(n); // advance write pointers

        // For some blocks that can change their produce/consume ratio
        // (the relative_rate), we might want to automatically update
        // based on the amount of items written/read.
        // In the block constructor, use enable_update_rate(true).
        if (m->update_rate()) {
            // rrate = ((double)(m->nitems_written(0))) / ((double)m->nitems_read(0));
            // if(rrate > 0.0)
            //  m->set_relative_rate(rrate);
            if ((n > 0) && (consumed() > 0))
                m->set_relative_rate((uint64_t)n, (uint64_t)consumed());
        }

        if (d_produce_or > 0) // block produced something
            return READY;

        // We didn't produce any output even though we called general_work.
        // We have (most likely) consumed some input.

        /*
        // If this is a source, it's broken.
        if(source_p()) {
          std::cerr << "block_executor: source " << m
                    << " produced no output.  We're marking it DONE.\n";
          // FIXME maybe we ought to raise an exception...
          goto were_done;
        }
        */

        // Have the caller try again...
        return READY_NO_OUTPUT;
    }
    assert(0);

were_done:
    LOG(*d_log << "  were_done\n");
    set_done(true);
    return DONE;
}


void block_executor::set_input(unsigned int which, buffer_reader_uptr reader)
{
    if (which >= d_ninputs)
        throw std::invalid_argument("block_executor::set_input");

    d_input[which] = std::move(reader);
}

void block_executor::set_output(unsigned int which, buffer_uptr buffer)
{
    if (which >= d_noutputs)
        throw std::invalid_argument("block_executor::set_output");

    d_output[which] = std::move(buffer);
}

void block_executor::set_done(bool done)
{
    d_done = done;
    for (unsigned int i = 0; i < d_noutputs; i++)
        d_output[i]->set_done(done);

    for (unsigned int i = 0; i < d_ninputs; i++)
        d_input[i]->set_done(done);
}

void block_executor::consume(int which_input, int how_many_items)
{
    d_consumed = how_many_items;
    if (how_many_items > 0) {
        input(which_input)->update_read_pointer(how_many_items);
    }
}

int block_executor::consumed() const { return d_consumed; }

void block_executor::consume_each(int how_many_items)
{
    d_consumed = how_many_items;
    if (how_many_items > 0) {
        for (int i = 0; i < ninputs(); i++) {
            d_input[i]->update_read_pointer(how_many_items);
        }
    }
}

void block_executor::produce(int which_output, int how_many_items)
{
    if (how_many_items > 0) {
        d_output[which_output]->update_write_pointer(how_many_items);
        d_produce_or |= how_many_items;
    }
}

void block_executor::produce_each(int how_many_items)
{
    if (how_many_items > 0) {
        for (int i = 0; i < noutputs(); i++) {
            d_output[i]->update_write_pointer(how_many_items);
        }
        d_produce_or |= how_many_items;
    }
}

uint64_t block_executor::nitems_read(unsigned int which_input)
{
    if (which_input >= d_ninputs)
        throw std::invalid_argument("block_executor::n_input_items");
    return d_input[which_input]->nitems_read();
}

uint64_t block_executor::nitems_written(unsigned int which_output)
{
    if (which_output >= d_noutputs)
        throw std::invalid_argument("block_executor::n_output_items");
    return d_output[which_output]->nitems_written();
}

void block_executor::reset_nitem_counters()
{
    for (unsigned int i = 0; i < d_ninputs; i++) {
        d_input[i]->reset_nitem_counter();
    }
    for (unsigned int o = 0; o < d_noutputs; o++) {
        d_output[o]->reset_nitem_counter();
    }
}

void block_executor::clear_tags()
{
    for (unsigned int i = 0; i < d_ninputs; i++) {
        uint64_t max_time = 0xFFFFFFFFFFFFFFFF; // from now to the end of time
        d_input[i]->buffer()->prune_tags(max_time);
    }
}

void block_executor::add_item_tag(unsigned int which_output, const tag_t& tag)
{
    d_output[which_output]->add_item_tag(tag);
}

void block_executor::remove_item_tag(unsigned int which_input, const tag_t& tag, long id)
{
    d_input[which_input]->buffer()->remove_item_tag(tag, id);
}

void block_executor::get_tags_in_range(std::vector<tag_t>& v,
                                       unsigned int which_input,
                                       uint64_t abs_start,
                                       uint64_t abs_end,
                                       long id)
{
    // get from gr_buffer_reader's deque of tags
    d_input[which_input]->get_tags_in_range(v, abs_start, abs_end, id);
}

void block_executor::get_tags_in_range(std::vector<tag_t>& v,
                                       unsigned int which_input,
                                       uint64_t abs_start,
                                       uint64_t abs_end,
                                       const std::string& key,
                                       long id)
{
    std::vector<tag_t> found_items;

    v.resize(0);

    // get from gr_buffer_reader's deque of tags
    d_input[which_input]->get_tags_in_range(found_items, abs_start, abs_end, id);

    // Filter further by key name
    std::string itemkey;
    std::vector<tag_t>::iterator itr;
    for (itr = found_items.begin(); itr != found_items.end(); itr++) {
        itemkey = (*itr).key;
        if (key == itemkey) {
            v.push_back(*itr);
        }
    }
}

void block_executor::set_processor_affinity(const std::vector<int>& mask)
{
    if (d_threaded) {
        try {
            gr::thread::thread_bind_to_processor(d_thread, mask);
        } catch (std::runtime_error& e) {
            std::cerr << "set_processor_affinity: invalid mask." << std::endl;
        }
    }
}

void block_executor::unset_processor_affinity()
{
    if (d_threaded) {
        gr::thread::thread_unbind(d_thread);
    }
}

int block_executor::thread_priority()
{
    if (d_threaded) {
        return gr::thread::thread_priority(d_thread);
    }
    return -1;
}

int block_executor::set_thread_priority(int priority)
{
    if (d_threaded) {
        return gr::thread::set_thread_priority(d_thread, priority);
    }
    return -1;
}

void block_executor::notify_upstream()
{
    // For each of our inputs, tell the guy upstream that we've
    // consumed some input, and that he most likely has more output
    // buffer space available.

    for (size_t i = 0; i < d_input.size(); i++) {
        // Can you say, "pointer chasing?"
        d_input[i]->buffer()->link()->executor()->set_output_changed();
    }
}

void block_executor::notify_downstream()
{
    // For each of our outputs, tell the guys downstream that they
    // have new input available.

    for (size_t i = 0; i < d_output.size(); i++) {
        buffer* buf = d_output[i].get();
        for (size_t j = 0, k = buf->nreaders(); j < k; j++)
            buf->reader(j)->link()->executor()->set_input_changed();
    }
}

void block_executor::notify_neighbors()
{
    notify_downstream();
    notify_upstream();
}


void block_executor::start_perf_counters()
{
    d_start_of_work = gr::high_res_timer_now_perfmon();
}

void block_executor::stop_perf_counters(int noutput_items, int nproduced)
{
    d_end_of_work = gr::high_res_timer_now_perfmon();
    gr::high_res_timer_type diff = d_end_of_work - d_start_of_work;

    if (d_pc_counter == 0) {
        d_ins_work_time = diff;
        d_avg_work_time = diff;
        d_var_work_time = 0;
        d_total_work_time = diff;
        d_ins_nproduced = nproduced;
        d_avg_nproduced = nproduced;
        d_var_nproduced = 0;
        d_ins_noutput_items = noutput_items;
        d_avg_noutput_items = noutput_items;
        d_var_noutput_items = 0;
        d_total_noutput_items = noutput_items;
        d_pc_start_time = (float)gr::high_res_timer_now();
        for (size_t i = 0; i < d_input.size(); i++) {
            buffer_reader* in_buf = d_input[i].get();
            gr::thread::scoped_lock guard(*in_buf->mutex());
            float pfull = static_cast<float>(in_buf->items_available()) /
                          static_cast<float>(in_buf->max_possible_items_available());
            d_ins_input_buffers_full[i] = pfull;
            d_avg_input_buffers_full[i] = pfull;
            d_var_input_buffers_full[i] = 0;
        }
        for (size_t i = 0; i < d_output.size(); i++) {
            buffer* out_buf = d_output[i].get();
            gr::thread::scoped_lock guard(*out_buf->mutex());
            float pfull = 1.0f - static_cast<float>(out_buf->space_available()) /
                                     static_cast<float>(out_buf->bufsize());
            d_ins_output_buffers_full[i] = pfull;
            d_avg_output_buffers_full[i] = pfull;
            d_var_output_buffers_full[i] = 0;
        }
    } else {
        float d = diff - d_avg_work_time;
        d_ins_work_time = diff;
        d_avg_work_time = d_avg_work_time + d / d_pc_counter;
        d_var_work_time = d_var_work_time + d * d;
        d_total_work_time += diff;

        d = nproduced - d_avg_nproduced;
        d_ins_nproduced = nproduced;
        d_avg_nproduced = d_avg_nproduced + d / d_pc_counter;
        d_var_nproduced = d_var_nproduced + d * d;

        d = noutput_items - d_avg_noutput_items;
        d_ins_noutput_items = noutput_items;
        d_avg_noutput_items = d_avg_noutput_items + d / d_pc_counter;
        d_var_noutput_items = d_var_noutput_items + d * d;
        d_total_noutput_items += noutput_items;
        d_pc_last_work_time = gr::high_res_timer_now();
        float monitor_time = (float)(d_pc_last_work_time - d_pc_start_time) /
                             (float)gr::high_res_timer_tps();
        d_avg_throughput = d_total_noutput_items / monitor_time;

        for (size_t i = 0; i < d_input.size(); i++) {
            buffer_reader* in_buf = d_input[i].get();
            gr::thread::scoped_lock guard(*in_buf->mutex());
            float pfull = static_cast<float>(in_buf->items_available()) /
                          static_cast<float>(in_buf->max_possible_items_available());

            d = pfull - d_avg_input_buffers_full[i];
            d_ins_input_buffers_full[i] = pfull;
            d_avg_input_buffers_full[i] = d_avg_input_buffers_full[i] + d / d_pc_counter;
            d_var_input_buffers_full[i] = d_var_input_buffers_full[i] + d * d;
        }

        for (size_t i = 0; i < d_output.size(); i++) {
            buffer* out_buf = d_output[i].get();
            gr::thread::scoped_lock guard(*out_buf->mutex());
            float pfull = 1.0f - static_cast<float>(out_buf->space_available()) /
                                     static_cast<float>(out_buf->bufsize());

            d = pfull - d_avg_output_buffers_full[i];
            d_ins_output_buffers_full[i] = pfull;
            d_avg_output_buffers_full[i] =
                d_avg_output_buffers_full[i] + d / d_pc_counter;
            d_var_output_buffers_full[i] = d_var_output_buffers_full[i] + d * d;
        }
    }

    d_pc_counter++;
}

void block_executor::reset_perf_counters() { d_pc_counter = 0; }

float block_executor::pc_noutput_items() { return d_ins_noutput_items; }

float block_executor::pc_nproduced() { return d_ins_nproduced; }

float block_executor::pc_input_buffers_full(size_t which)
{
    if (which < d_ins_input_buffers_full.size())
        return d_ins_input_buffers_full[which];
    else
        return 0;
}

std::vector<float> block_executor::pc_input_buffers_full()
{
    return d_ins_input_buffers_full;
}

float block_executor::pc_output_buffers_full(size_t which)
{
    if (which < d_ins_output_buffers_full.size())
        return d_ins_output_buffers_full[which];
    else
        return 0;
}

std::vector<float> block_executor::pc_output_buffers_full()
{
    return d_ins_output_buffers_full;
}

float block_executor::pc_work_time() { return d_ins_work_time; }

float block_executor::pc_noutput_items_avg() { return d_avg_noutput_items; }

float block_executor::pc_nproduced_avg() { return d_avg_nproduced; }

float block_executor::pc_input_buffers_full_avg(size_t which)
{
    if (which < d_avg_input_buffers_full.size())
        return d_avg_input_buffers_full[which];
    else
        return 0;
}

std::vector<float> block_executor::pc_input_buffers_full_avg()
{
    return d_avg_input_buffers_full;
}

float block_executor::pc_output_buffers_full_avg(size_t which)
{
    if (which < d_avg_output_buffers_full.size())
        return d_avg_output_buffers_full[which];
    else
        return 0;
}

std::vector<float> block_executor::pc_output_buffers_full_avg()
{
    return d_avg_output_buffers_full;
}

float block_executor::pc_work_time_avg() { return d_avg_work_time; }

float block_executor::pc_noutput_items_var()
{
    return d_var_noutput_items / (d_pc_counter - 1);
}

float block_executor::pc_nproduced_var() { return d_var_nproduced / (d_pc_counter - 1); }

float block_executor::pc_input_buffers_full_var(size_t which)
{
    if (which < d_avg_input_buffers_full.size())
        return d_var_input_buffers_full[which] / (d_pc_counter - 1);
    else
        return 0;
}

std::vector<float> block_executor::pc_input_buffers_full_var()
{
    std::vector<float> var(d_avg_input_buffers_full.size(), 0);
    for (size_t i = 0; i < d_avg_input_buffers_full.size(); i++)
        var[i] = d_avg_input_buffers_full[i] / (d_pc_counter - 1);
    return var;
}

float block_executor::pc_output_buffers_full_var(size_t which)
{
    if (which < d_avg_output_buffers_full.size())
        return d_var_output_buffers_full[which] / (d_pc_counter - 1);
    else
        return 0;
}

std::vector<float> block_executor::pc_output_buffers_full_var()
{
    std::vector<float> var(d_avg_output_buffers_full.size(), 0);
    for (size_t i = 0; i < d_avg_output_buffers_full.size(); i++)
        var[i] = d_avg_output_buffers_full[i] / (d_pc_counter - 1);
    return var;
}

float block_executor::pc_work_time_var() { return d_var_work_time / (d_pc_counter - 1); }

float block_executor::pc_work_time_total() { return d_total_work_time; }

float block_executor::pc_throughput_avg() { return d_avg_throughput; }


} /* namespace gr */
