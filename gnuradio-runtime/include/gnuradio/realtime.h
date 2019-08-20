/* -*- c++ -*- */
/*
 * Copyright 2006,2013 Free Software Foundation, Inc.
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

#ifndef INCLUDED_GR_REALTIME_H
#define INCLUDED_GR_REALTIME_H

#include <gnuradio/api.h>
#include <stdexcept>

namespace gr {

typedef enum { RT_OK = 0, RT_NOT_IMPLEMENTED, RT_NO_PRIVS, RT_OTHER_ERROR } rt_status_t;

enum rt_sched_policy {
    RT_SCHED_RR = 0,   // round robin
    RT_SCHED_FIFO = 1, // first in first out
};

/*
 * Define the range for our virtual priorities (don't change
 * these)
 *
 * Processes (or threads) with numerically higher priority values
 * are scheduled before processes with numerically lower priority
 * values. Thus, the value returned by rt_priority_max() will be
 * greater than the value returned by rt_priority_min().
 */
static inline int rt_priority_min() { return 0; }
static inline int rt_priority_max() { return 15; }
static inline int rt_priority_default() { return 1; }

struct GR_RUNTIME_API rt_sched_param {
    int priority;
    rt_sched_policy policy;

    rt_sched_param() : priority(rt_priority_default()), policy(RT_SCHED_RR) {}

    rt_sched_param(int priority_, rt_sched_policy policy_ = RT_SCHED_RR)
    {
        if (priority_ < rt_priority_min() || priority_ > rt_priority_max())
            throw std::invalid_argument("rt_sched_param: priority out of range");

        priority = priority_;
        policy = policy_;
    }
};

/*!
 * \brief If possible, enable high-priority "real time" scheduling.
 * \ingroup misc
 */
GR_RUNTIME_API rt_status_t enable_realtime_scheduling(rt_sched_param p = rt_sched_param());

} /* namespace gr */

#endif /* INCLUDED_GR_REALTIME_H */
