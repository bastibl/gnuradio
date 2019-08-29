/* -*- c++ -*- */
/*
 * Copyright 2005,2009-2011,2013 Free Software Foundation, Inc.
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

namespace gr {
namespace messages {

/*!
 * \brief thread-safe message queue
 */
class GR_RUNTIME_API msg_queue
{
public:
    msg_queue(unsigned int limit);
    ~msg_queue();

    /*!
     * \brief Delete message from head of queue and return it.
     * Block if no message is available.
     */
    pmt::pmt_t delete_head();

    //! Delete all messages from the queue
    void flush();

    //! is the queue empty?
    bool empty_p() const;

    //! is the queue full?
    bool full_p() const;

    //! return number of messages in queue
    unsigned int count() const;

    //! return limit on number of message in queue.  0 -> unbounded
    unsigned int limit() const;
};

typedef boost::shared_ptr<msg_queue> msg_queue_sptr;
GR_RUNTIME_API msg_queue_sptr make_msg_queue(unsigned int limit = 0);

}
}

/*
 * The following kludge-o-rama releases the Python global interpreter
 * lock around these potentially blocking calls.  We don't want
 * libgnuradio-runtime to be dependent on Python, thus we create these
 * functions that serve as replacements for the normal C++ delete_head
 * and insert_tail methods.  The %pythoncode smashes these new C++
 * functions into the gr.msg_queue wrapper class, so that everything
 * appears normal.  (An evil laugh is heard in the distance...)
 */
#ifdef SWIGPYTHON
%inline %{
    pmt::pmt_t py_msg_queue__delete_head(gr::messages::msg_queue_sptr q) {
    pmt::pmt_t msg;
    GR_PYTHON_BLOCKING_CODE(
        msg = q->delete_head();
    )
    return msg;
  }

    void py_msg_queue__insert_tail(gr::messages::msg_queue_sptr q, pmt::pmt_t msg) {
    GR_PYTHON_BLOCKING_CODE(
        q->insert_tail(msg);
    )
  }
%}

// smash in new python delete_head and insert_tail methods...
%template(msg_queue_sptr) boost::shared_ptr<gr::messages::msg_queue>;
%pythoncode %{
msg_queue_sptr.delete_head = py_msg_queue__delete_head
msg_queue_sptr.insert_tail = py_msg_queue__insert_tail
msg_queue_sptr.handle = py_msg_queue__insert_tail
msg_queue = make_msg_queue
%}
#endif	// SWIGPYTHON
