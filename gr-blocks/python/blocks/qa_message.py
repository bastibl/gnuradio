#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2004,2010,2013,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import time

from gnuradio import gr, gr_unittest, blocks
import pmt


def all_counts():
    return (gr.block_ncurrently_allocated(),
            gr.block_detail_ncurrently_allocated(),
            gr.buffer_ncurrently_allocated(),
            gr.buffer_reader_ncurrently_allocated())


class test_message(gr_unittest.TestCase):

    def setUp(self):
        self.msgq = gr.make_msg_queue()

    def tearDown(self):
        self.msgq = None

    def body_201(self):
        self.msgq.insert_tail(pmt.PMT_NIL)
        self.assertEquals(1, self.msgq.count())
        self.msgq.insert_tail(pmt.PMT_NIL)
        self.assertEquals(2, self.msgq.count())

    def test_300(self):
        input_data = (0,1,2,3,4,5,6,7,8,9)
        src = blocks.vector_source_b(input_data)
        dst = blocks.vector_sink_b()
        tb = gr.top_block()
        tb.connect(src, dst)
        tb.run()
        self.assertEquals(input_data, dst.data())

    def test_debug_401(self):
        msg = pmt.intern("TESTING")
        src = blocks.message_strobe(msg, 500)
        snk = blocks.message_debug()

        tb = gr.top_block()
        tb.msg_connect(src, "strobe", snk, "store")
        tb.start()
        time.sleep(1)
        tb.stop()
        tb.wait()

        rec_msg = snk.get_message(0)
        self.assertTrue(pmt.eqv(rec_msg, msg))


if __name__ == '__main__':
    gr_unittest.run(test_message, "test_message.xml")
