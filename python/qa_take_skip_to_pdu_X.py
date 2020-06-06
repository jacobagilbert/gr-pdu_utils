#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# <COPYRIGHT PLACEHOLDER>
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import pdu_utils_swig as pdu_utils
from gnuradio import gr, gr_unittest
from gnuradio import blocks
import pmt
import time
import numpy

class qa_take_skip_to_pdu_X (gr_unittest.TestCase):

    # this method is necessary because by default pmt.equal does not evaluate
    # the *contents* of a uniform vector
    def assertEqualPDU(self, pdu1, pdu2):
        # first check the equal() function:
        if not pmt.equal(pdu1, pdu2):
            self.assertTrue(False)
        # then check the dictionaries:
        if not pmt.equal(pmt.car(pdu1), pmt.car(pdu2)):
            self.assertTrue(False)
        # then check the elements of the respective vectors
        vec1 = pmt.cdr(pdu1)
        vec2 = pmt.cdr(pdu2)
        if not pmt.equal(vec1, vec2):
            self.assertTrue(False)
        if not (pmt.to_python(vec1) == pmt.to_python(vec2)).all():
            print "vectors not equal? " + repr(vec1) + repr(vec2)
            self.assertTrue(False)

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_f_32 (self):
        self.source = blocks.vector_source_f(range(0,32*3), False, 1, [])
        self.ts_pdu = pdu_utils.take_skip_to_pdu_f(32, 32)
        self.debug = blocks.message_debug()
        self.tb.connect((self.source, 0), (self.ts_pdu, 0))
        self.tb.msg_connect((self.ts_pdu, 'pdu_out'), (self.debug, 'store'))

        dic = pmt.dict_add(pmt.make_dict(), pmt.intern("pdu_num"), pmt.from_uint64(0))
        vec = pmt.init_f32vector(32, range(0,32))
        expected = pmt.cons(dic,vec)
        self.tb.run ()
        actual = self.debug.get_message(0)
        self.assertEqualPDU(actual, expected)

    def test_002_c_80 (self):
        self.source = blocks.vector_source_c(range(0,32*3), False, 1, [])
        self.ts_pdu = pdu_utils.take_skip_to_pdu_c(80, 32)
        self.debug = blocks.message_debug()
        self.tb.connect((self.source, 0), (self.ts_pdu, 0))
        self.tb.msg_connect((self.ts_pdu, 'pdu_out'), (self.debug, 'store'))

        dic = pmt.dict_add(pmt.make_dict(), pmt.intern("pdu_num"), pmt.from_uint64(0))
        vec = pmt.init_c32vector(80, range(0,80))
        expected = pmt.cons(dic,vec)
        self.tb.run ()
        actual = self.debug.get_message(0)
        self.assertEqualPDU(actual, expected)


    def test_003_s_2_11_7 (self):
        self.source = blocks.vector_source_s(range(0,32*3), False, 1, [])
        self.ts_pdu = pdu_utils.take_skip_to_pdu_s(2, 11)
        self.debug = blocks.message_debug()
        self.tb.connect((self.source, 0), (self.ts_pdu, 0))
        self.tb.msg_connect((self.ts_pdu, 'pdu_out'), (self.debug, 'store'))

        dic = pmt.dict_add(pmt.make_dict(), pmt.intern("pdu_num"), pmt.from_uint64(7))
        vec = pmt.init_s16vector(2, range(91,93))
        expected = pmt.cons(dic,vec)
        self.tb.run ()
        actual = self.debug.get_message(7)
        self.assertEqualPDU(actual, expected)


    def test_004_b_512 (self):
        self.source = blocks.vector_source_b(range(0,256)*4, False, 1, [])
        self.ts_pdu = pdu_utils.take_skip_to_pdu_b(512,1)
        self.debug = blocks.message_debug()
        self.tb.connect((self.source, 0), (self.ts_pdu, 0))
        self.tb.msg_connect((self.ts_pdu, 'pdu_out'), (self.debug, 'store'))

        dic = pmt.dict_add(pmt.make_dict(), pmt.intern("pdu_num"), pmt.from_uint64(0))
        vec = pmt.init_u8vector(512, range(0,256)*2)
        expected = pmt.cons(dic,vec)
        self.tb.run ()
        actual = self.debug.get_message(0)
        self.assertEqualPDU(actual, expected)


if __name__ == '__main__':
    gr_unittest.run(qa_take_skip_to_pdu_X, "qa_take_skip_to_pdu_X.xml")
