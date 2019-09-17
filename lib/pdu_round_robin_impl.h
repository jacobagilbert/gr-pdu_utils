/* -*- c++ -*- */
/* 
 * Copyright 2018 gr-pdu_utils author.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_PDU_UTILS_PDU_ROUND_ROBIN_IMPL_H
#define INCLUDED_PDU_UTILS_PDU_ROUND_ROBIN_IMPL_H

#include <pdu_utils/pdu_round_robin.h>

namespace gr {
  namespace pdu_utils {

    class pdu_round_robin_impl : public pdu_round_robin
    {
     private:
      int d_num_outputs;
      int d_counter;

      std::vector<std::string> d_port_names;

      void pdu_handler(pmt::pmt_t pdu);

     public:
      pdu_round_robin_impl(int num_outputs);
      ~pdu_round_robin_impl();
    };

  } // namespace pdu_utils
} // namespace gr

#endif /* INCLUDED_PDU_UTILS_PDU_ROUND_ROBIN_IMPL_H */

