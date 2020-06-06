/* -*- c++ -*- */
/*
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_PDU_UTILS_TIME_DELTA_IMPL_H
#define INCLUDED_PDU_UTILS_TIME_DELTA_IMPL_H

#include <pdu_utils/time_delta.h>
#include <pdu_utils/constants.h>

namespace gr {
  namespace pdu_utils {

    class time_delta_impl : public time_delta
    {
     private:
       boost::posix_time::ptime d_epoch;
       std::string d_name;

       // statistic tracking
       double d_sum_x, d_sum_x2;
       int    d_n;

     public:
      time_delta_impl(const std::string &name);
      ~time_delta_impl();

      // overloaded functions
      bool stop();

      void handle_pdu(pmt::pmt_t pdu);
    };

  } // namespace pdu_utils
} // namespace gr

#endif /* INCLUDED_PDU_UTILS_TIME_DELTA_IMPL_H */
