/* -*- c++ -*- */
/*
 * Copyright 2018-2021 National Technology & Engineering Solutions of Sandia, LLC
 * (NTESS). Under the terms of Contract DE-NA0003525 with NTESS, the U.S. Government
 * retains certain rights in this software.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PDU_UTILS_PDU_FINE_TIME_MEASURE_IMPL_H
#define INCLUDED_PDU_UTILS_PDU_FINE_TIME_MEASURE_IMPL_H

#include <pdu_utils/constants.h>
#include <pdu_utils/pdu_fine_time_measure.h>

namespace gr {
namespace pdu_utils {

class pdu_fine_time_measure_impl : public pdu_fine_time_measure
{
private:
    float d_pre_burst_time;
    float d_post_burst_time;
    size_t d_average_size;
    float d_threshold_db;

    bool d_pub_debug;
    size_t d_min_burst_length;
    std::vector<float> d_magnitude_squared_f;
    std::vector<float> d_mag_avg;

    void pdu_handler(pmt::pmt_t pdu);

public:
    pdu_fine_time_measure_impl(float pre_burst_time,
                               float post_burst_time,
                               size_t average_width,
                               float threshold_db);
    /**
     * Deconstructor
     */
    ~pdu_fine_time_measure_impl();
    bool start();
};

} // namespace pdu_utils
} // namespace gr

#endif /* INCLUDED_PDU_UTILS_PDU_FINE_TIME_MEASURE_IMPL_H */
