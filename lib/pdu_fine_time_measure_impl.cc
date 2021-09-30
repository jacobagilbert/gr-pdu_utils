/* -*- c++ -*- */
/*
 * Copyright 2018-2021 National Technology & Engineering Solutions of Sandia, LLC
 * (NTESS). Under the terms of Contract DE-NA0003525 with NTESS, the U.S. Government
 * retains certain rights in this software.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "pdu_fine_time_measure_impl.h"
#include <gnuradio/io_signature.h>
#include <pdu_utils/constants.h>
#include <volk/volk.h>
#include <numeric>

namespace gr {
namespace pdu_utils {

pdu_fine_time_measure::sptr pdu_fine_time_measure::make(float pre_burst_time,
                                                        float post_burst_time,
                                                        size_t average_width,
                                                        float buffer_percent)
{
    return gnuradio::make_block_sptr<pdu_fine_time_measure_impl>(
        pre_burst_time, post_burst_time, average_width, buffer_percent);
}

/*
 * The private constructor
 */
pdu_fine_time_measure_impl::pdu_fine_time_measure_impl(float pre_burst_time,
                                                       float post_burst_time,
                                                       size_t average_width,
                                                       float buffer_percent)
    : gr::block("pdu_fine_time_measure",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_pre_burst_time(pre_burst_time),
      d_post_burst_time(post_burst_time),
      d_average_size(average_width),
      d_buffer_percent(buffer_percent / 100.0),
      d_pub_debug(false)
{
    // force odd size so group delay will be an integer number of samples
    if (d_average_size % 2 == 0) {
        d_average_size++;
    }

    // bursts shorter than this will be passed as-is
    d_min_burst_length = (size_t )(1.5 * d_average_size);

    message_port_register_in(PMTCONSTSTR__pdu_in());
    message_port_register_out(PMTCONSTSTR__pdu_out());
    message_port_register_out(PMTCONSTSTR__debug());
    set_msg_handler(PMTCONSTSTR__pdu_in(),
                    [this](pmt::pmt_t msg) { this->pdu_handler(msg); });
}

/*
 * Our virtual destructor.
 */
pdu_fine_time_measure_impl::~pdu_fine_time_measure_impl() {}

bool pdu_fine_time_measure_impl::start()
{
    // check if there are any blocks subscribed to the `debug` port - if not, disable output
    if(pmt::is_pair(pmt::dict_ref(d_message_subscribers, PMTCONSTSTR__debug(), pmt::PMT_NIL))) {
        GR_LOG_INFO(d_logger, "Debug Message Port is connected - Enabling Output");
        d_pub_debug = true;
    } else {
        GR_LOG_INFO(d_logger, "Debug Message Port is not connected - Disabling Output");
        d_pub_debug = false;
    }
    return true;
}

void pdu_fine_time_measure_impl::pdu_handler(pmt::pmt_t pdu)
{
    /*      1. Make sure the PMT is a reasonable PDU
     *
     * it should be dropped if it is not a F32/C32 PDU with the optional
     * metadata fields: `sample_rate`, TODO: others??
     */ 
    if (!(pmt::is_pdu(pdu))) {
        GR_LOG_ERROR(this->d_logger, "PMT is not a PDU, dropping");
        return;
    }

    pmt::pmt_t metadata = pmt::car(pdu);
    pmt::pmt_t data = pmt::cdr(pdu);

    if (!pmt::is_c32vector(data)) {
        GR_LOG_WARN(d_logger, "PDU data not complex, dropping");
        return;
    }

    // short bursts arent a problem, but no further processing is required
    if (pmt::length(data) < d_min_burst_length) {
        message_port_pub(PMTCONSTSTR__pdu_out(), pdu);
        return;
    }

    // Divide the burst into pre-noise/burst/post_noise
    // This is, in a way, an SNR calculation
    float noiseSum = 0, burstSum = 0;
    size_t burst_size;
    
    // read in the PDU data and compute the magnitude squared
    const gr_complex* burst =
            (const gr_complex*)pmt::c32vector_elements(data, burst_size);
    d_magnitude_squared_f.resize(burst_size);
    volk_32fc_magnitude_squared_32f(&d_magnitude_squared_f[0], burst, burst_size);

    // average the magnitude squared data
    d_mag_avg.resize(burst_size - d_average_size + 1);
    for (auto ii=0; ii < (burst_size - d_average_size + 1); ii++) {
        float moving_avg = 0;
        for (auto jj=0; jj < d_average_size; jj++) {
            moving_avg += d_magnitude_squared_f[ii+jj];
        }
        d_mag_avg[ii] = moving_avg / d_average_size;
    }
    
    // determine the threshold; for now this is hard coded to be 4dB down from
    // the average of the middle 40% of the burst
    double peak = std::accumulate(d_mag_avg.begin() + (int )burst_size*0.3,
                                 d_mag_avg.begin() + (int )burst_size*0.7,
                                 0.0);
    double threshold = peak / (burst_size - d_average_size + 1) / exp(4.0 / 10);

    // find the first and last threshold crossings
    auto start_idx = std::distance(d_mag_avg.begin(), std::lower_bound(d_mag_avg.begin(), d_mag_avg.end(), threshold));
    auto end_idx = std::distance(d_mag_avg.begin(), std::lower_bound(d_mag_avg.rbegin(), d_mag_avg.rend(), threshold).base());

    // publish debug message
    if (d_pub_debug) {
        std::vector<gr_complex> dbg(burst_size);
        size_t end = burst_size;// - start;
        auto mv = *std::max_element(d_mag_avg.begin(), d_mag_avg.end());
        for (auto jj = 0; jj < (burst_size - d_average_size + 1); jj++) {
            if (jj >= start_idx && jj <= end_idx) {
                dbg[jj] = d_mag_avg[jj] + 1j * mv*1.5;
            } else {
                dbg[jj] = d_mag_avg[jj] + 1j * 0;
            }
        }
        message_port_pub(PMTCONSTSTR__debug(),
                         pmt::cons(pmt::PMT_NIL, pmt::init_c32vector(burst_size, &dbg[0])));
    }
    
    // offset our burst time and remove samples from the beginning
    pmt::pmt_t rate_key = pmt::dict_ref(metadata, PMTCONSTSTR__sample_rate(), pmt::PMT_NIL);
    if (!pmt::eqv(rate_key, pmt::PMT_NIL)) {
        double sample_rate = pmt::to_float(rate_key);
        pmt::pmt_t time_key = pmt::dict_ref(metadata, PMTCONSTSTR__start_time(), pmt::PMT_NIL);
        if (!pmt::eqv(time_key, pmt::PMT_NIL)) {
            double start_time = pmt::to_double(time_key);
            metadata = pmt::dict_add(metadata, PMTCONSTSTR__start_time(),
                                 pmt::from_double(start_time + (double(start_idx) / sample_rate)));
        }
        pmt::pmt_t dur_key = pmt::dict_ref(metadata, PMTCONSTSTR__duration(), pmt::PMT_NIL);
        if (!pmt::eqv(time_key, pmt::PMT_NIL)) {
            double start_time = pmt::to_double(dur_key);
            metadata = pmt::dict_add(metadata, PMTCONSTSTR__duration(),
                                 pmt::from_double(float(end_idx - start_idx) / sample_rate));
        }
    }

    if (pmt::is_c32vector(data)) {
        pmt::pmt_t pdu_vector = pmt::init_c32vector((end_idx - start_idx), burst + start_idx);
        message_port_pub(PMTCONSTSTR__pdu_out(), pmt::cons(metadata, pdu_vector));  
    }
}


} /* namespace pdu_utils */
} /* namespace gr */
