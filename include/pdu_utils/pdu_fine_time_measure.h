/* -*- c++ -*- */
/*
 * Copyright 2018-2021 National Technology & Engineering Solutions of Sandia, LLC
 * (NTESS). Under the terms of Contract DE-NA0003525 with NTESS, the U.S. Government
 * retains certain rights in this software.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PDU_UTILS_PDU_FINE_TIME_MEASURE_H
#define INCLUDED_PDU_UTILS_PDU_FINE_TIME_MEASURE_H

#include <gnuradio/sync_block.h>
#include <pdu_utils/api.h>

namespace gr {
namespace pdu_utils {

/*!
 * \brief Perform time domain fine burst start time estimate
 * \ingroup pdu_utils
 *
 * Perform time domain fine burst start time estimation. This general operation
 * of this block is as follows:
 *
 * 1. Compute an AM moving-average of the specified width.
 *
 * 2. Estimate the nominal magnitude of the peak power of the burst, sampled
 *    from the specified percentage of the center of the burst; establish a
 *    threshold as a specified percentage of this magnitude.
 *
 * 3. Identify the first and last place the burst crosses this threshold and 
 *    consider this, plus some margin, to be the boundary.
 
 ....TODO....
 
 *It considers
 * the pre/post burst times to be noise and the rest of the burst to
 * be signal.  The new start time is when a moving average goes above
 * .5*(Signal + Noise).
 *
 * Pre/post burst time - Extra time pre/appended to a burst, that is
 * presumed to be noise. Average Width - The number of samples used
 * in a moving average to decide the start of burst.
 *
 * Buffer Percent - We don't want to include transitions in our
 * Signal/Noise calculations.  Remove this percent of points from
 * consideration. (0 - 100%)
 *
 */
class PDU_UTILS_API pdu_fine_time_measure : virtual public gr::block
{
public:
    typedef std::shared_ptr<pdu_fine_time_measure> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of pdu_utils::pdu_fine_time_measure.
     *
     * @param pre_burst_time - Unit: sec
     * @param post_burst_time - Unit: sec
     * @param average_width -
     * @param buffer_percent - Range[0..100]
     */
    static sptr make(float pre_burst_time,
                     float post_burst_time,
                     size_t average_width,
                     float buffer_percent);
};

} // namespace pdu_utils
} // namespace gr

#endif /* INCLUDED_PDU_UTILS_PDU_FINE_TIME_MEASURE_H */
