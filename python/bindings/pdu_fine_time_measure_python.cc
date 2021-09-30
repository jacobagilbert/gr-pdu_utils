/*
 * Copyright 2021 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(pdu_fine_time_measure.h)                                   */
/* BINDTOOL_HEADER_FILE_HASH(1af768b1bf6f8296aa598c81fde0b4ab)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <pdu_utils/pdu_fine_time_measure.h>
// pydoc.h is automatically generated in the build directory
#include <pdu_fine_time_measure_pydoc.h>

void bind_pdu_fine_time_measure(py::module& m)
{

    using pdu_fine_time_measure = ::gr::pdu_utils::pdu_fine_time_measure;


    py::class_<pdu_fine_time_measure,
               gr::block,
               gr::basic_block,
               std::shared_ptr<pdu_fine_time_measure>>(
        m, "pdu_fine_time_measure", D(pdu_fine_time_measure))

        .def(py::init(&pdu_fine_time_measure::make),
             py::arg("pre_burst_time"),
             py::arg("post_burst_time"),
             py::arg("average_width"),
             py::arg("buffer_percent"),
             D(pdu_fine_time_measure, make))


        ;
}
