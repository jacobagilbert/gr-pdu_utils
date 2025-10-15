#pragma once

#include <fmt/core.h>
#include <pmt/pmt.h>

// Formatter for pmt::pmt_t
template <> 
struct fmt::formatter<pmt::pmt_t> : fmt::formatter<std::string_view> {
    auto format(const pmt::pmt_t& p, format_context& ctx) const {
        return fmt::formatter<std::string_view>::format(pmt::write_string(p), ctx);
    }
};

// Add formatter for window_type enum
template <>
struct fmt::formatter<gr::pdu_utils::window_type> : fmt::formatter<int> {
    auto format(const gr::pdu_utils::window_type& type, format_context& ctx) const {
        // Convert the enum to integer for formatting
        return fmt::formatter<int>::format(static_cast<int>(type), ctx);
    }
};