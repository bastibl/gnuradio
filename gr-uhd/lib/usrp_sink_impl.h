/* -*- c++ -*- */
/*
 * Copyright 2010-2016 Free Software Foundation, Inc.
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

#include "usrp_block_impl.h"
#include <gnuradio/uhd/usrp_sink.h>
#include <uhd/convert.hpp>

static const std::string SOB_KEY = "tx_sob";
static const std::string EOB_KEY = "tx_eob";
static const std::string TIME_KEY = "tx_time";
static const std::string FREQ_KEY = "tx_freq";
static const std::string COMMAND_KEY = "tx_command";

// Asynchronous message handling related PMTs
static const std::string ASYNC_MSG_KEY = "uhd_async_msg";
static const std::string CHANNEL_KEY = "channel";
static const std::string TIME_SPEC_KEY = "time_spec";
static const std::string EVENT_CODE_KEY = "event_code";
static const std::string BURST_ACK_KEY = "burst_ack";
static const std::string UNDERFLOW_KEY = "underflow";
static const std::string UNDERFLOW_IN_PACKET_KEY = "underflow_in_packet";
static const std::string SEQ_ERROR_KEY = "seq_error";
static const std::string SEQ_ERROR_IN_BURST_KEY = "seq_error_in_burst";
static const std::string TIME_ERROR_KEY = "time_error";
static const std::string ASYNC_MSGS_PORT_KEY = "async_msgs";


namespace gr {
namespace uhd {

inline io_signature::sptr args_to_io_sig(const ::uhd::stream_args_t& args)
{
    const size_t nchan = std::max<size_t>(args.channels.size(), 1);
    const size_t size = ::uhd::convert::get_bytes_per_item(args.cpu_format);
    return io_signature::make(nchan, nchan, size);
}

/***********************************************************************
 * UHD Multi USRP Sink Impl
 **********************************************************************/
class usrp_sink_impl : public usrp_sink, public usrp_block_impl
{
public:
    usrp_sink_impl(const ::uhd::device_addr_t& device_addr,
                   const ::uhd::stream_args_t& stream_args,
                   const std::string& length_tag_name);
    ~usrp_sink_impl();

    ::uhd::dict<std::string, std::string> get_usrp_info(size_t chan);
    double get_samp_rate(void);
    ::uhd::meta_range_t get_samp_rates(void);
    double get_center_freq(size_t chan);
    ::uhd::freq_range_t get_freq_range(size_t chan);
    double get_gain(size_t chan);
    double get_gain(const std::string& name, size_t chan);
    double get_normalized_gain(size_t chan);
    std::vector<std::string> get_gain_names(size_t chan);
    ::uhd::gain_range_t get_gain_range(size_t chan);
    ::uhd::gain_range_t get_gain_range(const std::string& name, size_t chan);
    std::string get_antenna(size_t chan);
    std::vector<std::string> get_antennas(size_t chan);
    ::uhd::sensor_value_t get_sensor(const std::string& name, size_t chan);
    std::vector<std::string> get_sensor_names(size_t chan);
    ::uhd::usrp::dboard_iface::sptr get_dboard_iface(size_t chan);
    std::vector<std::string> get_lo_names(size_t chan);
    const std::string get_lo_source(const std::string& name, size_t chan);
    std::vector<std::string> get_lo_sources(const std::string& name, size_t chan);
    bool get_lo_export_enabled(const std::string& name, size_t chan);
    double get_lo_freq(const std::string& name, size_t chan);
    ::uhd::freq_range_t get_lo_freq_range(const std::string& name, size_t chan);

    void set_subdev_spec(const std::string& spec, size_t mboard);
    std::string get_subdev_spec(size_t mboard);
    void set_samp_rate(double rate);
    ::uhd::tune_result_t set_center_freq(const ::uhd::tune_request_t tune_request,
                                         size_t chan);
    void set_gain(double gain, size_t chan);
    void set_gain(double gain, const std::string& name, size_t chan);
    void set_normalized_gain(double gain, size_t chan);
    void set_antenna(const std::string& ant, size_t chan);
    void set_bandwidth(double bandwidth, size_t chan);
    double get_bandwidth(size_t chan);
    ::uhd::freq_range_t get_bandwidth_range(size_t chan);
    void set_dc_offset(const std::complex<double>& offset, size_t chan);
    void set_iq_balance(const std::complex<double>& correction, size_t chan);
    void set_stream_args(const ::uhd::stream_args_t& stream_args);
    void set_start_time(const ::uhd::time_spec_t& time);
    void set_lo_source(const std::string& src,
                       const std::string& name = ALL_LOS,
                       size_t chan = 0);
    void set_lo_export_enabled(bool enabled,
                               const std::string& name = ALL_LOS,
                               size_t chan = 0);
    double set_lo_freq(double freq, const std::string& name, size_t chan);

    bool start(void);
    bool stop(void);

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

    inline void tag_work(int& ninput_items);

    void setup_rpc();

private:
    //! Like set_center_freq(), but uses _curr_freq and _curr_lo_offset
    ::uhd::tune_result_t _set_center_freq_from_internals(size_t chan,
                                                         pmt::pmt_t direction);

    ::uhd::tx_streamer::sptr _tx_stream;
    ::uhd::tx_metadata_t _metadata;
    double _sample_rate;

    // stream tags related stuff
    std::vector<tag_t> _tags;
    const std::string _length_tag_key;
    long _nitems_to_send;

    // asynchronous messages related stuff
    bool _async_event_loop_running;
    void async_event_loop();
    gr::thread::thread _async_event_thread;
};

} /* namespace uhd */
} /* namespace gr */
