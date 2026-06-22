//
// Copyright 2010-2012,2014-2015 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

//#include "wavetable.hpp"
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <filesystem>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <cmath>
#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>
//#include "matplotlibcpp.h" 
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cstdlib>





//namespace plt = matplotlibcpp;
namespace po = boost::program_options;
namespace fs = std::filesystem;


/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

/***********************************************************************
 * Utilities
 **********************************************************************/
//! Change to filename, e.g. from usrp_samples.dat to usrp_samples.00.dat,
//  but only if multiple names are to be generated.
std::string generate_out_filename(
    const std::string& base_fn, size_t n_names, size_t this_name)
{
    if (n_names == 1) {
        return base_fn;
    }

    boost::filesystem::path base_fn_fp(base_fn);
    base_fn_fp.replace_extension(boost::filesystem::path(
        str(boost::format("%02d%s") % this_name % base_fn_fp.extension().string())));
    return base_fn_fp.string();
}


/***********************************************************************
 * transmit_worker function
 * A function to be used in a thread for transmitting
 **********************************************************************/
void transmit_worker(std::vector<std::complex<float>> buff,
    std::vector<std::complex<float>> sig_tx,
    uhd::tx_streamer::sptr tx_streamer,
    uhd::tx_metadata_t metadata,
    int num_channels)
{
    std::vector<std::complex<float>*> buffs(num_channels, &buff.front());
    
      
    // send data until the signal handler gets called
    //while (not stop_signal_called) {
   
   for (int i=0; i<100; i++){
        // fill the buffer with the waveform
        for (size_t n = 0; n < sig_tx.size(); n++) {
            buff[n] = sig_tx[n];
            
        }
        
        // send the entire contents of the buffer
        tx_streamer->send(buffs, buff.size(), metadata);

        metadata.start_of_burst = false;
        metadata.has_time_spec  = false;
    }

    // send a mini EOB packet
    metadata.end_of_burst = true;
    tx_streamer->send("", 0, metadata);

    
}



/***********************************************************************
 * recv_to_file function
 **********************************************************************/
template <typename samp_type>
float recv_to_file(uhd::usrp::multi_usrp::sptr usrp,
    const std::string& cpu_format,
    const std::string& wire_format,
    const std::string& file,
    size_t samps_per_buff,
    int num_requested_samples,
    double settling_time,
    std::vector<size_t> rx_channel_nums,
    std::vector<std::complex<short>>& all_buffs)
{
    //reset data


    int num_total_samps = 0;
    // create a receive streamer
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    stream_args.channels             = rx_channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // Prepare buffers for received samples and metadata
    uhd::rx_metadata_t md;
    std::vector<std::vector<samp_type>> buffs(
        rx_channel_nums.size(), std::vector<samp_type>(samps_per_buff));
    // create a vector of pointers to point to each of the channel buffers
    std::vector<samp_type*> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++) {
        buff_ptrs.push_back(&buffs[i].front());
    }

    // Create one ofstream object per channel
    // (use shared_ptr because ofstream is non-copyable)
    std::vector<std::shared_ptr<std::ofstream>> outfiles;
    for (size_t i = 0; i < buffs.size(); i++) {
        const std::string this_filename = generate_out_filename(file, buffs.size(), i);
        outfiles.push_back(std::shared_ptr<std::ofstream>(
            new std::ofstream(this_filename.c_str(), std::ofstream::binary)));
    }
    UHD_ASSERT_THROW(outfiles.size() == buffs.size());
    UHD_ASSERT_THROW(buffs.size() == rx_channel_nums.size());
    bool overflow_message = true;
    // We increase the first timeout to cover for the delay between now + the
    // command time, plus 500ms of buffer. In the loop, we will then reduce the
    // timeout for subsequent receives.
    double timeout = settling_time + 0.5f;

    // setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = num_requested_samples;
    stream_cmd.stream_now = false;


             std::cout <<"\nsetteling "<<settling_time<<"\n";
    stream_cmd.time_spec  = usrp->get_time_now() + uhd::time_spec_t(settling_time);
    rx_stream->issue_stream_cmd(stream_cmd);

//===================================================================
         
         long double full_secs = stream_cmd.time_spec.get_full_secs();
         long double frac_secs = stream_cmd.time_spec.get_frac_secs();
         
         std::cout << full_secs << " s + " << frac_secs << " fractional seconds (dt + 0.2) \n";
         long double dt = full_secs + frac_secs - 0.2;
         std::cout << dt;
         
         
size_t write_idx = 0;

//===================================================================


    while (not stop_signal_called
           and (num_requested_samples > num_total_samps or num_requested_samples == 0)) {

        size_t num_rx_samps = rx_stream->recv(buff_ptrs, samps_per_buff, md, timeout);
        timeout             = 0.1f; // small timeout for subsequent recv

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << "Timeout while streaming" << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            if (overflow_message) {
                overflow_message = false;
                std::cerr
                    << boost::format(
                           "Got an overflow indication. Please consider the following:\n"
                           "  Your write medium must sustain a rate of %fMB/s.\n"
                           "  Dropped samples will not be written to the file.\n"
                           "  Please modify this example for your purposes.\n"
                           "  This message will not appear again.\n")
                           % (usrp->get_rx_rate() * sizeof(samp_type) / 1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            throw std::runtime_error("Receiver error " + md.strerror());
        }

        num_total_samps += num_rx_samps;
        


       const size_t n = num_rx_samps;

	std::memcpy(
	    all_buffs.data() + write_idx,
	    buffs[0].data(),
	    n * sizeof(std::complex<short>)
	);

	write_idx += n;

//all_buffs.insert(all_buffs.end(), buffs[0].begin(), buffs[0].end());
       

/*
        for (size_t i = 0; i < outfiles.size(); i++) {
            outfiles[i]->write(
                 reinterpret_cast<const char*> (buff_ptrs[i]), num_rx_samps * sizeof(samp_type));
        }
        */
    }
    

    // Shut down receiver
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    // Close files
  /*  for (size_t i = 0; i < outfiles.size(); i++) {
        outfiles[i]->close();
    }
   
   */ 

    return dt;
}

/**********************************************************************
Timestamp for folder name
**********************************************************************/

std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    std::tm *ptm = std::localtime(&now_c);

    std::ostringstream oss;
    oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S"); // format: 2026-03-27_14-30-05
    return oss.str();
}
/***********************************************************************
 * Post-Processing
 
 **********************************************************************/
void post_processing(std::vector<std::complex<float>> signal_tx, 
double rx_rate,
long double dt,
std::vector<std::complex<short>>& all_buffs,
std::string folder)    

{ 

 /*
  // read rx  
    std::ifstream file("usrp_samples.dat", std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: usrp_samples.dat");
    }
    std::vector<int16_t> rawData;
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    rawData.resize(fileSize / sizeof(int16_t));
    file.read(reinterpret_cast<char*>(rawData.data()), fileSize);

    // Convert interleaved I/Q to complex<double>
    std::vector<std::complex<double>> rx_complex;
    rx_complex.reserve(rawData.size() / 2);

    for (size_t i = 0; i + 1 < rawData.size(); i += 2) {
        double I = static_cast<double>(rawData[i]);
        double Q = static_cast<double>(rawData[i + 1]);
        rx_complex.emplace_back(I, Q);
    }
    
  */  
    // read tx
   //already in signal_tx variable
   //int N = 1; // rx_rate / tx_rate
   // match rate
  
    std::vector<std::complex<float>> samples_tx = signal_tx;
    //samples_tx.reserve(signal_tx.size() * N);
    /*
    for (auto chip : signal_tx) {
        std::complex<float> A = chip; // real ±1
        for (int i = 0; i < N; ++i) {
            samples_tx.push_back(A);
        }
    }
*/

   
   //truncate delay
   int Nrem = static_cast<int>((0.3 - dt) * rx_rate);
    std::vector<std::complex<short>> rx_trunc;
    if (Nrem < all_buffs.size()) {
        rx_trunc.assign(all_buffs.begin() + Nrem -1, all_buffs.end());
    }
    
  
      std::cout<<"\n0.3-dt = "<<0.3-dt<<"\n";
      std::cout<<"Nrem "<<Nrem<<"\n";
      std::cout<<"rx complex size "<<all_buffs.size()<<"\n";
      std::cout<<"trunc size "<<rx_trunc.size()<<"\n";
      
      
      std::cout<<"context:"<<rx_trunc[1]<<"  "<<rx_trunc[2]<<"  "<<rx_trunc[3]<<"  "<<rx_trunc[4]<<"  "<<rx_trunc[5]<<"\n";
      //save truncate
      





    int max_id = 0;

    for (auto& p : fs::directory_iterator(".")) {
        if (p.path().extension() == ".txt") {
            std::string name = p.path().stem().string();  // e.g., "scan14"
            if (name.rfind("scan", 0) == 0) {             // starts with "scan"
                int id = std::stoi(name.substr(4));
                if (id > max_id) max_id = id;
            }
        }
    }
    int idx = max_id + 1;
    

    std::string filename =  "scan" + std::to_string(idx) + ".txt";
    
    //binary write for faster disk write
    std::ofstream outFile_tr(filename, std::ios::binary);
if (!outFile_tr) {
    std::cerr << "Error opening file\n";
    return;
}

outFile_tr.write(
    reinterpret_cast<const char*>(rx_trunc.data()),
    rx_trunc.size() * sizeof(std::complex<float>)
);

outFile_tr.close();

/*


    // Write variable to file
    std::ofstream outFile_tr(filename);
    if (outFile_tr.is_open()) {
    
     for (size_t i = 0; i <2000000; ++i){ 
       outFile_tr << rx_trunc[i].real() << " " << rx_trunc[i].imag() << "\n"; 
     }

      outFile_tr.close();

    } else {
        std::cerr << "Error opening file for writing!" << std::endl;
    }
 */  
     // plot raw data 
     
/*
    int m = rx_trunc.size();
    
    // pulse shaping
    float beta = 0.25;  // roll-off factor
    int span = 6;
    int sps = 1;    
     // in symbols
    auto rrc = rrc_filter(beta, span, sps);
    
    rx_trunc = convolve(rx_trunc, rrc);
     
    std::vector<float> mag(m), phase(m), rx_dB(m), rx_real(m), rx_image(m), time(m), rx_real_dB(m);
    

    for (size_t i = 0; i < m; ++i) {
        mag[i] = std::abs(rx_trunc[i]);
        phase[i] = std::arg(rx_trunc[i]);
        time[i] = i / rx_rate;
        rx_dB[i] = 20 * std::log10(mag[i]);
        rx_real[i] = rx_trunc[i].real();
        rx_image[i] = rx_trunc[i].imag();
        rx_real_dB[i] = 20 * std::log10(rx_real[i]);
    }
    

    std::vector<float> tx_real(samples_tx.size());
    for (size_t i = 0; i < samples_tx.size(); i++)
          tx_real[i] = 200*std::real(samples_tx[i]);
     
    int n = tx_real.size();     
    std::vector<float> t_t(n);
    for (size_t i = 0; i < n; ++i) {
        t_t[i] = i / rx_rate;
    }
    
   
    */
    
    
}
/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{



    // preparing the host computer
    
    // network configuration
    system("sudo ip link set dev enp1s0f0 mtu 9000");
    system("sudo ip link set dev enp1s0f1 mtu 9000");
    system("sudo sysctl -w net.core.rmem_max=33445532");
    system("sudo sysctl -w net.core.wmem_max=33445532");
    system("sudo sysctl -w net.core.rmem_default=33445532");
    system("sudo sysctl -w net.core.wmem_default=33445532");
    system("sudo sysctl net.core.wmem_max");
    system("sudo sysctl -p");
    system("sudo ethtool -G enp1s0f0 tx 32768 rx 32768");
    system("sudo ethtool -G enp1s0f1 tx 32768 rx 32768");
    
    //CPU Tuning
    
    //system("bash -c 'for ((i=0;i<$(nproc --all);i++)); do sudo cpufreq-set -c $i -r -g performance; done'");
    //system("bash -c 'for ((i=0;i<$(nproc --all);i++)); do sudo cpufreq-set -c $i -r -g powersave; done'");
    

    
    
    
    
    // transmit variables to be set by po
    std::string tx_args, wave_type, tx_ant, tx_subdev, ref, otw, tx_channels;
    double tx_rate, tx_freq, tx_gain, wave_freq, tx_bw;
    float ampl;

    // receive variables to be set by po
    std::string rx_args, file, type, rx_ant, rx_subdev, rx_channels;
    size_t total_num_samps, spb;
    double rx_rate, rx_freq, rx_gain, rx_bw;
    double settling;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("tx-args", po::value<std::string>(&tx_args)->default_value("addr=192.168.30.2, second_addr = 192.168.40.2"), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&rx_args)->default_value("addr=192.168.30.2, second_addr = 192.168.40.2"), "uhd receive device address args")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type in file: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("settling", po::value<double>(&settling)->default_value(double(0.2)), "settling time (seconds) before receiving")
        ("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
        ("tx-rate", po::value<double>(&tx_rate), "rate of transmit outgoing samples")
        ("rx-rate", po::value<double>(&rx_rate), "rate of receive incoming samples")
        ("tx-freq", po::value<double>(&tx_freq), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&rx_freq), "receive RF center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("tx-gain", po::value<double>(&tx_gain), "gain for the transmit RF chain")
        ("rx-gain", po::value<double>(&rx_gain), "gain for the receive RF chain")
        ("tx-ant", po::value<std::string>(&tx_ant), "transmit antenna selection")
        ("rx-ant", po::value<std::string>(&rx_ant), "receive antenna selection")
        ("tx-subdev", po::value<std::string>(&tx_subdev), "transmit subdevice specification")
        ("rx-subdev", po::value<std::string>(&rx_subdev), "receive subdevice specification")
        ("tx-bw", po::value<double>(&tx_bw), "analog transmit filter bandwidth in Hz")
        ("rx-bw", po::value<double>(&rx_bw), "analog receive filter bandwidth in Hz")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("A"), "waveform type (A,B)")
        //("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("ref", po::value<std::string>(&ref), "reference source (internal, external, gpsdo, mimo)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("tx-channels", po::value<std::string>(&tx_channels)->default_value("0"), "which TX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("rx-channels", po::value<std::string>(&rx_channels)->default_value("0"), "which RX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("tx-int-n", "tune USRP TX with integer-N tuning")
        ("rx-int-n", "tune USRP RX with integer-N tuning")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << "UHD TXRX Loopback to File " << desc << std::endl;
        return ~0;
    }

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the transmit usrp device with: %s...") % tx_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr tx_usrp = uhd::usrp::multi_usrp::make(tx_args);
    std::cout << std::endl;
    std::cout << boost::format("Creating the receive usrp device with: %s...") % rx_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr rx_usrp = uhd::usrp::multi_usrp::make(rx_args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("tx-subdev"))
        tx_usrp->set_tx_subdev_spec(tx_subdev);
    if (vm.count("rx-subdev"))
        rx_usrp->set_rx_subdev_spec(rx_subdev);

    // detect which channels to use
    std::vector<std::string> tx_channel_strings;
    std::vector<size_t> tx_channel_nums;
    boost::split(tx_channel_strings, tx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < tx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(tx_channel_strings[ch]);
        if (chan >= tx_usrp->get_tx_num_channels()) {
            throw std::runtime_error("Invalid TX channel(s) specified.");
        } else
            tx_channel_nums.push_back(std::stoi(tx_channel_strings[ch]));
    }
    std::vector<std::string> rx_channel_strings;
    std::vector<size_t> rx_channel_nums;
    boost::split(rx_channel_strings, rx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < rx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(rx_channel_strings[ch]);
        if (chan >= rx_usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid RX channel(s) specified.");
        } else
            rx_channel_nums.push_back(std::stoi(rx_channel_strings[ch]));
    }


    // Lock mboard clocks
    if (vm.count("ref")) {
        tx_usrp->set_clock_source(ref);
        rx_usrp->set_clock_source(ref);
    }

    std::cout << "Using TX Device: " << tx_usrp->get_pp_string() << std::endl;
    std::cout << "Using RX Device: " << rx_usrp->get_pp_string() << std::endl;

    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    tx_usrp->set_tx_rate(tx_rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (tx_usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    rx_usrp->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (rx_usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the transmit center frequency
    if (not vm.count("tx-freq")) {
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }

    for (size_t ch = 0; ch < tx_channel_nums.size(); ch++) {
        size_t channel = tx_channel_nums[ch];
        if (tx_channel_nums.size() > 1) {
            std::cout << "Configuring TX Channel " << channel << std::endl;
        }
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (tx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t tx_tune_request(tx_freq);
        if (vm.count("tx-int-n"))
            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        tx_usrp->set_tx_freq(tx_tune_request, channel);
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                         % (tx_usrp->get_tx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the rf gain
        if (vm.count("tx-gain")) {
            std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain
                      << std::endl;
            tx_usrp->set_tx_gain(tx_gain, channel);
            std::cout << boost::format("Actual TX Gain: %f dB...")
                             % tx_usrp->get_tx_gain(channel)
                      << std::endl
                      << std::endl;
        }

        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % tx_bw
                      << std::endl;
            tx_usrp->set_tx_bandwidth(tx_bw, channel);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % tx_usrp->get_tx_bandwidth(channel)
                      << std::endl
                      << std::endl;
        }

        // set the antenna
        if (vm.count("tx-ant"))
            tx_usrp->set_tx_antenna(tx_ant, channel);
    }

    for (size_t ch = 0; ch < rx_channel_nums.size(); ch++) {
        size_t channel = rx_channel_nums[ch];
        if (rx_channel_nums.size() > 1) {
            std::cout << "Configuring RX Channel " << channel << std::endl;
        }

        // set the receive center frequency
        if (not vm.count("rx-freq")) {
            std::cerr << "Please specify the center frequency with --rx-freq"
                      << std::endl;
            return ~0;
        }
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (rx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t rx_tune_request(rx_freq);
        if (vm.count("rx-int-n"))
            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        rx_usrp->set_rx_freq(rx_tune_request, channel);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (rx_usrp->get_rx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the receive rf gain
        if (vm.count("rx-gain")) {
            std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain
                      << std::endl;
            rx_usrp->set_rx_gain(rx_gain, channel);
            std::cout << boost::format("Actual RX Gain: %f dB...")
                             % rx_usrp->get_rx_gain(channel)
                      << std::endl
                      << std::endl;
        }

        // set the receive analog frontend filter bandwidth
        if (vm.count("rx-bw")) {
            std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                      << std::endl;
            rx_usrp->set_rx_bandwidth(rx_bw, channel);
            std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                             % (rx_usrp->get_rx_bandwidth(channel) / 1e6)
                      << std::endl
                      << std::endl;
        }

        // set the receive antenna
        if (vm.count("rx-ant"))
            rx_usrp->set_rx_antenna(rx_ant, channel);
    }
    
      tx_usrp->set_clock_source("internal");
      tx_usrp->set_time_source("internal");
      tx_usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
      rx_usrp->set_clock_source("internal");
      rx_usrp->set_time_source("internal");
      rx_usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    
  
  

 std::string folder = "results_" + get_timestamp();

 fs::create_directory(folder);

 std::cout << "Created folder: " << folder << std::endl;
 
fs::copy_file("myClutterRemoival.m", folder + "/myClutterRemoival.m", fs::copy_options::overwrite_existing);
fs::copy_file("allone.txt", folder + "/allone.txt", fs::copy_options::overwrite_existing);

 std::cout << "Copied Post Processing " << std::endl;
 
fs::current_path(folder); 
 
system("/usr/local/MATLAB/R2023b/bin/matlab -batch \"myClutterRemoival.m\"&");
  
    // 10 scans
    for (int scan=0; scan<30; scan++){
     stop_signal_called = false;
      std::cout<< "iteration "<<scan<<"\n";
      // Align times in the RX USRP (the TX USRP does not require time-syncing)
      if (rx_usrp->get_num_mboards() > 1) {
          rx_usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
      }

	 
     //=================================================================
     
      std::ifstream inFile("allone.txt"); //allone or sfcw or golay
      if (!inFile) {
      		std::cerr << "Failed to open file.\n";
      		}
      		
      std::vector<std::complex<float>> chips_A, chips_B; 
      char comma; // for reading parentheses and comma 
      float a, b; 
      while (inFile >> a >> comma >> b ) { 
        if (comma == ',') { 
          chips_A.emplace_back(a); 
          chips_B.emplace_back(b); 
        } else { 
          std::cerr << "Format error in input file\n"; 
          break; 
        } 
    }
      
      inFile.close();
      std::cout << "Read " << chips_A.size() << " complex samples from file.\n";
      	/*	
      //=================================================================
      // upsample
      int sps = 1;
    
      std::vector<std::complex<float>> samples_A, samples_B;
      samples_A.reserve(chips_A.size() * sps);
      samples_B.reserve(chips_B.size() * sps);
      
      for (auto chip : chips_A) {
          std::complex<float> A = chip; // real ±1
          for (int i = 0; i < sps; ++i) {
              samples_A.push_back(A);
          }
      }
      
      for (auto chip : chips_B) {
          std::complex<float> B = chip; // real ±1
          for (int i = 0; i < sps; ++i) {
              samples_B.push_back(B);
          }
      }
      // IQ

*/
        // ===================================================================
      
    
  std::vector<std::complex<float>>  signal_tx = chips_A;
  //signal_tx(1, std::complex<float> (1.0,1.0));

      
      // create a transmit streamer
      // linearly map channels (index0 = channel0, index1 = channel1, ...)
      uhd::stream_args_t stream_args("fc32", otw);
      stream_args.channels             = tx_channel_nums;
      uhd::tx_streamer::sptr tx_stream = tx_usrp->get_tx_stream(stream_args);

      // allocate a buffer which we re-use for each channel
      if (spb == 0)

          spb = tx_stream->get_max_num_samps() * 10;  //removed *10
          
      std::cout<<spb<<"\n";
      std::vector<std::complex<float>> buff(spb);
      int num_channels = tx_channel_nums.size();

      // setup the metadata flags
      uhd::tx_metadata_t md;
      md.start_of_burst = true;
      md.end_of_burst   = false;
      md.has_time_spec  = true;
      md.time_spec = uhd::time_spec_t(0.5); // give us 0.5 seconds to fill the tx buffers


      // Check Ref and LO Lock detect
      std::vector<std::string> tx_sensor_names, rx_sensor_names;
      tx_sensor_names = tx_usrp->get_tx_sensor_names(0);
      if (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "lo_locked")
          != tx_sensor_names.end()) {
          uhd::sensor_value_t lo_locked = tx_usrp->get_tx_sensor("lo_locked", 0);
          std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                    << std::endl;
          UHD_ASSERT_THROW(lo_locked.to_bool());
      }
      rx_sensor_names = rx_usrp->get_rx_sensor_names(0);
      if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
          != rx_sensor_names.end()) {
          uhd::sensor_value_t lo_locked = rx_usrp->get_rx_sensor("lo_locked", 0);
          std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                    << std::endl;
          UHD_ASSERT_THROW(lo_locked.to_bool());
      }

      tx_sensor_names = tx_usrp->get_mboard_sensor_names(0);
      if ((ref == "mimo")
          and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "mimo_locked")
               != tx_sensor_names.end())) {
          uhd::sensor_value_t mimo_locked = tx_usrp->get_mboard_sensor("mimo_locked", 0);
          std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                    << std::endl;
          UHD_ASSERT_THROW(mimo_locked.to_bool());
      }
      if ((ref == "external")
          and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "ref_locked")
               != tx_sensor_names.end())) {
          uhd::sensor_value_t ref_locked = tx_usrp->get_mboard_sensor("ref_locked", 0);
          std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                    << std::endl;
          UHD_ASSERT_THROW(ref_locked.to_bool());
      }

      rx_sensor_names = rx_usrp->get_mboard_sensor_names(0);
      if ((ref == "mimo")
          and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
               != rx_sensor_names.end())) {
          uhd::sensor_value_t mimo_locked = rx_usrp->get_mboard_sensor("mimo_locked", 0);
          std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                    << std::endl;
          UHD_ASSERT_THROW(mimo_locked.to_bool());
      }
      if ((ref == "external")
          and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
               != rx_sensor_names.end())) {
          uhd::sensor_value_t ref_locked = rx_usrp->get_mboard_sensor("ref_locked", 0);
          std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                    << std::endl;
          UHD_ASSERT_THROW(ref_locked.to_bool());
      }

      if (total_num_samps == 0) {
          std::signal(SIGINT, &sig_int_handler);
          std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
      }
      
      
      
      std::vector<std::complex<short>> all_buffs;
      all_buffs.resize(total_num_samps);  // NOT reserve

      // reset usrp time to prepare for transmit/receive
      std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
      tx_usrp->set_time_now(uhd::time_spec_t(0.0));
      
      

      // start transmit worker thread
      std::thread transmit_thread([&]() {
          transmit_worker(buff, signal_tx, tx_stream, md,  num_channels);
      });


//



      // recv to file
          long double dt;
     // if (type == "double")
       //   dt=recv_to_file<std::complex<double>>(
        //      rx_usrp, "fc64", otw, file, spb, total_num_samps, settling, rx_channel_nums);
     // else if (type == "float")
        // dt= recv_to_file<std::complex<float>>(
          //    rx_usrp, "fc32", otw, file, spb, total_num_samps, settling, rx_channel_nums, all_buffs);
      //else if (type == "short")
          dt=recv_to_file<std::complex<short>>(
              rx_usrp, "sc16", otw, file, spb, total_num_samps, settling, rx_channel_nums, all_buffs);
     // else {
          // clean up transmit worker
       //   stop_signal_called = true;
         // transmit_thread.join();
       //   throw std::runtime_error("Unknown type " + type);
     // }

      // clean up transmit worker
      stop_signal_called = true;
      transmit_thread.join();
      
      post_processing( signal_tx,  rx_rate, dt, all_buffs, folder) ;

      // finished
      std::cout << std::endl << "Done!" << std::endl << std::endl;
      //std::this_thread::sleep_for(std::chrono::seconds(1)); // pause 1 second
      
      } //end of scan for
    return EXIT_SUCCESS;
}
