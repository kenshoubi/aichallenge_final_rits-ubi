// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE. 
//
// ****************************************************************************

#include <septentrio_gnss_driver/node/rosaic_node.hpp>
#include <regex>

/**
 * @file rosaic_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */
 
rosaic_node::ROSaicNode::ROSaicNode() : Node("septentrio_gnss")
{
	g_use_gnss_time = declare_parameter<bool>("use_gnss_time", true);
	g_frame_id = declare_parameter<std::string>("frame_id", (std::string) "gnss");
	g_publish_gpst = declare_parameter<bool>("publish.gpst", true);
	g_publish_navsatfix = declare_parameter<bool>("publish.navsatfix", true);
	g_publish_gpsfix = declare_parameter<bool>("publish.gpsfix", true);
	g_publish_pose = declare_parameter<bool>("publish.pose", true);
	g_publish_diagnostics = declare_parameter<bool>("publish.diagnostics", true);
	int leap_seconds = declare_parameter<int>("leap_seconds", 18);
	getROSInt("leap_seconds", g_leap_seconds, static_cast<uint32_t>(18), leap_seconds);

	RCLCPP_DEBUG(this->get_logger(), "Called ROSaicNode() constructor..");
	
	// Parameters must be set before initializing IO
	connected_ = false;
	getROSParams();
	
	// Initializes Connection
	initializeIO();
	
	// Subscribes to all requested Rx messages by adding entries to the C++ multimap 
	// storing the callback handlers and publishes ROS messages
    defineMessages();
	
	// Sends commands to the Rx regarding which SBF/NMEA messages it should output and sets all 
	// its necessary corrections-related parameters
	if (!g_read_from_sbf_log)
	{
		while(!connected_) reconnect();
		configureRx();
	}
	
	// Since we already have a ros::Spin() elsewhere, we use waitForShutdown() here
	RCLCPP_DEBUG(this->get_logger(), "Leaving ROSaicNode() constructor..");
}

//! The send() method of AsyncManager class is paramount for this purpose.
//! Note that std::to_string() is from C++11 onwards only.
//! Since ROSaic can be launched before booting the Rx, we have to watch out for escape characters that are sent 
//! by the Rx to indicate that it is in upgrade mode. Those characters would then be mingled with the first command 
//! we send to it in this method and could result in an invalid command. Hence we first enter command mode via "SSSSSSSSSS".
void rosaic_node::ROSaicNode::configureRx()
{
	RCLCPP_DEBUG(this->get_logger(), "Called configureRx() method");
	
	// It is imperative to hold a lock on the mutex "g_response_mutex" while modifying the variable 
	// "g_response_received". Same for "g_cd_mutex" and "g_cd_received".
	std::unique_lock<std::mutex> lock(g_response_mutex);
	std::unique_lock<std::mutex> lock_cd(g_cd_mutex);
	
	// Determining communication mode: TCP vs USB/Serial
	unsigned stream = 1;
	std::smatch match;
	std::regex_match(device_, match, std::regex("(tcp)://(.+):(\\d+)"));
	std::string proto(match[1]);
	std::string rx_port;
	if (proto == "tcp") 
	{
		// Escape sequence (escape from correction mode), ensuring that we can send our real commands afterwards...
		IO.send("\x0DSSSSSSSSSSSSSSSSSSS\x0D\x0D"); 
		// We wait for the connection descriptor before we send another command, otherwise the latter would not be processed.
		g_cd_condition.wait(lock_cd, [](){return g_cd_received;}); 
		g_cd_received = false;
		rx_port = g_rx_tcp_port;
	}
	else
	{
		rx_port = rx_serial_port_;
		// After booting, the Rx sends the characters "x?" to all ports, which could potentially mingle with our first command.
		// Hence send a safeguard command "lif", whose potentially false processing is harmless.
		IO.send("lif, Identification \x0D");
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	uint32_t rx_period_pvt = parsing_utilities::convertUserPeriodToRxCommand(polling_period_pvt_);
	uint32_t rx_period_rest = parsing_utilities::convertUserPeriodToRxCommand(polling_period_rest_);
	std::string pvt_sec_or_msec;
	std::string rest_sec_or_msec;
	if (polling_period_pvt_ == 1000 || polling_period_pvt_ == 2000 || polling_period_pvt_ == 5000 || 
		polling_period_pvt_ == 10000) pvt_sec_or_msec = "sec";
	else pvt_sec_or_msec = "msec";
	if (polling_period_rest_ == 1000 || polling_period_rest_ == 2000 || polling_period_rest_ == 5000 || 
		polling_period_rest_ == 10000) rest_sec_or_msec = "sec";
	else rest_sec_or_msec = "msec";
	
	// Turning off all current SBF/NMEA output 
	// Authentication, leaving anonymous mode
	IO.send("login, Tibor, Tibor \x0D");
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	IO.send("sso, all, none, none, off \x0D");
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	IO.send("sno, all, none, none, off \x0D");
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	
	// Setting the datum to be used by the Rx (not the NMEA output though, which only provides MSL and undulation 
	// (by default with respect to WGS84), but not ellipsoidal height)
	{
		std::stringstream ss;
		ss << "sgd, " << datum_ << "\x0D"; 
		IO.send(ss.str());
	}
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	
	// Setting SBF/NMEA output of Rx
	if (publish_gpgga_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GGA, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_gprmc_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", RMC, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_gpgsa_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GSA, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_gpgsv_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GSV, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_pvtcartesian_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PVTCartesian, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_pvtgeodetic_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PVTGeodetic, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_poscovcartesian_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PosCovCartesian, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_poscovgeodetic_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PosCovGeodetic, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_atteuler_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", AttEuler, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_attcoveuler_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", AttCovEuler, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (g_publish_gpsfix == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", ChannelStatus, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); // avoids invoking the std::string constructor
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", MeasEpoch, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", DOP, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", VelCovGeodetic, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (g_publish_diagnostics == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", ReceiverStatus, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", QualityInd, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", ReceiverSetup, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	
	// Setting the marker-to-ARP offsets. This comes after the "sso, ..., ReceiverSetup, ..." command, 
	// since the latter is only generated when a user-command is entered to change one or more values in the block.
	{
		std::stringstream ss;
		ss << "sao, Main, " << string_utilities::trimString(std::to_string(delta_e_)) << ", " << 
			string_utilities::trimString(std::to_string(delta_n_)) << ", " << 
			string_utilities::trimString(std::to_string(delta_u_)) << ", \"" << ant_type_ << "\", \"" << ant_serial_nr_ << 
			"\", 0 \x0D"; 
		IO.send(ss.str());
	}
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	
	// Configuring the NTRIP connection
	// First disable any existing NTRIP connection on NTR1
	{
		std::stringstream ss;
		ss << "snts, NTR1, off \x0D"; 
		IO.send(ss.str());
	}
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	if (rx_has_internet_)
	{
		if (mode_ == "off")
		{
		}
		else if (mode_ == "Client")
		{
			{
				std::stringstream ss;
				ss << "snts, NTR1, " << mode_ << ", " << caster_ << ", " << std::to_string(caster_port_) << ", " << 
					username_ << ", " << password_ << ", " << mountpoint_ << ", " << ntrip_version_ << ", " << send_gga_ << " \x0D"; 
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
		}
		else if (mode_ == "Client-Sapcorda")
		{
			{
				std::stringstream ss;
				ss << "snts, NTR1, Client-Sapcorda, , , , , , , , \x0D"; 
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
		}
		else 
		{
			RCLCPP_ERROR(this->get_logger(), "Invalid mode specified for NTRIP settings.");
		}
	}
	else 	// Since the Rx does not have internet (and you will not be able to share it via USB), 
			//we need to forward the corrections ourselves, though not on the same port.
	{
		if (proto == "tcp")
		{
			{
				std::stringstream ss;
				// In case IPS1 was used before, old configuration is lost of course.
				ss << "siss, IPS1, " << std::to_string(rx_input_corrections_tcp_) << ", TCP2Way \x0D"; 
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
			{
				std::stringstream ss;
				ss << "sno, Stream" << std::to_string(stream) << ", IPS1, GGA, " << pvt_sec_or_msec << 
					std::to_string(rx_period_pvt) << " \x0D";
				++stream;
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
		}
		{
			std::stringstream ss;
			if (proto == "tcp")
			{
				ss << "sdio, IPS1, " << rtcm_version_ << ", +SBF+NMEA \x0D"; 
			}
			else
			{
				ss << "sdio, " << rx_input_corrections_serial_ << ", " << rtcm_version_ << ", +SBF+NMEA \x0D"; 
			}
			IO.send(ss.str());
		}
	}
	RCLCPP_DEBUG(this->get_logger(), "Leaving configureRx() method");
}
void rosaic_node::ROSaicNode::getROSParams() 
{
	// Communication parameters
	device_ = declare_parameter<std::string>("device", std::string("/dev/ttyACM0"));
    int serial_baudrate = declare_parameter<int>("serial.baudrate", 115200);
	getROSInt("serial.baudrate", baudrate_, static_cast<uint32_t>(115200), serial_baudrate);
	hw_flow_control_ = declare_parameter<std::string>("serial.hw_flow_control", std::string("off"));
	rx_serial_port_ = declare_parameter<std::string>("serial.rx_serial_port", std::string("USB1"));
	
	reconnect_delay_s_ = declare_parameter<int>("reconnect_delay_s", 4.0f);
	
	// Polling period parameters
	int polling_period_pvt = declare_parameter<int>("polling_period.pvt", 1000);
	getROSInt("polling_period.pvt", polling_period_pvt_, static_cast<uint32_t>(1000), polling_period_pvt);
	if (polling_period_pvt_ != 10 && polling_period_pvt_ != 20 && polling_period_pvt_ != 50 && polling_period_pvt_ != 100 
	&& polling_period_pvt_ != 200 && polling_period_pvt_ != 250 && polling_period_pvt_ != 500 && polling_period_pvt_ != 1000 
	&& polling_period_pvt_ != 2000 && polling_period_pvt_ != 5000 && polling_period_pvt_ != 10000)
	{
		RCLCPP_ERROR(this->get_logger(), "Please specify a valid polling period for PVT-related SBF blocks and NMEA messages.");
	}
	int polling_period_rest = declare_parameter<int>("polling_period.rest", 1000);
	getROSInt("polling_period.rest", polling_period_rest_, static_cast<uint32_t>(1000), polling_period_rest);
	if (polling_period_rest_ != 10 && polling_period_rest_ != 20 && polling_period_rest_ != 50 && polling_period_rest_ != 100 
	&& polling_period_rest_ != 200 && polling_period_rest_ != 250 && polling_period_rest_ != 500 && polling_period_rest_ != 1000 
	&& polling_period_rest_ != 2000 && polling_period_rest_ != 5000 && polling_period_rest_ != 10000)
	{
		RCLCPP_ERROR(this->get_logger(), "Please specify a valid polling period for PVT-unrelated SBF blocks and NMEA messages.");
	}
	
	// Datum and marker-to-ARP offset
	datum_ = declare_parameter<std::string>("datum", std::string("ETRS89"));
	ant_type_ = declare_parameter<std::string>("ant_type", std::string("Unknown"));
	ant_serial_nr_ = declare_parameter<std::string>("ant_serial_nr", std::string("Unknown"));
	delta_e_ = declare_parameter<float>("marker_to_arp.delta_e", 0.0f);
	delta_n_ = declare_parameter<float>("marker_to_arp.delta_n", 0.0f);
	delta_u_ = declare_parameter<float>("marker_to_arp.delta_u", 0.0f);
	
	// Correction service parameters
	mode_ = declare_parameter<std::string>("ntrip_settings.mode", std::string("off"));
	caster_ = declare_parameter<std::string>("ntrip_settings.caster", std::string());
    int ntrip_settings_caster_port = declare_parameter("ntrip_settings.caster_port", 0);
	getROSInt("ntrip_settings.caster_port", caster_port_, static_cast<uint32_t>(0), ntrip_settings_caster_port);
	username_ = declare_parameter<std::string>("ntrip_settings.username", std::string());
	password_ = declare_parameter<std::string>("ntrip_settings.password", std::string());
	mountpoint_ = declare_parameter<std::string>("ntrip_settings.mountpoint", std::string());
	ntrip_version_ = declare_parameter<std::string>("ntrip_settings.ntrip_version", std::string("v2"));
	send_gga_ = declare_parameter<std::string>("ntrip_settings.send_gga", std::string("auto"));
	rx_has_internet_ = declare_parameter<bool>("ntrip_settings.rx_has_internet", false);
	rtcm_version_ = declare_parameter<std::string>("ntrip_settings.rtcm_version", std::string("RTCMv3"));
    int ntrip_settings_rx_input_corrections_tcp = declare_parameter("ntrip_settings.rx_input_corrections_tcp", 28785);
	getROSInt("ntrip_settings.rx_input_corrections_tcp", rx_input_corrections_tcp_, static_cast<uint32_t>(28785), ntrip_settings_rx_input_corrections_tcp);
	rx_input_corrections_serial_ = declare_parameter<std::string>("ntrip_settings.rx_input_corrections_serial", std::string("USB2"));
	
	// Publishing parameters, the others remained global since they need to be accessed in the callbackhandlers.hpp file
	publish_gpgga_ = declare_parameter<bool>("publish.gpgga", true);
	publish_gprmc_ = declare_parameter<bool>("publish.gprmc", true);
	publish_gpgsa_ = declare_parameter<bool>("publish.gpgsa", true);
	publish_gpgsv_ = declare_parameter<bool>("publish.gpgsv", true);
	publish_pvtcartesian_ = declare_parameter<bool>("publish.pvtcartesian", true);
	publish_pvtgeodetic_ = declare_parameter<bool>("publish.pvtgeodetic", true);
	publish_poscovcartesian_ = declare_parameter<bool>("publish.poscovcartesian", true);
	publish_poscovgeodetic_ = declare_parameter<bool>("publish.poscovgeodetic", true);
	publish_atteuler_ = declare_parameter<bool>("publish.atteuler", true);
	publish_attcoveuler_ = declare_parameter<bool>("publish.attcoveuler", true);

	// To be implemented: RTCM, setting datum, raw data settings, PPP, SBAS, fix mode...
	RCLCPP_DEBUG(this->get_logger(), "Finished getROSParams() method");

}

void rosaic_node::ROSaicNode::initializeIO() 
{
	RCLCPP_DEBUG(this->get_logger(), "Called initializeIO() method");
	std::smatch match;
	// In fact: smatch is a typedef of match_results<string::const_iterator>
	if (std::regex_match(device_, match, std::regex("(tcp)://(.+):(\\d+)")))
	// C++ needs \\d instead of \d: Both mean decimal.
	// Note that regex_match can be used with a smatch object to store results, or without. In any case, 
	// true is returned if and only if it matches the !complete! string.
	{
		// The first sub_match (index 0) contained in a match_result always represents the full match 
		// within a target sequence made by a regex, and subsequent sub_matches represent sub-expression 
		// matches corresponding in sequence to the left parenthesis delimiting the sub-expression in the regex,
		// i.e. $n Perl is equivalent to m[n] in boost regex.
		tcp_host_ = match[2];
		tcp_port_ = match[3];
		
		serial_ = false;
		g_read_from_sbf_log = false;
		connect();
	}
	else if (std::regex_match(device_, match, std::regex("(file_name):(\\w+.sbf)")))
	{
		serial_ = false;
		g_read_from_sbf_log = true;
		g_unix_time = rclcpp::Time(0, 0);
		prepareSBFFileReading(match[2]);
		//std::thread temporary_thread(boost::bind(&ROSaicNode::prepareSBFFileReading, this, match[2]));
		//temporary_thread.detach();
	}
	else if (std::regex_match(device_, match, std::regex("(serial):(.+)")))
	{
		serial_ = true;
		g_read_from_sbf_log = false;
		std::string proto(match[2]);
		std::stringstream ss;
		ss << "Searching for serial port" << proto;
		RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
		connect();
		//std::thread temporary_thread(boost::bind(&ROSaicNode::connect, this));
		//temporary_thread.detach();
	}
	else
	{
		std::stringstream ss;
		ss << "Device[" << device_ << "] is unsupported. Perhaps you meant 'tcp://host:port' or 'file_name:xxx.sbf' or 'serial:/path/to/device'?";
		RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
	}
	RCLCPP_DEBUG(this->get_logger(), "Leaving initializeIO() method");
}

void rosaic_node::ROSaicNode::prepareSBFFileReading(std::string file_name)
{
	try
	{
		std::stringstream ss;
		ss << "Setting up everything needed to read from" << file_name;
		RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
		IO.initializeSBFFileReading(file_name);
	}
	catch (std::runtime_error& e)
	{
		std::stringstream ss;
		ss << "Comm_IO::initializeSBFFileReading() failed for SBF File" << file_name << " due to: " << e.what();
		RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
	}
	
}


void rosaic_node::ROSaicNode::connect() 
{
	RCLCPP_DEBUG(this->get_logger(), "Called connect() method");
	RCLCPP_DEBUG(this->get_logger(), "Setting ROS timer for calling reconnect() method until connection succeds");
	const auto period_ns =
	    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(reconnect_delay_s_));
	auto timer_callback = std::bind(&ROSaicNode::reconnect, this);
	reconnect_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
	    get_clock(), period_ns, std::move(timer_callback),
	    get_node_base_interface()->get_context());
    get_node_timers_interface()->add_timer(reconnect_timer_, nullptr);

	RCLCPP_DEBUG(this->get_logger(), "Leaving connect() method"); // This will never be output since ros::spin() is on the line above.
}

//! In serial mode (not USB, since the Rx port is then called USB1 or USB2), please ensure that you are 
//! connected to the Rx's COM1, COM2 or COM3 port, !if! you employ UART hardware flow control.
void rosaic_node::ROSaicNode::reconnect()
{
	RCLCPP_DEBUG(this->get_logger(), "Called reconnect() method");
	if (connected_ == true)
	{
		reconnect_timer_->cancel();
		RCLCPP_DEBUG(this->get_logger(), "Stopped ROS timer since successully connected.");
	}
	else
	{
		if (serial_)
		{
			bool initialize_serial_return = false;
			try
			{
				RCLCPP_INFO(this->get_logger(), "Connecting serially to device %s, targeted baudrate: %u", device_.c_str(), baudrate_);
				initialize_serial_return = IO.initializeSerial(device_, baudrate_, hw_flow_control_);
			}
			catch (std::runtime_error& e)
			{
				{
					std::stringstream ss;
					ss << "IO.initializeSerial() failed for device " << device_ << " due to: " << e.what();
					RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
				}
			}
			if (initialize_serial_return)
			{
				std::unique_lock<std::mutex> lock(connection_mutex_);
				//std::mutex::scoped_lock lock(connection_mutex_);
				connected_ = true;
				lock.unlock();
				connection_condition_.notify_one();
			}
		}
		else
		{
			bool initialize_tcp_return = false;
			try
			{
				RCLCPP_INFO(this->get_logger(), "Connecting to tcp://%s:%s ...", tcp_host_.c_str(), tcp_port_.c_str());
				initialize_tcp_return = IO.initializeTCP(tcp_host_, tcp_port_);
			}
			catch (std::runtime_error& e)
			{
				{
					std::stringstream ss;
					ss << "IO.initializeTCP() failed for host " << tcp_host_ << " on port " << tcp_port_ << 
						" due to: " << e.what();
					RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
				}
			}
			if (initialize_tcp_return)
			{
				std::unique_lock<std::mutex> lock(connection_mutex_);
				//std::mutex::scoped_lock lock(connection_mutex_);
				connected_ = true;
				lock.unlock();
				connection_condition_.notify_one();
			}
		}
	}
	RCLCPP_DEBUG(this->get_logger(), "Leaving reconnect() method");
}

//! initializeSerial is not self-contained: The for loop in Callbackhandlers' handle method would 
//! never open a specific handler unless the handler is added (=inserted) to the C++ map via this 
//! function. This way, the specific handler can be called, in which in turn RxMessage's read() method is 
//! called, which publishes the ROS message.
void rosaic_node::ROSaicNode::defineMessages() 
{
	RCLCPP_DEBUG(this->get_logger(), "Called defineMessages() method");
    rclcpp::QoS durable_qos(g_ROS_QUEUE_SIZE);
    durable_qos.transient_local();
	
	if (publish_gpgga_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::Gpgga>("$GPGGA");
		g_gpgga_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::Gpgga>("gpgga", durable_qos);
	}
	if (publish_gprmc_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::Gprmc>("$GPRMC");
		g_gprmc_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::Gprmc>("gprmc", durable_qos);
	}
	if (publish_gpgsa_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::Gpgsa>("$GPGSA");
		g_gpgsa_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::Gpgsa>("gpgsa", durable_qos);
	}
	if (publish_gpgsv_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::Gpgsv>("$GPGSV");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::Gpgsv>("$GLGSV");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::Gpgsv>("$GAGSV");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::Gpgsv>("$GBGSV");
		g_gpgsv_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::Gpgsv>("gpgsv", durable_qos);
	}
	if (publish_pvtcartesian_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::PVTCartesian>("4006");
		g_pvtcartesian_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::PVTCartesian>("pvtcartesian", durable_qos);
	}
	if (publish_pvtgeodetic_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::PVTGeodetic>("4007");
		g_pvtgeodetic_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::PVTGeodetic>("pvtgeodetic", durable_qos);
	}
	if (publish_poscovcartesian_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::PosCovCartesian>("5905");
		g_poscovcartesian_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::PosCovCartesian>("poscovcartesian", durable_qos);
	}
	if (publish_poscovgeodetic_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::PosCovGeodetic>("5906");
		g_poscovgeodetic_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::PosCovGeodetic>("poscovgeodetic", durable_qos);
	}
	if (publish_atteuler_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::AttEuler>("5938");
		g_atteuler_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::AttEuler>("atteuler", durable_qos);
	}
	if (publish_attcoveuler_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver_msgs::msg::AttCovEuler>("5939");
		g_attcoveuler_publisher = create_publisher<septentrio_gnss_driver_msgs::msg::AttCovEuler>("attcoveuler", durable_qos);
	}
	if (g_publish_gpst == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("GPST");
		g_gpst_publisher = create_publisher<sensor_msgs::msg::TimeReference>("gpst", durable_qos);
	}
	if (g_publish_navsatfix == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
		{
			RCLCPP_ERROR(this->get_logger(), "For a proper NavSatFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
		}
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<sensor_msgs::msg::NavSatFix>("NavSatFix");
		g_navsatfix_publisher = create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix", durable_qos);
	}
	if (g_publish_gpsfix == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
		{
			RCLCPP_ERROR(this->get_logger(), "For a proper GPSFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
		}
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<gps_msgs::msg::GPSFix>("GPSFix");
		// The following blocks are never published, yet are needed for the construction of the GPSFix message, hence we have empty callbacks.
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4013"); // ChannelStatus block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4027"); // MeasEpoch block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4001"); // DOP block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("5908"); // VelCovGeodetic block
		g_gpsfix_publisher = create_publisher<gps_msgs::msg::GPSFix>("gpsfix", durable_qos);
	}
	if (g_publish_pose == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false || publish_atteuler_ == false || 
			publish_attcoveuler_ == false)
		{
			RCLCPP_ERROR(this->get_logger(), "For a proper PoseWithCovarianceStamped message, please set the publish/pvtgeodetic, publish/poscovgeodetic, publish_atteuler and publish_attcoveuler ROSaic parameters all to true.");
		}
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<geometry_msgs::msg::PoseWithCovarianceStamped>("PoseWithCovarianceStamped");
		g_posewithcovariancestamped_publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", durable_qos);
	}
	if (g_publish_diagnostics == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<diagnostic_msgs::msg::DiagnosticArray>("DiagnosticArray");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4014"); // ReceiverStatus block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4082"); // QualityInd block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("5902"); // ReceiverSetup block
		g_diagnosticarray_publisher = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", durable_qos);
	}
	// so on and so forth...
	RCLCPP_DEBUG(this->get_logger(), "Leaving defineMessages() method");
}


//! If true, the ROS message headers' unix time field is constructed from the TOW (in the SBF case) 
//! and UTC (in the NMEA case) data. 
//! If false, times are constructed within the driver via time(NULL) of the \<ctime\> library.
bool g_use_gnss_time;
//! Whether or not to publish the sensor_msgs::TimeReference message with GPST
bool g_publish_gpst;
//! Whether or not to publish the sensor_msgs::NavSatFix message
bool g_publish_navsatfix;
//! Whether or not to publish the gps_common::GPSFix message
bool g_publish_gpsfix;
//! Whether or not to publish the geometry_msgs::PoseWithCovarianceStamped message
bool g_publish_pose;
//! Whether or not to publish the diagnostic_msgs::DiagnosticArray message
bool g_publish_diagnostics;
//! The frame ID used in the header of every published ROS message
std::string g_frame_id;
//! The number of leap seconds that have been inserted into the UTC time
uint32_t g_leap_seconds;
//! Mutex to control changes of global variable "g_response_received"
std::mutex g_response_mutex;
//! Determines whether a command reply was received from the Rx
bool g_response_received;
//! Condition variable complementing "g_response_mutex"
std::condition_variable g_response_condition;
//! Mutex to control changes of global variable "g_cd_received"
std::mutex g_cd_mutex;
//! Determines whether the connection descriptor was received from the Rx
bool g_cd_received;
//! Condition variable complementing "g_cd_mutex"
std::condition_variable g_cd_condition;
//! Whether or not we still want to read the connection descriptor, which we only want in the very beginning to 
//! know whether it is IP10, IP11 etc.
bool g_read_cd;
//! Rx TCP port, e.g. IP10 or IP11, to which ROSaic is connected to
std::string g_rx_tcp_port;
//! Since after SSSSSSSSSSS we need to wait for second connection descriptor, we have to count the connection descriptors
uint32_t g_cd_count;
//! For GPSFix: Whether the ChannelStatus block of the current epoch has arrived or not
bool g_channelstatus_has_arrived_gpsfix;
//! For GPSFix: Whether the MeasEpoch block of the current epoch has arrived or not
bool g_measepoch_has_arrived_gpsfix;
//! For GPSFix: Whether the DOP block of the current epoch has arrived or not
bool g_dop_has_arrived_gpsfix;
//! For GPSFix: Whether the PVTGeodetic block of the current epoch has arrived or not
bool g_pvtgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the PosCovGeodetic block of the current epoch has arrived or not
bool g_poscovgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the VelCovGeodetic block of the current epoch has arrived or not
bool g_velcovgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the AttEuler block of the current epoch has arrived or not
bool g_atteuler_has_arrived_gpsfix;
//! For GPSFix: Whether the AttCovEuler block of the current epoch has arrived or not
bool g_attcoveuler_has_arrived_gpsfix;
//! For NavSatFix: Whether the PVTGeodetic block of the current epoch has arrived or not
bool g_pvtgeodetic_has_arrived_navsatfix;
//! For NavSatFix: Whether the PosCovGeodetic block of the current epoch has arrived or not
bool g_poscovgeodetic_has_arrived_navsatfix;
//! For PoseWithCovarianceStamped: Whether the PVTGeodetic block of the current epoch has arrived or not
bool g_pvtgeodetic_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the PosCovGeodetic block of the current epoch has arrived or not
bool g_poscovgeodetic_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the AttEuler block of the current epoch has arrived or not
bool g_atteuler_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the AttCovEuler block of the current epoch has arrived or not
bool g_attcoveuler_has_arrived_pose;
//! For DiagnosticArray: Whether the ReceiverStatus block of the current epoch has arrived or not
bool g_receiverstatus_has_arrived_diagnostics;
//! For DiagnosticArray: Whether the QualityInd block of the current epoch has arrived or not
bool g_qualityind_has_arrived_diagnostics;
//! When reading from an SBF file, the ROS publishing frequency is governed by the time stamps found in the SBF blocks
//! therein.
rclcpp::Time g_unix_time;
//! Whether or not we are reading from an SBF file
bool g_read_from_sbf_log;
//! A C++ map for keeping track of the SBF blocks necessary to construct the GPSFix ROS message
std::map<std::string, uint32_t> g_GPSFixMap;
//! A C++ map for keeping track of the SBF blocks necessary to construct the NavSatFix ROS message
std::map<std::string, uint32_t> g_NavSatFixMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the PoseWithCovarianceStamped ROS message
std::map<std::string, uint32_t> g_PoseWithCovarianceStampedMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the DiagnosticArray ROS message
std::map<std::string, uint32_t> g_DiagnosticArrayMap;
//! ROS2 Publisher for the ROSaic node
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::Gpgga>> g_gpgga_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::Gprmc>> g_gprmc_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::Gpgsa>> g_gpgsa_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::Gpgsv>> g_gpgsv_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::PVTCartesian>> g_pvtcartesian_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::PVTGeodetic>> g_pvtgeodetic_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::PosCovCartesian>> g_poscovcartesian_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::PosCovGeodetic>> g_poscovgeodetic_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::AttEuler>> g_atteuler_publisher;
std::shared_ptr<rclcpp::Publisher<septentrio_gnss_driver_msgs::msg::AttCovEuler>> g_attcoveuler_publisher;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::TimeReference>> g_gpst_publisher;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> g_navsatfix_publisher;
std::shared_ptr<rclcpp::Publisher<gps_msgs::msg::GPSFix>> g_gpsfix_publisher;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> g_posewithcovariancestamped_publisher;
std::shared_ptr<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>> g_diagnosticarray_publisher;
//! You must initialize the NodeHandle in the "main" function (or in any method called 
//! indirectly or directly by the main function). 
//! Queue size for ROS publishers
const uint32_t g_ROS_QUEUE_SIZE = 1;

int main(int argc, char** argv) 
{
	rclcpp::init(argc, argv);
	
	g_response_received = false;
	g_cd_received = false;
	g_read_cd = true;
	g_cd_count = 0;
	g_channelstatus_has_arrived_gpsfix = false;
	g_measepoch_has_arrived_gpsfix = false;
	g_dop_has_arrived_gpsfix = false;
	g_pvtgeodetic_has_arrived_gpsfix = false;
	g_pvtgeodetic_has_arrived_navsatfix = false;
	g_pvtgeodetic_has_arrived_pose = false;
	g_poscovgeodetic_has_arrived_gpsfix = false;
	g_poscovgeodetic_has_arrived_navsatfix = false;
	g_poscovgeodetic_has_arrived_pose = false;
	g_velcovgeodetic_has_arrived_gpsfix = false;
	g_atteuler_has_arrived_gpsfix = false;
	g_atteuler_has_arrived_pose = false;
	g_attcoveuler_has_arrived_gpsfix = false;
	g_attcoveuler_has_arrived_pose = false;
	g_receiverstatus_has_arrived_diagnostics = false;
	g_qualityind_has_arrived_diagnostics = false;
	
	rclcpp::spin(std::make_shared<rosaic_node::ROSaicNode>());
	rclcpp::shutdown();
	return 0;
}
