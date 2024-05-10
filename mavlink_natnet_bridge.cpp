

#include <thread>
#include <atomic>
#include <signal.h>
#include <chrono>

#include "mavlink_socket.h"
#include "natnet_socket.h"

// ############################################################################

#define DRONE_PORT	14550	// mavlink port to drone
#define GROUND_PORT	14560	// mavlink port to groundstation
#define NATNET_BODY	2		// natnet rigid-body id
#define NATNET_RATE	100		// natnet update rate [hz]

#define ORIG_LAT	468536490	// origin GPS coordinate of NATNET local coordinates
#define ORIG_LON	95145790	//
#define ORIG_ALT	0			//

// ############################################################################

// ctrl+c event handler
std::atomic_bool running = true;
void exit_handler(int s) { running = false; }

// send set_gps_global_origin message to mavlink socket
void set_gps_global_origin(MAVLinkSocket* dst, int32_t lat, int32_t lon, int32_t alt);

// ############################################################################


int main(int argc, char* argv[]) 
{
	signal(SIGINT, exit_handler);	// register ctrl+c callback

	MAVLinkSocket drone ("0.0.0.0",   DRONE_PORT, true);	// open mavlink drone socket (server)
	MAVLinkSocket ground("127.0.0.1", GROUND_PORT, false);	// open mavlink groundstation socket (client)
	NatNetSocket  mocap (NATNET_BODY);						// open natnet mocap socket for specified rigid-body
	// natnet expected to be configured as z-axis = up !!!

	std::thread drone_to_ground_relay (MAVLinkRelay, &running, &drone, &ground);			 // relay mavlink traffic from drone to groundstation
	std::thread ground_to_drone_relay (MAVLinkRelay, &running, &ground,&drone);				 // relay mavlink traffic from groundstation to drone
	std::thread mocap_to_drone_relay  (NatNetRelay,  &running, &mocap, &drone, NATNET_RATE); // relay mocap traffic to drone
	std::thread mocap_to_ground_relay (NatNetRelay,  &running, &mocap, &ground,NATNET_RATE); // relay mocap traffic to ground
	
	// run until user terminates program
	while(running) {
		// regularly resend mavlink command to set global origin for local position
		// currently not working due to: https://github.com/PX4/PX4-Autopilot/pull/21996
		// set_gps_global_origin(&drone, ORIG_LAT, ORIG_LON, ORIG_ALT);
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}

	drone_to_ground_relay.join();	// join relay threads
	ground_to_drone_relay.join();	//
	mocap_to_drone_relay.join();	//
	mocap_to_ground_relay.join();	//

	return 0;
}

// ############################################################################

// send set_gps_global_origin message to mavlink socket
void set_gps_global_origin(MAVLinkSocket* dst, int32_t lat, int32_t lon, int32_t alt) {

	auto time_now = std::chrono::system_clock::now();
	uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch()).count();

	// build set_gps_global_origin message
	mavlink_message_t message;
	mavlink_msg_set_gps_global_origin_pack_chan(
		255, MAV_COMP_ID_PERIPHERAL, MAVLINK_COMM_0, &message,
		1, lat, lon, alt, timestamp
	);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	const int len = mavlink_msg_to_send_buffer(buffer, &message);

	// send set_gps_global_origin message
	if (!dst->is_srv || dst->connected) {
		sendto(dst->socket_fd, buffer, len, 0, (const struct sockaddr*) &(dst->rmt_addr), dst->rmt_addr_len);
	}

}

// ############################################################################