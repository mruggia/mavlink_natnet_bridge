

#include "mavlink_socket.h"

// bind mavlink socket to ip/port, and indicate if ip was INADDR_ANY = "server"
MAVLinkSocket::MAVLinkSocket(std::string _ip, int _port, bool _is_srv) : is_srv(_is_srv), connected(false)
{
	// open UDP socket
	socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
	if (socket_fd < 0) { printf("socket error: %s\n", strerror(errno)); return; }

	memset(&loc_addr, 0, sizeof(struct sockaddr_in));
	memset(&rmt_addr, 0, sizeof(struct sockaddr_in));

	// configure socket depending on if it's a client or a server
	if (is_srv) {
		loc_ip = _ip; loc_port = _port;

		loc_addr.sin_family = AF_INET;
		inet_pton(AF_INET, loc_ip.c_str(), &(loc_addr.sin_addr));
		loc_addr.sin_port = htons(loc_port);

		const int bind_ret = bind(socket_fd, (struct sockaddr*)(&loc_addr), sizeof(loc_addr));
		if (bind_ret != 0) { printf("bind error: %s\n", strerror(errno)); return; }

	} else {
		rmt_ip = _ip; rmt_port = _port;
		loc_ip = _ip; loc_port = _port;	// wrong, but is unused anyways

		rmt_addr.sin_family = AF_INET;
		inet_pton(AF_INET, rmt_ip.c_str(), &(rmt_addr.sin_addr));
		rmt_addr.sin_port = htons(rmt_port);

	}

	// set recv timeout to 100ms
	struct timeval tv;
	tv.tv_sec = 0; tv.tv_usec = 100000;
	const int setsock_ret = setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	if (setsock_ret < 0) { printf("setsockopt error: %s\n", strerror(errno)); return; }

}

// receive new mavlink data from socket
bool MAVLinkSocket::recv() 
{
	// receive data
	recv_len = recvfrom(socket_fd, recv_buffer, sizeof(recv_buffer), 0, (struct sockaddr*)(&rmt_addr), &rmt_addr_len);
	if (recv_len <= 0) { return false; }

	// check if this is the first message received on the socket
	if (!connected) {
		connected = true;

		// save first connection to a server as remote address
		if (is_srv) {
			char ip[INET_ADDRSTRLEN]; uint16_t port;
			inet_ntop(AF_INET, &(rmt_addr.sin_addr), ip, sizeof(ip));
			port = htons(rmt_addr.sin_port);
			rmt_ip = ip; rmt_port = port;
		}

		printf("mavlink connected: %s:%d <-> %s:%d\n", loc_ip.c_str(), loc_port, rmt_ip.c_str(), rmt_port);
	}

	/*// decode mavlink messages
	for (int i = 0; i < recv_len; ++i) {
		mavlink_message_t message;
		mavlink_status_t status;
		if (mavlink_parse_char(MAVLINK_COMM_0, recv_buffer[i], &message, &status) == 1) {

			printf("Received message %d from %d/%d\n", message.msgid, message.sysid, message.compid);

			// if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
			// 	mavlink_heartbeat_t heartbeat;
			// 	mavlink_msg_heartbeat_decode(&message, &heartbeat);
			// }
		}
	}*/

	return true;
}

// forward current receive buffer to an other mavlink socket
void MAVLinkSocket::forward(MAVLinkSocket* dst) 
{
	if (dst->is_srv && !dst->connected) { return; }
	sendto(dst->socket_fd, recv_buffer, recv_len, 0, (const struct sockaddr*) &(dst->rmt_addr), dst->rmt_addr_len);
}

// relay all mavlink traffic from src to dst socket (to be run in a separate thread) (a mavlink socket can be a relay source only once!)
void MAVLinkRelay(std::atomic_bool* running, MAVLinkSocket* src, MAVLinkSocket* dst)
{
	while(*running) {
		if( src->recv() ) {
			src->forward(dst);
		}
	}
}