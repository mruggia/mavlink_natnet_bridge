
#pragma once

#include <stdio.h>
#include <errno.h>
#include <string>
#include <atomic>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <mavlink/common/mavlink.h>

struct MAVLinkSocket 
{
	// bind mavlink socket to ip/port, and indicate if ip was INADDR_ANY = "server"
	MAVLinkSocket(std::string _ip, int _port, bool _is_srv);

	// receive new mavlink data from socket
	bool recv();

	// forward current receive buffer to an other mavlink socket
	void forward(MAVLinkSocket* dst);

	int socket_fd;	// socket file descriptor
	bool connected;	// flag indicating if socket has received data
	bool is_srv;	// flag indicating if socket is bound to INADDR_ANY

	// local and remote ip/port/socket
	std::string loc_ip; int loc_port; struct sockaddr_in loc_addr; socklen_t loc_addr_len = sizeof(struct sockaddr_in);
	std::string rmt_ip; int rmt_port; struct sockaddr_in rmt_addr; socklen_t rmt_addr_len = sizeof(struct sockaddr_in);

	// current receive buffer
	char recv_buffer[2048]; int recv_len = 0;

};

// relay all mavlink traffic from src to dst socket (to be run in a separate thread) (a mavlink socket can be a relay source only once!)
void MAVLinkRelay(std::atomic_bool* running, MAVLinkSocket* src, MAVLinkSocket* dst);
