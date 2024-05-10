
#pragma once

#include <stdio.h>
#include <chrono>
#include <mutex>
#include <atomic>
#include <thread>

#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include "mavlink_socket.h"

struct NatNetSocket
{
	NatNetSocket(int _body_id);	// connect to any natnet client on the network look for specified rigid-body id
	~NatNetSocket();			// disconnect from natnet client

	NatNetClient* g_pClient = NULL;	// handle to natnet client
	int body_id;					// id of rigid-body
	bool connected = false;			// flag indicating if socket has received data
	int sent_old = 0; 				// counter how many old packets have been sent to mavlink before last new natnet packet

	float pos[3] = {0.0, 0.0, 0.0};			// natnet rigid-body position
	float rot[4] = {1.0, 0.0, 0.0, 0.0};	// natnet rigid-body orientation
	std::mutex mtx;							// mutex for reading/writing pos/rot

};

// callback for natnet traffic
void NatNetCallback(sFrameOfMocapData *data, void* pUserData);

// relay all natnet traffic from src to a mavlink dst sockets at a specific hz rate (to be run in a separate thread)
void NatNetRelay(std::atomic_bool* running, NatNetSocket* src, MAVLinkSocket* dst1, int rate_hz);
