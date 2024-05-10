

#include "natnet_socket.h"

// connect to any natnet client on the network look for specified rigid-body id
NatNetSocket::NatNetSocket(int _body_id) : body_id(_body_id)
{
	// create NatNet client
	g_pClient = new NatNetClient();

	// discover server
	sNatNetDiscoveredServer servers[4];
	int actualNumDescriptions = 4;
	NatNet_BroadcastServerDiscovery( servers, &actualNumDescriptions );
	if (actualNumDescriptions < 1) { printf("could not find natnet server!\n"); return; }

	// create server description
	char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
	snprintf(
		g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr, "%d.%d.%d.%d",
		servers[0].serverDescription.ConnectionMulticastAddress[0],
		servers[0].serverDescription.ConnectionMulticastAddress[1],
		servers[0].serverDescription.ConnectionMulticastAddress[2],
		servers[0].serverDescription.ConnectionMulticastAddress[3]
	);
	sNatNetClientConnectParams g_connectParams;
	g_connectParams.connectionType    = servers[0].serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
	g_connectParams.serverCommandPort = servers[0].serverCommandPort;
	g_connectParams.serverDataPort    = servers[0].serverDescription.ConnectionDataPort;
	g_connectParams.serverAddress     = servers[0].serverAddress;
	g_connectParams.localAddress      = servers[0].localAddress;
	g_connectParams.multicastAddress  = g_discoveredMulticastGroupAddr;

	// init client and connect to NatNet server
	int retCode = g_pClient->Connect( g_connectParams );
	if (retCode != ErrorCode_OK) { printf("unable to connect to natnet server. error code: %d\n", retCode); return; }

	// set the frame callback handler
	g_pClient->SetFrameReceivedCallback(NatNetCallback, this);

}

// disconnect from natnet client
NatNetSocket::~NatNetSocket()
{
	if (g_pClient) {
		g_pClient->Disconnect();
		delete g_pClient; g_pClient = NULL;
	}
}

// callback for natnet traffic
void NatNetCallback(sFrameOfMocapData *data, void* pUserData)
{
	//search for rigid-body pose in mocap frame
	NatNetSocket* socket = (NatNetSocket*) pUserData;
	static float pos_old[3], rot_old[4];
	for(int i=0; i < data->nRigidBodies; i++) {
		if (data->RigidBodies[i].ID == socket->body_id) {

			// save rigid-body pose data so socket
			// natnet expected to be configured as z-axis=up, but mavlink is z-axis=down !!!
			socket->mtx.lock();
			socket->pos[0] =  data->RigidBodies[i].x;
			socket->pos[1] = -data->RigidBodies[i].y;
			socket->pos[2] = -data->RigidBodies[i].z;
			socket->rot[0] =  data->RigidBodies[i].qw;
			socket->rot[1] =  data->RigidBodies[i].qx;
			socket->rot[2] = -data->RigidBodies[i].qy;
			socket->rot[3] = -data->RigidBodies[i].qz;
			if (socket->pos[0]!=pos_old[0] || socket->pos[1]!=pos_old[1] || socket->pos[2]!=pos_old[2] ||
				socket->rot[0]!=rot_old[0] || socket->rot[1]!=rot_old[1] || socket->rot[2]!=rot_old[2] || socket->rot[2]!=rot_old[2]) {
				socket->sent_old = 0;
			}
			pos_old[0] = socket->pos[0]; pos_old[1] = socket->pos[1]; pos_old[2] = socket->pos[2];
			rot_old[0] = socket->rot[0]; rot_old[1] = socket->rot[1]; rot_old[2] = socket->rot[2]; rot_old[3] = socket->rot[3]; 
			socket->mtx.unlock();

			// check if this is the first message received on the socket
			if (!socket->connected) {
				socket->connected = true;
				printf("natnet connected: ID=%d\n", socket->body_id);
			}
		}
	}
}

// relay all natnet traffic from src to a mavlink dst sockets at a specific hz rate (to be run in a separate thread)
void NatNetRelay(std::atomic_bool* running, NatNetSocket* src, MAVLinkSocket* dst, int rate_hz)
{
	auto time_last = std::chrono::system_clock::now();
	auto rate = std::chrono::milliseconds(1000/rate_hz);

	// estimated covariance data
	float cov[21] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};

	while(*running) {
		
		// run at specified rate
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		auto time_now = std::chrono::system_clock::now();
		if (time_now-time_last > rate) {
			if (time_now-time_last > 10*rate) {
				time_last = time_now;
			} else {
				time_last += rate;
			}

			// build odometry mavlink message with current natnet rigid-body pose
			src->mtx.lock();
			if (src->sent_old >= 10) { src->mtx.unlock(); continue; }
			src->sent_old += 1;
			mavlink_message_t message;
			uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch()).count();
			mavlink_msg_odometry_pack_chan(
				255, MAV_COMP_ID_PERIPHERAL, MAVLINK_COMM_0, &message,
				timestamp, MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_NED,
				src->pos[0], src->pos[1], src->pos[2], src->rot,
				NAN, NAN, NAN, NAN, NAN, NAN,
				cov, cov,
				0, MAV_ESTIMATOR_TYPE_VIO, 0
			);
			src->mtx.unlock();
			uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
			const int len = mavlink_msg_to_send_buffer(buffer, &message);

			// send odometry mavlink message to destination mavlink socket
			if (!dst->is_srv || dst->connected) {
				sendto(dst->socket_fd, buffer, len, 0, (const struct sockaddr*) &(dst->rmt_addr), dst->rmt_addr_len);
			}

		}
	}

}
