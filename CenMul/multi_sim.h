#ifndef MULTI_SIM_H
#define MULTI_SIM_H

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/csma-module.h"

#include "utils.h"


using namespace ns3;

class CMultiMain
{
public:
	CMultiMain();
	~CMultiMain();
	void Simulate(int argc, char *argv[]);
protected:
	void SetDefault();
	void ParseArguments(int argc, char *argv[]);
	void LoadTraffic();
	void ConfigNode();
	void ConfigChannels();
	void ConfigDevices();
	void ConfigMobility();
	void ConfigApp();
	void ConfigTracing();
	void Run();
	void ProcessOutputs();
	bool CheckActive(Node node);
	void Look_at_clock();
private:
	Ptr<Socket> source;
	std::string traceFile;
	std::string logFile;
	std::string phyMode;
	std::string lossModle;
	std::string homepath;
	std::string folder;
	std::ofstream os;
	double freq2;//CCH
	double txp2;//CCH
	double range2;//CCH
	uint32_t packetSize; // bytes
	uint32_t numPackets;
	double interval; // seconds
	bool verbose;
	uint32_t vehNum;
	uint32_t conNum;
	double duration;
	NodeContainer m_vehicles; //Cars
	NodeContainer m_controllers; //Vehicles
	NetDeviceContainer m_VehDevices, m_ConDevices, m_CSMADevices;
	Ipv4InterfaceContainer m_VehInterface, m_ConInterface, m_CSMAInterface;
	//////////TongJi////////////
	uint32_t Rx_Routing_Bytes, Tx_Routing_Bytes;
	uint32_t RX_Routing_Pkts, TX_Routing_Pkts;
	uint32_t Rx_Data_Bytes, Tx_Data_Bytes;
	uint32_t Rx_Data_Pkts, Tx_Data_Pkts;
	uint32_t m_port;
	ApplicationContainer m_source, m_sink, m_cars, m_controller;
	Ptr<ns3::vanetmobility::VANETmobility> VMo;
	void ReceiveDataPacket (Ptr<Socket> socket);
	void SendDataPacket ();

	Utils util;
	std::vector<std::string> vec_str; //save the information of vehicles
	double speed_idle;  //the speed when vehicles are in idle area
	double speed_waiting; //the speed when vehicles are in waiting area
	Seconds goStraight;  // the time for going Straight to pass the intersection
	Seconds turnLeft;	// the time for turning left to pass the intersection
	uint32_t state;
};

#endif // MULTI_SIM_H
