/*
 * multi_sim.cc
 *
 *  Created on: Thu 28, 2015
 *      Author: nw
 */

#include "multi_sim.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <dirent.h>//DIR*

NS_LOG_COMPONENT_DEFINE ("SDN");


using namespace ns3;

static void
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
   Vector pos = mobility->GetPosition ();
   Vector vel = mobility->GetVelocity ();
   std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
   << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
   << ", z=" << vel.z << std::endl;
}


CMultiMain::CMultiMain()
{
	phyMode = "OfdmRate6MbpsBW10MHz";
	freq2 = 5.890e9;  //802.11p CCH CH178
	txp2 = 20;  // CCH
	range2 = 300.0; //CCH
	packetSize = 1000; // bytes
	numPackets = 1;
	interval = 0.1; // seconds
	verbose = false;
	duration = -1;
	vehNum = 0;
	conNum = 5;
	speed_idle = 20;
	speed_waiting = 12;
	goStraight = 3.0;
	turnLeft = 4.0;
	Rx_Data_Bytes = 0;
	Rx_Data_Pkts = 0;
	Rx_Routing_Bytes = 0;
	RX_Routing_Pkts = 0;
	Tx_Data_Bytes = 0;
	Tx_Data_Pkts = 0;
	Tx_Routing_Bytes = 0;
	TX_Routing_Pkts = 0;
	m_port = 65419;
	homepath = getenv("HOME");
	folder="csim";
}

CMultiMain::~CMultiMain()
{
	os.close();
}

void CMultiMain::Simulate(int argc, char *argv[])
{
	SetDefault();
	ParseArguments(argc, argv);
	LoadTraffic();
	ConfigNode();
	ConfigChannels();
	ConfigDevices();
	ConfigMobility();
	ConfigApp();
	ConfigTracing();
	Run();
	ProcessOutputs();
}

void CMultiMain::SetDefault()
{
	//Handle By Constructor
}

void CMultiMain::ParseArguments(int argc, char *argv[])
{
	CommandLine cmd;
//	cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
	cmd.AddValue ("vehNum", "Number of vehicles", vehNum);
//	cmd.AddValue ("conNum", "Number of controllers", conNum);
	cmd.AddValue ("duration", "Duration of Simulation", duration);
//	cmd.AddValue ("logFile", "Log file", logFile);
	cmd.AddValue ("folder", "Working Directory", folder);
	cmd.AddValue ("txp2", "TX power for CCH", txp2);
	cmd.AddValue ("range2", "Range for CCH", range2);
	cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
	cmd.Parse (argc,argv);

	// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
			      StringValue (phyMode));

}

/**
 * @brief CMultiMain::LoadTraffic get paths of vehicles from configure file
 */
void
CMultiMain::LoadTraffic()
{
	DIR* dir = NULL;
	//DIR* subdir=NULL;
	std::string temp(homepath+"/test/"+folder);
	if((dir = opendir(temp.data()))==NULL)
		NS_FATAL_ERROR("Cannot open input path "<<temp.data()<<", Aborted.");


	std::string vehicles_trace = temp + "/traces.txt";

	std::string output = temp + "/result.txt";

	vehNum = util.readTraceFile(vehicles_trace, vec_str);
	std::cout << vehNnum << " vehicles to be scheduled" << std::endl;
	os.open(output.data(),std::ios::out);
}


void
CMultiMain::CreateNode()
{
	m_vehicles.Create( vehNum );//Cars
	m_controllers.Create ( conNum );
	/*Only Apps Are Different Between Different kind of Nodes*/
	// Name nodes
	for (uint32_t i = 0; i < vehNum; ++i)
	{
		std::ostringstream os;
		os << "vehicle-" << i;
		Names::Add(os.str(), m_vehicles.Get(i));
	}
	for (uint32_t i = 0; i < conNum; ++i)
	{
		std::ostringstream os;
		os << "controller-" << i;
		Names::Add(os.str(), m_vehicles.Get(i));
	}
}


void
CMultiMain::CreateChannels()
{
	//===channel
	YansWifiChannelHelper CCHChannel;
	CCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	CCHChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
				DoubleValue(range2));

	// the channelg
	Ptr<YansWifiChannel> CCH = CCHChannel.Create();

	//===wifiphy
	YansWifiPhyHelper CCHPhy =  YansWifiPhyHelper::Default ();
	CCHPhy.SetChannel (CCH);
	CCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
	CCHPhy.Set ("TxPowerStart",DoubleValue (txp2));
	CCHPhy.Set ("TxPowerEnd", DoubleValue (txp2));

	// 802.11p mac
	NqosWaveMacHelper CCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper CCH80211p = Wifi80211pHelper::Default ();

	CCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
											"DataMode",StringValue (phyMode),
											"ControlMode",StringValue (phyMode));
	Ssid ssid = Ssid ("ns-3-ssid");
	CCH80211pMac.SetType ("ns3::StaWifiMac",
		     "Ssid", SsidValue (ssid),
		     "ActiveProbing", BooleanValue (false));

	m_VehDevices = CCH80211p.Install(CCHPhy, CCH80211pMac, m_vehicles);

	CCH80211pMac.SetType ("ns3::ApWifiMac",
		     "Ssid", SsidValue (ssid));
	m_ConDevices = CCH80211p.Install(CCHPhy, CCH80211pMac, m_controllers);

	CsmaHelper csma;
	csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
	csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

	m_CSMADevices = csma.Install (csmaNodes);

}


void
CMultiMain::ConfigMobility()
{
	MobilityHelper mobility;

	//configure the position for five constrollers
	Ptr<ListPositionAllocator> positionAlloc =
	  CreateObject<ListPositionAllocator> ();
	positionAlloc->Add(Vector(1960,3000,0));
	positionAlloc->Add(Vector(3000,1960,0));
	positionAlloc->Add(Vector(1960,920,0));
	positionAlloc->Add(Vector(920,1960,0));
	positionAlloc->Add(Vector(1960,1960,0));

	mobility.SetPositionAllocator (positionAlloc);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (m_controllers);

	//here we will set the position of vehicles and set their mobility model

	std::vector<Ptr<MobilityModel> > mobilityStack;
	std::deque<Waypoint> waypoints;
//	bool lazyNotify = true;

	ObjectFactory mobilityFactory;
	mobilityFactory.SetTypeId ("ns3::WaypointMobilityModel");
//	mobilityFactory.Set ("LazyNotify", BooleanValue (lazyNotify));

	// Populate the vector of mobility models.
	for (uint32_t i = 0; i < vehNum; i++)
	{
		// Create a new mobility model.
		Ptr<WaypointMobilityModel> model = mobilityFactory.Create ()->GetObject<WaypointMobilityModel> ();

		// Add this mobility model to the stack.
		mobilityStack.push_back (model);
		Simulator::Schedule (Seconds (0.0), &Object::Initialize, model);
	}

	uint32_t im = 0;
	for(std::vector<std::string>::iterator vec_it1 = vec_str.begin (); vec_it1 != vec_str.end (); vec_it1++) {

	    std::vector<std::string> tmp_vec;
	    util.split (*vec_it1, tmp_vec);

	    Ptr<WaypointMobilityModel> mob = mobilityStack[im]->GetObject<WaypointMobilityModel> ();

	    ns3::Vector initPos(atof(tmp_vec[1]), atof(tmp_vec[2]), 0.0);
	    Waypoint wpt (Seconds (tmp_vec[0]), initPos);
	    mob->AddWaypoint (wpt);

	    double travel = util.travelTime (initPos, (uint32_t)atoi(tmp_vec[3]), speed_idle);
	    wpt.time += travel;
	    wpt.position.x = initPos.x;
	    wpt.position.y = initPos.y;
	    mob->AddWaypoint (wpt);
	    m_vehicles.Get(im)->AggregateObject(mob);
	  }

	for(int i = 0; i < vehNum; i++) {
	    std::ostringstream oss;
	    oss << "/NodeList/"
		<< m_vehicles.Get (i)->GetId ()
		<< "/$ns3::MobilityModel/CourseChange";
	    Config::Connect (oss.str (),
				MakeCallback (&CourseChange));
	  }
}


void
CMultiMain::InstallInternetStack ()
{
//  AodvHelper aodv;
  InternetStackHelper stack;
 // stack.SetRoutingHelper(aodv);
  stack.Install (m_vehicles);
  Ipv4AddressHelper address;
  NS_LOG_INFO ("Assign IP Addresses.");
  address.SetBase ("10.1.0.0", "255.255.0.0");
  m_VehInterface = address.Assign (m_VehDevices);
  m_ConInterface = address.Assign (m_ConDevices);

  address.SetBase ("10.2.0.0", "255.255.0.0");
  m_CSMAInterface = address.Assign (m_CSMADevices);

}


void
CMultiMain::ConfigApp()
{
	//===Routing
	InternetStackHelper internet;
	if (mod != 1)
	{
		OlsrHelper olsr;
		//Ipv4ListRoutingHelper list;
		//list.Add(olsr,100);
		internet.SetRoutingHelper(olsr);
		std::cout<<"OLSR"<<std::endl;
	}
	else
	{
		std::cout<<"SDN"<<std::endl;
	}
	internet.Install (m_vehicles);

	std::cout<<"Now Internet Done"<<std::endl;

	//===IP ADDRESS
	Ipv4AddressHelper ipv4S;
	NS_LOG_INFO ("Assign IP Addresses.");
	ipv4S.SetBase ("10.1.1.0", "255.255.255.0");//SCH
	m_SCHInterface = ipv4S.Assign (m_SCHDevices);

	if (mod ==1)
	{
		Ipv4AddressHelper ipv4C;
		NS_LOG_INFO ("Assign IP-C Addresses.");
		ipv4C.SetBase("192.168.0.0","255.255.255.0");//CCH
		m_CCHInterface = ipv4C.Assign(m_CCHDevices);
	}


	//===Traffic
	//source

	//onoff
	Address remote (InetSocketAddress(m_SCHInterface.GetAddress(vehNum+2), m_port));
	OnOffHelper Source("ns3::UdpSocketFactory",remote);//SendToSink
	Source.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

	m_source = Source.Install(m_vehicles.Get(vehNum+1));//Insatll on Source
	m_source.Stop(Seconds(duration));//Default Start time is 0.

	/*
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	source = Socket::CreateSocket (m_nodes.Get(vehNum+1), tid);
	Simulator::Schedule(Seconds(0.0), &CMultiMain::SendDataPacket, this);
	*/

	//sink
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (m_vehicles.Get(vehNum+2), tid);//The Sink
	InetSocketAddress local = InetSocketAddress(m_SCHInterface.GetAddress(vehNum+2),m_port);
	sink->Bind(local);
	sink->SetRecvCallback(MakeCallback(&CMultiMain::ReceiveDataPacket, this));
}

void CMultiMain::ReceiveDataPacket(Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	while ((packet = socket->Recv()))
	{
		Rx_Data_Bytes += packet->GetSize();
		Rx_Data_Pkts++;
		std::cout<<".";
	}
}

void CMultiMain::SendDataPacket()
{
	/*Ptr<Packet> packet = Create<Packet> (packetSize);
	source->SendTo(packet, 0, )
	Simulator::Schedule(Seconds(interval), &CMultiMain::SendDataPacket, this);*/
	//TODO
}

void CMultiMain::ConfigTracing()
{
	//TODO
}

void CMultiMain::ProcessOutputs()
{
	Ptr<OnOffApplication> app = DynamicCast<OnOffApplication>(m_source.Get(0));
	Tx_Data_Pkts = app->Tx_packets;
	std::cout<<Tx_Data_Pkts<<std::endl;
	std::cout<<Rx_Data_Pkts<<std::endl;
}

void CMultiMain::Run()
{
	Simulator::Schedule(Seconds(0.0), &CMultiMain::Look_at_clock, this);
	std::cout << "Starting simulation for " << duration << " s ..."<< std::endl;
	Simulator::Stop(Seconds(duration));
	Simulator::Run();
	Simulator::Destroy();

}

void CMultiMain::Look_at_clock()
{
	std::cout<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	os<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	Ptr<OutputStreamWrapper> osw = Create<OutputStreamWrapper> (&std::cout);
	m_vehicles.Get(vehNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw);
	Ptr<OutputStreamWrapper> osw2 = Create<OutputStreamWrapper> (&os);
	m_vehicles.Get(vehNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw2);
	/*2  Ptr<MobilityModel> Temp;
	Vector vt;
	for (int i = 0;i<=vehNum+2;++i)
	{
		Temp = m_nodes.Get(i)->GetObject<MobilityModel>();
		vt = Temp->GetPosition();
		std::cout<<i<<":"<<vt.x<<","<<vt.y<<","<<vt.z<<";"<<std::flush;
	}
	std::cout<<std::endl;*/
	ProcessOutputs();

	Simulator::Schedule(Seconds(1.0), &CMultiMain::Look_at_clock, this);
}

// Example to use ns2 traces file in ns3
int main (int argc, char *argv[])
{
	CMultiMain SDN_test;
	SDN_test.Simulate(argc, argv);
	return 0;
}



