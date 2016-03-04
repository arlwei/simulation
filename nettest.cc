#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include <string>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");


int
main (int argc, char *argv[])
{

  bool verbose = true;
  bool tracing = false;

  CommandLine cmd;
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);

  // Check for valid number of csma or wifi nodes
  // 250 should be enough, otherwise IP addresses
  // soon become an issue

  if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  NodeContainer m_vehicles, m_controllers;
  uint32_t vehNum = 20, conNum = 5;
  m_vehicles.Create( vehNum );//Cars
  m_controllers.Create ( conNum );
  /*Only Apps Are Different Between Different kind of Nodes*/
  // Name nodes
//  for (uint32_t i = 0; i < vehNum; ++i)
//  {
//          std::ostringstream os;
//          os << "vehicle-" << i;
//          Names::Add(os.str(), m_vehicles.Get(i));
//  }
//  for (uint32_t i = 0; i < conNum; ++i)
//  {
//          std::ostringstream os;
//          os << "controller-" << i;
//          Names::Add(os.str(), m_vehicles.Get(i));
//  }

  //===channel
  YansWifiChannelHelper CCHChannel = YansWifiChannelHelper::Default ();
  CCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  CCHChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
                          DoubleValue(1200.0));
  Ptr<YansWifiChannel> CCH = CCHChannel.Create();

  //===wifiphy
  YansWifiPhyHelper wavePhy =  YansWifiPhyHelper::Default ();
  wavePhy.SetChannel (CCH);
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  wavePhy.Set ("TxPowerStart",DoubleValue (20));
  wavePhy.Set ("TxPowerEnd", DoubleValue (20));

////  // 802.11p mac
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();

  Wifi80211pHelper waveHelper = Wifi80211pHelper::Default ();
  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                                                                    "DataMode",StringValue ("OfdmRate6MbpsBW10MHz"),
                                                                                    "ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"));


//  YansWifiPhyHelper wavePhy = YansWifiPhyHelper::Default ();
//  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
//  wavePhy.SetChannel (wifiChannel.Create ());
//  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
//  Wifi80211pHelper waveHelper = Wifi80211pHelper::Default ();

//  Ssid ssid = Ssid ("ns-3-ssid");
//  wifi80211pMac.SetType ("ns3::StaWifiMac",
//               "Ssid", SsidValue (ssid),
//               "ActiveProbing", BooleanValue (false));

  NetDeviceContainer m_VehDevices;
  m_VehDevices = waveHelper.Install(wavePhy, wifi80211pMac, m_vehicles);

//  wifi80211pMac.SetType ("ns3::ApWifiMac",
//               "Ssid", SsidValue (ssid));
  NetDeviceContainer m_ConDevices;
  m_ConDevices = waveHelper.Install(wavePhy, wifi80211pMac, m_controllers);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

  NetDeviceContainer m_CSMADevices;
  m_CSMADevices = csma.Install (m_controllers);

  InternetStackHelper stack;
 // stack.SetRoutingHelper(aodv);
  stack.Install (m_vehicles);
  stack.Install (m_controllers);

  Ipv4InterfaceContainer m_VehInterface, m_ConInterface, m_CSMAInterface;
  Ipv4AddressHelper address;
  NS_LOG_INFO ("Assign IP Addresses.");
  address.SetBase ("10.1.0.0", "255.255.0.0");
  m_VehInterface = address.Assign (m_VehDevices);
  m_ConInterface = address.Assign (m_ConDevices);

  address.SetBase ("10.2.0.0", "255.255.0.0");
  m_CSMAInterface = address.Assign (m_CSMADevices);




    MobilityHelper mobility;

    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (5.0),
                                   "DeltaY", DoubleValue (10.0),
                                   "GridWidth", UintegerValue (11),
                                   "LayoutType", StringValue ("RowFirst"));

    mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
    mobility.Install (m_vehicles);

    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (m_controllers);


    UdpEchoServerHelper echoServer (9);

    ApplicationContainer serverApps = echoServer.Install (m_controllers.Get (3));
    serverApps.Start (Seconds (1.0));
    serverApps.Stop (Seconds (10.0));

    UdpEchoClientHelper echoClient (m_ConInterface.GetAddress (3), 9);
    echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
    echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
    echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

    ApplicationContainer clientApps =
      echoClient.Install (m_vehicles.Get (3));
    clientApps.Start (Seconds (2.0));
    clientApps.Stop (Seconds (10.0));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    Simulator::Stop (Seconds (10.0));

    if (tracing == true)
      {
        wavePhy.EnablePcap ("testnet", m_VehDevices.Get (3));
        wavePhy.EnablePcap ("testnet", m_ConDevices.Get (3));
      }

    Simulator::Run ();
    Simulator::Destroy ();
    return 0;
}
