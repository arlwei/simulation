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
#include <fstream>
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include<string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("testwave");

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

  NodeContainer m_vehicles;
  uint32_t vehNum = 5;
  m_vehicles.Create( vehNum );//Cars
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



/**
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
                          DoubleValue(1200.0));
   Ptr<YansWifiChannel> channel = wifiChannel.Create ();

   // The below set of helpers will help us to put together the wifi NICs we want
 //  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
 //  wifiPhy.SetChannel (channel);
   // ns-3 supports generate a pcap trace
 //  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

   YansWavePhyHelper wifiPhy =  YansWavePhyHelper::Default ();
   wifiPhy.SetChannel (channel);
   wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
   wifiPhy.Set ("TxPowerStart",DoubleValue (5.890e9));
   wifiPhy.Set ("TxPowerEnd", DoubleValue (5.890e9));
 
   // Setup WAVE PHY and MAC
//   NqosWaveMacHelper mac = NqosWaveMacHelper::Default ();
//   mac.SetType ("ns3::AdhocWifiMac");

   QosWaveMacHelper mac = QosWaveMacHelper::Default ();

  //Wave Helper
   WaveHelper waveHelper = WaveHelper::Default ();
 //  Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
   std::string ofdm = "OfdmRate6MbpsBW10MHz";
   waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                                                                  "DataMode",StringValue (ofdm),
                                                                                  "ControlMode",StringValue (ofdm));

  //===channel
  //YansWifiChannelHelper CCHChannel = YansWifiChannelHelper::Default ();
  //CCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  //CCHChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
//                          DoubleValue(1200.0));


  //Ptr<YansWifiChannel> CCH = CCHChannel.Create();

  //===wifiphy
  //YansWifiPhyHelper CCHPhy =  YansWifiPhyHelper::Default ();
  //CCHPhy.SetChannel (CCH);
  //CCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  //CCHPhy.Set ("TxPowerStart",DoubleValue (5.890e9));
  //CCHPhy.Set ("TxPowerEnd", DoubleValue (5.890e9));

//  // 802.11p mac
//  NqosWaveMacHelper CCH80211pMac = NqosWaveMacHelper::Default ();
//  Wifi80211pHelper CCH80211p = Wifi80211pHelper::Default ();

//  CCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
//                                                                                  "DataMode",StringValue ("OfdmRate6MbpsBW10MHz"),
//                                                                                  "ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"));

//  WifiHelper wifi = WifiHelper::Default ();
//  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

//  NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();

//  Ssid ssid = Ssid ("ns-3-ssid");
//  mac.SetType ("ns3::StaWifiMac",
//               "Ssid", SsidValue (ssid),
//             "ActiveProbing", BooleanValue (false));

  NetDeviceContainer m_VehDevices;
  m_VehDevices = waveHelper.Install(wifiPhy, mac, m_vehicles);
**/


/**
YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
wavePhy.SetChannel (wifiChannel.Create ());
QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
WaveHelper waveHelper = WaveHelper::Default ();
NetDeviceContainer m_VehDevices;
m_VehDevices = waveHelper.Install (wavePhy, waveMac, m_vehicles);
**/


YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
wifiPhy.SetChannel (wifiChannel.Create ());
NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
Wifi80211pHelper _80211pHelper = Wifi80211pHelper::Default ();
NetDeviceContainer m_VehDevices;
m_VehDevices = _80211pHelper.Install (wifiPhy, wifi80211pMac, m_vehicles);

  InternetStackHelper stack;
 // stack.SetRoutingHelper(aodv);
  stack.Install (m_vehicles);

  Ipv4InterfaceContainer m_VehInterface, m_ConInterface, m_CSMAInterface;
  Ipv4AddressHelper address;
  NS_LOG_INFO ("Assign IP Addresses.");
  address.SetBase ("10.1.0.0", "255.255.0.0");
  m_VehInterface = address.Assign (m_VehDevices);





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

 //   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  //  mobility.Install (m_controllers);


    UdpEchoServerHelper echoServer (9);

    ApplicationContainer serverApps = echoServer.Install (m_vehicles.Get (3));
    serverApps.Start (Seconds (1.0));
    serverApps.Stop (Seconds (10.0));

    UdpEchoClientHelper echoClient (m_VehInterface.GetAddress (3), 9);
    echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
    echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
    echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

    ApplicationContainer clientApps =
      echoClient.Install (m_vehicles.Get (1));
    clientApps.Start (Seconds (2.0));
    clientApps.Stop (Seconds (10.0));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    Simulator::Stop (Seconds (10.0));

    if (tracing == true)
      {
        wifiPhy.EnablePcapAll("testwave");
      }

    Simulator::Run ();
    Simulator::Destroy ();
    return 0;
}

