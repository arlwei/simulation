#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/simulator.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/waypoint-mobility-model.h"
#include <iostream>
#include <stack>
#include <deque>

using namespace ns3;

void
printOut(Ptr<Node> object) {
  std::cout << "*****************" << std::endl;
  Ptr<WaypointMobilityModel> position = object->GetObject<WaypointMobilityModel> ();
  Vector pos = position->GetPosition ();
  std::cout <<  Simulator::Now().GetSeconds() << "S : x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;

  if(Simulator::Now().GetSeconds() > 45.0 && Simulator::Now().GetSeconds() < 61.0){
      position->AddWaypoint(Waypoint(Seconds(60),Vector(4125, 2550.0, 0)));
    }

}

int main (int argc, char *argv[])
{
	NodeContainer sta;
	sta.Create (5);
	Ptr<ListPositionAllocator> positionAlloc =
	  CreateObject<ListPositionAllocator> ();
	positionAlloc->Add(Vector(2675,4125,0));
	positionAlloc->Add(Vector(4125,2675,0));
	positionAlloc->Add(Vector(2675,1225,0));
	positionAlloc->Add(Vector(1225,2675,0));
	positionAlloc->Add(Vector(2675,2675,0));
	MobilityHelper mobility;
	mobility.SetPositionAllocator (positionAlloc);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (sta);
	for (NodeContainer::Iterator j = sta.Begin ();
		   j != sta.End (); ++j)
	{
		  Ptr<Node> object = *j;
		  Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
		  Vector pos = position->GetPosition ();
		  std::cout << "x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
	 }

	NodeContainer veh;
	veh.Create(2);
/**	Waypoint way1(Time(30),Vector(1000,2550.0));
	Waypoint way2(Time(38),Vector(1200,2550.0));
	Waypoint way3(Time(42),Vector(1250,2550.0));
	Ptr<WaypointMobilityModel> wayAlloc =
	  CreateObject<WaypointMobilityModel> ();
	wayAlloc->AddWaypint(way1);
	wayAlloc->AddWaypint(way2);
	wayAlloc->AddWaypint(way3);
**/

	std::vector<Ptr<MobilityModel> > mobilityStack;
	std::deque<Waypoint> waypoints;
	bool lazyNotify = true;

	mobility.SetMobilityModel("ns3::WaypointMobilityModel");

	ObjectFactory mobilityFactory;
	mobilityFactory.SetTypeId ("ns3::WaypointMobilityModel");
	mobilityFactory.Set ("LazyNotify", BooleanValue (lazyNotify));

	// Populate the vector of mobility models.
	for (uint32_t i = 0; i < 1; i++)
	{
		// Create a new mobility model.
		Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel>(mobilityFactory.Create ()->GetObject<MobilityModel> ());

		// Add this mobility model to the stack.
		mobilityStack.push_back (model);
		Simulator::Schedule (Seconds (0.0), &Object::Initialize, model);
	}

	Waypoint wpt (Seconds (0.0), Vector (0.0, 2550.0, 0.0));
	waypoints.push_back(wpt);
	wpt.time = Seconds(0.000001);
	waypoints.push_back(wpt);
	wpt.time += Seconds(29.999999);
	wpt.position.x = 1000.0;
	waypoints.push_back(wpt);
	wpt.time += Seconds(8.0);
	wpt.position.x = 1200;
	waypoints.push_back(wpt);
	wpt.time += Seconds(4.0);
	wpt.position.x = 1250;
	waypoints.push_back(wpt);

//	// Create waypoints
//	for ( uint32_t iw = 0; iw < waypointCount; ++iw )
//	{
//		wpt.time += Seconds (1.0);
//		waypoints.push_back (wpt);
//	}

	// Add the same waypoints to each node
	std::vector<Ptr<MobilityModel> >::iterator i;
	for (i = mobilityStack.begin (); i != mobilityStack.end (); ++i)
	{
		Ptr<WaypointMobilityModel> mob = (*i)->GetObject<WaypointMobilityModel> ();

		for ( std::deque<Waypoint>::iterator w = waypoints.begin (); w != waypoints.end (); ++w )
		{
			mob->AddWaypoint (*w);
		}
		veh.Get(0)->AggregateObject(mob);
		mobility.Install(veh.Get(0));
	}
	std::cout << "hello" << std::endl;

	Simulator::Schedule (Seconds (15.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (30.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (31.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (38.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (44.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (45.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (50.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (55.0), printOut, veh.Get(0));
	Simulator::Schedule (Seconds (60.0), printOut, veh.Get(0));

	std::cout << "world" << std::endl;


	Simulator::Stop();
	Simulator::Run();
	Simulator::Destroy();
}
