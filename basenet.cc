/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation;
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*
 * multi_intersection_simulation
 *
 *  Created on: Thu 28, 2015
 *      Author: nw
 */


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
#include <iostream>
#include <dirent.h>
#include <stdlib.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Test network");

typedef struct rpList {
  uint32_t vehicle_id;
  uint32_t lane_id;
  uint32_t next_lane_id;
  uint32_t next_inter_id;
  bool pass;
  double time;
  double x;
  double y;
}rp;

typedef struct pos {
  uint32_t vehicle_id;
  uint8_t  direction; // 1:y-; 2:x+;3:x-;4:y+;
  double   time_send;
  double   time_need;
}pos_unit;

/*****************************************************************************************************/
class Utils
{
public:

  Utils();

  int        readTraceFile(const char* fileName, std::vector<std::string> &vec_str);
  std::string trim(const std::string& str);
  int        split(const std::string& str, std::vector<std::string>& ret_, std::string sep);

  bool       isInIdleState(Vector &pos) const;
  bool       isInWaitingArea(Vector &pos) const;

  double     travelTime( Vector &pos, const int nextWaiting, const double speed) const;
  bool       isEnteringWaitingArea(Vector &pos) const;
  uint32_t   whichIntersection (Vector &pos) const;
  uint32_t   whichIntersection(uint32_t) const;
  uint32_t   whichWaitingArea(uint32_t) const;
  uint32_t   whichLane(uint32_t enterArea, uint32_t leavingArea);
  double     predictTime(uint32_t lane, uint32_t inter, double x, double y);
  void       getPosition(uint32_t lane,uint32_t inter_id,uint32_t count,double &x,double &y);
  double     passIntersectionWait(uint32_t lane, uint32_t inter_id, double &x, double &y);
  double     passIntersectionCore(uint32_t lane, uint32_t inter_id, double &x, double &y);
  double     passIntersectionIdle(uint32_t lane, uint32_t inter_id, double &x, double &y);
  uint32_t   nextIntersection (uint32_t lane, uint32_t inter_id);

  bool doubleEquals(double a,double b) const;

private:
  double go_straight;
  double turn_left;
  double speed_idle;  //the speed when vehicles are in idle state
  double speed_waiting; //the speed when vehicles are in waiting area and waiting state
  double wal; //waiting_area_length
  double cal; //core_area_length
  double ial;  //idle_area_length
  double space_occupy;
};
/***********************************************************************************************************/


Utils::Utils() {
  go_straight = 3.0;
  turn_left = 4.0;
  speed_idle = 20.0;
  speed_waiting = 12.0;
  wal = 100.0;
  cal = 40.0;
  ial = 800.0;
  space_occupy = 4;
}

int
Utils::readTraceFile(const char* fileName, std::vector<std::string> &vec_str) {
  std::ifstream fin(fileName);
  std::string  s;
  int count = 0;
  while ( getline(fin,s) )
  {
    vec_str.push_back (s);
    count++;
  }
  fin.close();
  return count;
}

std::string
Utils::trim(const std::string& str)
{
    std::string::size_type pos = str.find_first_not_of(' ');
    if (pos == std::string::npos)
    {
        return str;
    }
    std::string::size_type pos2 = str.find_last_not_of(' ');
    if (pos2 != std::string::npos)
    {
        return str.substr(pos, pos2 - pos + 1);
    }
    return str.substr(pos);
}


int
Utils::split(const std::string& str, std::vector<std::string>& ret_, std::string sep = ",")
{
    if (str.empty())
    {
        return 0;
    }

    std::string tmp;
    std::string::size_type pos_begin = str.find_first_not_of(sep);
    std::string::size_type comma_pos = 0;

    while (pos_begin != std::string::npos)
    {
        comma_pos = str.find(sep, pos_begin);
        if (comma_pos != std::string::npos)
        {
            tmp = str.substr(pos_begin, comma_pos - pos_begin);
            pos_begin = comma_pos + sep.length();
        }
        else
        {
            tmp = str.substr(pos_begin);
            pos_begin = comma_pos;
        }

        if (!tmp.empty())
        {
            ret_.push_back(tmp);
            tmp.clear();
        }
    }
    return 0;
}


bool
Utils::isInIdleState(Vector &pos) const{
  double x = pos.x;
  double y = pos.y;
  if(x < 800 || ((y < 1840 || y > 2080) && x > 900 && x < 940) || (1840 > x && x > 1040 && 1980 > y && y > 1940)
     || y < 800 || ((1980 > x && x > 1940) && (y > 3120 || (2880 > y && y > 2080) || (1840 > y && y > 1040)))
     || (940 > y && y > 900 && (x < 1840 || x > 2080)) || (3020 > y && y > 2980 && (x < 1840 || x > 2080))
     || ( 2880 > x && x > 2080 && 1980 > y && y > 1940) || (3020 > x && x > 2980 && (y < 1840 || y > 2080)) || x > 3120
     )
    return true;
  else
    return false;
}

bool
Utils::isInWaitingArea(Vector &pos) const{
  double x = pos.x;
  double y = pos.y;
  if((((900 >= x && x >= 800) || (1040 >= x && x >= 940)) && 1980>= y && y >= 1940) ||
        (940 >= x && x >= 900 && ((2080 >= y && y >= 1980) || (1940 >= y && y >= 1840))) ||
     (((1940 >= x && x >= 1840) || (2080 >= x && x >= 1980)) && 1980 >= y && y >= 1940) ||
     (1980 >= x && x >= 1940 && ((2080 >= y && y >= 1980) || (1940 >= y && y >=1840))) ||
     (((1940 >= x && x >= 1840) || (2080 >= x && x >= 1980)) && 3020 >= y && y >= 2980) ||
     (1980 >= x && x >= 1940 && ((3120 >= y && y >= 3020) || (2980 >= y && y >= 2880))) ||
     (((1940 >= x && x >= 1840) || (2080 >= x && x >= 1980)) && 940 >= y && y >= 900) ||
     (1980 >= x && x >= 1940 && ((1040 >= y && y >= 940) || (900 >= y && y >= 800))) ||
     (((2980 >= x && x >= 2880) || (3120 >= x && x >= 3020)) && 1980 >= y && y >= 1940) ||
     (3020 >= x && x >= 2980 && ((2080 >= y && y >= 1980) || (1940 >= y && y >=1840)))
     )
    return true;
  else
    return false;
}


/**
 * @brief Utils::travelTime
 * @param object
 * @param nextWaiting
 * @return
 * enter the waiting area from idle area
 */
double
Utils::travelTime(Vector &pos, const int nextWaiting, const double speed) const{

  //vehicles are in the idle area
  if(isInIdleState (pos)) {

      double time = 0.0;

      //change in the x direction
      if(nextWaiting%2 == 0) {
          switch (nextWaiting) {
            case 12:
            case 32:
            case 52:
              time = fabs(2080 - pos.x) / speed;
              pos.x = 2080;
              break;
            case 14:
            case 34:
            case 54:
              time = fabs(1840 - pos.x) / speed;
              pos.x = 1840;
              break;
            case 22:
              time = fabs(3120 - pos.x) / speed;
              pos.x = 3120;
              break;
            case 24:
              time = fabs( 2880 - pos.x) / speed;
              pos.x = 2880;
              break;
            case 42:
              time = fabs( 1040 - pos.x) /speed;
              pos.x = 1040;
              break;
            case 44:
              time = fabs( 800 - pos.x) /speed;
              pos.x = 800;
              break;
            default:
              std::cout << "judge error in TravelTime from class Util";
              break;
            }
        }
      else {
          switch (nextWaiting) {
            case 11:
              time = fabs(3120 - pos.y) / speed;
              pos.y = 3120;
              break;
            case 13:
              time = fabs(2880 - pos.y) /speed;
              pos.y = 2880;
            case 21:
            case 41:
            case 51:
              time = fabs( 2080 - pos.y) /speed;
              pos.y = 2080;
              break;
            case 23:
            case 43:
            case 53:
              time = fabs( 1840 - pos.y) / speed;
              pos.y = 1840;
              break;
            case 31:
              time = fabs( 1040 - pos.y) / speed;
              pos.y = 1040;
              break;
            case 33:
              time = fabs( 800 - pos.y) / speed;
              pos.y = 800;
              break;
            default:
              std::cout << "judge error in TravelTime from class Util";
              break;
            }
        }
      return time;
    }
  return -1;
}


 bool
 Utils::isEnteringWaitingArea(Vector &pos) const {
   double x = pos.x;
   double y = pos.y;

   if(doubleEquals(x,800) || doubleEquals(x, 1040) || doubleEquals(x,1840) || doubleEquals(x,2080) || doubleEquals(x,2880) || doubleEquals(x, 3120)
      || doubleEquals(y,800) || doubleEquals(y,1040) || doubleEquals(y,1840) || doubleEquals(y, 2080) || doubleEquals(y,2880) || doubleEquals(y,3120))
     return true;
   else
     return false;
 }

 uint32_t
 Utils::whichIntersection (Vector &pos) const {
   double x = pos.x;
   double y = pos.y;

   uint32_t inter = 8;

   if(3120 >= y && y >= 2880 && 2080 >= x && x >= 1840)
     inter = 1;
   else if(2080 >= y && y >= 1840 && 3120 >= x && x >= 2880)
     inter = 2;
   else if(1040 >= y && y >= 800 && 2080 >= x && x >= 1840)
     inter = 3;
   else if(2080 >= y && y >= 1840 && 1040 >= x && x >= 800)
     inter = 4;
   else if(2080 >= y && y >= 1840 && 2080 >= x && x >= 1840)
     inter = 5;
   return inter;
 }

 double
 Utils::predictTime(uint32_t lane, uint32_t inter, double x, double y) {
   double time;
   if(lane == 0 || lane == 2 || lane == 4 || lane ==6)
     time = go_straight;
   else
     time = turn_left;
   switch (inter) {
     case 1:
       if(lane == 0) {
           time += (y - 3020.0)/speed_waiting;
         }
       else if(lane == 3) {
           time += (x - 1980.0)/speed_waiting;
         }
       else
         return 0;
       break;
     case 2:
       if(lane == 3) {
           time += (x - 3020.0)/speed_waiting;
         }
       else if(lane == 5) {
           time += (2980.0 - y)/speed_waiting;
         }
       else
         return 0;
       break;
     case 3:
       if(lane == 4) {
           time += (900.0 - y)/speed_waiting;
         }
       else if(lane == 7) {
           time += (1940.0 - x)/speed_waiting;
         }
       else
         return 0;
       break;
     case 4:
       if(lane == 1) {
           time += (y - 1980.0)/speed_waiting;
         }
       else if(lane == 6) {
           time += (900.0 - x)/speed_waiting;
         }
       else
         return 0;
       break;
     case 5:
       if(lane == 0 || lane == 1) {
           time += (y - 1980.0)/speed_waiting;
         }
       else if(lane == 2 || lane == 3) {
           time += (x - 1980.0)/speed_waiting;
         }
       else if(lane == 4 || lane == 5) {
           time += (1940.0 - y)/speed_waiting;
         }
       else if(lane == 6 || lane == 7) {
           time += (1940.0 - x)/speed_waiting;
         }
       else
         return 0;
       break;
     default:
       return 0;
       break;
     }
   time += (100.0 + 800.0)/speed_idle;  //waiting area + idle area
   return time;
 }

 /**
  * @brief Utils::getPosition
  * @param lane
  * @param inter_id
  * @param count
  * @param x
  * @param y
  * get the coordinate of new comer when there are "count" vehicles ahead of it.
  */
 void
 Utils::getPosition(uint32_t lane,uint32_t inter_id,uint32_t count,double &x,double &y) {
   double queue_length = count * space_occupy;
   switch (inter_id) {
     case 1:
       switch (lane) {
         case 0:
         case 1:
           y = 3020 + queue_length;
           break;
         case 2:
         case 3:
           x = 1980 + queue_length;
           break;
         case 4:
         case 5:
           y = 2980 - queue_length;
           break;
         case 6:
         case 7:
           x = 1940 - queue_length;
           break;
         default:
           break;
         }
       break;
     case 2:
       switch (lane) {
         case 0:
         case 1:
           y = 1980 + queue_length;
           break;
         case 2:
         case 3:
           x = 3020 + queue_length;
           break;
         case 4:
         case 5:
           y = 1940 - queue_length;
           break;
         case 6:
         case 7:
           x = 2980 - queue_length;
           break;
         default:
           break;
         }
       break;
     case 3:
       switch (lane) {
         case 0:
         case 1:
           y = 940 + queue_length;
           break;
         case 2:
         case 3:
           x = 1980 + queue_length;
           break;
         case 4:
         case 5:
           y = 900 - queue_length;
           break;
         case 6:
         case 7:
           x = 1940 - queue_length;
           break;
         default:
           break;
         }
       break;
     case 4:
       switch (lane) {
         case 0:
         case 1:
           y = 1980 + queue_length;
           break;
         case 2:
         case 3:
           x = 940 + queue_length;
           break;
         case 4:
         case 5:
           y = 1940 - queue_length;
           break;
         case 6:
         case 7:
           x = 900 - queue_length;
           break;
         default:
           break;
         }
       break;
     case 5:
       switch (lane) {
         case 0:
         case 1:
           y = 1980 + queue_length;
           break;
         case 2:
         case 3:
           x = 1980 + queue_length;
           break;
         case 4:
         case 5:
           y = 1940 - queue_length;
           break;
         case 6:
         case 7:
           x = 1940 - queue_length;
           break;
         default:
           break;
         }
       break;
     default:
       break;
     }
 }


 /**
  * @brief Utils::passIntersectionWait
  * @param lane
  * @param inter_id
  * @param x
  * @param y
  * @return the time of passing waiting area
  */
 double
 Utils::passIntersectionWait (uint32_t lane, uint32_t inter_id, double &x, double &y)
 {
   double time = 0.0;
   switch (inter_id) {
     case 1:
       switch (lane) {
         case 0:
         case 1:
           time = (y - 3020.0) / speed_waiting;
           y = 3020.0;
           break;
         case 2:
         case 3:
           time = (x - 1980.0) / speed_waiting;
           x = 1980.0;
           break;
         case 4:
         case 5:
           time = (2980.0 - y) / speed_waiting;
           y = 2980.0;
           break;
         case 6:
         case 7:
           time = (1940.0 - x) / speed_waiting;
           x = 1940.0;
           break;
         default:
           break;
         }
       break;
     case 2:
       switch (lane) {
         case 0:
         case 1:
           time = (y - 1980.0) / speed_waiting;
           y = 1980.0;
           break;
         case 2:
         case 3:
           time = (x - 3020.0) / speed_waiting;
           x = 3020.0;
           break;
         case 4:
         case 5:
           time = (1940.0 - y) / speed_waiting;
           y = 1940.0;
           break;
         case 6:
         case 7:
           time = (2980.0 - x) / speed_waiting;
           x = 2980.0;
           break;
         default:
           break;
         }
       break;
     case 3:
       switch (lane) {
         case 0:
         case 1:
           time = (y - 940.0) / speed_waiting;
           y = 940.0;
           break;
         case 2:
         case 3:
           time = (x - 1980.0) / speed_waiting;
           x = 1980.0;
           break;
         case 4:
         case 5:
           time = (900.0 - y) / speed_waiting;
           y = 900.0;
           break;
         case 6:
         case 7:
           time = (1940.0 - x) / speed_waiting;
           x = 1940.0;
           break;
         default:
           break;
         }
       break;
     case 4:
       switch (lane) {
         case 0:
         case 1:
           time = (y - 1980.0) / speed_waiting;
           y = 1980.0;
           break;
         case 2:
         case 3:
           time = (x - 940.0) / speed_waiting;
           x = 940.0;
           break;
         case 4:
         case 5:
           time = (1940.0 - y) / speed_waiting;
           y = 1940.0;
           break;
         case 6:
         case 7:
           time = (900.0 - x) / speed_waiting;
           x = 900.0;
           break;
         default:
           break;
         }
       break;
     case 5:
       switch (lane) {
         case 0:
         case 1:
           time = (y - 1980.0) / speed_waiting;
           y = 1980.0;
           break;
         case 2:
         case 3:
           time = (x - 1980.0) / speed_waiting;
           x = 1980.0;
           break;
         case 4:
         case 5:
           time = (1940.0 - y) / speed_waiting;
           y = 1940.0;
           break;
         case 6:
         case 7:
           time = (1940.0 - x) / speed_waiting;
           x = 1940.0;
           break;
         default:
           break;
         }
       break;
     default:
       break;
     }
   return time;
 }


 /**
  * @brief Utils::passIntersectionCore
  * @param lane
  * @param inter_id
  * @param x
  * @param y
  * @return the time of passing core area
  */
 double
 Utils::passIntersectionCore (uint32_t lane, uint32_t inter_id, double &x, double &y)
 {
   double time;
   switch (lane) {
     case 0:
       switch (inter_id) {
         case 1:
          y = 2980.0;
          break;
         case 2:
         case 4:
         case 5:
          y = 1940.0;
          break;
         case 3:
          y = 900.0;
          break;
         default:
           break;
         }
       time = go_straight;
       break;
     case 1:
       switch (inter_id) {
         case 1:
           x = 1980.0;
           y = 2990.0;
           break;
         case 2:
           x = 3020.0;
           y = 1950.0;
           break;
         case 3:
           x = 1980.0;
           y = 910.0;
           break;
         case 4:
           x = 940.0;
           y = 1950.0;
           break;
         case 5:
           x = 1980.0;
           y = 1950.0;
           break;
         default:
           break;
         }
        time = turn_left;
       break;
     case 2:
       switch (inter_id) {
         case 1:
         case 3:
         case 5:
           x = 1940.0;
           break;
         case 2:
           x = 2980.0;
           break;
         case 4:
           x = 900.0;
           break;
         default:
           break;
         }
       time = go_straight;
       break;
     case 3:
       switch (inter_id) {
         case 1:
           x = 1950.0;
           y = 2980.0;
           break;
         case 2:
           x = 2990.0;
           y = 1940.0;
           break;
         case 3:
           x = 1950.0;
           y = 900.0;
           break;
         case 4:
           x = 910.0;
           y = 1940.0;
           break;
         case 5:
           x = 1950.0;
           y = 1940.0;
           break;
         default:
           break;
         }
       time = turn_left;
       break;
     case 4:
       switch (inter_id) {
         case 1:
           y = 3020.0;
           break;
         case 2:
         case 4:
         case 5:
           y = 1980.0;
           break;
         case 3:
           y = 940.0;
           break;
         default:
           break;
         }
       time = go_straight;
       break;
     case 5:
       switch (inter_id) {
         case 1:
           x = 1940.0;
           y = 3010.0;
           break;
         case 2:
           x = 2980.0;
           y = 1970.0;
           break;
         case 3:
           x = 1940.0;
           y = 930.0;
           break;
         case 4:
           x = 900.0;
           y = 1970.0;
           break;
         case 5:
           x = 1940.0;
           y = 1970.0;
           break;
         default:
           break;
         }
       time = turn_left;
       break;
     case 6:
       switch (inter_id) {
         case 1:
         case 3:
         case 5:
           x = 1980.0;
           break;
         case 2:
           x = 3020.0;
           break;
         case 4:
           x = 940.0;
           break;
         default:
           break;
         }
       time = go_straight;
       break;
     case 7:
       switch (inter_id) {
         case 1:
           x = 1970.0;
           y = 3020.0;
           break;
         case 2:
           x = 3010.0;
           y = 1980.0;
           break;
         case 3:
           x = 1970.0;
           y = 940.0;
           break;
         case 4:
           x = 930.0;
           y = 1980.0;
           break;
         case 5:
           x = 1970.0;
           y = 1980.0;
           break;
         default:
           break;
         }
       time = turn_left;
       break;
     default:
       break;
     }
   return time;
 }

 double
 Utils::passIntersectionIdle (uint32_t lane, uint32_t inter_id, double &x, double &y)
 {
   double time = 900.0 / speed_idle;
   switch (inter_id) {
     case 1:
       switch (lane) {
         case 0:
         case 3:
           y = 2080.0;
           break;
         case 1:
         case 6:
           x = 2880.0;
           break;
         case 2:
         case 5:
           x = 1040.0;
           break;
         case 4:
         case 7:
           y = 3920.0;
           break;
         default:
           break;
         }
       break;
     case 2:
       switch (lane) {
         case 0:
         case 3:
           y = 1040.0;
           break;
         case 1:
         case 6:
           x = 3920.0;
           break;
         case 2:
         case 5:
           x = 2080.0;
           break;
         case 4:
         case 7:
           y = 2880.0;
           break;
         default:
           break;
         }
       break;
     case 3:
       switch (lane) {
         case 0:
         case 3:
           y = 0.0;
           break;
         case 1:
         case 6:
           x = 2880.0;
           break;
         case 2:
         case 5:
           x = 1040.0;
           break;
         case 4:
         case 7:
           y = 1840.0;
           break;
         default:
           break;
         }
       break;
     case 4:
       switch (lane) {
         case 0:
         case 3:
           y = 1040.0;
           break;
         case 1:
         case 6:
           x = 1840.0;
           break;
         case 2:
         case 5:
           x = 0.0;
           break;
         case 4:
         case 7:
           y = 2880.0;
           break;
         default:
           break;
         }
       break;
     case 5:
       switch (lane) {
         case 0:
         case 3:
           y = 1040.0;
           break;
         case 1:
         case 6:
           x = 2880.0;
           break;
         case 2:
         case 5:
           x = 1040.0;
           break;
         case 4:
         case 7:
           y = 2880.0;
           break;
         default:
           break;
         }
       break;
     default:
       break;
     }
   return time;
 }

 uint32_t
 Utils::nextIntersection (uint32_t lane, uint32_t inter_id) {
   uint32_t next_inter = 8;
   switch (inter_id) {
     case 1:
       switch (lane) {
         case 0:
         case 3:
           next_inter = 5;
           break;
         default:
           next_inter = 8;
           break;
         }
       break;
     case 2:
       switch (lane) {
         case 2:
         case 5:
           next_inter = 5;
           break;
         default:
           next_inter = 8;
           break;
         }
       break;
     case 3:
       switch (lane) {
         case 4:
         case 7:
           next_inter = 5;
           break;
         default:
           next_inter = 8;
           break;
         }
       break;
     case 4:
       switch (lane) {
         case 6:
         case 1:
           next_inter = 5;
           break;
         default:
           next_inter = 8;
           break;
         }
       break;
     case 5:
       switch (lane) {
         case 0:
         case 3:
           next_inter = 3;
           break;
         case 1:
         case 6:
           next_inter = 2;
           break;
         case 2:
         case 5:
           next_inter = 4;
           break;
         case 4:
         case 7:
           next_inter = 7;
           break;
         default:
           next_inter = 8;
           break;
         }
       break;
     default:
       break;
     }
   return next_inter;
 }

 bool
 Utils::doubleEquals(double a,double b) const
 {
   if (fabs(a-b) <= 1e-6)
     return true;
  return false;
 }


 /**
 * @brief Utils::whichIntersection
 * @param next the id of the intersection and waiting area
 * @return
 */
uint32_t
Utils::whichIntersection (uint32_t next) const {
  return next/10;
}

uint32_t
Utils::whichWaitingArea (uint32_t next) const {
  return next%10;
}

uint32_t
Utils::whichLane(uint32_t enterArea, uint32_t leavingArea) {
  if(enterArea == 1 && leavingArea == 3)
    return 0;
  else if(enterArea == 1 && leavingArea == 2)
    return 1;
  else if(enterArea == 2 && leavingArea == 4)
    return 2;
  else if(enterArea == 2 && leavingArea == 3)
    return 3;
  else if(enterArea == 3 && leavingArea == 1)
    return 4;
  else if(enterArea == 3 && leavingArea == 4)
    return 5;
  else if(enterArea == 4 && leavingArea == 2)
    return 6;
  else if(enterArea == 4 && leavingArea == 1)
    return 7;
  else
    return 8;  //means error
}


/*********************************************************************************************************/


class intersection : public Application
{
public:

          intersection ();
  virtual ~intersection();

          void HandleRead (Ptr<Socket> socket);
          void Start(void);
          void SetUp(Ipv4InterfaceContainer csmaInterface, uint32_t inter_id);
          std::vector<rp>& getPending();

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);
          static void recvCallback(Ptr<Socket> sock);
          bool updateRP (uint32_t vehicle, uint32_t lane, uint32_t next_lane, uint32_t next_inter, double x, double y, bool &last);
          void sendPermit (Ipv4Address &address, uint32_t vehicle, uint32_t lane, bool last);
          void sendPermit (uint32_t lane, uint32_t num);
          void sendPredict (uint32_t veh_id, uint32_t lane_id, uint32_t next_lane, uint32_t next_inter, double x, double y);
          void sendPredict( uint32_t lane, uint32_t num, uint32_t next_inter);
          bool needPredict(Ipv4Address &address, uint32_t next_inter);
          void setSendSocket(Ipv4Address &address, uint16_t port);
          void removeVehicle(uint32_t lane, uint32_t num);
          uint32_t  decideNum(uint32_t lane);


  Ptr<Socket> m_socket_accept;  //the address for receiving packets from vehicles
  Ptr<Socket> m_socket_send;   //the address for sending packets
  uint16_t m_port_accept; //the port for receiving packets from vehicles
  uint16_t m_port_send; //the port for sending vehicles
  bool s_lock[8]; // locks for lanes
  uint32_t m_pass;  //how many vehicles have passed from this intersection
  uint32_t s_packetSize;  //the size of packet
  Utils util; //tools
  std::vector<rp> pending;
  std::vector<pos_unit> pos_helper;
  uint32_t intersection_id;  //the id of intersection
  Ipv4InterfaceContainer controllerAddress; //the container for addresses of vehicles
};



/***********************************************************************************************************/


intersection::intersection ()
{
    m_socket_accept = 0;
    m_socket_send=0;
    m_port_accept = 1024;
    m_port_send = 9;
    s_packetSize = 128;
    m_pass = 0;

    for(int i = 0;i < 8;i++)
        s_lock[i] = false;  //init the locks.
}

intersection::~intersection()
{
  m_socket_accept = 0;
  m_socket_send = 0;
}

void
intersection::StartApplication (void)
{
//  NS_LOG_FUNCTION (this);

  if (m_socket_accept == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket_accept = Socket::CreateSocket (GetNode (), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), m_port_accept);
      m_socket_accept->Bind (local);
      std::cout << "intersection " << intersection_id << " set accept address" << std::endl;
    }
  m_socket_accept->SetRecvCallback (MakeCallback (&intersection::HandleRead, this));
}

std::vector<rp>&
intersection::getPending() {
  return pending;
}

void
intersection::StopApplication ()
{
   if (m_socket_accept != 0)
    {
      m_socket_accept->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
}

void
intersection::SetUp (Ipv4InterfaceContainer csmaInterface, uint32_t inter_id) {
  controllerAddress = csmaInterface;
  this->intersection_id = inter_id;
}

void
intersection::HandleRead (Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    std::cout << "enter handle read function" << std::endl;

    while (packet = socket->RecvFrom (from))
      {
        if (InetSocketAddress::IsMatchingType (from))
          {
            std::cout << Simulator::Now ().GetSeconds () << "s"<< std::endl;
            std::cout<< "Controller " << intersection_id << " received " << packet->GetSize () << " bytes from " <<
                         InetSocketAddress::ConvertFrom (from).GetIpv4 () <<std::endl;

            Ipv4Address address = InetSocketAddress::ConvertFrom (from).GetIpv4 ();

            uint32_t messageType; // 1 represents request, 2 represents unlock, 3 represents predict, 4 represents permit

            SeqTsHeader seqTss;
            packet->RemoveHeader (seqTss);
            messageType = seqTss.GetSeq ();

            switch (messageType) {
              case 1: //request
                {
                uint32_t vehID, laneID, nextLaneID, nextInterID, x1, x2, y1, y2;
                bool last = true;  //if it is the last predicted vehicle

                packet->RemoveHeader (seqTss);
                vehID = seqTss.GetSeq ();
                packet->RemoveHeader (seqTss);
                laneID = seqTss.GetSeq ();
                packet->RemoveHeader (seqTss);
                nextLaneID = seqTss.GetSeq (); //id of lane in next intersection
                packet->RemoveHeader (seqTss);
                nextInterID = seqTss.GetSeq (); //id of next intersection
                packet->RemoveHeader (seqTss);
                x1= seqTss.GetSeq ();
                packet->RemoveHeader (seqTss);
                x2 = seqTss.GetSeq ();
                packet->RemoveHeader (seqTss);
                y1 = seqTss.GetSeq ();
                packet->RemoveHeader (seqTss);
                y2 = seqTss.GetSeq ();
                packet->RemoveAllPacketTags ();
                packet->RemoveAllByteTags ();

                double x = x2, y = y2;
                x /= 1000.0;  //the client multiple it with 1000.0
                x += x1;
                y /= 1000.0;
                y += y1;

                std::cout << "|||||||||||||||||| vehicle " << vehID << "(" << x << "," << y << ")" << std::endl;

                bool passing = updateRP(vehID, laneID, nextLaneID, nextInterID, x, y,last);    //update the request pending list
                if(passing) {
                    sendPermit(address, vehID, laneID, last); //scheduled to pass the intersection
                    sendPredict (vehID, laneID, nextLaneID, nextInterID, x, y);
                  }
                if(laneID == 0 || laneID == 2 || laneID == 4 || laneID == 6) //try to lock corresponding lanes
                  {
                    if(!s_lock[laneID] && !s_lock[(laneID+8-1)%8] && !s_lock[(laneID+8-2)%8])
                      {
                        std::cout << "notice this is an test" << std::endl;
                        s_lock[laneID] = true;    //lock corresponding lanes
                        s_lock[(laneID+8-1)%8] = true;
                        s_lock[(laneID+8-2)%8] = true;

                        uint32_t num = decideNum (laneID);
                        sendPermit(laneID, num);
                        sendPredict(laneID, num, nextInterID);
                        removeVehicle(laneID, num); //remove vehicles which are passing
                      }
                  }
                if(laneID == 1 || laneID == 3 || laneID == 5 || laneID == 7)
                  {
                    if(!s_lock[laneID] && !s_lock[(laneID+1)%8] && !s_lock[(laneID+2)%8])
                      {
                        s_lock[laneID] = true;
                        s_lock[(laneID+1)%8] = true;
                        s_lock[(laneID+2)%8] = true;

                        uint32_t num = decideNum (laneID);  //how many vehicles are allowed to pass the intersection
                        sendPermit(laneID, num);
                        sendPredict(laneID, num, nextInterID);
                        removeVehicle(laneID, num); //remove vehicles which are passing
                      }
                  }
                }
                break;
              case 2:   //unlock
                {
                std::cout << "recieve unlock message" << std::endl;
                uint32_t lane_num;
                packet->RemoveHeader (seqTss);
                lane_num = seqTss.GetSeq ();
                packet->RemoveAllPacketTags ();
                packet->RemoveAllByteTags ();

                if(lane_num == 0 || lane_num == 2 || lane_num == 4 || lane_num == 6) {
                    s_lock[lane_num] = false;    //lock corresponding lanes
                    s_lock[(lane_num+8-1)%8] = false;
                    s_lock[(lane_num+8-2)%8] = false;
                  }
                if(lane_num == 1 || lane_num == 3 || lane_num == 5 || lane_num == 7) {
                    s_lock[lane_num] = false;
                    s_lock[(lane_num+1)%8] = false;
                    s_lock[(lane_num+2)%8] = false;
                  }
                if(!pending.empty ()) {
                    uint32_t pass_lane = pending[0].lane_id;
                    std::cout << "pass lane is " << pass_lane  << "   " <<pending[0].vehicle_id << std::endl;
                    uint32_t num = decideNum (pass_lane);  //how many vehicles are allowed to pass the intersection
                    sendPermit(pass_lane, num);
                    uint32_t next_inter = util.nextIntersection(pass_lane, intersection_id);
                    sendPredict(pass_lane, num, next_inter);
                    removeVehicle(pass_lane, num); //remove vehicles which are passing
                  }
                }
                std::cout << "end of unlock" << std::endl;
                break;
              case 3: //predict
                {
                uint32_t t1, t2, ve_id,la_id, pre_num;
                double time = 0.0;
                packet->RemoveHeader (seqTss);
                pre_num = seqTss.GetSeq ();

                while(pre_num != 0) {
                    packet->RemoveHeader (seqTss);
                    ve_id = seqTss.GetSeq ();
                    packet->RemoveHeader (seqTss);
                    la_id = seqTss.GetSeq ();
                    packet->RemoveHeader (seqTss);
                    t1 = seqTss.GetSeq ();
                    packet->RemoveHeader (seqTss);
                    t2 = seqTss.GetSeq ();
                    time = double(t2)/1000.0;
                    time += t1;
                    rp predict_vehile = {ve_id, la_id, 8, 8, false, time, 0, 0};
                    pending.push_back (predict_vehile);
                    pre_num--;
                    std::cout << "lane " << la_id << ", vehicle "<< ve_id << " are predicted to arrive at " << time << std::endl;
                    time = 0;
                  }

                packet->RemoveAllPacketTags ();
                packet->RemoveAllByteTags ();
                }
                break;
              default:
                std::cout << "message type error" << std::endl;
                break;
              }
          }
      }
}

/**
 * @brief intersection::updateRP
 * @param vehicle
 * @param lane
 * @return "true" means whether the vehicle has been scheduled to pass, the vehicle is removed from rp list
 */
bool
intersection::updateRP(uint32_t vehicle, uint32_t lane, uint32_t next_lane, uint32_t next_inter, double x, double y, bool &last) {
  std::cout << "update request pending list" << std::endl;
  std::cout << "Controller " << intersection_id << " recieve a request message from vehicle " << vehicle << " on lane " << lane << std::endl;
  bool recorded = false; //the vehicle has been predicted
  std::vector<rp>::iterator it;
  for(it = pending.begin (); it != pending.end (); it++) {
      if(vehicle == it->vehicle_id) {
            if(it->pass) {  //the vehicle has been scheduled to pass the intersection
                pending.erase (it); //remove vehicle from pending list
                for(std::vector<rp>::iterator tmp = pending.begin (); tmp != pending.end (); tmp++) {
                    if(tmp->lane_id == lane && tmp->pass) {  //the lane is same and the state is passing
                        last = false;
                      }
                  }
                return true;
              }
            recorded = true; //recorded
            it->next_lane_id = next_lane;
            it->next_inter_id = next_inter;
            it->time = 0; //0 means the vehicle has arrived at this intersection
            break;
        }
    }

  if(!recorded) // vehicle has not been recorded and scheduled
    {
      rp tmp = {vehicle, lane, next_lane, next_inter, false, 0, x, y};
      std::cout << "push back stack " << vehicle << "  " << lane << std::endl;
      pending.push_back (tmp);
    }


  //calculate how many vehicles are ahead of the newcomer
  uint32_t count = 0;
  for(it = pending.begin (); it != pending.end (); it++) {
      if(it->lane_id == lane && util.doubleEquals (it->time, 0) && it->vehicle_id != vehicle) {
          count++;
        }
    }
  //set the coordinate of vehicle
  util.getPosition(lane, intersection_id, count, x, y);
  for(it = pending.begin (); it != pending.end () && it->vehicle_id == vehicle; it++) {
      it->x = x;
      it->y = y;
    }
  return false;
}

bool
intersection::updatePosHelper(uint32_t veh_id, uint32_t lane, double time_left) {
  uint8_t direction;
  double now = Simulator::Now ().GetSeconds ();
  switch (lane) {
    case 0:
    case 3:
      direction = 1;    //y-
      break;
    case 1:
    case 6:
      direction = 2;    //x+
      break;
    case 2:
    case 5:
      direction = 3;    //x-
      break;
    case 4:
    case 7:
      direction = 4;    //y+
      break;
    default:
      break;
    }
  pos_unit unit = {veh_id, direction, now, time_left};
  pos_helper.push_back (unit);
}

/**
 * @brief intersection::sendPermit
 * @param address
 * @param vehicle
 * send permit message to single vehicle
 */
void
intersection::sendPermit(Ipv4Address &address, uint32_t vehicle, uint32_t lane, bool last) {
  setSendSocket(address, m_port_send);
  Ptr<Packet> packet = Create<Packet> (s_packetSize);
  SeqTsHeader seqTss;

  seqTss.SetSeq(vehicle); //the vehicle
  packet->AddHeader (seqTss);
  seqTss.SetSeq(1); //the number of vehicles
  packet->AddHeader (seqTss);
  if(last) {
      seqTss.SetSeq(vehicle);     //the vehicle for unlocking
      packet->AddHeader (seqTss);
    }
  else {
      uint32_t last_id;
      for(std::vector<rp>::iterator it = pending.begin (); it != pending.end (); it++) {
          if(lane == it->lane_id && it->pass) { //on the same lane, and haven't arrived at the waiting area
              last_id = it->vehicle_id;
            }
        }
      seqTss.SetSeq(last_id);     //the vehicle for unlocking
      packet->AddHeader (seqTss);
    }
  seqTss.SetSeq(4);                //4 means passing is allowed.
  packet->AddHeader (seqTss);
  m_socket_send->Send(packet);

  m_pass++;
  std::cout << Simulator::Now ().GetSeconds () << "s"<< std::endl;
  std::cout<<"the vehicle "<< vehicle <<" also Aallowed to pass and no cars following"<<std::endl;
  std::cout << "-------------- " << m_pass << " vehicles have passed the intersection" << std::endl;
}

/**
 * @brief intersection::sendPermit
 * @param vehicle
 * @param num
 * broadcast permit message to a number of vehicls
 */
void
intersection::sendPermit ( uint32_t lane, uint32_t num) {
  Ipv4Address broadcastAdd = Ipv4Address("255.255.255.255");
  setSendSocket(broadcastAdd, m_port_send);
  Ptr<Packet> packet = Create<Packet> (s_packetSize);
  SeqTsHeader seqTss;

  uint32_t count = num;
  uint32_t last_vehicle;  //responsible for unlocking
  std::vector<rp>::iterator it;

  for(it = pending.begin (); it != pending.end () && num > 0; it++) {
      if(it->lane_id == lane && util.doubleEquals (it->time, 0.0))  //some vehicles may appear in accident
        {

          seqTss.SetSeq (it->vehicle_id);
          packet->AddHeader (seqTss);
          it->pass = true;
          last_vehicle = it->vehicle_id;
          num--;
          m_pass++;
        }
    }


  seqTss.SetSeq(count - num); //the number of vehicles
  packet->AddHeader (seqTss);

  for(it = pending.begin (); it != pending.end () && num > 0; it++) {
      if(it->lane_id == lane && !(util.doubleEquals (it->time, 0.0))) //some vehicles may be the predicted vehicles
        {
          std::cout << "have predicted vehicles" << std::endl;
          it->pass = true;  //this means vehicles are scheduled to pass the intersection
          last_vehicle = it->vehicle_id;
          num--;
        }
    }
  seqTss.SetSeq(last_vehicle);     //the vehicle for unlocking
  packet->AddHeader (seqTss);
  seqTss.SetSeq(4);                //4 means passing is allowed.
  packet->AddHeader (seqTss);

  m_socket_send->Send(packet);

  std::cout << "on lane " << lane  << ", " << count << " vehicles are allowed to pass " << std::endl;
  std::cout << "at intersection " << intersection_id << " , " << m_pass << " have been allowed to pass the intersection" << std::endl;
}

/**
 * @brief intersection::sendPredict
 * @param veh_id
 * @param next_lane
 * for a single vehicle
 */
void
intersection::sendPredict (uint32_t veh_id, uint32_t lane_id, uint32_t next_lane, uint32_t next_inter, double x, double y) {
  Ipv4Address address;
  bool flag = needPredict (address, next_inter);    //some lanes may not need to be predicted

  if(flag) {
      setSendSocket(address, m_port_accept);
      Ptr<Packet> packet = Create<Packet> (s_packetSize);
      SeqTsHeader seqTss;

      //transport time separatly, for example, 1.23s, 1 is t1, 230 is t2.
      double time = util.predictTime (lane_id, intersection_id, x, y);
      time += Simulator::Now ().GetSeconds ();
      uint32_t t1 = time;
      time -= (double)t1;
      uint32_t t2 = time * 1000.0;
      seqTss.SetSeq (t2);
      packet->AddHeader (seqTss);
      seqTss.SetSeq (t1);
      packet->AddHeader (seqTss);

      std::cout << "=========================================================================" << t1 << std::endl;

      seqTss.SetSeq (next_lane);
      packet->AddHeader (seqTss);

      seqTss.SetSeq (veh_id);
      packet->AddHeader (seqTss);

      seqTss.SetSeq (1);  //only one vehicle
      packet->AddHeader (seqTss);

      seqTss.SetSeq (3);  //message type: predict
      packet->AddHeader (seqTss);

      m_socket_send->Send (packet);

      std::cout << "controller" << intersection_id << " send predict vehicle id" << veh_id << std::endl;
    }
}

/**
 * @brief intersection::sendPredict
 * @param lane
 * @param next_lane
 * @param num
 * for a number of vehicles
 */
void
intersection::sendPredict( uint32_t lane, uint32_t num, uint32_t next_inter) {
  std::cout << "vehicles at lane " << lane << " will start to pass the intersection" << std::endl;
  Ipv4Address address;
  bool flag = needPredict (address, next_inter);  //need to send predictive message?

  if(flag)
    {
      setSendSocket(address,m_port_accept);
      Ptr<Packet> packet = Create<Packet> (s_packetSize);
      SeqTsHeader seqTss;
      std::vector<rp>::iterator it;
      uint32_t count = num;

      for(it = pending.begin (); it != pending.end () && num > 0; it++) {
          if(it->lane_id == lane && util.doubleEquals (it->time, 0.0))  //some vehicles may appear in accident
            {

              double time = util.predictTime (lane, intersection_id, it->x, it->y);
              std::cout << it->y << "=========================================================================" << time << std::endl;
              time += Simulator::Now ().GetSeconds ();
              uint32_t t1 = time;
              time -= (double)t1;
              uint32_t t2 = time * 1000.0;
              seqTss.SetSeq (t2);
              packet->AddHeader (seqTss);
              seqTss.SetSeq (t1);
              packet->AddHeader (seqTss);
              std::cout << "=========================================================================" << t1 << std::endl;

              seqTss.SetSeq (it->next_lane_id);
              packet->AddHeader (seqTss);
              seqTss.SetSeq (it->vehicle_id);
              packet->AddHeader (seqTss);
              num--;
              std::cout << "$$$$ " <<  it->vehicle_id << std::endl;
            }
        }

      seqTss.SetSeq(count - num); //the number of vehicles
      packet->AddHeader (seqTss);
      seqTss.SetSeq (3);  //predict message
      packet->AddHeader (seqTss);

      m_socket_send->Send (packet);
    }
}

/**
 * @brief intersection::needPredict
 * @param address   the ip of next controller
 * @param lane  the lane of controller
 * @return
 */
bool
intersection::needPredict(Ipv4Address &address, uint32_t next_inter) {
  std::cout << "next intersection " << next_inter << " need predict"  << std::endl;
  if(next_inter != 8) //8 means out of these five intersections
    {
      address = controllerAddress.GetAddress (next_inter - 1);
      return true;
    }
  else
    return false;
}

/**
 * @brief intersection::removeVehicle
 * @param lane
 * @param num
 * remove vehicles from rp list
 */
void
intersection::removeVehicle(uint32_t lane, uint32_t num) {
  std::vector<rp>::iterator it;
  for(it = pending.begin (); it!= pending.end ();) {
      if(it->lane_id == lane && it->pass && util.doubleEquals (it->time, 0) && num > 0)
        {
          pending.erase (it);
          num--;
        }
      else
        it++;
    }
}

void
intersection::setSendSocket(Ipv4Address &address, uint16_t port) {
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  m_socket_send = Socket::CreateSocket(GetNode(), tid);
  m_socket_send->Bind();	//为套接字分配一个本地Ipv4终端
  m_socket_send->SetAllowBroadcast(true);	//允许广播
  m_socket_send->Connect(InetSocketAddress(address, port));	//初始化一个到远程主机的连接
}


/**
 * @brief intersection::decideNum decide how many vehicles on lane can pass the intersection
 * @param lane
 */
uint32_t
intersection::decideNum(uint32_t lane) {
   uint32_t count = 0;
   for(std::vector<rp>::iterator it = pending.begin (); it != pending.end (); it++) {
       if( it->lane_id == lane && it->time < (Simulator::Now ().GetSeconds () + 5) )
         count++;
     }
   return count;
}



/******************************************************************************************************************/
class vehicle : public Application
{
public:

  vehicle ();
  virtual ~vehicle();

  void Setup (const Ipv4InterfaceContainer &address,uint16_t port_send, uint32_t packetSize,
              DataRate dataRate, Ptr<WaypointMobilityModel> waypointModel, uint32_t v_id,
              std::vector<uint32_t> &traces, std::vector< Ptr<intersection> > &intersections_ptr);
  void sendRequest ();

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);
          void HandleRead (Ptr<Socket> socket);
          void sendUnlock ();
          static void recvCallback(Ptr<Socket> sock);
          void Begin(void);
          void setConnectSocket(uint32_t inter);
          void passIntersection();
          void pos_xy(uint32_t lane, uint32_t inter_id, double &x, double &y, std::vector< Ptr<intersection> > &intsect);





  Ptr<Socket>     m_socket_send;
  Ptr<Socket>     m_socket_accept;
  Ipv4InterfaceContainer     m_CSMAInterface;
  uint16_t        m_port_send;
  uint16_t        m_port_accept;
  uint32_t        m_packetSize;
  uint32_t        m_nPackets;
  DataRate        m_dataRate;
  uint32_t        vehicle_id;
  uint32_t        intersection_id;
  uint32_t        lane_id;
  std::vector<uint32_t>      paths;
  std::vector< Ptr<intersection> >   control_apps;
  uint32_t        path_index;
  Ptr<WaypointMobilityModel> mobility;
  bool            last;
  bool            begin;
  Utils           util;
  double          speed_waiting;
  double          speed_idle;
};



vehicle::vehicle ()
{
    m_socket_send = 0;
    m_socket_accept = 0;
    m_port_send = 1024;
    m_port_accept = 9;
    m_packetSize = 1024;
    m_nPackets = 0;
    m_dataRate = 0;
    intersection_id= 8; //init, there is no lane named 8
    lane_id = 8;  //init, there is no lane named 8
    path_index = 0;
    last = false;
    begin = true;
    speed_waiting = 12.0;
    speed_idle = 20.0;
}

vehicle::~vehicle ()
{
  m_socket_send = 0;
  m_socket_accept = 0;
}

void
vehicle::Setup(const Ipv4InterfaceContainer &address,uint16_t port_send, uint32_t packetSize, DataRate dataRate,
               Ptr<WaypointMobilityModel> waypointModel, uint32_t v_id, std::vector<uint32_t> &traces, std::vector< Ptr<intersection> > &intersections_ptr)
{
  m_CSMAInterface = address;
  m_port_send = port_send;
  m_packetSize = packetSize;
  m_dataRate = dataRate;
  mobility = waypointModel;
  vehicle_id = v_id;
  paths = traces;
  control_apps = intersections_ptr;
}

void
vehicle::StartApplication (void)
{
  Begin();
  if (intersection_id < 6)
    {
        setConnectSocket (intersection_id);

    }
}

void
vehicle::Begin(void)
{
  if (m_socket_accept == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket_accept = Socket::CreateSocket (GetNode (), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), m_port_accept);
      m_socket_accept->Bind (local);
      std::cout << "vehicle " << vehicle_id << " set the accept address" << std::endl;
    }
   m_socket_accept->SetRecvCallback (MakeCallback (&vehicle::HandleRead, this));
}

void
vehicle::StopApplication (void)
{
  if (m_socket_send)
    {
      m_socket_send->Close ();
    }
  if (m_socket_accept)
    {
      m_socket_accept->Close ();
    }
}

void
vehicle::HandleRead (Ptr<Socket> socket)
{
  std::cout << "vehicle " << vehicle_id << " recieve message from controller" << std::endl;
  Ptr<Packet> packet;
  Address from;
  while (packet = socket->RecvFrom (from))
    {
      if (InetSocketAddress::IsMatchingType (from))
        {
          SeqTsHeader seqTss;
          packet->RemoveHeader (seqTss);

          uint32_t m_type, m_last_id, pass_num, veh_id;

          m_type = seqTss.GetSeq ();

          if(m_type == 4) {
              packet->RemoveHeader (seqTss);
              m_last_id = seqTss.GetSeq ();

              if(m_last_id == vehicle_id) {
                  std::cout << "vehicle " << m_last_id << " is the last permitted vehicle in the list" << std::endl;
                  last = true;
                }
              packet->RemoveHeader (seqTss);
              pass_num = seqTss.GetSeq ();
              double t_last;
              while(pass_num) {
                  packet->RemoveHeader (seqTss);
                  veh_id = seqTss.GetSeq ();

                  if(veh_id == vehicle_id)  //the vehicle is in the perimit list
                    {
                      Vector pos = mobility->GetPosition ();

                      std::cout << Simulator::Now ().GetSeconds () << "s  (" << pos.x << "," << pos.y << ") "  << std::endl;
                      Waypoint next_point = mobility->GetNextWaypoint ();
                      std::cout << next_point.position.x << " <><><><>  " << next_point.position.y << "   "<< next_point.time.GetSeconds ()<< std::endl;
                      //schedule the vehicle to pass the intersection
                      std::cout << lane_id << "  " << vehicle_id << std::endl;

                      double time = 0;
                      double tmp_t = 0;
                      bool flag = true;
                      bool pass_head = false;
                      Vector now_pos = pos;
                      if(next_point.time.GetSeconds () > Simulator::Now().GetSeconds ()) {  //the vehicle get the privilege to pass immediately
                          Vector next_pos = next_point.position;
                          util.passIntersectionWait (lane_id, intersection_id, now_pos.x, now_pos.y);
                          if(!(util.doubleEquals (next_pos.x, now_pos.x) && util.doubleEquals (next_pos.y, now_pos.y)))  //the destination of vehicle isn't the edge of core
                            {
                              time = util.passIntersectionWait (lane_id, intersection_id, next_pos.x, next_pos.y);
                              pos = next_pos;
                              tmp_t = time + next_point.time.GetSeconds ();
                              std::cout << "=============1===========  " << tmp_t << std::endl;
                            }
                          else {  //the destination of vehicle is at edge
                              std::cout << "+++++++++++++++++++++++" << std::endl;
                              pass_head = true;
                              flag = false;
                            }
                        }
                      else {  //the vehicle get the privilege to pass after waiting
                          time = util.passIntersectionWait (lane_id, intersection_id, now_pos.x, now_pos.y);
                          if(!(util.doubleEquals (now_pos.x, pos.x) && util.doubleEquals (now_pos.y, pos.y))) { //the destination of vehicle isn't the edge of core
                              pos = now_pos;
                              tmp_t = time + Simulator::Now ().GetSeconds ();
                            }
                          else {
                               flag = false;
                            }
                        }
                      Waypoint wpt (Seconds(tmp_t), pos);
                      if(flag)
                          mobility->AddWaypoint (wpt);
                      std::cout << tmp_t << "s later (" << pos.x << "," << pos.y << ")" << std::endl;

                      time = util.passIntersectionCore (lane_id, intersection_id, pos.x, pos.y);
                      if(vehicle_id == 4) {
                          std::cout << pos.x << " asdfg " <<  pos.y << std::endl;
                        }
                      if(flag) {
                        tmp_t += time;
                        std::cout << "==============3===========   " << tmp_t <<std::endl;
                        }
                      else if(pass_head) {
                        tmp_t = time + mobility->GetNextWaypoint ().time.GetSeconds ();
                        std::cout << "==============4===========   " << tmp_t <<std::endl;
                        }
                      else{
                        tmp_t = time + Simulator::Now ().GetSeconds ();
                        std::cout << "==============5===========   " << mobility->GetNextWaypoint ().time.GetSeconds () <<std::endl;
                        }
                      std::cout << "==============2===========   " << tmp_t <<std::endl;
                      //the last vehicle
                      if(vehicle_id == m_last_id) {
                        t_last = tmp_t;
                      }
                      wpt.position = pos;
                      wpt.time = Seconds(tmp_t);
                      mobility->AddWaypoint (wpt);
                      std::cout << "the time pass the core of intersection " << wpt.time.GetSeconds () << std::endl;

                      if(vehicle_id == m_last_id) { //send unlock message
                          std::cout << "vehicle " << vehicle_id << " need " << (t_last- Simulator::Now ().GetSeconds ()) << "s pass the intersection" << std::endl;
                          Time time_unlock = Seconds (t_last- Simulator::Now ().GetSeconds ()) ;
                          Simulator::Schedule (time_unlock, &vehicle::sendUnlock, this);
                        }

                      time = util.passIntersectionIdle (lane_id, intersection_id, pos.x, pos.y);
                      if(vehicle_id == 4) {
                          std::cout << "asdf " << intersection_id << "  " << lane_id << " "  << pos.x << "  " << pos.y << std::endl;
                        }
                      tmp_t += time;
                      wpt.position = pos;
                      std::cout << "********************* " << tmp_t << " x:" << pos.x << " y: " << pos.y << std::endl;
                      wpt.time = Seconds(tmp_t);
                      mobility->AddWaypoint (wpt);
                      Simulator::Schedule (Seconds (tmp_t - Simulator::Now ().GetSeconds ()), &vehicle::sendRequest, this);

                      std::cout << "%%%%%%%%%%%%%%%%" << mobility->WaypointsLeft ()<< std::endl;
                      std::cout << "vehicle " << vehicle_id << " start to pass the intersection " << intersection_id << std::endl;
                      break;
                    }
                  pass_num--;
                }
            }
          packet->RemoveAllPacketTags ();
          packet->RemoveAllByteTags ();
       }
    }
}


void
vehicle::sendRequest()
{
      Ptr<Packet> packet = Create<Packet> (m_packetSize);
      SeqTsHeader seqTss;

      Vector pos = mobility->GetPosition ();
      double x = pos.x;
      double y = pos.y;
      std::cout << Simulator::Now ().GetSeconds () << " s, vehicle " << vehicle_id << " at (" << x << "," << y << ") prepare to send a request message" << std::endl;

      //the coordinate of vehicle
      uint32_t x1, x2, y1, y2;
      x1 = x;
      y1 = y;
      x2 = (x - x1) * 1000;
      y2 = (y - y1) * 1000;
      seqTss.SetSeq (y2);
      packet->AddHeader (seqTss);
      seqTss.SetSeq (y1);
      packet->AddHeader (seqTss);
      seqTss.SetSeq (x2);
      packet->AddHeader (seqTss);
      seqTss.SetSeq (x1);
      packet->AddHeader (seqTss);

      if(begin) { //the first time that the vehicle send request
          std::cout << "the first time to pass intersection" << std::endl;
          path_index = 0;
          begin = false;
        }
      intersection_id = util.whichIntersection (paths[path_index]);
      if(intersection_id == 8) {
          x += rand()/50;
          y += rand()/50;
          Waypoint ran(Seconds (Simulator::Now ().GetSeconds () + 0.1), Vector(x, y, 0));
          mobility->AddWaypoint (ran);
          return;
        }
      lane_id = util.whichLane (util.whichWaitingArea (paths[path_index]), util.whichWaitingArea (paths[path_index + 1]));
      setConnectSocket (intersection_id);
      uint32_t next_inter = intersection_id;
      uint32_t tmp_index = path_index;
      uint32_t next_lane;
      while(tmp_index + 1 < paths.size () && next_inter == intersection_id)
        {
          tmp_index++;
          next_inter = util.whichIntersection (paths[tmp_index]);
        }
      path_index = tmp_index;
      next_lane = util.whichLane (util.whichWaitingArea(paths[tmp_index]), util.whichWaitingArea (paths[tmp_index+1]));
      seqTss.SetSeq (next_inter);
      packet->AddHeader (seqTss);
      seqTss.SetSeq (next_lane);
      packet->AddHeader (seqTss);

      seqTss.SetSeq (lane_id);
      packet->AddHeader (seqTss);
      seqTss.SetSeq (vehicle_id);
      packet->AddHeader (seqTss);
      seqTss.SetSeq (1);
      packet->AddHeader (seqTss);

      int success = m_socket_send->Send (packet);

      std::cout << success << "~~~~~~~~~~~~x:" << x << " y:" << y << "  now: " << Simulator::Now ().GetSeconds () << std::endl;
      //the vehicle will move to the intersection
      double next_x = x;
      double next_y = y;
      pos_xy(lane_id, intersection_id, next_x, next_y, control_apps);
      double time = (fabs(x - next_x) + fabs(y - next_y))/speed_waiting + Simulator::Now ().GetSeconds ();
      pos.x = next_x;
      pos.y = next_y;
      Waypoint wpt (Seconds(time), pos);
      Waypoint w(Seconds (Simulator::Now ().GetSeconds () + 0.0001), Vector(x, y, 0));
      mobility->AddWaypoint (w);
      mobility->AddWaypoint (wpt);

      std::cout << "x:" << next_x << " y:" << next_y << " time:" << time << "  " << mobility->GetVelocity ().y << std::endl;
      std::cout << "vehicle " << vehicle_id << " at intersection " << intersection_id << " send request"<< std::endl;
}


void
vehicle::sendUnlock ()
{
//      setConnectSocket(intersection_id);
      Ptr<Packet> packet = Create<Packet> (m_packetSize);
      SeqTsHeader seqTss;

      seqTss.SetSeq (lane_id); //添加车道号，发送解锁信号
      packet->AddHeader (seqTss);
      seqTss.SetSeq (2);  //解锁标志
      packet->AddHeader (seqTss);
      m_socket_send->Send (packet);

      std::cout<< Simulator::Now ().GetSeconds () << "s, " << lane_id<<"  channel now unlock the locks"<<std::endl;
}


void
vehicle::setConnectSocket(uint32_t inter)
{
      if(m_socket_send != 0) {
          m_socket_send->Close ();
          m_socket_send = 0;
        }
      Ipv4Address address;
      if(inter <= 5 )
        address = m_CSMAInterface.GetAddress (inter - 1);
      else
        std::cout << "the id of intersection is error" << std::endl;
      TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
      m_socket_send = Socket::CreateSocket(GetNode(), tid);
      m_socket_send->Bind();
      m_socket_send->SetAllowBroadcast(true);
      m_socket_send->Connect(InetSocketAddress(address, m_port_send));
      m_socket_send->SetRecvCallback (MakeCallback (&vehicle::HandleRead, this));
}


/**
 * @brief vehicle::pos_xy
 * @param lane
 * @param inter_id
 * @param x
 * @param y
 * when the vehicle arrive at waiting area, it need to know how many vehicles are ahead of itself.
 */
void
vehicle::pos_xy(uint32_t lane, uint32_t inter_id, double &x, double &y, std::vector< Ptr<intersection> > &intsect)
{
  //calculate how many vehicles are ahead of the newcomer
  uint32_t count = 0;
  std::vector<rp> req_pending = intsect[inter_id - 1]->getPending();
  for(std::vector<rp>::iterator it = req_pending.begin ();
      it != req_pending.end (); it++) {
      if(it->lane_id == lane && util.doubleEquals (it->time, 0)) {
          count++;
        }
    }
  //set the coordinate of vehicle
  util.getPosition(lane, inter_id, count, x, y);
}




/**********************************************************************************************************/

class CMultiMain
{

public:
	CMultiMain();
	~CMultiMain();
	void Simulate(int argc, char *argv[]);


private:
	Ptr<Socket> source;
	std::string traceFile;
	std::string logFile;
	std::string phyMode;
	std::string lossModle;
	std::string homepath;
	std::string folder;
	std::ofstream os;
	double freq;//CCH
	double txp;
	double range;
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

	Utils util;
	std::vector< std::string > vec_str; //save the information of vehicles, one line represents the information of one vehicle
	std::vector< std::vector<uint32_t> > vec_vec_uint_path; //save paths of vehicles
	std::vector< Ptr<intersection> > vec_ptr_inter;	// keep the smart point of intersections
	double speed_idle;  //the speed when vehicles are in idle area
	double speed_waiting; //the speed when vehicles are in waiting area
	uint32_t goStraight;  // the time for going Straight to pass the intersection
	uint32_t turnLeft;	// the time for turning left to pass the intersection
	uint32_t waitingAreaLength;
	uint32_t coreAreaLength;
	uint32_t idleAreaLength;
	std::vector<Node> vec_vehicles;  //the vector to keep all vehicles object
	std::vector<intersection> vec_intersections;  //the vector to keep all intersections object
	uint32_t spaceOccupied;	// the space of a vehicle occupied when it is in the waiting lane
	std::vector<bool> req;

	void CourseChange (std::string foo, Ptr<const MobilityModel> mobility);
	void SetDefault();
	void ParseArguments(int argc, char *argv[]);
	void LoadTraffic();
	void CreateNode();
	void CreateChannels();
	void InstallInternetStack();
	void ConfigMobility();
	void Run();
//	void CMultiMain::enterAndRequest(Ptr<const MobilityModel> mobility, vehicle &veh, uint32_t currentIndex, Vector &pos);
	void SetUpApplication();
};


CMultiMain::CMultiMain()
{
	phyMode = "OfdmRate6MbpsBW10MHz";
	freq = 5.890e9;  //802.11p
	txp = 20;  // CCH
	range = 300.0; //CCH
	packetSize = 1000; // bytes
	numPackets = 1;
	interval = 0.1; // seconds
	verbose = false;
	duration = 180;
	vehNum = 0;
	conNum = 5;
	speed_idle = 20;
	speed_waiting = 12;
	goStraight = 3.0;
	turnLeft = 4.0;

	spaceOccupied = 4;
	waitingAreaLength = 100;
	coreAreaLength = 40;
	idleAreaLength = 800;
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
	CreateNode();
	CreateChannels();
	InstallInternetStack();
	ConfigMobility();
	SetUpApplication();
	Run();
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
	cmd.AddValue ("txp", "TX power for CCH", txp);
	cmd.AddValue ("range", "Range for CCH", range);
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

	vehNum = util.readTraceFile(vehicles_trace.c_str (), vec_str);
	std::cout << vehNum << " vehicles to be scheduled" << std::endl;
	os.open(output.data(),std::ios::out);
}


void
CMultiMain::CreateNode()
{
	m_vehicles.Create( vehNum );//Cars
	m_controllers.Create ( conNum );  //cotrollers

	for(uint32_t i = 0; i < vehNum; i++) {
	    req.push_back (false);
	  }
}


void
CMultiMain::CreateChannels()
{
	//===channel
	YansWifiChannelHelper channelHelper;
	channelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	channelHelper.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
				DoubleValue(range));

	// the channelg
	Ptr<YansWifiChannel> waveChannel = channelHelper.Create();

	//===wifiphy
	YansWifiPhyHelper wavePhy =  YansWifiPhyHelper::Default ();
	wavePhy.SetChannel (waveChannel);
	wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
	wavePhy.Set ("TxPowerStart",DoubleValue (txp));
	wavePhy.Set ("TxPowerEnd", DoubleValue (txp));

	// 802.11p mac
	NqosWaveMacHelper waveMac = NqosWaveMacHelper::Default ();

	Wifi80211pHelper waveHelper = Wifi80211pHelper::Default ();
	waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
											"DataMode",StringValue (phyMode),
											"ControlMode",StringValue (phyMode));

	m_VehDevices = waveHelper.Install(wavePhy, waveMac, m_vehicles);
	m_ConDevices = waveHelper.Install(wavePhy, waveMac, m_controllers);

	CsmaHelper csma;
	csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
	csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

	m_CSMADevices = csma.Install (m_controllers);

}

void
CMultiMain::InstallInternetStack ()
{
        InternetStackHelper stack;
        // stack.SetRoutingHelper(aodv);
        stack.Install (m_vehicles);
        stack.Install (m_controllers);

        Ipv4AddressHelper address;
        NS_LOG_INFO ("Assign IP Addresses.");
        address.SetBase ("10.1.0.0", "255.255.0.0");
        m_VehInterface = address.Assign (m_VehDevices);
        m_ConInterface = address.Assign (m_ConDevices);

        address.SetBase ("10.2.0.0", "255.255.0.0");
        m_CSMAInterface = address.Assign (m_CSMADevices);


        Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}



void
CMultiMain::CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
   Vector pos = mobility->GetPosition ();
   std::vector<std::string> ret;
   util.split (foo, ret, "/");
   uint32_t veh_node = atoi(ret[1].c_str());
//   std::vector<bool> visit = req[veh_node];
   if( util.isEnteringWaitingArea (pos) && !(req[veh_node])) {
       req[veh_node] = true;
       Ptr<Node> node = m_vehicles.Get (veh_node); //get the vehicle node id and the vehicle object
       Ptr<vehicle> appVeh = DynamicCast<vehicle>(node->GetApplication (0));
       appVeh->sendRequest ();
       std::cout << "**************( node " << veh_node << " ) " << std::endl;
       std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
       << ", z=" << pos.z << std::endl;

   }
   std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
   << ", z=" << pos.z << std::endl;
}

void
CMultiMain::ConfigMobility()
{
	std::cout << "Config mobility" << std::endl;
	//here we will set the position of vehicles and set their mobility model
	std::vector<Ptr<MobilityModel> > mobilityStack;

	ObjectFactory mobilityFactory;
	mobilityFactory.SetTypeId ("ns3::WaypointMobilityModel");

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

	    //get the paths of vehicles
	    std::vector<std::string> tmp_vec;
	    util.split (*vec_it1, tmp_vec);

	    std::vector<uint32_t> tmp_path;
	    for(std::vector<std::string>::iterator vec_it2 = tmp_vec.begin () + 3; vec_it2 != tmp_vec.end (); vec_it2++)
		tmp_path.push_back (atoi ((*vec_it2).c_str()));
	    vec_vec_uint_path.push_back (tmp_path);

	    Ptr<WaypointMobilityModel> mob = DynamicCast<WaypointMobilityModel>(mobilityStack[im]);

	    ns3::Vector initPos(atof(tmp_vec[1].c_str()), atof(tmp_vec[2].c_str()), 0.0);
	    Waypoint wpt (Seconds (atof(tmp_vec[0].c_str())), initPos);
	    mob->AddWaypoint (wpt);

	    double travel = util.travelTime (initPos, (uint32_t)atoi(tmp_vec[3].c_str()), speed_idle);
	    double next_time = wpt.time.GetSeconds () + travel;
	    std::cout << tmp_vec[0] << " , next time: " << travel << std::endl;
	    wpt.time = Seconds(next_time);
	    wpt.position = initPos;
	    std::cout << wpt.position.x << "  " << wpt.position.y << std::endl;
	    mob->AddWaypoint (wpt);
	    m_vehicles.Get(im)->AggregateObject(mob);
	    im++;
	  }

	for(uint32_t i = 0; i < vehNum; i++) {
	    std::ostringstream oss;
	    oss << "/NodeList/"
		<< m_vehicles.Get (i)->GetId ()
		<< "/$ns3::MobilityModel/CourseChange";
	    Config::Connect (oss.str (),
				MakeCallback (&CMultiMain::CourseChange, this));
	  }


	//here we set the mobility for the controllers
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
}





void
CMultiMain::SetUpApplication ()
{
       uint16_t port_controller = 1024;

      for(uint32_t i = 0; i < conNum; i++) {
          Ptr<intersection> appCon = CreateObject<intersection> ();
          vec_ptr_inter.push_back (appCon);
          appCon->SetUp (m_CSMAInterface,i + 1);
          m_controllers.Get (i)->AddApplication (appCon);
          appCon->SetStartTime (Seconds (1.));
          appCon->SetStopTime (Seconds (duration));
          std::cout << "init app of controllers"  << i << std::endl;
        }

       for(uint32_t i = 0; i < vehNum; i++) {
           Ptr<vehicle> appVeh = CreateObject<vehicle> ();
           appVeh->Setup (m_ConInterface, port_controller, 1024, DataRate ("1Mbps"),
                          m_vehicles.Get(i)->GetObject<WaypointMobilityModel>(), i + 1, vec_vec_uint_path[i], vec_ptr_inter);
           m_vehicles.Get (i)->AddApplication (appVeh);
           appVeh->SetStartTime (Seconds (1.));
           appVeh->SetStopTime (Seconds (duration));
           std::cout << "init app of vehicles" << i << std::endl;
         }
}


void CMultiMain::Run()
{
	std::cout << "Starting simulation for " << duration << " s ..."<< std::endl;
	Simulator::Stop(Seconds(duration));
	Simulator::Run();
	Simulator::Destroy();
}


int main (int argc, char *argv[])
{
	CMultiMain test;
	test.Simulate(argc, argv);
	return 0;
}



