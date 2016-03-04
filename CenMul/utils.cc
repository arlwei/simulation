#include "utils.h"

Utils::Utils()
{
}


/**
 * @brief readTraceFile
 * @param fileName
 * @param vec_str
 * @return the number of lines
 */
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
Utils::split(const std::string& str, std::vector<string>& ret_, std::string sep = ",")
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
Utils::isInIdleState(Ptr<Node> object) const{
  Ptr<WaypointMobilityModel> position = object->GetObject<WaypointMobilityModel> ();
  Vector pos = position->GetPosition ();
  double x = pos.x;
  double y = pos.y;
  if(x < 800 || ((y < 1840 || y > 2080) && x > 900 && x < 940) || (1840 > x && x > 1040 && 1980 > y && y > 1940)
     || y < 800 || ((1980 > x && x > 1940) && (y > 3120 || 2880 > y && y > 2080 || 1840 > y && y > 1040))
     || (940 > y && y > 900 && (x < 1840 || x > 2080)) || (3020 > y && y > 2980 && (x < 1840 || x > 2080))
     || ( 2880 > x && x > 2080 && 1980 > y && y > 1940) || (3020 > x && x > 2980 && (y < 1840 || y > 2080)) || x > 3120
     )
    return true;
  else
    return false;
}

bool
Utils::isInWaitingArea(Ptr<Node> object) const{
  Ptr<WaypointMobilityModel> position = object->GetObject<WaypointMobilityModel> ();
  Vector pos = position->GetPosition ();
  double x = pos.x;
  double y = pos.y;
  if(((900 >= x && x >= 800 || 1040 >= x && x >= 940) && 1980>= y && y >= 1940) ||
        (940 >= x && x >= 900 && (2080 >= y && y >= 1980 || 1940 >= y && y >= 1840)) ||
     ((1940 >= x && x >= 1840 || 2080 >= x && x >= 1980) && 1980 >= y && y >= 1940) ||
     (1980 >= x && x >= 1940 && (2080 >= y && y >= 1980 || 1940 >= y && y >=1840)) ||
     ((1940 >= x && x >= 1840 || 2080 >= x && x >= 1980) && 3020 >= y && y >= 2980) ||
     (1980 >= x && x >= 1940 && (3120 >= y && y >= 3020 || 2980 >= y && y >= 2880)) ||
     ((1940 >= x && x >= 1840 || 2080 >= x && x >= 1980) && 940 >= y && y >= 900) ||
     (1980 >= x && x >= 1940 && (1040 >= y && y >= 940 || 900 >= y && y >= 800)) ||
     ((2980 >= x && x >= 2880 || 3120 >= x && x >= 3020) && 1980 >= y && y >= 1940) ||
     (3020 >= x && x >= 2980 && (2080 >= y && y >= 1980 || 1940 >= y && y >=1840))
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

  Ptr<WaypointMobilityModel> position = object->GetObject<WaypointMobilityModel> ();
  pos = position->GetPosition ();

  //vehicles are in the idle area
  if(isInIdleState (object)) {

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
}


 bool
 Utils::isEnteringWaitingArea(Vector &pos) const {
   double x = pos.x;
   double y = pos.y;

   if(x == 800 || x== 1040 || x == 1840 || x == 2080 || x == 2880 || x == 3120
      || y = 800 || y == 1040 || y == 1840 || y == 2080 || y == 2880 || y == 3120)
     return true;
   else
     return false;
 }


 bool
 Utils::doubleEquals(double a,double b) const
 {
   if (fabs(a-b) <= 1e-6)
     return true;
  return false;
 }
