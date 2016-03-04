#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <iostream>
#include "ns3/mobility-module.h"

using namespace ns3;
class Utils
{
public:
  Utils();
  ~Utils();

  int readTraceFile(const char* fileName, std::vector<std::string> &vec_str);
  std::string trim(const std::string& str);
  int split(const std::string& str, std::vector<string>& ret_, std::string sep = ",");

  bool isInIdleState(Ptr<Node> object) const;
  bool isInWaitingArea(Ptr<Node> object) const;

  double travelTime( Vector &pos, const int nextWaiting, const double speed) const;
  bool isEnteringWaitingArea(Vector &pos) const;

  bool doubleEquals(double a,double b) const;

};

#endif // UTILS_H
