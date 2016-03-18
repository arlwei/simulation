#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
using namespace std;

const int  Num_Min_main = 1;
const int  Num_Min_minor = 1;
vector<string> int1_0;
vector<string> int1_3;
vector<string> int2_2;
vector<string> int2_5;
vector<string> int3_4;
vector<string> int3_7;
vector<string> int4_6;
vector<string> int4_1;
vector<string> int5;

void init() {
  string str1_0[] = {"11,13,51,53,31,32,88,88","11,13,51,53,31,33,88,88","11,13,51,52,24,22,88,88","11,13,51,52,24,21,88,88"};
  for(int i = 0; i < 4; i++) {
      int1_0.push_back(str1_0[i]);
    }
  string str1_3[] = {"12,13,51,53,31,32,88,88","12,13,51,53,31,33,88,88","12,13,51,52,24,22,88,88","12,13,51,52,24,21,88,88"};
  for(int i = 0; i < 4; i++) {
      int1_3.push_back(str1_3[i]);
    }
  string str2_2[] = {"22,24,52,54,42,43,88,88","22,24,52,54,42,44,88,88","22,24,52,53,31,32,88,88","22,24,52,53,31,33,88,88"};
  for(int i = 0; i < 4; i++) {
      int2_2.push_back(str2_2[i]);
    }
  string str2_5[] = {"23,24,52,54,42,43,88,88","23,24,52,54,42,44,88,88","23,24,52,53,31,32,88,88","23,24,52,53,31,33,88,88"};
  for(int i = 0; i < 4; i++) {
      int2_5.push_back(str2_5[i]);
    }
  string str3_4[] = {"33,31,53,51,13,11,88,88","33,31,53,51,13,14,88,88","33,31,53,54,42,44,88,88","33,31,53,54,42,43,88,88"};
  for(int i = 0; i < 4; i++) {
      int3_4.push_back(str3_4[i]);
    }
  string str3_7[] = {"34,31,53,51,13,11,88,88","34,31,53,51,13,14,88,88","34,31,53,54,42,44,88,88","34,31,53,54,42,43,88,88"};
  for(int i = 0; i < 4; i++) {
      int3_7.push_back(str3_7[i]);
    }
  string str4_6[] = {"44,42,54,52,24,22,88,88","44,42,54,52,24,21,88,88","44,42,54,51,13,11,88,88","44,42,54,51,13,14,88,88"};
  for(int i = 0; i < 4; i++) {
      int4_6.push_back(str4_6[i]);
    }
  string str4_1[] = {"41,42,54,52,24,22,88,88","41,42,54,52,24,21,88,88","41,42,54,51,13,11,88,88","41,42,54,51,13,14,88,88"};
  for(int i = 0; i < 4; i++) {
      int4_1.push_back(str4_1[i]);
    }
  string str5[] = {
    "51,53,31,32,88,88",
    "51,53,31,33,88,88",
    "51,52,24,21,88,88",
    "51,52,24,22,88,88",
    "52,54,42,44,88,88",
    "52,54,42,43,88,88",
    "52,53,31,32,88,88",
    "52,53,31,33,88,88",
    "53,51,13,11,88,88",
    "53,51,13,14,88,88",
    "53,54,42,44,88,88",
    "53,54,42,43,88,88",
    "54,52,24,22,88,88",
    "54,52,24,21,88,88",
    "54,51,13,11,88,88",
    "54,51,13,14,88,88"
  };
  for(int i = 0; i < 16; i++) {
      int5.push_back(str5[i]);
    }
}

double getFloat()
{
  return (rand()%100/100.0 + rand()%60);
}

double randomUniform( double dMinValue = 0.0, double dMaxValue = 60.0)
{
    double pRandomValue = (double)(rand()/(double)RAND_MAX);
    pRandomValue = pRandomValue*(dMaxValue-dMinValue)+dMinValue;
    return pRandomValue;
}

int main() {
  double time  = 60.0;
  double base_time = 0.0;
  int count = time/60;
  int round = 0;
  init();
  cout << "mission completed!" << endl;
//  srand((unsigned int)time(NULL));
  freopen("traces.txt", "w", stdout);
    while (round < count) {
        int t = 0;
        int k = 0;
        for(int i = 1; i < 6; i++) {  //for all intersections
             switch(i) {
                case 1:
                   {
                   for(int j = 0; j < 8; j++) { //for all lanes
                       switch(j) {
                          case 0:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (1941,1950) << "," << randomUniform (3121,3920) << "," <<  int1_0[t] << endl;
                             }
                           break;
                          case 1:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1951,1960) << "," << randomUniform (3121,3920) << "," <<  "11,12,88,88" << endl;
                             }
                           break;
                          case 2:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2880) << "," << randomUniform (3010,3019) << "," <<  "12,14,88,88" << endl;
                             }
                           break;
                          case 3:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2880) << "," << randomUniform (3001,3010) << "," <<  int1_3[t] << endl;
                             }
                           break;
                          case 4:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1970,1979) << "," << randomUniform (2081,2799) << "," <<  "13,11,88,88" << endl;
                             }
                           break;
                          case 5:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1961,1970) << "," << randomUniform (2081,2799) << "," <<  "13,14,88,88" << endl;
                             }
                           break;
                          case 6:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (2981,2990) << "," <<  "14,12,88,88" << endl;
                             }
                           break;
                          case 7:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (2990,2999) << "," <<  "14,11,88,88" << endl;
                             }
                           break;
                       }
                     }
                   }
                   break;
                case 2:
                 {
                   for(int j = 0; j < 8; j++) { //for all lanes
                       switch(j) {
                         case 0:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (2981,2990) << "," << randomUniform (2081,2879) << "," <<  "21,23,88,88" << endl;
                             }
                           break;
                         case 1:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (2990,2999) << "," << randomUniform (2081,2879) << "," <<  "21,22,88,88" << endl;
                             }
                           break;
                         case 2:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (3121,3919) << "," << randomUniform (1970,1979) << "," <<  int2_2[t] << endl;
                             }
                           break;
                         case 3:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (3121,3919) << "," << randomUniform (1961,1970) << "," <<  "22,23,88,88" << endl;
                             }
                           break;
                         case 4:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (3010,3019) << "," << randomUniform (1041,1839) << "," <<  "23,21,88,88" << endl;
                             }
                           break;
                         case 5:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (3001,3010) << "," << randomUniform (1041,1839) << "," <<  int2_5[t] << endl;
                             }
                           break;
                         case 6:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2879) << "," << randomUniform (1941,1950) << "," <<  "24,22,88,88" << endl;
                             }
                           break;
                         case 7:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2879) << "," << randomUniform (1950,1959) << "," <<  "24,21,88,88" << endl;
                             }
                           break;
                         }
                     }
                 }
                   break;
                case 3:
                 {
                   for(int j = 0; j < 8; j++) { //for all lanes
                       switch(j) {
                         case 0:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1941,1950) << "," << randomUniform (1041,1839) << "," <<  "31,33,88,88" << endl;
                             }
                           break;
                         case 1:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1950,1959) << "," << randomUniform (1041,1839) << "," <<  "31,32,88,88" << endl;
                             }
                           break;
                         case 2:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2879) << "," << randomUniform (930,939) << "," <<  "32,34,88,88" << endl;
                             }
                           break;
                         case 3:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2879) << "," << randomUniform (921,930) << "," <<  "32,33,88,88" << endl;
                             }
                           break;
                         case 4:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (1970,1979) << "," << randomUniform (1,799) << "," <<  int3_4[t] << endl;
                             }
                           break;
                         case 5:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1961,1970) << "," << randomUniform (1,799) << "," <<  "33,34,88,88" << endl;
                             }
                           break;
                         case 6:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (901,910) << "," <<  "34,32,88,88" << endl;
                             }
                           break;
                         case 7:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (910,919) << "," <<  int3_7[t] << endl;
                             }
                           break;
                         }
                     }
                 }
                   break;
                case 4:
                 {
                   for(int j = 0; j < 8; j++) { //for all lanes
                       switch(j) {
                         case 0:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (901,910) << "," << randomUniform (2081,2879) << "," <<  "41,43,88,88" << endl;
                             }
                           break;
                         case 1:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (910,919) << "," << randomUniform (2081,2879) << "," <<  int4_1[t] << endl;
                             }
                           break;
                         case 2:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (1970,1979) << "," <<  "42,44,88,88" << endl;
                             }
                           break;
                         case 3:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (1961,1970) << "," <<  "42,43,88,88" << endl;
                             }
                           break;
                         case 4:
                           for(k = Num_Min_main; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (930,939) << "," << randomUniform (1041,1839) << "," <<  "43,41,88,88" << endl;
                             }
                           break;
                         case 5:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (921,930) << "," << randomUniform (1041,1839) << "," <<  "43,44,88,88" << endl;
                             }
                           break;
                         case 6:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%4;
                               cout << randomUniform () + base_time << "," << randomUniform (1,799) << "," << randomUniform (1941,1950) << "," <<  int4_6[t] << endl;
                             }
                           break;
                         case 7:
                           for(k = Num_Min_minor; k > 0; k--) {
                               cout << randomUniform () + base_time << "," << randomUniform (1,799) << "," << randomUniform (1950,1959) << "," <<  "44,41,88,88" << endl;
                             }
                           break;
                         }
                     }
                 }
                   break;
                case 5:
                 {
                   for(int j = 0; j < 8; j++) { //for all lanes
                       switch(j) {
                         case 0:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (1941,1950) << "," << randomUniform (2081,2879) << "," <<  int5[t] << endl;
                             }
                           break;
                         case 1:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (1950,1959) << "," << randomUniform (2081,2879) << "," <<  int5[t + 2] << endl;
                             }
                           break;
                         case 2:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2879) << "," << randomUniform (1970,1979) << "," <<  int5[t + 4] << endl;
                             }
                           break;
                         case 3:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (2081,2879) << "," << randomUniform (1961,1970) << "," <<  int5[t + 6] << endl;
                             }
                           break;
                         case 4:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (1970,1979) << "," << randomUniform (1041,1839) << "," <<  int5[t + 8] << endl;
                             }
                           break;
                         case 5:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (1961,1970) << "," << randomUniform (1041,1839) << "," <<  int5[t + 10] << endl;
                             }
                           break;
                         case 6:
                           for(k = Num_Min_main; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (1941,1950) << "," <<  int5[t + 12] << endl;
                             }
                           break;
                         case 7:
                           for(k = Num_Min_minor; k > 0; k--) {
                               t = rand()%2;
                               cout << randomUniform () + base_time << "," << randomUniform (1041,1839) << "," << randomUniform (1950,1959) << "," <<  int5[t + 14] << endl;
                             }
                           break;
                          }
                     }
                 }
                   break;     
             }

        }
        round++;
        base_time = round*60;
      }
}
