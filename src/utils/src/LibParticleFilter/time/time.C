#include "time.H"

#include <string>
#include <cmath>

using namespace std;

Time& Time::operator+=(const Duration &d){
//--------------------------------------------------  
  double 
    s;
//--------------------------------------------------
  modf(d.get()/1000.0, &s);
  time.tv_sec += (long) s;
  time.tv_usec += (long)((d.get() - s * 1000.0) * 1000);
  if(time.tv_usec >= 1000000){
    time.tv_sec += 1;
    time.tv_usec -= 1000000;
  }
  return(*this);
};

Time& Time::operator-=(const Duration &d){
//--------------------------------------------------
  double 
    s;
//--------------------------------------------------
  modf(d.get()/1000.0, &s);
  time.tv_sec -= (long) s;
  time.tv_usec -= (long)((d.get() - s * 1000.0) * 1000);
  if(time.tv_usec < 0){
    time.tv_sec -= 1;
    time.tv_usec += 1000000;
  }
  return(*this);
}; 

ostream& operator<<(ostream& s, const Time &d){
  s << "Seconds: " << d.time.tv_sec << " MuSeconds: " << d.time.tv_usec;
  return s;
};

istream& operator>>(istream& s, Time& d){
//--------------------------------------------------
  string
    buf;
//--------------------------------------------------
  s >> buf;
  if(buf == "Seconds:") s >> d.time.tv_sec;
  
  s >> buf;
  if(buf =="MuSeconds:") s >> d.time.tv_usec;

  return s;
};

ostream& operator<<(ostream& s, const Duration &d){
  s << "Duration in ms: " << d.dur;
  return s;
};

istream& operator>>(istream& s, Duration& d){
//--------------------------------------------------
  string
    buf;
//--------------------------------------------------
  s >> buf;
  if(buf == "Duration") s >> buf >> buf >> d.dur;
 
  return s;
};

void sleep(const Duration d){
  struct timeval t;
  t.tv_sec = (int)d/1000;
  t.tv_usec = (int)((d - t.tv_sec * 1000) * 1000);
  select(0, NULL, NULL, NULL, &t);
};
