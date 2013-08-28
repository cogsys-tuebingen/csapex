/*****************************************************************************
 *
 * Filename: time.H
 *
 * Purpose:  classes for C++ time handling (wrapper for timeval) with 
 *           additional functionality
 *
 * Language: C++
 *
 * Compiler: g++ 2.91.66
 *
 * Authors:  Michael Plagge
 *
 * Revision info:
 *           $Revision: 1.2 $
 *           $Date: 2002/07/19 17:15:06 $
 *           $Author: plagge $
 *
 * Copyright (C) Dept. Computer Architecture, University of Tuebingen, Germany
 *
 ****************************************************************************/
/** @file time.H 
 * 
 */

#ifndef _TIMECLASS_INCLUDE_H_
#define _TIMECLASS_INCLUDE_H_

#include <sys/time.h>
#include <iostream>

class Duration;


//======================================================================
// class Time
//======================================================================
/// A class to handle time and durations 
/// \nosubgrouping
class Time {
public:

  /** @name Constructors
   */
  //@{
  /// Standard constuctor initialize object with the current time
  Time() {
    gettimeofday(&time, NULL);
  }

  /// Construcor to initialize a Time object with a C timeval struct
  Time(struct timeval& t){time = t;};
  
  /// Initialize a Time object with a time in milliseconds
  Time(long int ms){
    time.tv_sec  = ms / 1000;
    time.tv_usec = (ms - time.tv_sec * 1000) * 1000; 
  }
  //@}
  
  void set(){
    gettimeofday(&time, NULL); 
  };
  
  
  /** @name Operators
   */
  //@{
  /// compares two objects of type Time
  /** \returns true if the two objects are equal
   */
  bool operator==(const Time& t) const{
    if((time.tv_sec == t.time.tv_sec ) &&
       (time.tv_usec == t.time.tv_usec))
      return true;
    return false;
  };
  
  /// compares two objects of type Time
  /** \returns true if the two objects are not equal
   */
  bool operator!=(const Time& t) const{
    return(!(*this == t));
  };
  
  /// compares two objects of type Time
  /** \returns true if the object on the left side of the operator 
   *  describes a later point in time.
   */
  bool operator>(const Time& t) const{
    if((time.tv_sec > t.time.tv_sec) || 
       ((time.tv_sec == t.time.tv_sec) && time.tv_usec > t.time.tv_usec))
      return true;
    return false;
  };
  
  /// compares two objects of type Time
  /** \returns true if the object on the right side of the operator 
   *  describes a later point in time.
   */
  bool operator<(const Time& t) const{
    if((time.tv_sec < t.time.tv_sec) || 
       ((time.tv_sec == t.time.tv_sec) && time.tv_usec < t.time.tv_usec))
      return true;
    return false;
  };
  
  /// compares two objects of type Time
  /** \returns true if the object on the left side of the operator 
   *  describes the same or a later point in time.
   */
  bool operator>=(const Time& t) const{
    return(!(*this < t));
  };
  
  /// compares two objects of type Time
  /** \returns true if the object on the right side of the operator 
   *  describes the same or a later point in time.
   */ 
  bool operator<=(const Time& t) const{
    return(!(*this > t));
  };
  
  /// adds a Duration object to a Time object
  /** \return reference to the Time object
   */ 
  Time& operator+=(const Duration &d);
  
  /// subtracts a Duration from a Time object
  /** \return reference to the Time object
   */ 
  Time& operator-=(const Duration &d);


  /// adds a Duration object and to a Time object
  /** \return a new time object
   */ 
  Time operator+(const Duration &d) const{
   Time ret(*this);
   return(ret += d);
  };
  
  /// subtracts a Duration object and to a Time object
  /** \return a new time object
   */ 
  Time operator-(const Duration &d) const{
    Time ret(*this);
    return(ret -= d);
  };
  //@}

  friend std::ostream& operator<<(std::ostream& s, const Time &d);
  friend std::istream& operator>>(std::istream& s, Time &d);
  friend class Duration;

private:
//--------------------------------------------------
  struct timeval
  time;
//--------------------------------------------------
};

//======================================================================
// class Duration
//======================================================================
/// A class to handle the difference (duration) between two objects of type Time
class Duration{
public:

  Duration(const Time& from, const Time& to){
    set(from, to);
  };
  
  Duration(const Time& from){
    Time to;
    set(from, to);
  };
  
  Duration(const Duration &d){dur = d.dur;};

  Duration(){dur = 0;};

  Duration(double ms){dur = ms;};
  
  Duration& operator+=(const Duration &d){
    dur += d.dur;
    return(*this);
  };
  
  Duration& operator-=(const Duration &d){
    dur -= d.dur;
    return(*this);
  };
  
  Duration operator+(const Duration &d){
    Duration ret(*this);
    return(ret += d);
  };
  
  Duration operator-(const Duration &d){
    Duration ret(*this);
    return(ret -= d);
  };

  bool operator==(const Duration &d) const{
    if(dur == d.dur) return true;
    return false;
  };
  
  bool operator!=(const Duration &d) const{
    if(dur != d.dur) return true;
    return false;
  }
  
  bool operator>(const Duration &d) const{
    if(dur > d.dur) return true;
    return false;
  }
  
  bool operator<(const Duration &d) const{
    if(dur < d.dur) return true;
    return false;
  }

  bool operator>=(const Duration &d) const{
    if(dur >= d.dur) return true;
    return false;
  }
  
  bool operator<=(const Duration &d) const{
    if(dur <= d.dur) return true;
    return false;
  }
  
  void set(const Time& from){
    Time to;
    set(from, to);
  };

  void set(const Time& from, const Time& to){
    if(from == to) dur = 0;
    else if(from < to){
      //calc seconds
      dur = (to.time.tv_sec - from.time.tv_sec) * 1000.0;
      if(from.time.tv_usec <= to.time.tv_usec)
        dur += (to.time.tv_usec - from.time.tv_usec)/1000.0;
      else
        dur += ((1000000 - from.time.tv_usec + to.time.tv_usec)/1000.0 
                - 1000.0);
    }
    else{
      dur = (from.time.tv_sec - to.time.tv_sec) * 1000.0;
      if(to.time.tv_usec <= from.time.tv_usec)
        dur += (from.time.tv_usec - to.time.tv_usec)/1000.0;
      else
        dur += ((1000000 - to.time.tv_usec + from.time.tv_usec)/1000.0 
                - 1000.0);
      dur *= -1.0;
    }
  };

  double get() const {return dur;};
  operator double() const {return dur;};
  

  friend std::ostream& operator<<(std::ostream& s, const Duration &d);
  friend std::istream& operator>>(std::istream& s, Duration &d);
  friend void sleep(const Duration d);
  
private:
  double dur;
};

#endif

/****************************************************************************
 * END OF FILE
 ****************************************************************************/
