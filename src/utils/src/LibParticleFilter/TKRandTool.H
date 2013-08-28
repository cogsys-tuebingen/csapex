////////////////////////////////////////////////////////////////////////////
//                                                                        //
// File: TKRandTool.h (header file of class TKRandTool).                  //
//                                                                        //
// Version: 2006-05-06                                                    //
//                                                                        //
// History:                                                               //
//                                                                        //
// 2006-05-06: Added d() methods.                                         //
// 2006-03-10: Added shootInt and operator() methods.                     //
//             Added several specialized uniform generator methods.       //
//             Added static cast to avoid compile warning.                //
// 2005-02-11: Initial version.                                           //
//                                                                        //
// Description: The purpose of this class is to provide an easy-to-use    //
//              reliable source of high quality random numbers for        //
//              standalone programs, without requirering users to         //
//              introduce new external dependencies.                      //
//                                                                        //
// Homepage: http://www.nbi.dk/~kittel/programsandscripts/#randtool       //
//                                                                        //
// Copyright: Thomas H. Kittelmann <kittel AT nbi DOT dk>                 //
//                                                                        //
// License: GNU General Public License version 2 (note however that a     //
//          clearly marked part of the implementation file TKRandTool.cxx //
//          comes from the Mersenne Twister implementation by R. Wagner,  //
//          found at                                                      //
//     http://www-personal.engin.umich.edu/~wagnerr/MersenneTwister.html  //
//          so that part of the code is under the BSD-revised license.    //
//                                                                        //
// This program is free software; you can redistribute it and/or modify   //
// it under the terms of the GNU General Public License as published by   //
// the Free Software Foundation; either version 2 of the License, or (at  //
// your option) any later version.                                        //
//                                                                        //
// This program is distributed in the hope that it will be useful, but    //
// WITHOUT ANY WARRANTY; without even the implied warranty of             //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU      //
// General Public License for more details.                               //
//                                                                        //
// You should have received a copy of the GNU General Public License      //
// along with this program; if not, write to the Free Software            //
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307    //
// USA                                                                    //
////////////////////////////////////////////////////////////////////////////

#ifndef TKRANDTOOL_H
#define TKRANDTOOL_H

// The purpose of this class is to provide an easy-to-use reliable
// source of random numbers for standalone programs.
//
// Author: Thomas Kittelmann
// Licence: GPL v2.
//
// (The file MersenneTwister.h has as of now been pasted into
//  TKRandTool.cxx. So that part of the file is under the BSD)

//_________________________________________________________________

class TKRandTool  {
public:
  TKRandTool(unsigned seed = 0);//0 for time (or /dev/urandom if present).
  ~TKRandTool();

  //Uniform "real" random numbers:
  double shoot();//Output in 0.0 ... 1.0
  double shoot( const double& max );//Output in 0.0 ... max
  double shoot( const double& min, const double& max );//Output in min ... max

  //Uniform integers
  unsigned shootInt( const unsigned& n );//Output in 0,...,n-1
  unsigned shootInt( const unsigned& nmin, const unsigned& nmax );//output in nmin,...,nmax
  int shootInt( const int& nmin, const int& nmax );//output in nmin,...,nmax

  //Various special distributions:
  double shootGaussian( const double & mean = 0.0, const double & width = 1.0 );
  unsigned shootBinomial( const unsigned & n, const double & p );// Assumes p in [0,1]
  double shootExponential( const double & lambda = 1.0 );
  unsigned shootPoisson(const double & mu );

  //Overloaded call operators with functionality like shoot() and shootInt(n):
  unsigned operator()( const unsigned& );
  double operator()();

  //For dice:
  unsigned d( const unsigned& dicetype );
  unsigned d( const unsigned& ndice, const unsigned& dicetype );

private:
  class Clockwork;
  Clockwork * m_cw;

};

#endif
