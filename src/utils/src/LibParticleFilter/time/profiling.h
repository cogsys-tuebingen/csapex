//
// profiling.h -- nützliche Tools zum Profiling
//
// -- Michael Drueing
//
// abgeaendert/erweitert von Andreas Masselli

/*
 * Quick usage introduction:
 *
 * #include <profiling.h>
 *
 * use #define DEBUG_PROFILING to enable profiling
 * (if disabled, no runtime overhead is generated)
 *
 * define Profiling variables globally with
 * DECLARE_PROFILING_VARIABLE(name)
 *
 * then enclose the code to be profiled with
 * START_PROFILING_SECTION(name)
 * ...<your code here>...
 * END_PROFILING_SECTION(name)
 *
 * and add debug output with
 * DISPLAY_PROFILE(name)
 *
 * debug output will only be generated evey PROFILE_PRINT_AMOUNT run
 *
 */
  
#ifndef PROFILING_H
#define PROFILING_H

#ifdef DEBUG_PROFILING

#ifndef PROFILE_PRINT_AMOUNT
#define PROFILE_PRINT_AMOUNT 100
#endif

#include "time.H"
#include <iostream>
#include <iomanip>

struct ProfileData
{
	char*  name;
	Time   start;
	Time   end;
	double duration;
	double min;
	double max;
	long   numRuns;
};

#define DECLARE_PROFILING_VARIABLE(pdata) \
	ProfileData pdata = {#pdata, Time(), Time(), 0.0, 99999999999.0, -1.0, 0};

#define START_PROFILING_SECTION(pdata) \
	pdata.start.set();

#define END_PROFILING_SECTION(pdata) \
{ \
	pdata.end.set(); \
	Duration d(pdata.start, pdata.end); \
	double dd = (double)d;\
	pdata.duration += dd; \
	if (pdata.min > dd) pdata.min = dd; \
	if (pdata.max < dd) pdata.max = dd; \
	++pdata.numRuns; \
}

#define DISPLAY_PROFILE(pdata) \
if ((pdata.numRuns % PROFILE_PRINT_AMOUNT) == 0) \
{ \
	cerr << pdata.name << ": " \
	     << setiosflags(ios::fixed) \
	     << setw(8) << setprecision(2) << pdata.duration << " ms in " \
	     << setw(5) << pdata.numRuns << " runs --> " \
	     << setw(6) << setprecision(3) << ((pdata.duration)/(double)pdata.numRuns) << " ms per run (min = " \
	     << pdata.min << ", max = " << pdata.max << ")" << endl; \
}

#else

#define DECLARE_PROFILING_VARIABLE(pdata) 
#define START_PROFILING_SECTION(pdata) 
#define END_PROFILING_SECTION(pdata) 
#define DISPLAY_PROFILE(pdata) 

#endif

#endif
