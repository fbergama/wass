/*************************************************************************

WASS - Wave Acquisition Stereo System
Copyright (C) 2016  Filippo Bergamasco

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*************************************************************************/

#include "hires_timer.h"

#if defined(__unix)
#include <sys/time.h>
#endif

namespace cvlab {

#ifdef WIN32
/*
 *	Win32 Hi-res timer code
 */

double LI1D( const LARGE_INTEGER *i )
{
   return(i->LowPart+(i->HighPart*4294967296.0));
}

void HiresTimer::start()
{
   started = true;
   stopped = false;

   QueryPerformanceFrequency(&frequency);
   QueryPerformanceCounter(&start_time);
}

double HiresTimer::elapsed() const
{
   if( !started )
      return 0.0;

   if( !stopped ) {
      LARGE_INTEGER end_time;
      QueryPerformanceCounter(&end_time);
      elapsed_time = (LI1D(&end_time) - LI1D(&start_time)) / LI1D(&frequency);
   }
   return elapsed_time; // sec.
}

void HiresTimer::stop()
{
   LARGE_INTEGER end_time;
   QueryPerformanceCounter(&end_time);
   elapsed_time = (LI1D(&end_time) - LI1D(&start_time)) / LI1D(&frequency);
   stopped = true;
}

void HiresTimer::reset()
{
   started = false;
   stopped = false;
}

#else

#if defined(__unix) || defined(__APPLE__)
/*
 *	Unix Hi-res timer code
 */

void HiresTimer::start()
{
   started = true;
   stopped = false;
   start_time = 0.0;
   struct timeval start_timeval;
   gettimeofday( &start_timeval, 0 );
   start_time = (double)start_timeval.tv_sec + (double)start_timeval.tv_usec/1000000.0;
}

double HiresTimer::elapsed() const
{
   if( !started )
      return 0.0;

   if( !stopped ) {
      struct timeval end_timeval;
      gettimeofday( &end_timeval, 0 );
      double tnow = (double)end_timeval.tv_sec + (double)end_timeval.tv_usec/1000000.0;
      elapsed_time = tnow-start_time;
   }
   return elapsed_time;
}

void HiresTimer::stop()
{
   struct timeval end_timeval;
   gettimeofday( &end_timeval, 0 );
   double tnow = (double)end_timeval.tv_sec + (double)end_timeval.tv_usec/1000000.0;
   elapsed_time = tnow-start_time;
   stopped = true;
}

void HiresTimer::reset()
{
   started = false;
   stopped = false;
}

#endif
#endif

void HiresTimer::operator<<( const std::string evt_name )
{
    std::pair<double,std::string> curr_evt;
    curr_evt.first = elapsed();
    curr_evt.second = evt_name;
    events.push_back( curr_evt );
}

}
