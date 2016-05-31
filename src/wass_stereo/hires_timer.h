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

#ifndef HIRES_TIMER_H
#define HIRES_TIMER_H

#ifdef WIN32
#include <windows.h>
#endif

#if defined(UNIX) || defined(__APPLE__)
#include <time.h>
#include <sys/time.h>
#endif

#include <vector>
#include <string>

#undef max
#undef min

namespace cvlab {

class HiresTimer
{
   public:
      HiresTimer() : started(false), stopped(false) {}
      ~HiresTimer() {}
      void start();
      double elapsed() const;
      void stop();
      void reset();

      void operator<<( const std::string evt_name );
      const std::vector< std::pair<double,std::string> >::const_iterator evts_begin() const { return events.begin(); }
      const std::vector< std::pair<double,std::string> >::const_iterator evts_end() const { return events.end(); }

   private:
      bool started;
      bool stopped;
      mutable double elapsed_time;
      std::vector< std::pair<double,std::string> > events;

   #ifdef WIN32
      LARGE_INTEGER frequency;
      LARGE_INTEGER start_time;
   #endif

   #if defined(__unix) || defined(__APPLE__)
      double start_time;
   #endif
};


}

#endif // HIRES_TIMER_H
