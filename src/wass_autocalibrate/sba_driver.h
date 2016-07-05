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


#ifndef _SBA_DRIVER_H
#define _SBA_DRIVER_H

#include <vector>
#include <opencv2/core.hpp>



void sba_driver( const std::vector< cv::Matx44d >& cam_poses,
                 const std::vector< cv::Vec3d >& pts3d,
                 const std::vector< std::vector< cv::Vec2d > >& pts2d,
                 std::vector< cv::Mat >& Rsba,
                 std::vector< cv::Mat >& Tsba );

#endif
