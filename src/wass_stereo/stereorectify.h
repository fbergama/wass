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

#ifndef _STEREO_RECTIFY_H_
#define _STEREO_RECTIFY_H_

#include <opencv2/core/core.hpp>

void stereoRectifyUndistorted( const cv::Matx33d K0,
                               const cv::Matx33d K1,
                               const cv::Matx33d R,
                               const cv::Vec3d T,
                               const cv::Size I0Size,
                               const cv::Size I1Size,
                               const cv::Size outSize,
                               cv::Matx33d& H0, cv::Matx33d& H1,
                               cv::Rect& ROI );
#endif

