#ifndef _EPIPOLAR_H_
#define _EPIPOLAR_H_

#include <vector>
#include <opencv2/core.hpp>

namespace WASS
{
namespace epi
{

struct ErrorStats
{
    double avg;
    double min;
    double max;
    double std;
};


ErrorStats evaluate_epipolar_error( const cv::Matx33d& F, const std::vector< cv::Vec2d >& p0, const std::vector< cv::Vec2d >& p1 );
ErrorStats evaluate_structure_error( const std::vector< cv::Vec3d >& pts3d,
                                     const std::vector< cv::Vec2d >& pts0,
                                     const std::vector< cv::Vec2d >& pts1,
                                     cv::Mat R, cv::Mat T, cv::Mat K0, cv::Mat K1 );
}
}



#endif
