#include "epipolar.h"

namespace WASS { namespace epi {



ErrorStats evaluate_epipolar_error( const cv::Matx33d& F, const std::vector< cv::Vec2d >& p0, const std::vector< cv::Vec2d >& p1 )
{
    size_t Npts = std::min< size_t >( p0.size(), p1.size() );
    std::vector<double> errors(Npts);
    ErrorStats es;
    es.max = -std::numeric_limits<double>::max();
    es.min = std::numeric_limits<double>::max();
    es.avg = 0;

    for( size_t i=0; i<Npts; ++i )
    {
        const cv::Vec3d l( p0[i][0], p0[i][1], 1 );
        const cv::Vec3d r( p1[i][0], p1[i][1], 1 );

        cv::Vec3d Fl = F*l;
        cv::Vec3d Fr = F.t()*r;

        errors[i] = 0.5 * ( std::fabs( Fl.ddot(r) / sqrt(Fl[0]*Fl[0] + Fl[1]*Fl[1]) ) + std::fabs( Fr.ddot(l) / sqrt(Fr[0]*Fr[0] + Fr[1]*Fr[1]) ));
        es.avg += errors[i];
        es.max = std::max< double >( es.max, errors[i] );
        es.min = std::min< double >( es.min, errors[i] );
        //std::cout << errors[i] << std::endl;
    }

    es.avg /= static_cast<double>(Npts);
    es.std = 0;
    for( std::vector<double>::const_iterator it=errors.begin(); it!=errors.end(); ++it )
    {
        es.std += (*it - es.avg)*(*it - es.avg);
    }
    es.std = sqrt(es.std / Npts);
    return es;
}


ErrorStats evaluate_structure_error( const std::vector< cv::Vec3d >& pts3d,
                                     const std::vector< cv::Vec2d >& pts0,
                                     const std::vector< cv::Vec2d >& pts1,
                                     const cv::Mat R, const cv::Mat T, cv::Mat K0, cv::Mat K1 )
{
    cv::Matx33d Rx((double*)(R.clone().ptr()));
    cv::Vec3d Tx( T.at<double>(0), T.at<double>(1), T.at<double>(2) );
    cv::Matx33d K0x((double*)(K0.clone().ptr()));
    cv::Matx33d K1x((double*)(K1.clone().ptr()));

    std::vector<double> errors(pts3d.size());
    ErrorStats es;
    es.max = -std::numeric_limits<double>::max();
    es.min = std::numeric_limits<double>::max();
    es.avg = 0;

    for( size_t i=0; i<pts3d.size(); ++i )
    {
        const cv::Vec3d& p3d = pts3d[i];
        cv::Vec3d rep_0 = K0x * p3d; rep_0 = rep_0 / rep_0[2];
        cv::Vec3d rep_1 = K1x * (Rx*p3d + Tx); rep_1 = rep_1 / rep_1[2];

        errors[i] = 0.5 * ( cv::norm(cv::Vec2d(rep_0[0], rep_0[1]) - pts0[i]) +
                            cv::norm(cv::Vec2d(rep_1[0], rep_1[1]) - pts1[i]));

        es.avg += errors[i];
        es.max = std::max< double >( es.max, errors[i] );
        es.min = std::min< double >( es.min, errors[i] );
    }

    es.avg /= static_cast<double>(pts3d.size());
    es.std = 0;
    for( std::vector<double>::const_iterator it=errors.begin(); it!=errors.end(); ++it )
    {
        es.std += (*it - es.avg)*(*it - es.avg);
    }
    es.std = sqrt(es.std / pts3d.size());
    return es;
}

}}
