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


#ifndef _POVMESH_H_
#define _POVMESH_H_

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>


class PovMesh
{

public:
    struct Point
    {
        Point()
        {
            valid = false;
            visited = false;
            component_id = -1;
            p3d[0]=p3d[1]=p3d[2]=0;
            color[0]=color[1]=color[2]=0;
        }

        bool valid;

        bool visited;
        int component_id;

        cv::Vec3d p3d;
        unsigned char color[3]; // RGB color
    };


    PovMesh( const int _width, const int _height );
    ~PovMesh(void);

    cv::Mat to_image( ) const;

    const int width() const;
    const int height() const;
    void set_point( int u, int v, cv::Vec3d p3d, unsigned char red=255, unsigned char green=255, unsigned char blue=255 );

    bool ransac_find_plane( const size_t n_ransac_rounds, const double distance_threshold );
    void refine_plane( double xmin, double xmax, double ymin, double ymax, std::vector<cv::Vec3d>* dbg_inliers );
    void crop_plane( const double distance_threshold );

    std::vector<double> get_plane_params( ) const;
    void align_plane();

	void crop(int top, int left, int bottom, int right);
    void erode_borders();
    bool save_as_ply_points( std::string filename ) const ;
    bool save_as_xyz( std::string filename ) const ;
    bool save_as_xyz_binary( std::string filename ) const ;
    bool save_as_xyz_compressed( std::string filename ) const ;
    bool save_as_triangulated_ply(std::string filename, double MAX_Z_GAP) const;

    double compute_zgap_percentile(const double percentile=99 );
    void cluster_biggest_connected_component( const boost::filesystem::path workdir, const double zgap );

    void laplacian_smooth( size_t steps );

    void RT_from_plane( cv::Matx33d& R, cv::Vec3d& T, cv::Matx33d& Rinv, cv::Vec3d& Tinv) const;
    static void RT_from_plane( const double a, const double b, const double c, const double d, cv::Matx33d& R, cv::Vec3d& T, cv::Matx33d& Rinv, cv::Vec3d& Tinv);

private:
    struct PovMeshImpl* pImpl;
};


#endif
