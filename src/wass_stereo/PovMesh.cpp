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


#include "PovMesh.h"
#include <fstream>
#include <boost/cstdint.hpp>
#include <time.h>
#include "log.hpp"
#include "incfg.hpp"

#ifdef USE_PCL
// PCL stuff
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/octree/octree_impl.h>
//#include <pcl/octree/octree_pointcloud_singlepoint.h>
#endif


struct PointLocation
{
    size_t u;
    size_t v;

    explicit PointLocation( size_t _u, size_t _v ) : u(_u), v(_v) { }

    inline bool operator<( const PointLocation& other )
    {
        if( u == other.u )
            return v<other.v;
        return u<other.u;
    }
};


struct PovMeshImpl
{
    PovMeshImpl( const int _W, const int _H ) : width(_W), height(_H), plane_set(false)
    {
        data.resize(width*height);
    }

    inline const size_t size() const { return data.size(); }
    inline PovMesh::Point& PT( const int u, const int v ) { return data[v*width+u]; }
    inline PovMesh::Point& PT( const size_t idx ) { return data[idx]; }
    inline PovMesh::Point& PT( const PointLocation& l ) { return data[l.v*width+l.u];  }

    inline const PovMesh::Point& PTc( const int u, const int v ) const { return data[v*width+u]; }
    inline const PovMesh::Point& PTc( const PointLocation& l ) const { return data[l.v*width+l.u]; }
    inline const PovMesh::Point& PTc( const size_t idx ) const { return data[idx]; }

    size_t num_valid_points() const
    {
        size_t n_valid = 0;
        for( size_t i=0; i<size(); ++i )
            n_valid += PTc(i).valid ? 1 : 0;

        return n_valid;
    }

    void set_all_valid( bool valid )
    {
        for( size_t i=0; i<size(); ++i )
            PT(i).valid = valid;
    }

    void set_all_visited( bool visited )
    {
        for( size_t i=0; i<size(); ++i )
            PT(i).visited = visited;
    }

    void extract_component( int id )
    {
        for( size_t i=0; i<size(); ++i )
            PT(i).valid = PT(i).component_id == id;
    }

    cv::Vec3d centroid() const
    {
        cv::Vec3d c;
        double n=0.0;

        for( size_t i=0; i<size(); ++i )
        {
            if( PTc(i).valid )
            {
                c += PTc(i).p3d;
                n+=1.0;
            }
        }

        return c/n;
    }

    void min_max( double& minx, double& maxx,
                  double& miny, double& maxy,
                  double& minz, double& maxz )
    {
        minx=miny=minz=1E30;
        maxx=maxy=maxz=-1E30;
        for( size_t i=0; i<size(); ++i )
        {
            if( PTc(i).valid )
            {
                const cv::Vec3d& pp = PTc(i).p3d;
                minx = std::min( minx, pp[0] );
                maxx = std::max( maxx, pp[0] );
                miny = std::min( miny, pp[1] );
                maxy = std::max( maxy, pp[1] );
                minz = std::min( minz, pp[2] );
                maxz = std::max( maxz, pp[2] );
            }
        }
    }

    void rotate_translate( cv::Matx33d R, cv::Vec3d T )
    {
        for( size_t i=0; i<size(); ++i )
            PT(i).p3d = R*PT(i).p3d + T;
    }


    size_t get_non_visited_neighbours( const PointLocation& p, std::vector< PointLocation >& neigh, const double MAX_ZGAP=1E20  )
    {
        neigh.clear();
        const PovMesh::Point& pref = PTc(p);

        if( !pref.valid )
            return 0;

        if( p.u>0 )
        {
            PointLocation left( p.u-1, p.v );
            if( PTc(left).valid && !PTc(left).visited && std::abs( pref.p3d[2] - PTc(left).p3d[2] ) < MAX_ZGAP )
            {
                neigh.push_back( left );
            }
        }
        if( p.u<width-1 )
        {
            PointLocation right( p.u+1, p.v );
            if( PTc(right).valid && !PTc(right).visited && std::abs( pref.p3d[2] - PTc(right).p3d[2] ) < MAX_ZGAP )
            {
                neigh.push_back( right );
            }
        }
        if( p.v>0 )
        {
            PointLocation top( p.u, p.v-1 );
            if( PTc(top).valid && !PTc(top).visited && std::abs( pref.p3d[2] - PTc(top).p3d[2] ) < MAX_ZGAP )
            {
                neigh.push_back( top );
            }
        }
        if( p.v<height-1 )
        {
            PointLocation bottom( p.u, p.v+1 );
            if( PTc(bottom).valid && !PTc(bottom).visited && std::abs( pref.p3d[2] - PTc(bottom).p3d[2] ) < MAX_ZGAP )
            {
                neigh.push_back( bottom );
            }
        }
        return neigh.size();
    }

    bool get_non_visited( PointLocation& p )
    {
        for( p.u=0; p.u<width; ++p.u )
        {
            for( p.v=0; p.v<height; ++p.v )
            {
                if( PTc(p).valid && !PTc(p).visited )
                {
                    return true;
                }
            }
        }
        return false;
    }

    cv::Mat get_RGB_image()
    {
        cv::Mat img = cv::Mat::zeros( height,width, CV_8UC3 );
        for( int u=0; u<width; ++u )
        {
            for( int v=0; v<height; ++v )
            {
                const PovMesh::Point& p = PTc( u, v );
                if( p.valid )
                {
                    img.at<cv::Vec3b>( v,u ) = cv::Vec3b(p.color[0],p.color[1],p.color[2]);
                }
            }
        }
        return img;
    }

    cv::Mat get_Components_image( int biggest_comp_id = -1 )
    {
        cv::Mat img = cv::Mat::zeros( height,width, CV_8UC3 );
        std::vector< cv::Vec3b > components_palette;
        components_palette.push_back( cv::Vec3b(255,0,0) );
        components_palette.push_back( cv::Vec3b(255,255,0) );
        components_palette.push_back( cv::Vec3b(255,255,255) );
        components_palette.push_back( cv::Vec3b(0,255,255) );
        components_palette.push_back( cv::Vec3b(255,0,255) );
        components_palette.push_back( cv::Vec3b(100,0,0) );
        components_palette.push_back( cv::Vec3b(100,100,0) );
        components_palette.push_back( cv::Vec3b(100,100,100) );
        components_palette.push_back( cv::Vec3b(0,100,0) );
        components_palette.push_back( cv::Vec3b(0,100,100) );
        components_palette.push_back( cv::Vec3b(100,0,100) );
        components_palette.push_back( cv::Vec3b(0,255,0) );

        for( int u=0; u<width; ++u )
        {
            for( int v=0; v<height; ++v )
            {
                const PovMesh::Point& p = PTc( u, v );
                if( p.valid && p.component_id>=0)
                {
                    img.at<cv::Vec3b>( v,u ) = (p.component_id == biggest_comp_id) ? components_palette.back() : components_palette[ p.component_id % (components_palette.size()-1) ];
                }
            }
        }
        return img;
    }

    const int width;
    const int height;
    std::vector< PovMesh::Point > data;

    bool plane_set;
    cv::Vec3d planeN;
    double plane_coeffs[4];
};





PovMesh::PovMesh( const int _width, const int _height )
{
    pImpl = new PovMeshImpl(_width,_height);
}


PovMesh::~PovMesh(void)
{
    delete pImpl;
    pImpl = 0;
}


const int PovMesh::width() const
{
    return pImpl->width;
}


const int PovMesh::height() const
{
    return pImpl->height;
}


void PovMesh::set_point( int u, int v, cv::Vec3d p3d, unsigned char red, unsigned char green, unsigned char blue )
{
    if( u>=0 && u<pImpl->width && v>=0 && v<pImpl->height ) {
        PovMesh::Point& pt = pImpl->PT(u,v);

        pt.p3d = p3d;
        pt.color[0] = red;
        pt.color[1] = green;
        pt.color[2] = blue;
        pt.valid = true;
    }
}

cv::Mat PovMesh::to_image( ) const
{
    cv::Mat img = cv::Mat::zeros( pImpl->height, pImpl->width, CV_8UC3 );
    for( int r=0; r<img.rows; ++r )
    {
        unsigned char* pr = img.ptr( r );

        for( int col=0; col<img.cols; ++col )
        {
            const PovMesh::Point& pt = pImpl->PTc(col, r);
            *pr++ = pt.color[0];
            *pr++ = pt.color[1];
            *pr++ = pt.color[2];
        }
    }
    return img;
}

bool PovMesh::save_as_xyz( std::string filename ) const
{
    std::ofstream ofs( filename.c_str() );
    ofs.precision(15);
    ofs << std::scientific;

    if( ofs.fail() )
        return false;

    for( size_t i=0; i<pImpl->size(); ++i )
    {
        const PovMesh::Point& pt = pImpl->PTc(i);
        if( pt.valid )
        {
            ofs << pt.p3d[0] << " " << pt.p3d[1] << " " << pt.p3d[2] << std::endl;
        }
    }

    ofs.flush();
    ofs.close();
    return true;
}


bool PovMesh::save_as_xyz_binary( std::string filename ) const
{
    std::ofstream ofs( filename.c_str(), std::ios::binary );
    if( ofs.fail() )
        return false;

    boost::uint32_t datasize = static_cast<boost::uint32_t>( pImpl->num_valid_points() );
    ofs.write( (char*)&datasize, sizeof( boost::uint32_t) );

    float* ptdata = new float[datasize*3];

    size_t idx=0;
    for( size_t i=0; i<pImpl->size(); ++i )
    {
        const PovMesh::Point& pt = pImpl->PTc(i);
        if( pt.valid )
        {
            ptdata[idx++] = static_cast<float>(pt.p3d[0]);
            ptdata[idx++] = static_cast<float>(pt.p3d[1]);
            ptdata[idx++] = static_cast<float>(pt.p3d[2]);
        }
    }

    ofs.write( (char*)ptdata, sizeof(float)*datasize*3 );
    delete[] ptdata;

    ofs.flush();
    ofs.close();
    return true;
}

bool PovMesh::save_as_xyz_compressed( std::string filename ) const
{
    LOG_SCOPE("save_as_xyz_compressed");
    std::ofstream ofs( filename.c_str(), std::ios::binary );
    if( ofs.fail() )
        return false;

    boost::uint32_t datasize = static_cast<boost::uint32_t>( pImpl->num_valid_points() );
    ofs.write( (char*)&datasize, sizeof( boost::uint32_t) );

    LOGI << "saving mesh as compressed xyz file..." ;;

    // Transform data and compute limits
    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double minz = std::numeric_limits<double>::max();
    double maxx = -std::numeric_limits<double>::max();
    double maxy = -std::numeric_limits<double>::max();
    double maxz = -std::numeric_limits<double>::max();
    cv::Matx33d R,Rinv;
    cv::Vec3d T,Tinv;
    RT_from_plane( R,T,Rinv,Tinv);

    for( size_t i=0; i<pImpl->size(); ++i )
    {
        if( pImpl->PTc(i).valid )
        {
            const cv::Vec3d pto = pImpl->PTc(i).p3d;
            const cv::Vec3d pt = R*pto + T;
            minx = std::min( minx, pt[0] );
            miny = std::min( miny, pt[1] );
            minz = std::min( minz, pt[2] );

            maxx = std::max( maxx, pt[0] );
            maxy = std::max( maxy, pt[1] );
            maxz = std::max( maxz, pt[2] );
        }
    }

    // Find data limits
    LOGI << "Xrange: [" << minx << " ... " << maxx << "]" ;
    LOGI << "Yrange: [" << miny << " ... " << maxy << "]" ;
    LOGI << "Zrange: [" << minz << " ... " << maxz << "]" ;

    const double MV = static_cast<double>( static_cast<boost::uint16_t>(0xFFFF) );
    //LOGI << "Max v: " << MV;

    const double xscale = MV / (maxx-minx);
    const double yscale = MV / (maxy-miny);
    const double zscale = MV / (maxz-minz);

    ofs.write( (char*)&xscale, sizeof(double) );
    ofs.write( (char*)&yscale, sizeof(double) );
    ofs.write( (char*)&zscale, sizeof(double) );
    ofs.write( (char*)&minx, sizeof(double) );
    ofs.write( (char*)&miny, sizeof(double) );
    ofs.write( (char*)&minz, sizeof(double) );

    ofs.write( (char*)Rinv.val, sizeof(double)*9 );
    ofs.write( (char*)Tinv.val, sizeof(double)*3 );

    boost::uint16_t* ptdata = new boost::uint16_t[datasize*3];

    size_t idx=0;
    for( size_t i=0; i<pImpl->size(); ++i )
    {
        const PovMesh::Point& pt = pImpl->PTc(i);
        if( pt.valid )
        {
            const cv::Vec3d p3d = R*pt.p3d + T;
            ptdata[idx++] = static_cast<boost::uint16_t>( (p3d[0] - minx) * xscale );
            ptdata[idx++] = static_cast<boost::uint16_t>( (p3d[1] - miny) * yscale );
            ptdata[idx++] = static_cast<boost::uint16_t>( (p3d[2] - minz) * zscale );
        }
    }

    ofs.write( (char*)ptdata, sizeof(boost::uint16_t)*datasize*3 );
    delete[] ptdata;

    ofs.flush();
    LOGI << "total data size: " << (static_cast<double>(ofs.tellp())/(1E6)) << " MB";
    ofs.close();
    return true;
}


bool PovMesh::save_as_ply_points( std::string filename ) const
{
    const size_t n_valid_points=pImpl->num_valid_points();

    // Generate header
    {
        std::ofstream ofs( filename.c_str() );
        if( ofs.fail() )
            return false;

        ofs << "ply" << std::endl;
        ofs << "format binary_little_endian 1.0" << std::endl;
        ofs << "element vertex " << n_valid_points << std::endl;
        ofs << "property float x" << std::endl;
        ofs << "property float y" << std::endl;
        ofs << "property float z" << std::endl;
        ofs << "property uchar red" << std::endl;
        ofs << "property uchar green" << std::endl;
        ofs << "property uchar blue" << std::endl;
        ofs << "end_header" << std::endl;
        ofs.close();
    }

    // Save data
    {
        std::ofstream ofs( filename.c_str(), std::ios::app | std::ios::binary );
        if( ofs.fail() )
            return false;

        size_t datasize = (sizeof(float)*3+sizeof(unsigned char)*3)*n_valid_points;
        char* data = new char[ datasize ];
        char* ptrD=data;

        for( size_t i=0; i<pImpl->size(); ++i )
        {
            const PovMesh::Point& pt = pImpl->PTc(i);
            if( pt.valid )
            {
                *((float*)ptrD) = (float)pt.p3d[0]; ptrD+=sizeof(float);
                *((float*)ptrD) = (float)pt.p3d[1]; ptrD+=sizeof(float);
                *((float*)ptrD) = (float)pt.p3d[2]; ptrD+=sizeof(float);
                *((unsigned char*)ptrD) = pt.color[0]; ptrD+=sizeof(unsigned char);
                *((unsigned char*)ptrD) = pt.color[1]; ptrD+=sizeof(unsigned char);
                *((unsigned char*)ptrD) = pt.color[2]; ptrD+=sizeof(unsigned char);
            }
        }

        ofs.write(data,datasize);
        delete[] data;
        ofs.flush();
        ofs.close();
    }

    return true;
}

void PovMesh::erode_borders()
{
    std::vector< size_t > bad_indices;

    for( int v=0; v<height(); ++v)
    {
        for( int u=0; u<width(); ++u )
        {
            bool good=pImpl->PTc(u,v).valid;

            if( good && v>0  )
            {
                good &= pImpl->PTc(u,v-1).valid;

                if( good && u>0 )
                    good &= pImpl->PTc(u-1,v-1).valid;

                if( good && u<width()-1 )
                    good &= pImpl->PTc(u+1,v-1).valid;
            }
            if( good && v<height()-1  )
            {
                good &= pImpl->PTc(u,v+1).valid;

                if( good && u>0 )
                    good &= pImpl->PTc(u-1,v+1).valid;

                if( good && u<width()-1 )
                    good &= pImpl->PTc(u+1,v+1).valid;
            }

            if( good && u>0 )
                good &= pImpl->PTc(u-1,v).valid;
            if( good && u<width()-1 )
                good &= pImpl->PTc(u+1,v).valid;

            if(!good)
                bad_indices.push_back(v*width()+u);
        }
    }

    for( std::vector< size_t >::const_iterator it = bad_indices.begin(); it!=bad_indices.end(); ++it )
        pImpl->PT(*it).valid = false;
}


void PovMesh::crop(int top, int left, int bottom, int right)
{
    for( int v=0; v<height(); ++v)
    {
        for( int u=0; u<width(); ++u )
        {
            pImpl->PT(u,v).valid = v>top && v<bottom && u>left && u<right;
        }
    }
}


INCFG_REQUIRE( bool, PLANE_WEIGHT_PROPORTIONAL_TO_DISTANCE, true, "use point to camera distance as weight during LLS plane fitting" )
INCFG_REQUIRE( bool, PLANE_USE_CENTRAL_THIRD_ONLY, false, "use only the central third of the image to estimate the mean sea plane" )
INCFG_REQUIRE( double, PLANE_REFINEMENT_MAX_DISTANCE, 70.0, "max point distance for plane refinement" )

void PovMesh::refine_plane( double xmin, double xmax, double ymin, double ymax, std::vector<cv::Vec3d>* dbg_inliers )
{
    LOG_SCOPE("refine_plane");
    
    const int umin = INCFG_GET(PLANE_USE_CENTRAL_THIRD_ONLY) ? pImpl->width / 4 : 0;
    const int umax = INCFG_GET(PLANE_USE_CENTRAL_THIRD_ONLY) ? pImpl->width*3 / 4 : pImpl->width-1;
    const int vmin = INCFG_GET(PLANE_USE_CENTRAL_THIRD_ONLY) ? pImpl->height / 4 : 0;
    const int vmax = INCFG_GET(PLANE_USE_CENTRAL_THIRD_ONLY) ? pImpl->height*2 / 3 : pImpl->height-1;

    for( int v=vmin; v<=vmax; ++v )
    {
        for( int u=umin; u<=umax; ++u )
        {
            const PovMesh::Point& pmp = pImpl->PTc(u,v);
            if( pmp.valid  )
            {
                cv::Vec3d p = pmp.p3d;
                if( p[0]>xmin && p[0]<xmax &&
                    p[1]>ymin && p[1]<ymax  &&
                    sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]) < INCFG_GET(PLANE_REFINEMENT_MAX_DISTANCE) )
                {
                    dbg_inliers->push_back( p );
                }
            }
        }
    }

    LOGI << "refinement inliers (after cropping): " << dbg_inliers->size();
    LOGI << "computing weights/centroid...";

    std::vector< double > weights(dbg_inliers->size());

    cv::Vec3d wcentr;
    double wsum=0.0;
    for( size_t i=0; i<dbg_inliers->size(); ++i )
    {
        const double& px=dbg_inliers->at(i)[0];
        const double& py=dbg_inliers->at(i)[1];
        const double& pz=dbg_inliers->at(i)[2];
        const double dist = sqrt(px*px + py*py + pz*pz);
        const double w = INCFG_GET(PLANE_WEIGHT_PROPORTIONAL_TO_DISTANCE) ? dist : 1.0;
        weights[i] = w;
        wsum+=w;

        wcentr[0]+=px*w;
        wcentr[1]+=py*w;
        wcentr[2]+=pz*w;
    }
    wcentr=wcentr/wsum;

    LOGI << "creating covariance matrix";
    cv::Matx33d A = cv::Matx33d::zeros();

    for( size_t i=0; i<dbg_inliers->size(); ++i )
    {
        cv::Vec3d pt = dbg_inliers->at(i) - wcentr;
        A = A + weights[i] * pt * pt.t();
    }

    LOGI << "computing svd";
    cv::SVD svd;
    svd( (cv::Mat)A );

    cv::Vec3d cff( svd.vt.at<double>(2,0),svd.vt.at<double>(2,1),svd.vt.at<double>(2,2) );
    cff = cv::normalize(cff); // just to be sure in case that svd is not normalized
    if( cff[2]<0 )
    {
        cff *= -1.0;    // Ensure that normal is facing toward the camera
    }

    const double d = (-cff.t()*wcentr)[0];

    pImpl->planeN = cff;
    pImpl->plane_coeffs[0] = cff[0];
    pImpl->plane_coeffs[1] = cff[1];
    pImpl->plane_coeffs[2] = cff[2];
    pImpl->plane_coeffs[3] = d;

    LOGI << "estimated plane coeffs: " << pImpl->plane_coeffs[0] << " " << pImpl->plane_coeffs[1] << " " << pImpl->plane_coeffs[2] << " " << pImpl->plane_coeffs[3];
}




bool PovMesh::ransac_find_plane( const size_t n_ransac_rounds, const double distance_threshold  )
{
    LOG_SCOPE("ransac_find_plane");

    const size_t NPTS=pImpl->size();
    size_t best_inliers=0;
    double best_d=0;;
    const int iW = pImpl->width;
    const int iH = pImpl->height;
    const double mindist = iH * 0.01;
    cv::Vec3d best_n;


    for( int round=0; round<n_ransac_rounds; ++round )
    {
         cv::Vec2i p1coord( rand()%iW, rand()%iH );
         cv::Vec2i p2coord( rand()%iW, rand()%iH );
         cv::Vec2i p3coord( rand()%iW, rand()%iH );

        if( cv::norm(p1coord-p2coord)<mindist ||
            cv::norm(p2coord-p3coord)<mindist ||
            cv::norm(p1coord-p3coord)<mindist )
        {
            // Points are too close, try again
            round--;
            continue;
        }

        const PovMesh::Point& p1M = pImpl->PTc(p1coord[0],p1coord[1]);
        const PovMesh::Point& p2M = pImpl->PTc(p2coord[0],p2coord[1]);
        const PovMesh::Point& p3M = pImpl->PTc(p3coord[0],p3coord[1]);

        if( !p1M.valid ||  !p2M.valid || !p3M.valid )
        {
            continue;
        }

        // Extract 3 points
        cv::Vec3d p1 = p1M.p3d;
        cv::Vec3d p2 = p2M.p3d;
        cv::Vec3d p3 = p3M.p3d;

        // Get plane
        cv::Vec3d n = (p2-p1).cross( p3-p1 );
        n = n/cv::norm(n);
        if( n[2]<0 )
            n=n*-1.0;

        double d = -n.ddot(p1);

        // Count inliers
        size_t n_inliers=0;
        for( size_t i=0; i<NPTS; ++i )
        {
            if( pImpl->PTc( i ).valid )
            {
                double dist = fabs(n.ddot( pImpl->PTc( i ).p3d ) + d);
                if( dist<distance_threshold )
                {
                    ++n_inliers;
                }
            }
        }

        if( n_inliers>best_inliers)
        {
            best_inliers = n_inliers;
            best_n = n;
            best_d = d;
        }
    }

    LOGI << n_ransac_rounds << " ransac rounds, " << best_inliers << " best inliers";


    // Save plane and inliers
    if( best_n[2]<0 )
    {
        LOGI << "swapping normal" << std::endl;
    }
    pImpl->planeN = best_n;
    pImpl->plane_coeffs[0] = best_n[0];
    pImpl->plane_coeffs[1] = best_n[1];
    pImpl->plane_coeffs[2] = best_n[2];
    pImpl->plane_coeffs[3] = best_d;


    LOGI << "ransac plane coeffs: " << pImpl->plane_coeffs[0] << " " << pImpl->plane_coeffs[1] << " " << pImpl->plane_coeffs[2] << " " << pImpl->plane_coeffs[3];

#if 0
    size_t k=0;
    pImpl->set_all_visited(false);
    for( size_t i=0; i<NPTS; ++i )
    {
        if( pImpl->PTc( i ).valid )
        {
            double d = fabs( best_n.ddot( pImpl->PTc( i ).p3d ) + best_d );
            if( d<distance_threshold )
            {
                pImpl->PT( i ).visited = true;
                ++k;
            }
        }
    }

    LOGI << "Ransac out: " << k << std::endl;
#endif

    if( best_inliers < NPTS/10 )
        return false;

    return true;
}


void PovMesh::crop_plane( const double distance_threshold )
{
    LOG_SCOPE("crop_plane");
    const size_t NPTS=pImpl->size();
    cv::Vec3d best_n;
    double best_d;
    size_t k=0;

    pImpl->planeN = best_n;
    best_n[0] = pImpl->plane_coeffs[0];
    best_n[1] = pImpl->plane_coeffs[1];
    best_n[2] = pImpl->plane_coeffs[2];
    best_d = pImpl->plane_coeffs[3];

    pImpl->set_all_visited(false);
    for( size_t i=0; i<NPTS; ++i )
    {
        if( pImpl->PTc( i ).valid )
        {
            double d = fabs( best_n.ddot( pImpl->PTc( i ).p3d ) + best_d );
            if( d<distance_threshold )
            {
                pImpl->PT( i ).visited = true;
                ++k;
            }
        }
    }

    for( size_t i=0; i<NPTS; ++i )
    {
        pImpl->PT( i ).valid = pImpl->PT( i ).visited;
    }
    pImpl->set_all_visited(false);

    LOGI << "number of points after plane cropping: " << k;
}


std::vector<double> PovMesh::get_plane_params( ) const
{
    std::vector<double> p(4);
    p[0] = pImpl->plane_coeffs[0];
    p[1] = pImpl->plane_coeffs[1];
    p[2] = pImpl->plane_coeffs[2];
    p[3] = pImpl->plane_coeffs[3];
    return p;
}


void PovMesh::align_plane()
{
    /*
    if( !pImpl->plane_set )
        LOGI << "Unable to align, no plane found." << std::endl;

    cv::Vec3d N = cv::normalize(pImpl->planeN);
    if( N.ddot(cv::Vec3d(0,0,1))>0.0 )
        N*=-1.0;

    cv::Vec3d v2= cv::normalize(cv::Vec3d(0,-N[2],N[1]));
    cv::Vec3d v1= cv::normalize( N.cross(v2) );

    cv::Matx33d R;
    R(0,0) = v1[0]; R(0,1) = v1[1]; R(0,2) = v1[2];
    R(1,0) = v2[0]; R(1,1) = v2[1]; R(1,2) = v2[2];
    R(2,0) =  N[0]; R(2,1) =  N[1]; R(2,2) = N[2];

    LOGI << N << " " << v1 << " " << v2 << " " << (cv::Mat)R << std::endl;
    LOGI << "DET: " << cv::determinant( R );

    cv::Vec3d T;
    pImpl->rotate_translate(R,T);
    */

    cv::Vec3d centroid = pImpl->centroid();


    if( !pImpl->plane_set )
        LOGE << "unable to align, no plane found.";

    cv::Vec3d N = cv::normalize(pImpl->planeN);
    if( N.ddot(cv::Vec3d(0,0,1))>0.0 )
        N*=-1.0;

    cv::Vec3d v2= cv::normalize(cv::Vec3d(0,-N[2],N[1]));
    cv::Vec3d v1= cv::normalize( N.cross(v2) );


    cv::Matx33d R = cv::Matx33d::eye();
    R(0,0) = v1[0]; R(0,1) = v1[1]; R(0,2) = v1[2];
    R(1,0) = v2[0]; R(1,1) = v2[1]; R(1,2) = v2[2];
    R(2,0) =  -N[0]; R(2,1) =  -N[1]; R(2,2) = -N[2];


    pImpl->rotate_translate(R,R*cv::Vec3d(-centroid[0],-centroid[1],-centroid[2]) );
}



struct triangle_indices {
    triangle_indices() : id0(0), id1(0), id2(0) {}
    triangle_indices(std::size_t _id0, std::size_t _id1, std::size_t _id2) : id0(_id0), id1(_id1), id2(_id2) {}
    std::size_t id0;
    std::size_t id1;
    std::size_t id2;
};


double PovMesh::compute_zgap_percentile( const double percentile )
{
    LOG_SCOPE("compute_zgap");
    std::vector< double > zgaps;
    zgaps.reserve( width()*height()*3 );
    for( int i=1; i<height(); i++ )
    {
        for( int j=1; j<width()-1; j++ )
        {
            const PovMesh::Point& pt0 = pImpl->PTc(j,i);

            if( pImpl->PTc(j,i).valid  )
            {
                const PovMesh::Point& ptA = pImpl->PTc(j-1,i-1);
                const PovMesh::Point& ptB = pImpl->PTc(j  ,i-1);
                const PovMesh::Point& ptC = pImpl->PTc(j+1,i-1);

                if( ptA.valid )
                {
                    zgaps.push_back( std::abs( pt0.p3d[2] - ptA.p3d[2] ) );
                }
                if( ptB.valid )
                {
                    zgaps.push_back( std::abs( pt0.p3d[2] - ptB.p3d[2] ) );
                }
                if( ptC.valid )
                {
                    zgaps.push_back( std::abs( pt0.p3d[2] - ptC.p3d[2] ) );
                }
            }
        }
    }

    LOGI << "computing " << percentile << "th percentile";

    std::sort( zgaps.begin(), zgaps.end() );
    double zgap_percentile = zgaps[ std::floor( percentile/100.0 * (double)(zgaps.size()) ) ];
    return zgap_percentile;
}


void PovMesh::cluster_biggest_connected_component( const boost::filesystem::path workdir, const double zgap_percentile )
{
    LOG_SCOPE("cluster");
    LOGI << "extracting connected-components";
    //cv::imwrite("meshcol.png",pImpl->get_RGB_image());
    pImpl->set_all_visited( false );

    int curr_comp_id = 0;
    int biggest_component_id = curr_comp_id;
    size_t biggest_component_size = 0;
    size_t remaining_nodes = pImpl->num_valid_points();

    PointLocation px(0,0);
    std::vector< PointLocation > neigh;

    while( pImpl->get_non_visited(px) )
    {
        size_t component_size=0;
        std::vector< PointLocation > visit_list;
        visit_list.push_back(px);

        while( !visit_list.empty() )
        {
            px = visit_list.back(); visit_list.pop_back();
            if( !pImpl->PTc(px).visited )
            {
                pImpl->PT(px).visited = true;
                pImpl->PT(px).component_id = curr_comp_id;
                component_size++;

                pImpl->get_non_visited_neighbours(px, neigh, zgap_percentile);
                for( size_t kk=0; kk<neigh.size(); ++kk )
                {
                    visit_list.push_back( neigh[kk] );
                }
            }
        }

        if( component_size > biggest_component_size )
        {
            biggest_component_size = component_size;
            biggest_component_id = curr_comp_id;
        }

        remaining_nodes -= component_size;
        if( remaining_nodes < biggest_component_size )
            break;

        curr_comp_id++;
    }

    LOGI << "biggest component: " << biggest_component_id << " size: " << biggest_component_size << " (px)";

    cv::Mat components_img;
    cv::resize(pImpl->get_Components_image(biggest_component_id),components_img,cv::Size(),0.5,0.5);
    cv::imwrite( (workdir/"/graph_components.jpg").string(), components_img);

    pImpl->extract_component( biggest_component_id );
}


void PovMesh::laplacian_smooth( size_t steps )
{
    LOG_SCOPE("laplacian_smooth");

    std::vector< double > zvals( pImpl->size() );
    std::vector< double > orig_z( pImpl->size() );

    for( size_t k=0; k<pImpl->size(); ++k )
        orig_z[k] = pImpl->PT(k).p3d[2];

    LOGI << "laplacian smoothing...";
    for( size_t step=0; step<steps; ++step)
    {
        LOGI << "step " << (step+1);
        for( int v=1; v<pImpl->height-1; ++v )
        {
            for( int u=1; u<pImpl->width-1; ++u )
            {
                const PovMesh::Point& pog = pImpl->PTc(u,v);
                if( !pog.valid )
                    continue;

                size_t N=0;
                double vs=0;
                for( int vv=v-1; vv <= v+1; ++vv )
                {
                    for( int uu=u-1; uu <= u+1; ++uu )
                    {
                        const PovMesh::Point& p = pImpl->PTc(uu,vv);
                        if( p.valid )
                        {
                            vs += p.p3d[2];
                            ++N;
                        }
                    }
                }

                zvals[ v*pImpl->width + u ] =  N>3 ? vs/(double)N : -1;
            }
        }

        // Copy smoothed values
        for( size_t k=0; k<pImpl->size(); ++k )
            pImpl->PT(k).p3d[2] = zvals[k]>0?zvals[k]:pImpl->PT(k).p3d[2];
    }

    LOGI << "DONE";

    for( size_t k=0; k<pImpl->size(); ++k )
        pImpl->PT(k).p3d[2] -= orig_z[k];
}



void PovMesh::RT_from_plane( const double a, const double b, const double c, const double d, cv::Matx33d& R, cv::Vec3d& T, cv::Matx33d& Rinv, cv::Vec3d& Tinv)
{
    double q = (1-c)/(a*a + b*b);

    // Build matrix R
    R(0,0) = 1-a*a*q;
    R(0,1) = -a*b*q;
    R(0,2) = -a;

    R(1,0) = -a*b*q;
    R(1,1) = 1-b*b*q;
    R(1,2) = -b;

    R(2,0) = a;
    R(2,1) = b;
    R(2,2) = c;

    Rinv = R.t();

    // Build matrix T
    T(0)=0;
    T(1)=0;
    T(2)=d;

    Tinv = Rinv*(-T);
}

void PovMesh::RT_from_plane( cv::Matx33d& R, cv::Vec3d& T, cv::Matx33d& Rinv, cv::Vec3d& Tinv ) const
{
    return RT_from_plane(pImpl->plane_coeffs[0], pImpl->plane_coeffs[1], pImpl->plane_coeffs[2], pImpl->plane_coeffs[3], R,T,Rinv,Tinv);
}


bool PovMesh::save_as_triangulated_ply( std::string filename, double MAX_Z_GAP ) const
{
    //Keep only triangles with relative verices

    std::size_t* old_to_new_index = new std::size_t[ width()*height()+1 ];
    #define ON_INVALID_INDEX ((boost::uint32_t)0xFFFFFFFF)
    for( int i=0; i<width()*height()+1; i++ ) {
        old_to_new_index[i] = ON_INVALID_INDEX;
    }

    std::vector< PovMesh::Point > pts;
    pts.reserve( width()*height() );
    std::vector< triangle_indices > triangles;
    triangles.reserve( width()*height() );

    for( int i=1; i<height(); i++ ) {
        for( int j=1; j<width(); j++ ) {
            if( pImpl->PTc(j,i).valid  ) {

                if(  pImpl->PTc(j-1,i-1).valid && std::abs( pImpl->PTc(j-1,i-1).p3d[2] - pImpl->PTc(j,i).p3d[2] ) < MAX_Z_GAP ) {
                    if( pImpl->PTc(j,i-1).valid && std::abs( pImpl->PTc(j,i-1).p3d[2] - pImpl->PTc(j,i).p3d[2] ) < MAX_Z_GAP ) {
                        std::size_t idx1 = i*width() + j;
                        std::size_t idx2 = (i-1)*width() + j;
                        std::size_t idx3 = (i-1)*width() + (j-1);

                        if( old_to_new_index[idx1] == ON_INVALID_INDEX ) {
                            PovMesh::Point pt = pImpl->PTc(j,i);
                            pts.push_back( pt );
                            old_to_new_index[ idx1 ] = pts.size()-1;
                        }
                        if( old_to_new_index[idx2] == ON_INVALID_INDEX ) {
                            PovMesh::Point pt = pImpl->PTc(j,i-1);
                            pts.push_back( pt );
                            old_to_new_index[ idx2 ] = pts.size()-1;
                        }
                        if( old_to_new_index[idx3] == ON_INVALID_INDEX ) {
                            PovMesh::Point pt = pImpl->PTc(j-1,i-1);
                            pts.push_back( pt );
                            old_to_new_index[ idx3 ] = pts.size()-1;
                        }

                        triangles.push_back( triangle_indices( old_to_new_index[idx1], old_to_new_index[idx2] , old_to_new_index[idx3] ) );
                    }

                    if( pImpl->PTc(j-1,i).valid && pImpl->PTc(j-1,i-1).valid && std::abs( pImpl->PTc(j-1,i).p3d[2] - pImpl->PTc(j,i).p3d[2] ) < MAX_Z_GAP ) {
                        boost::uint32_t idx1 = i*width() + j;
                        boost::uint32_t idx2 = (i-1)*width() + (j-1);
                        boost::uint32_t idx3 = (i)*width() + (j-1);

                        if( old_to_new_index[idx1] == ON_INVALID_INDEX ) {
                            PovMesh::Point pt = pImpl->PTc(j,i);
                            pts.push_back( pt );
                            old_to_new_index[ idx1 ] = pts.size()-1;
                        }
                        if( old_to_new_index[idx2] == ON_INVALID_INDEX ) {
                            PovMesh::Point pt = pImpl->PTc(j-1,i-1);
                            pts.push_back( pt );
                            old_to_new_index[ idx2 ] = pts.size()-1;
                        }
                        if( old_to_new_index[idx3] == ON_INVALID_INDEX ) {
                            PovMesh::Point pt = pImpl->PTc(j-1,i);
                            pts.push_back( pt );
                            old_to_new_index[ idx3 ] = pts.size()-1;
                        }
                        triangles.push_back( triangle_indices(old_to_new_index[idx1], old_to_new_index[idx2], old_to_new_index[idx3]) );
                    }
                }
            }
        }
    }

    delete[] old_to_new_index;

    const size_t n_valid_points=pts.size();

    // Generate header
    {
        std::ofstream ofs( filename.c_str() );
        if( ofs.fail() )
            return false;

        ofs << "ply" << std::endl;
        ofs << "format binary_little_endian 1.0" << std::endl;
        ofs << "element vertex " << n_valid_points << std::endl;
        ofs << "property float x" << std::endl;
        ofs << "property float y" << std::endl;
        ofs << "property float z" << std::endl;
        ofs << "property uchar red" << std::endl;
        ofs << "property uchar green" << std::endl;
        ofs << "property uchar blue" << std::endl;
        ofs << "element face " << triangles.size() << std::endl;
        ofs << "property list uchar int vertex_index" << std::endl;
        ofs << "end_header" << std::endl;
        ofs.close();
    }

    // Save data
    {
        std::ofstream ofs( filename.c_str(), std::ios::app | std::ios::binary );
        if( ofs.fail() )
            return false;

        size_t datasize = (sizeof(float)*3+sizeof(unsigned char)*3)*n_valid_points;
        char* data = new char[ datasize ];
        char* ptrD=data;

        // Write vertices
        for( size_t i=0; i<pts.size(); ++i )
        {
            const PovMesh::Point& pt = pts[i];
            *((float*)ptrD) = (float)pt.p3d[0]; ptrD+=sizeof(float);
            *((float*)ptrD) = (float)pt.p3d[1]; ptrD+=sizeof(float);
            *((float*)ptrD) = (float)pt.p3d[2]; ptrD+=sizeof(float);
            *((unsigned char*)ptrD) = pt.color[0]; ptrD+=sizeof(unsigned char);
            *((unsigned char*)ptrD) = pt.color[1]; ptrD+=sizeof(unsigned char);
            *((unsigned char*)ptrD) = pt.color[2]; ptrD+=sizeof(unsigned char);
        }

        ofs.write(data,datasize);
        delete[] data;


        // Write indices
        datasize = (sizeof(unsigned char)+sizeof(boost::uint32_t)*3)*triangles.size();
        data = new char[ datasize ];
        ptrD=data;
        for( size_t i=0; i<triangles.size(); ++i )
        {
            const triangle_indices& t = triangles[i];

            *((unsigned char*)ptrD) = 3; ptrD+=sizeof(unsigned char);
            *((boost::uint32_t*)ptrD) = static_cast<boost::uint32_t>(t.id0); ptrD+=sizeof(boost::uint32_t);
            *((boost::uint32_t*)ptrD) = static_cast<boost::uint32_t>(t.id1); ptrD+=sizeof(boost::uint32_t);
            *((boost::uint32_t*)ptrD) = static_cast<boost::uint32_t>(t.id2); ptrD+=sizeof(boost::uint32_t);
        }

        ofs.write(data,datasize);
        ofs.flush();
        ofs.close();
    }


    return true;

}
