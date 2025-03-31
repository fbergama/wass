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


#ifndef _WASS_UTILS_HPP_
#define _WASS_UTILS_HPP_

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <log.hpp>

namespace WASS
{

    inline cv::Mat load_matrix( boost::filesystem::path filename )
    {
        if( !filename.has_extension() )
        {
            LOGE << filename << " has no extension";
            return cv::Mat();
        }

        if( boost::filesystem::path(filename).extension().compare(".xml") == 0 )
        {
            cv::FileStorage fs;
            boost::filesystem::path currfile;
            if( !fs.open( filename.string(), cv::FileStorage::READ ) )
            {
                LOGE << "Unable to load " << filename;
                return cv::Mat();
            }

            cv::Mat M;
            fs.getFirstTopLevelNode() >> M;
            fs.release();
            return M;
        }
        else if( boost::filesystem::path(filename).extension().compare(".txt") == 0 )
        {
            LOGI << "Load TXT not implemented yet";
        }
        else
        {
            LOGE << "Unrecognized extension: " << boost::filesystem::path(filename).extension();
        }

        return cv::Mat();

    }


    template <typename MatrixType>
    bool save_matrix_txt( std::string filename, const cv::Mat& m )
    {
        std::ofstream ofs( filename.c_str() );
        if( ofs.fail() )
            return false;

        ofs.precision(16);
        ofs << std::scientific;

        for( int i=0; i<m.rows; ++i )
        {
            for( int j=0; j<m.cols; ++j )
            {
                ofs << m.at<MatrixType>(i,j);
                if( j!=m.cols-1 )
                    ofs << " ";
            }
            if( i!=m.rows-1)
                ofs << std::endl;
        }

        return true;
    }


}


#endif
