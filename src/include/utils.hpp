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

        if( boost::filesystem::extension(filename).compare(".xml") == 0 )
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
        else if( boost::filesystem::extension(filename).compare(".txt") == 0 )
        {
            LOGI << "Load TXT not implemented yet";
        }
        else
        {
            LOGE << "Unrecognized extension: " << boost::filesystem::extension( filename );
        }

        return cv::Mat();

    }

}


#endif
