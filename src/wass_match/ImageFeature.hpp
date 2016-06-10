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


#ifndef _IMAGEFEATURE_H_
#define _IMAGEFEATURE_H_

#include <opencv2/opencv.hpp>
#include <vector>


namespace WASS
{
namespace match
{

class Feature
{
public:
    explicit Feature( cv::Vec2d _p, float _scale, float _ang ) : position(_p), scale(_scale), angle(_ang)
    {   }

    explicit Feature( float x, float y, float _scale, float _ang ) : scale(_scale), angle(_ang)
    {
        position[0]=x;
        position[1]=y;
    }

    explicit Feature( float* pData, size_t desc_size )
    {
        position[0]=*pData; pData++;
        position[1]=*pData; pData++;
        scale=*pData; pData++;
        angle=*pData; pData++;

        descriptor.resize( desc_size );
        for( size_t i=0; i<desc_size; ++i )
        {
            descriptor[i] = *pData; pData++;
        }
    }

    void copyBinary( float* pData ) const
    {
        *pData = position[0]; pData++;
        *pData = position[1]; pData++;
        *pData = scale; pData++;
        *pData = angle; pData++;

        for( size_t i=0; i<descriptor.size(); ++i )
        {
            *pData = descriptor[i]; pData++;
        }
    }

    size_t size_bytes() const
    {
        return (4+descriptor.size())*sizeof(float);
    }

    static size_t size_bytes( size_t descriptor_size )
    {
        return (4+descriptor_size)*sizeof(float);
    }


    inline double spatial_distance( const Feature& other ) const
    {
        return sqrt( (x()-other.x())*(x()-other.x()) + (y()-other.y())*(y()-other.y()) );
    }

    inline bool operator==( const Feature& other ) const
    {
        return (position==other.position) && (scale==other.scale) && (angle==other.angle);
    }


    cv::Vec2f position;
    float scale;
    float angle;
    std::vector< float > descriptor;

    inline const float x() const { return position[0]; }
    inline const float y() const { return position[1]; }
};

}}

#endif
