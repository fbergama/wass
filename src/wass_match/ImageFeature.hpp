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
