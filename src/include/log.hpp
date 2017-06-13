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

#ifndef _WASS_LOG_HPP_
#define _WASS_LOG_HPP_

#include <iostream>
#include <fstream>

#ifdef WASS_USE_BOOST_LOG

#include <iomanip>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/keywords/severity.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/expressions.hpp>
#include <boost/core/null_deleter.hpp>

#define LOG_SCOPE(name) BOOST_LOG_NAMED_SCOPE(name)
#define LOG(a) BOOST_LOG_TRIVIAL(a)
#define LOGI BOOST_LOG_TRIVIAL(info )
#define LOGE BOOST_LOG_TRIVIAL(error)
#define LOGW BOOST_LOG_TRIVIAL(warn )


namespace WASS
{
    BOOST_LOG_ATTRIBUTE_KEYWORD(scope, "Scope", boost::log::attributes::named_scope::value_type )

    inline void setup_logger( std::string logfilename = std::string() )
    {
        boost::shared_ptr< boost::log::core > core = boost::log::core::get();
        core->add_global_attribute("Scope", boost::log::attributes::named_scope());

        typedef boost::log::sinks::synchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;
        boost::shared_ptr< text_sink > sink = boost::make_shared< text_sink >();

        if( logfilename.length()>=1 )
        {
            boost::shared_ptr< std::ofstream > ofs = boost::make_shared< std::ofstream >(  logfilename.c_str() );
            if( ofs && !ofs->fail() )
                sink->locked_backend()->add_stream( ofs );
        }

        sink->locked_backend()->add_stream( boost::shared_ptr< std::ostream >(&std::cout, boost::null_deleter()) );

        sink->set_formatter( boost::log::expressions::stream << scope << " (" << boost::log::trivial::severity << "): " << boost::log::expressions::smessage );

        sink->locked_backend()->auto_flush(true);

        core->add_sink(sink);
    }
}

#else

namespace WASS
{

    struct NaiveLoggerProperties
    {
        std::string scope;
        std::string severity;
        std::ostream* outs;
        NaiveLoggerProperties() : outs( 0 ) {}

        static inline NaiveLoggerProperties& instance()
        {
            static NaiveLoggerProperties myself;
            return myself;
        }
        static inline NaiveLoggerProperties& instance( std::string _severity )
        {
            NaiveLoggerProperties& prop = instance();
            prop.severity = _severity;
            return prop;
        }
    };

    struct NaiveLogger
    {
        NaiveLoggerProperties prp;

        inline void log_prefix()
        {
            std::cout << prp.scope << " [" << prp.severity << "] ";
            if( prp.outs )
                (*prp.outs) << prp.scope << " [" << prp.severity << "] ";
        }

        inline NaiveLogger( )
        {
            prp = NaiveLoggerProperties::instance();
            log_prefix();
        }

        inline NaiveLogger( std::string _severity )
        {
            prp = NaiveLoggerProperties::instance( _severity );
            log_prefix();
        }

        inline ~NaiveLogger() // Log suffix
        {
            std::cout << std::endl;
            if( prp.outs )
                (*prp.outs) << std::endl;
        }

        template< typename T >
        NaiveLogger& operator<<( T in )
        {

            std::cout << in;
            if( prp.outs )
                (*prp.outs) << in;

            return *this;
        }

        NaiveLogger& operator<<( std::ostream&(*pManip)(std::ostream&) )
        {
            (*pManip)( std::cout );

            if( prp.outs )
                (*pManip)( *prp.outs );

            return *this;
        }

    };


    inline void setup_logger( std::string logfilename = std::string() )
    {
        if( logfilename.length()>=1 )
        {
            NaiveLoggerProperties::instance().outs = new std::ofstream( logfilename.c_str() );
        }
    }
}

#define LOG_SCOPE(name) (WASS::NaiveLoggerProperties::instance().scope=std::string(name))
#define LOG(a) WASS::NaiveLogger::instance(a)
#define LOGI WASS::NaiveLogger("info ")
#define LOGE WASS::NaiveLogger("error")
#define LOGW WASS::NaiveLogger("warn ")

#endif

#endif

