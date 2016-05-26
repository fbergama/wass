#ifndef _WASS_LOG_HPP_
#define _WASS_LOG_HPP_

#include <fstream>
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
#define LOGI BOOST_LOG_TRIVIAL(info)
#define LOGE BOOST_LOG_TRIVIAL(error)


namespace WASS
{
    BOOST_LOG_ATTRIBUTE_KEYWORD(scope, "Scope", boost::log::attributes::named_scope::value_type )

    inline void setup_logger( std::string logfilename = std::string() )
    {
        boost::shared_ptr< boost::log::core > core = boost::log::core::get();
        core->add_global_attribute("Scope", boost::log::attributes::named_scope());

        typedef boost::log::sinks::synchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;
        boost::shared_ptr< text_sink > sink = boost::make_shared< text_sink >();

        if( logfilename.length()>1 )
            sink->locked_backend()->add_stream( boost::make_shared< std::ofstream >(logfilename.c_str() ));

        sink->locked_backend()->add_stream( boost::shared_ptr< std::ostream >(&std::cout, boost::null_deleter()) );

        sink->set_formatter( boost::log::expressions::stream << "(" << scope << ") " << boost::log::trivial::severity << ": " << boost::log::expressions::smessage );

        boost::log::core::get()->add_sink(sink);
    }
}

#endif

