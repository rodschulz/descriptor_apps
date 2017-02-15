/**
 * Author: rodrigo
 * 2016
 */
#define BOOST_TEST_MODULE TestMain

#include <boost/test/included/unit_test.hpp>
#include <yaml-cpp/yaml.h>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>


#define LOGGING_LOCATION "config/logging.yaml"


struct LogggingSetup
{
	LogggingSetup()
	{
		static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
		plog::init(plog::severityFromString(YAML::LoadFile(LOGGING_LOCATION)["level"].as<std::string>().c_str()), &consoleAppender);
	}
	~LogggingSetup()  {
	}
};


BOOST_GLOBAL_FIXTURE(LogggingSetup);
