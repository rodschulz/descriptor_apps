/**
 * Author: rodrigo
 * 2016
 */
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include "Utils.hpp"


#define LOGGING_LOCATION "config/logging.yaml"


void crawlDirectory(const std::string &directory_,
					std::map<std::string, std::string> &cloudMap_,
					std::map<std::string, std::vector<std::string> > &files_)
{
	boost::filesystem::path target(directory_);
	boost::filesystem::directory_iterator it(target), eod;
	BOOST_FOREACH(boost::filesystem::path const & filePath, std::make_pair(it, eod))
	{
		if (is_regular_file(filePath))
		{
			// Extract object id
			std::string id = filePath.filename().string();
			int index = id.find_last_of('_');
			index = id.find_last_of('_', index - 1);
			id = id.substr(0, index);

			LOGD << "File: " << filePath.filename().string();

			std::string extension = filePath.extension().string();
			if (boost::iequals(extension, ".yaml"))
				files_[id].push_back(filePath.string());
			else if (boost::iequals(extension, ".pcd"))
			{
				if (cloudMap_.find(id) == cloudMap_.end())
					cloudMap_[id] = filePath.string();
				else
					LOGW << "Duplicated id (" << id << "), ignoring new value";
			}
		}
		else
			crawlDirectory(filePath.string(), cloudMap_, files_);
	}
}


int main(int _argn, char **_argv)
{
	static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
	plog::init(plog::severityFromString(YAML::LoadFile(LOGGING_LOCATION)["level"].as<std::string>().c_str()), &consoleAppender);


	std::string workingDir = Utils::getWorkingDirectory();
	clock_t begin = clock();
	try
	{
		// Check if enough arguments were given
		if (_argn < 2)
			throw std::runtime_error("Not enough params given\n\tUsage: Reprocessor <input_directory>");
		std::string inputDirectory = _argv[1];

		LOGI << "START!";

		LOGI << "Crawling of files";
		std::map<std::string, std::string> cloudMap;
		std::map<std::string, std::vector<std::string> > dataFiles;
		crawlDirectory(inputDirectory, cloudMap, dataFiles);

		// LOGD << "CHECK!";
		// for (std::map<std::string, std::string>::const_iterator it = cloudMap.begin(); it != cloudMap.end(); it++)
		// {
		// 	LOGD << "=====" << it->first << "=====";
		// 	for (size_t i = 0; i < dataFiles[it->first].size(); i++)
		// 		LOGD << dataFiles[it->first][i];
		// }

		for (std::map<std::string, std::string>::const_iterator it = cloudMap.begin(); it != cloudMap.end(); it++)
		{
		}
	}
	catch (std::exception &_ex)
	{
		LOGE << _ex.what();
	}

	clock_t end = clock();
	double elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
	LOGI << std::fixed << std::setprecision(3) << "Finished in " << elapsedTime << " [s]";

	return EXIT_SUCCESS;
}
