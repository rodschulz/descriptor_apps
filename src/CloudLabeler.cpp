/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include "Config.hpp"
#include "DCH.hpp"
#include "ClusteringUtils.hpp"
#include "MetricFactory.hpp"
#include "Loader.hpp"
#include "Writer.hpp"


#define CONFIG_LOCATION "config/config_cloud_labeler.yaml"


int main(int _argn, char **_argv)
{
	static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
	plog::init(plog::info, &consoleAppender);


	// Get the current exec directory
	std::string workingDir = Utils::getWorkingDirectory();


	// Start measuring execution time
	clock_t begin = clock();
	try
	{
		// Check if enough arguments were given
		if (_argn < 2)
			throw std::runtime_error("Not enough exec params given\n\tUsage: CloudLabeler <input_cloud_file>");
		std::string cloudFilename = _argv[1];

		LOGI << "START!";

		// Create the output folder in case it doesn't exists
		if (system("mkdir -p " OUTPUT_DIR) != 0)
			throw std::runtime_error("Can't create the output folder: " + workingDir + OUTPUT_DIR);

		// Clean the output directory
		if (system("rm -rf " OUTPUT_DIR "*") != 0)
			LOGW << "Can't clean output directory: " + workingDir + OUTPUT_DIR;

		// Load the configuration file
		LOGI << "Loading configuration";
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error("Error reading config at " + workingDir + CONFIG_LOCATION);


		// Retrieve useful parameters
		double normalEstimationRadius = Config::getNormalEstimationRadius();
		std::string cacheLocation = Config::getCacheDirectory();
		DescriptorParamsPtr descriptorParams = Config::getDescriptorParams();
		CloudSmoothingParams smoothingParams = Config::getCloudSmoothingParams();


		// Load point cloud
		LOGI << "Loading point cloud at " << cloudFilename;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
		if (!Loader::loadCloud(cloudFilename, normalEstimationRadius, smoothingParams, cloud))
			throw std::runtime_error("Can't load cloud at " + workingDir + cloudFilename);
		LOGI << "...loaded " << cloud->size() << " points in cloud";


		// Descriptor dense evaluation over the point cloud
		LOGI << "Starting descriptor dense evaluation";
		cv::Mat descriptors;
		if (!Loader::loadDescriptors(cacheLocation, cloudFilename, normalEstimationRadius, descriptorParams, smoothingParams, descriptors))
		{
			LOGI << "...cache not found, performing descriptor dense evaluation";
			DCH::calculateDescriptors(cloud, descriptorParams, descriptors);
			Writer::writeDescriptorsCache(descriptors, cacheLocation, cloudFilename, normalEstimationRadius, descriptorParams, smoothingParams);
		}
		LOGI << "...done";


		// Read data from files
		LOGI << "Loading centers for labeling";
		cv::Mat centers;
		std::map<std::string, std::string> metadata;
		if (!Loader::loadMatrix(Config::get()["codebookLocation"].as<std::string>(), centers, &metadata))
			throw std::runtime_error("Unable to load centers file at " + workingDir + Config::get()["codebookLocation"].as<std::string>());


		// Label data only if descriptor matches the given codebook
		cv::Mat labels;
		if (descriptors.cols == centers.cols)
		{
			if (boost::iequals(Config::get()["labeling"]["type"].as<std::string>(), "metric"))
			{
				LOGI << "Performing metric based labeling";
				std::vector<std::string> metricDetails = Config::get()["labeling"]["args"].as<std::vector<std::string> >();
				MetricPtr metric = MetricFactory::create(Metric::toType(metricDetails[0]), metricDetails);
				ClusteringUtils::labelData(descriptors, centers, metric, labels);
			}
			else
			{
				LOGI << "Performing SVM based labeling";
				SVMPtr svm = ClusteringUtils::prepareClassifier(centers, metadata);
				ClusteringUtils::labelData(descriptors, svm, labels);
			}
		}
		else
			throw std::runtime_error("Mismatching dims between computed descriptor and codebook (" + boost::lexical_cast<std::string>(descriptors.cols) + " / " + boost::lexical_cast<std::string>(centers.cols) + ")");


		// Write the labeled cloud
		LOGI << "Writing labeled cloud";
		Writer::writeClusteredCloud(OUTPUT_DIR "cloud" CLOUD_FILE_EXTENSION, cloud, labels);
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
