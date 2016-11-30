/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include "Config.hpp"
#include "Calculator.hpp"
#include "ClusteringUtils.hpp"
#include "MetricFactory.hpp"
#include "Loader.hpp"
#include "Writer.hpp"
#include <pcl/io/pcd_io.h>


#define CONFIG_LOCATION "config/config_cloud_labeler.yaml"


int main(int _argn, char **_argv)
{
	// Get the current exec directory
	std::string workingDir = Utils::getWorkingDirectory();

	// Start measuring execution time
	clock_t begin = clock();
	try
	{
		// Check if enough arguments were given
		if (_argn < 2)
			throw std::runtime_error("Not enough exec params given\nUsage: CloudLabeler <input_cloud_file>");
		std::string cloudFilename = _argv[1];

		// Create the output folder in case it doesn't exists
		if (system("mkdir -p " OUTPUT_DIR) != 0)
			throw std::runtime_error("can't create the output folder: " + workingDir + OUTPUT_DIR);

		// Clean the output directory
		if (system("rm -rf " OUTPUT_DIR "*") != 0)
			std::cout << (std::string) "WARNING: can't clean output directory: " + workingDir + OUTPUT_DIR << std::endl;

		// Load the configuration file
		std::cout << "Loading configuration" << std::endl;
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error((std::string) "Error reading config at " + workingDir + CONFIG_LOCATION);

		// Retrieve useful parameters
		double normalEstimationRadius = Config::getNormalEstimationRadius();
		std::string cacheLocation = Config::getCacheDirectory();
		DescriptorParams descriptorParams = Config::getDescriptorParams();
		CloudSmoothingParams smoothingParams = Config::getCloudSmoothingParams();

		// Load point cloud
		std::cout << "Loading point cloud at " << cloudFilename << std::endl;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
		if (!Loader::loadCloud(cloudFilename, normalEstimationRadius, smoothingParams, cloud))
			throw std::runtime_error("Can't load cloud at " + workingDir + cloudFilename);
		std::cout << "...loaded " << cloud->size() << " points in cloud" << std::endl;

		// Descriptor dense evaluation over the point cloud
		std::cout << "Starting descriptor dense evaluation" << std::endl;
		cv::Mat descriptors;
		if (!Loader::loadDescriptors(cacheLocation, cloudFilename, normalEstimationRadius, descriptorParams, smoothingParams, descriptors))
		{
			std::cout << "...cache not found, performing descriptor dense evaluation" << std::endl;
			Calculator::calculateDescriptors(cloud, descriptorParams, descriptors);
			Writer::writeDescriptorsCache(descriptors, cacheLocation, cloudFilename, normalEstimationRadius, descriptorParams, smoothingParams);
		}
		std::cout << "...done" << std::endl;

		// Read data from files
		std::cout << "Loading centers for labeling" << std::endl;
		cv::Mat centers;
		std::map<std::string, std::string> metadata;
		if (!Loader::loadMatrix(Config::get()["centersLocation"].as<std::string>(), centers, &metadata))
			throw std::runtime_error("Unable to load centers file at " + workingDir + Config::get()["centersLocation"].as<std::string>());

		// Perform the labeling
		cv::Mat labels;
		if (boost::iequals(Config::get()["labeling"]["type"].as<std::string>(), "metric"))
		{
			std::cout << "Performing metric based labeling" << std::endl;
			std::vector<std::string> metricDetails = Config::get()["labeling"]["args"].as<std::vector<std::string> >();
			MetricPtr metric = MetricFactory::createMetric(Utils::getMetricType(metricDetails[0]), metricDetails);
			ClusteringUtils::labelData(descriptors, centers, metric, labels);
		}
		else
		{
			std::cout << "Performing SVM based labeling" << std::endl;
			CvSVMPtr svm = ClusteringUtils::prepareClassifier(centers, metadata);
			ClusteringUtils::labelData(descriptors, svm, labels);
		}

		// Write the labeled cloud
		std::cout << "Writing labeled cloud" << std::endl;
		Writer::writeClusteredCloud(OUTPUT_DIR "cloud" CLOUD_FILE_EXTENSION, cloud, labels);
	}
	catch (std::exception &_ex)
	{
		std::cout << "ERROR: " << _ex.what() << std::endl;
	}

	clock_t end = clock();
	double elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << std::fixed << std::setprecision(3) << "Finished in " << elapsedTime << " [s]\n";

	return EXIT_SUCCESS;
}
