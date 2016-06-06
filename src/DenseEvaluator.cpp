/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <opencv2/core/core.hpp>
#include "Config.hpp"
#include "Loader.hpp"
#include "Calculator.hpp"
#include "Writer.hpp"
#include "Clustering.hpp"

#define CONFIG_LOCATION "config/config_dense_evaluator.yaml"

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
			throw std::runtime_error("Not enough exec params given\nUsage: DenseEvaluator <input_cloud_file>");
		std::string cloudFilename = _argv[1];

		// Create the output folder in case it doesn't exists
		if (system("mkdir -p " OUTPUT_FOLDER) != 0)
			throw std::runtime_error("can't create the output folder: " + workingDir + OUTPUT_FOLDER);

		// Clean the output directory
		if (system("rm -rf " OUTPUT_FOLDER "*") != 0)
			std::cout << (std::string) "WARNING: can't clean output directory: " + workingDir + OUTPUT_FOLDER << std::endl;

		// Load the configuration file
		std::cout << "Loading configuration" << std::endl;
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error((std::string) "Error reading config at " + workingDir + CONFIG_LOCATION);

		// Retrieve useful parameters
		double normalEstimationRadius = Config::getNormalEstimationRadius();
		std::string cacheLocation = Config::getCacheDirectory();
		DescriptorParams descriptorParams = Config::getDescriptorParams();
		ClusteringParams clusteringParams = Config::getClusteringParams();
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

		std::cout << "Performing data size reduction (clustering)" << std::endl;
		ClusteringResults results;
		Clustering::searchClusters(descriptors, Config::getClusteringParams(), results);

		// Generate outputs
		std::cout << "Writing reduced data" << std::endl;
		Writer::writeClustersCenters(OUTPUT_FOLDER "centers.dat", results.centers, descriptorParams, clusteringParams, smoothingParams);

		if (clusteringParams.generateDistanceMatrix)
			Writer::writeDistanceMatrix(OUTPUT_FOLDER, descriptors, results.centers, results.labels, clusteringParams.metric);

		if (clusteringParams.generateElbowCurve)
			Clustering::generateElbowGraph(descriptors, clusteringParams);
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
