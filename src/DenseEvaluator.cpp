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
#include <boost/lexical_cast.hpp>
#include "Config.hpp"
#include "Loader.hpp"
#include "DCH.hpp"
#include "Writer.hpp"
#include "Clustering.hpp"
#include "SHOT.hpp"


#define CONFIG_LOCATION "config/config_dense_evaluator.yaml"
#define LOGGING_LOCATION "config/logging.yaml"


std::string genFilename(const int rows_, const int cols_, const DescriptorType type_)
{
	std::string filename = "centers_"
						   + descType[type_].substr(11)
						   + "_" + boost::lexical_cast<std::string>(rows_)
						   + "_" + boost::lexical_cast<std::string>(cols_);
	return filename;
}


int main(int _argn, char **_argv)
{
	static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
	plog::init(plog::severityFromString(YAML::LoadFile(LOGGING_LOCATION)["level"].as<std::string>().c_str()), &consoleAppender);


	// Get the current exec directory
	std::string workingDir = Utils::getWorkingDirectory();

	// Start measuring execution time
	clock_t begin = clock();
	try
	{
		// Check if enough arguments were given
		if (_argn < 2)
			throw std::runtime_error("Not enough exec params given\n\tUsage: DenseEvaluator <input_cloud_file>");
		std::string cloudFilename = _argv[1];

		LOGI << "START!";

		// Create the output folder in case it doesn't exists
		if (system("mkdir -p " OUTPUT_DIR) != 0)
			throw std::runtime_error("Can't create the output folder: " + workingDir + OUTPUT_DIR);

		// Clean output and debug directory
		if (system("rm -rf " OUTPUT_DIR "*") != 0)
			LOGW << "Can't clean output directory: " + workingDir + OUTPUT_DIR;
		if (system("rm -rf " DEBUG_DIR "*") != 0)
			LOGW << "Can't clean debug directory: " + workingDir + DEBUG_DIR;

		// Load the configuration file
		LOGI << "Loading configuration";
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error((std::string) "Error reading config at " + workingDir + CONFIG_LOCATION);

		// Retrieve useful parameters
		double normalEstimationRadius = Config::getNormalEstimationRadius();
		std::string cacheLocation = Config::getCacheDirectory();
		DescriptorParamsPtr descriptorParams = Config::getDescriptorParams();
		ClusteringParams clusteringParams = Config::getClusteringParams();
		CloudSmoothingParams smoothingParams = Config::getCloudSmoothingParams();

		// Load point cloud
		LOGI << "Loading point cloud at " << cloudFilename;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
		if (!Loader::loadCloud(cloudFilename, normalEstimationRadius, smoothingParams, cloud))
			throw std::runtime_error("Can't load cloud at " + workingDir + cloudFilename);
		LOGI << "...loaded " << cloud->size() << " points in cloud";

		// Dense evaluation over the point cloud
		LOGI << "Starting dense evaluation (" << descType[descriptorParams->type] << ")";
		cv::Mat descriptors;
		if (!Loader::loadDescriptors(cacheLocation, cloudFilename, normalEstimationRadius, descriptorParams, smoothingParams, descriptors))
		{
			LOGI << "...cache not found, performing dense evaluation";

			switch (descriptorParams->type)
			{
			default:
				LOGW << "WARNING: wrong descriptor type, assuming DCH";
			case DESCRIPTOR_DCH:
				DCH::calculateDescriptors(cloud, descriptorParams, descriptors);
				break;

			case DESCRIPTOR_SHOT:
				SHOT::computeDense(cloud, descriptorParams, descriptors);
				break;
			}
			Writer::writeDescriptorsCache(descriptors, cacheLocation, cloudFilename, normalEstimationRadius, descriptorParams, smoothingParams);
		}

		LOGI << "Performing data size reduction";
		ClusteringResults results;
		Clustering::searchClusters(descriptors, clusteringParams, results);


		// Generate outputs
		LOGI << "Writing reduced data";
		std::string filename = genFilename(results.centers.rows, results.centers.cols, descriptorParams->type);
		Writer::writeClustersCenters(OUTPUT_DIR + filename,
									 results.centers,
									 descriptorParams,
									 clusteringParams,
									 smoothingParams);


		if (clusteringParams.generateDistanceMatrix)
			Writer::writeDistanceMatrix(OUTPUT_DIR,
										descriptors,
										results.centers,
										results.labels,
										clusteringParams.metric);

		if (clusteringParams.generateElbowCurve)
			Clustering::generateElbowGraph(descriptors,
										   clusteringParams);
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
