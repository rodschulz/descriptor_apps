/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include "Clustering.hpp"
#include "KMeans.hpp"
#include "MetricFactory.hpp"
#include "Calculator.hpp"
#include "Extractor.hpp"
#include "Hist.hpp"
#include "CloudFactory.hpp"
#include "Loader.hpp"
#include "Writer.hpp"
#include "Config.hpp"
#include "CloudUtils.hpp"

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
			throw std::runtime_error("Not enough exec params given\nUsage: Descriptor <input_file>");

		// Clean the output directory
		if (system("rm -rf " OUTPUT_FOLDER "*") != 0)
			std::cout << (std::string) "WARNING: can't clean output directory: " + workingDir + OUTPUT_FOLDER << std::endl;

		// Load the configuration file
		std::cout << "Loading configuration" << std::endl;
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error((std::string) "Error reading config at " + workingDir + CONFIG_LOCATION);

		// Load point cloud
		std::cout << "Loading point cloud at " << _argv[1] << std::endl;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
		if (!Loader::loadCloud(_argv[1], Config::getNormalEstimationRadius(), Config::getCloudSmoothingParams(), cloud))
			throw std::runtime_error("Can't load cloud at " + workingDir + _argv[1]);
		std::cout << "...loaded " << cloud->size() << " points in cloud" << std::endl;

		// Descriptor dense evaluation over the point cloud
		std::cout << "Starting descriptor dense evaluation" << std::endl;
		cv::Mat descriptors;
//		if (!Loader::loadDescriptors(descriptors, params))
//		{
//			std::cout << "...Cache not found, calculating descriptors\n";
		Calculator::calculateDescriptors(cloud, Config::getDescriptorParams(), descriptors);
//		Writer::writeDescriptorsCache(descriptors, params);

//		}

		std::cout << "Performing data size reduction (clustering)" << std::endl;
		ClusteringResults results;
		MetricPtr metric = MetricFactory::createMetric(params.metric, params.getSequenceLength(), params.useConfidence);
//			if (!params.labelData)
//			{
				Clustering::searchClusters(descriptors, Config::getClusteringParams(), metric, results);
				std::cout << "Generating SSE plot" << std::endl;
				Writer::writePlotSSE("sse", "SSE Evolution", results.errorEvolution);
//			}
//			else
//			{
//				std::cout << "Loading centers" << std::endl;
//
//				if (!Loader::loadCenters(params.centersLocation, results.centers))
//					throw std::runtime_error("Can't load clusters centers");
//
//				Clustering::labelData(descriptors, results.centers, params, results.labels);
//			}

// Generate outputs
//			std::cout << "Writing outputs" << std::endl;
//			pcl::io::savePCDFileASCII("./output/visualization.pcd", *Clustering::generateClusterRepresentation(cloud, results.labels, results.centers, params));
//			Writer::writeClusteredCloud("./output/clusters.pcd", cloud, results.labels);
//			Writer::writeClustersCenters("./output/", results.centers);
//
//			if (params.genDistanceMatrix)
//				Writer::writeDistanceMatrix("./output/", descriptors, results.centers, results.labels, metric);
//			if (params.genElbowCurve)
//				Clustering::generateElbowGraph(descriptors, params);
//		}
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
