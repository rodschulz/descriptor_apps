/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
// #include <opencv2/core/core.hpp>
// #include <pcl/io/pcd_io.h>
// #include "Clustering.hpp"
// #include "KMeans.hpp"
// #include "MetricFactory.hpp"
#include "Calculator.hpp"
// #include "Extractor.hpp"
// #include "Hist.hpp"
#include "CloudFactory.hpp"
#include "Loader.hpp"
// #include "Writer.hpp"
#include "Config.hpp"
// #include "CloudUtils.hpp"


#define CONFIG_LOCATION "./config/config.yaml"


void genPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_, const SynCloudType cloudType_)
{
	std::cout << "Generating cloud: " << cloudType[cloudType_] << "\n";

	switch (cloudType_)
	{
	default:
		std::cout << "WARNING, wrong synthetic cloud params, assuming cube\n";

	case CLOUD_CUBE:
		cloud_ = CloudFactory::createCube(5, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case CLOUD_CYLINDER:
		cloud_ = CloudFactory::createCylinderSection(2 * M_PI, 5, 10, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case CLOUD_SPHERE:
		cloud_ = CloudFactory::createSphereSection(2 * M_PI, 10, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case CLOUD_HALF_SPHERE:
		cloud_ = CloudFactory::createSphereSection(M_PI, 10, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case CLOUD_PLANE:
		cloud_ = CloudFactory::createHorizontalPlane(-50, 50, 200, 300, 30, 20000);
		break;
	}
}


int main(int _argn, char **_argv)
{
	// Get the current exec directory
	std::string workingDir = Utils::getWorkingDirectory();

	// Start measuring execution time
	clock_t begin = clock();
	try
	{
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

		// Load or generate the cloud
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
		SyntheticCloudsParams synCloudParams = Config::getSyntheticCloudParams();
		if (synCloudParams.useSynthetic)
			genPointCloud(cloud, synCloudParams.synCloudType);
		else
		{
			// Check if enough params were given
			if (_argn < 2)
				throw std::runtime_error("Not enough exec params given\nUsage: Descriptor <input_file>");

			std::string cloudFilename = _argv[1];

			// Retrieve useful parameters
			double normalEstimationRadius = Config::getNormalEstimationRadius();
			CloudSmoothingParams smoothingParams = Config::getCloudSmoothingParams();

			// Load the point cloud
			if (!Loader::loadCloud(cloudFilename, normalEstimationRadius, smoothingParams, cloud))
				throw std::runtime_error("Can't load cloud at " + workingDir + cloudFilename);
		}



		std::cout << "...calculating descriptor" << std::endl;
		DescriptorParams descriptorParams = Config::getDescriptorParams();
		Descriptor descriptor = Calculator::calculateDescriptor(cloud, descriptorParams, Config::getTargetPoint());



	}
	catch (std::exception &_ex)
	{
		std::cout << "ERROR: " << _ex.what() << std::endl;
	}

	clock_t end = clock();
	double elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << std::fixed << std::setprecision(3) << "Finished in " << elapsedTime << " [s]\n";

	return EXIT_SUCCESS;



//	try
//	{
//		if (system("rm -rf ./output/*") != 0)
//			std::cout << "WARNING: can't clean output folder\n";
//
//		std::cout << "Loading configuration file\n";
//		if (!Config::load(CONFIG_LOCATION))
//			throw std::runtime_error("Problem reading configuration file at " CONFIG_LOCATION);
//		ExecutionParams params = Config::getExecutionParams();
//
//		// Check if enough params were given
//		if (_argn < 2 && !params.useSynthetic)
//			throw std::runtime_error("Not enough exec params given\nUsage: Descriptor <input_file>");
//
//		if (!params.useSynthetic)
//			params.inputLocation = _argv[1];
//
//		// Do things according to the execution type
//		if (params.executionType == EXECUTION_METRIC)
//			Metric::evaluateMetricCases("./output/metricEvaluation", params.inputLocation, params.targetMetric, params.metricArgs);
//		else
//		{
//			// Load cloud
//			pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
//			if (!getPointCloud(cloud, params))
//				throw std::runtime_error("Can't load cloud");
//
//			// Select execution type
//			if (params.executionType == EXECUTION_DESCRIPTOR)
//			{
//				std::cout << "...Execution for descriptor calculation\n";
//
//				std::cout << "Target point: " << params.targetPoint << "\n";
//				Descriptor descriptor = Calculator::calculateDescriptor(cloud, params);
//
//				// Calculate histograms
//				std::cout << "Generating histograms\n";
//				std::vector<Hist> histograms = Calculator::generateAngleHistograms(descriptor, params.useProjection);
//
//				// Write output
//				std::cout << "Writing output\n";
//				Writer::writeOuputData(cloud, descriptor, histograms, params);
//			}
//			else if (params.executionType == EXECUTION_CLUSTERING)
//			{
//				std::cout << "...Execution for clustering\n";
//
//				cv::Mat descriptors;
//				if (!Loader::loadDescriptors(descriptors, params))
//				{
//					std::cout << "Cache not found, calculating descriptors\n";
//					Calculator::calculateDescriptors(cloud, params, descriptors);
//					Writer::writeDescriptorsCache(descriptors, params);
//				}
//
//				ClusteringResults results;
//				MetricPtr metric = MetricFactory::createMetric(params.metric, params.getSequenceLength(), params.useConfidence);
//				if (!params.labelData)
//				{
//					Clustering::searchClusters(descriptors, params, results);
//
//					std::cout << "Generating SSE plot" << std::endl;
//					Writer::writePlotSSE("sse", "SSE Evolution", results.errorEvolution);
//				}
//				else
//				{
//					std::cout << "Loading centers" << std::endl;
//
//					if (!Loader::loadCenters(params.centersLocation, results.centers))
//						throw std::runtime_error("Can't load clusters centers");
//
//					Clustering::labelData(descriptors, results.centers, params, results.labels);
//				}
//
//				// Generate outputs
//				std::cout << "Writing outputs" << std::endl;
//				pcl::io::savePCDFileASCII("./output/visualization.pcd", *Clustering::generateClusterRepresentation(cloud, results.labels, results.centers, params));
//				Writer::writeClusteredCloud("./output/clusters.pcd", cloud, results.labels);
//				Writer::writeClustersCenters("./output/", results.centers);
//
//				if (params.genDistanceMatrix)
//					Writer::writeDistanceMatrix("./output/", descriptors, results.centers, results.labels, metric);
//				if (params.genElbowCurve)
//					Clustering::generateElbowGraph(descriptors, params);
//			}
//		}
//	}
//	catch (std::exception &_ex)
//	{
//		std::cout << "ERROR: " << _ex.what() << std::endl;
//	}

	// clock_t end = clock();
	// double elapsedTime = double(end - begin) / CLOCKS_PER_SEC;

	// std::cout << std::fixed << std::setprecision(3) << "Finished in " << elapsedTime << " [s]\n";
	// return EXIT_SUCCESS;
}
