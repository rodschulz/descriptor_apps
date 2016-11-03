/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include "Calculator.hpp"
#include "CloudFactory.hpp"
#include "Loader.hpp"
#include "Writer.hpp"
#include "Config.hpp"


#define CONFIG_LOCATION "config/config_descriptor.yaml"


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

		// Evaluate the descriptor around the target point
		std::cout << "...calculating descriptor" << std::endl;
		int targetPoint = Config::getTargetPoint();
		DescriptorParams descriptorParams = Config::getDescriptorParams();
		Descriptor descriptor = Calculator::calculateDescriptor(cloud, descriptorParams, targetPoint);

		// Generate histograms
		std::cout << "...generating histograms" << std::endl;
		std::vector<Hist> histograms = Calculator::generateAngleHistograms(descriptor, descriptorParams.useProjection);

		// Write output
		std::cout << "...writing output" << std::endl;
		Writer::writeOuputData(cloud, descriptor, histograms, descriptorParams, targetPoint);


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
