/**
 * Author: rodrigo
 * 2015
 */
#include <boost/lexical_cast.hpp>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include "DCH.hpp"
#include "CloudFactory.hpp"
#include "Loader.hpp"
#include "Writer.hpp"
#include "Config.hpp"


#define CONFIG_LOCATION "config/config_descriptor.yaml"
#define LOGGING_LOCATION "config/logging.yaml"


void genPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
				   const Params::SynCloudType cloudType_)
{
	LOGI << "Generating cloud: " << Params::cloudType[cloudType_];

	switch (cloudType_)
	{
	default:
		LOGW << "Wrong synthetic cloud params, assuming cube";

	case Params::CLOUD_CUBE:
		cloud_ = CloudFactory::createCube(5, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case Params::CLOUD_CYLINDER:
		cloud_ = CloudFactory::createCylinderSection(2 * M_PI, 5, 10, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case Params::CLOUD_SPHERE:
		cloud_ = CloudFactory::createSphereSection(2 * M_PI, 10, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case Params::CLOUD_HALF_SPHERE:
		cloud_ = CloudFactory::createSphereSection(M_PI, 10, Eigen::Vector3f(0, 0, 0), 20000);
		break;

	case Params::CLOUD_PLANE:
		cloud_ = CloudFactory::createHorizontalPlane(-50, 50, 200, 300, 30, 20000);
		break;
	}
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
		LOGI << "START!";
		Utils::cleanDirectories(workingDir);

		// Load the configuration file
		LOGI << "Loading configuration";
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
				throw std::runtime_error("Not enough exec params given\n\tUsage: Descriptor <input_file>");

			std::string cloudFilename = _argv[1];


			// Retrieve useful parameters
			double normalEstimationRadius = Config::getNormalEstimationRadius();
			CloudSmoothingParams smoothingParams = Config::getCloudSmoothingParams();


			// Load the point cloud
			if (!Loader::loadCloud(cloudFilename, normalEstimationRadius, smoothingParams, cloud))
				throw std::runtime_error("Can't load cloud at " + workingDir + cloudFilename);
		}


		int targetPoint = Config::getTargetPoint();
		DescriptorParamsPtr descriptorParams = Config::getDescriptorParams();
		DCHParams *params = dynamic_cast<DCHParams *>(descriptorParams.get());
		params->angle = DEG2RAD(Config::get()["descriptor"]["DCH"]["angle"].as<float>(0));
		LOGD << "Runtime angle: " << params->angle;

		if (params)
		{
			if (targetPoint < 0 || targetPoint >= (int)cloud->size())
				throw std::runtime_error("Target point out of range (cloud size: " + boost::lexical_cast<std::string>(cloud->size()) + ")");


			// Evaluate the descriptor around the target point
			LOGI << "...calculating descriptor at " << targetPoint;
			std::vector<BandPtr> descriptor = DCH::calculateDescriptor(cloud, descriptorParams, targetPoint);


			// Write output
			LOGI << "...writing output";
			Writer::writeOuputData(cloud, descriptor, descriptorParams, targetPoint);
		}
		else
			LOGW << "App only works with DCH descriptor";
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
