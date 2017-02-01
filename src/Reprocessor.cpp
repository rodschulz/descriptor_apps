/**
 * Author: rodrigo
 * 2016
 */
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "Utils.hpp"
#include "Loader.hpp"
#include "Config.hpp"
#include "CloudFactory.hpp"
#include "PointFactory.hpp"
#include "DCH.hpp"
#include "SHOT.hpp"
#include "PFH.hpp"
#include "FPFH.hpp"
#include "SpinImage.hpp"


#define CONFIG_LOCATION "config/config_reprocessor.yaml"
#define LOGGING_LOCATION "config/logging.yaml"


void crawlDirectory(const std::string &directory_,
					std::map<std::string, std::string> &cloudMap_,
					std::map<std::string, std::vector<boost::filesystem::path> > &files_)
{
	boost::filesystem::path target(directory_);
	boost::filesystem::directory_iterator it(target), eod;
	BOOST_FOREACH(boost::filesystem::path const & filepath, std::make_pair(it, eod))
	{
		if (is_regular_file(filepath))
		{
			// Extract object id
			std::string id = filepath.string();
			int index = id.find_last_of('_');
			index = id.find_last_of('_', index - 1);
			id = id.substr(0, index);

			LOGD << "File: " << filepath.filename().string();

			std::string extension = filepath.extension().string();
			if (boost::iequals(extension, ".yaml"))
				files_[id].push_back(filepath);
			else if (boost::iequals(extension, ".pcd"))
			{
				if (cloudMap_.find(id) == cloudMap_.end())
					cloudMap_[id] = filepath.string();
				else
					LOGW << "Duplicated id (" << id << "), ignoring new value";
			}
		}
		else
			crawlDirectory(filepath.string(), cloudMap_, files_);
	}
}

void showPointLoc(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_,
				  const int pointIndex_,
				  const std::string &id_)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored = CloudFactory::createColorCloud(cloud_, Utils::palette12(0));
	(*colored)[pointIndex_].rgba = Utils::getColor(255, 0, 0);

	pcl::io::savePCDFileASCII(OUTPUT_DIR + id_ + "_point.pcd", *colored);
}

void showGrasp(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_,
			   const YAML::Node &pose_,
			   const std::string &id_)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr grasp = CloudFactory::createColorCloud(cloud_, Utils::palette12(0));

	// Add the grasping point to the cloud
	float x = pose_["position"]["x"].as<float>();
	float y = pose_["position"]["y"].as<float>();
	float z = pose_["position"]["z"].as<float>();
	grasp->push_back(PointFactory::createPointXYZRGBNormal(x, y, z, 0, 0, 0, 0, Utils::getColor(255, 0, 0)));

	float dx = pose_["orientation"]["x"].as<float>();
	float dy = pose_["orientation"]["y"].as<float>();
	float dz = pose_["orientation"]["z"].as<float>();
	float dw = pose_["orientation"]["w"].as<float>();

	Eigen::Vector3f origin = Eigen::Vector3f(x, y, z);
	Eigen::Vector3f direction = Eigen::Quaternionf(dw, dx, dy, dz) * Eigen::Vector3f(1, 0, 0);
	Eigen::ParametrizedLine<float, 3> line(origin, direction);

	// Add a set of point showing the orientation
	float delta = 0.2;
	float step = 0.001;
	for (float t = step; t < delta; t += step)
	{
		Eigen::Vector3f p = line.pointAt(t);
		grasp->push_back(PointFactory::createPointXYZRGBNormal(p.x(), p.y(), p.z(), 0, 0, 0, 0, Utils::getColor(0, 255, 0)));
	}

	pcl::io::savePCDFileASCII(OUTPUT_DIR + id_ + "_grasp.pcd", *grasp);
}

int findTargetPoint(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_,
					const YAML::Node &pose_)
{
	// Add the grasping point to the cloud
	float x = pose_["position"]["x"].as<float>();
	float y = pose_["position"]["y"].as<float>();
	float z = pose_["position"]["z"].as<float>();

	float dx = pose_["orientation"]["x"].as<float>();
	float dy = pose_["orientation"]["y"].as<float>();
	float dz = pose_["orientation"]["z"].as<float>();
	float dw = pose_["orientation"]["w"].as<float>();

	Eigen::Vector3f origin = Eigen::Vector3f(x, y, z);
	Eigen::Vector3f direction = Eigen::Quaternionf(dw, dx, dy, dz) * Eigen::Vector3f(1, 0, 0);
	Eigen::ParametrizedLine<float, 3> line(origin, direction);


	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud_);

	int target = -1;
	float distance = std::numeric_limits<float>::max();
	float step = 0.001;
	for (float t = step; t < 0.2; t += step)
	{
		Eigen::Vector3f p = line.pointAt(t);
		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.nearestKSearch(PointFactory::createPointNormal(p.x(), p.y(), p.z(), 0, 0, 0), 1, indices, distances);

		if (target == -1 || distances[0] < distance)
		{
			target = indices[0];
			distance = distances[0];
		}
	}

	return target;
}

void updateFile(const Eigen::VectorXf &descriptor_,
				const int target_,
				const DescriptorParamsPtr &params_,
				YAML::Node &node_)
{
	node_["descriptor"] = params_->toNode();
	node_["descriptor"]["point_index"] =  target_;

	std::vector<float> data(descriptor_.data(), descriptor_.data() + descriptor_.rows() * descriptor_.cols());
	node_["descriptor"]["data"] = data;
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
		Utils::cleanDirectories(workingDir);


		LOGI << "Loading configuration";
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error("Error reading config at " + workingDir + CONFIG_LOCATION);
		DescriptorParamsPtr params = Config::getDescriptorParams();


		LOGI << "Crawling files";
		std::map<std::string, std::string> cloudMap;
		std::map<std::string, std::vector<boost::filesystem::path> > dataFiles;
		crawlDirectory(inputDirectory, cloudMap, dataFiles);


		LOGI << "Processing data";
		bool debug = Config::debugEnabled();
		int processed = 0, generated = 0;
		for (std::map<std::string, std::vector<boost::filesystem::path> >::const_iterator it = dataFiles.begin(); it != dataFiles.end(); it++)
		{
			LOGI << "Loading cloud " << cloudMap[it->first];
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
			if (!Loader::loadCloud(cloudMap[it->first], -1, CloudSmoothingParams(), cloud))
				LOGE << "Can't load cloud " << it->first << ", skipping";


			for (std::vector<boost::filesystem::path>::const_iterator f = it->second.begin(); f != it->second.end(); f++)
			{
				LOGI << "Reprocessing " << f->filename();
				YAML::Node file =  YAML::LoadFile(f->string());
				processed++;


				int target = findTargetPoint(cloud, file["grasp"]["grasp_pose"]["pose"]);
				std::string id = f->stem().string();
				LOGI << "...target point: " << target;

				if (debug)
				{
					showPointLoc(cloud, file["descriptor"]["point_index"].as<int>(), id);
					showGrasp(cloud, file["grasp"]["grasp_pose"]["pose"], id);
					showPointLoc(cloud, target, id);
				}


				LOGI << "...running " + Params::descType[params->type];
				Eigen::VectorXf descriptor;
				switch (params->type)
				{
				default:
					throw std::runtime_error("Descriptor calculation not implemented");

				case Params::DESCRIPTOR_DCH:
				{
					DCHParams *p = dynamic_cast<DCHParams *>(params.get());
					p->angle = M_PI / 2 - file["orientation"]["angle"].as<float>(); // pi/2 to align the zero band
					if (Config::get()["idDebug"].as<bool>())
						DCH::computePoint(cloud, params, target, descriptor, f->stem().string());
					else
						DCH::computePoint(cloud, params, target, descriptor);
				}
				break;

				case Params::DESCRIPTOR_SHOT:
					SHOT::computePoint(cloud, params, target, descriptor);
					break;

				// case Params::DESCRIPTOR_USC:
				// 	USC::computePoint(cloud, params, target, descriptor);
				// 	break;

				case Params::DESCRIPTOR_PFH:
					PFH::computePoint(cloud, params, target, descriptor);
					break;

				case Params::DESCRIPTOR_FPFH:
					FPFH::computePoint(cloud, params, target, descriptor);
					break;

				// case Params::DESCRIPTOR_ROPS:
				// 	ROPS::computePoint(cloud, params, target, descriptor);
				// 	break;

				case Params::DESCRIPTOR_SPIN_IMAGE:
					SpinImage::computePoint(cloud, params, target, descriptor);
					break;
				}


				LOGI << "...generating output file";
				updateFile(descriptor, target, params, file);
				file["reprocessed"] = true;

				std::string outname = OUTPUT_DIR + f->filename().string();
				std::ofstream fout(outname.c_str());
				fout << file;

				generated++;
			}
		}

		LOGI << "Processed " << processed << " files -- Generated " << generated << " files";
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
