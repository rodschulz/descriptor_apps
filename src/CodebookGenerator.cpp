/**
 * Author: rodrigo
 * 2016
 */
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <opencv2/core/core.hpp>
#include <boost/lexical_cast.hpp>
#include "Clustering.hpp"
#include "Config.hpp"
#include "Loader.hpp"
#include "Writer.hpp"
#include "ClosestPermutationMetric.hpp"


#define CONFIG_LOCATION "config/config_codebook_generator.yaml"
#define LOGGING_LOCATION "config/logging.yaml"


struct CodebookFeatures
{
	Params::DescriptorType type;
	int nbands, nbins, descriptorSize;
	float searchRadius;
	bool bidirectional;
	Params::Statistic stat;
};


std::string generateFilename(const int dataRows_,
							 const int dataCols_,
							 const ClusteringParams &params_,
							 const Params::DescriptorType type_,
							 const Params::Statistic stat_)
{
	std::string statStr = type_ == Params::DESCRIPTOR_DCH ? "_" + Params::toString(stat_) : "";
	std::string str = "codebook";
	str += "_" + Params::descType[type_].substr(11)
		   + "_" + boost::lexical_cast<std::string>(dataRows_) + "-" + boost::lexical_cast<std::string>(dataCols_)
		   + statStr
		   + "_c" + boost::lexical_cast<std::string>(params_.clusterNumber);

	char buffer[50];
	sprintf(buffer, "%1.0E", params_.stopThreshold);
	str += "_" + std::string(buffer);

	switch (params_.implementation)
	{
	case Params::CLUSTERING_OPENCV:
		str += "_opencv";
		break;
	case Params::CLUSTERING_KMEANS:
		str += "_kmeans";
		break;
	case Params::CLUSTERING_STOCHASTIC:
		str += "_stochastic";
		break;
	}
	str += ".dat";

	return str;
}


CodebookFeatures validateCenters(std::vector<std::pair<cv::Mat, std::map<std::string, std::string> > > &centers_)
{
	Params::DescriptorType type = Params::DESCRIPTOR_UNKNOWN;
	int nbands = -1;
	int nbins = -1;
	int ncols = 0;
	float searchRadius = 0;
	bool bidirectional = false;
	Params::Statistic stat = Params::STAT_MEAN;

	for (size_t i = 0; i < centers_.size(); i++)
	{
		LOGD << "type:" << centers_[i].second["type"]
			 << " - nbands:" << centers_[i].second["bandNumber"]
			 << " - searchRadius:" << centers_[i].second["searchRadius"]
			 << " - binNumber:" << centers_[i].second["binNumber"]
			 << " - sequenceBin:" << centers_[i].second["sequenceBin"];

		Params::DescriptorType auxType = DescriptorParams::toType(centers_[i].second["type"]);
		int auxBands = nbands;
		int auxBins = nbins;
		int auxCols = centers_[i].first.cols;
		float auxSearchRadius = searchRadius;
		bool auxBidir = bidirectional;
		Params::Statistic auxStat = stat;

		switch (auxType)
		{
		default:
			std::runtime_error("Wrong descriptor type read from file");

		case Params::DESCRIPTOR_DCH:
			auxBands = boost::lexical_cast<int>(centers_[i].second["bandNumber"]);
			auxSearchRadius = boost::lexical_cast<float>(centers_[i].second["searchRadius"]);
			auxBidir = boost::iequals(centers_[i].second["bidirectional"], "true");
			auxStat = Params::toStatType(centers_[i].second["stat"]);

			if (centers_[i].second.find("binNumber") != centers_[i].second.end())
				auxBins = boost::lexical_cast<int>(centers_[i].second["binNumber"]);
			else
				auxBins = (int)(auxSearchRadius / boost::lexical_cast<float>(centers_[i].second["sequenceBin"]));
			break;

		case Params::DESCRIPTOR_SHOT:
		case Params::DESCRIPTOR_USC:
		case Params::DESCRIPTOR_PFH:
		case Params::DESCRIPTOR_FPFH:
		case Params::DESCRIPTOR_ROPS:
		case Params::DESCRIPTOR_SPIN_IMAGE:
			auxSearchRadius = boost::lexical_cast<float>(centers_[i].second["searchRadius"]);
			break;
		}

		if (i == 0)
		{
			type = auxType;
			nbands = auxBands;
			nbins = auxBins;
			ncols = auxCols;
			searchRadius = auxSearchRadius;
			bidirectional = auxBidir;
			stat = auxStat;
		}
		else
		{
			if (type != auxType)
				throw std::runtime_error("Mixing descriptors (" + Params::descType[type] + " != " + Params::descType[auxType] + ")");
			if (ncols != auxCols)
				throw std::runtime_error((std::string)"Mixing descriptor sizes (" + boost::lexical_cast<std::string>(ncols) + " != " + boost::lexical_cast<std::string>(auxCols) + ")");
			if (nbands != auxBands)
				LOGW << "Mixing number of bands (" << nbands << " != " << auxBands << ")";
			if (nbins != auxBins)
				LOGW << "Mixing bins (" << nbins << " != " << auxBins << ")";
			if (fabs(searchRadius - auxSearchRadius) > 1e-5)
				LOGW << "Mixing search radiuses (" << searchRadius << " != " << auxSearchRadius << ")";
			if (bidirectional != auxBidir)
				LOGW << "Mixing bidirectional and no bidirectional bands";
			if (stat != auxStat)
				LOGW << "Mixing computed statistics (" << Params::stat[stat] << " != " << Params::stat[auxStat] << ")";
		}
	}

	CodebookFeatures features;
	features.type = type;
	features.nbands = nbands;
	features.nbins = nbins;
	features.descriptorSize = ncols;
	features.searchRadius = searchRadius;
	features.bidirectional = bidirectional;
	features.stat = stat;

	return features;
}


bool validateClustering(CodebookFeatures &features_, ClusteringParams &params_)
{
	bool customClustering = params_.implementation == Params::CLUSTERING_KMEANS ||
							params_.implementation == Params::CLUSTERING_STOCHASTIC;
	bool metricClosest = params_.metric->getType() == METRIC_CLOSEST_PERMUTATION ||
						 params_.metric->getType() == METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE;

	bool validConfiguration = true;
	if (customClustering && metricClosest)
	{
		if (features_.type != Params::DESCRIPTOR_DCH)
		{
			LOGE << "Wrong metric-descriptor combination";
			LOGE << "\t"
				 << metricType[params_.metric->getType()]
				 << " can be used only with DCH (current "
				 << Params::descType[features_.type] << ")";
			validConfiguration = false;
		}
		else
		{
			float bandSize = features_.descriptorSize / features_.nbands;
			ClosestPermutationMetric *closestMetric = (ClosestPermutationMetric *)params_.metric.get();
			if (abs(closestMetric->getPermutationSize() - bandSize) > 0)
			{
				LOGE << "Wrong metric params";
				LOGE << "\tbands size mismatch metric config ("
					 << bandSize << " != " << closestMetric->getPermutationSize() << ")";
				validConfiguration = false;
			}
		}
	}

	return validConfiguration;
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
			throw std::runtime_error("Not enough exec params given\n\tUsage: CodebookGenerator <input_directory>");
		std::string inputDirectory = _argv[1];

		LOGI << "START!";
		Utils::cleanDirectories(workingDir);

		// Load the configuration file
		LOGI << "Loading configuration";
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error((std::string) "Error reading config at " + workingDir + CONFIG_LOCATION);


		// Read data from files
		LOGI << "Loading data from files";
		std::pair<int, int> dimensions(0, 0);
		std::vector<std::pair<cv::Mat, std::map<std::string, std::string> > > centers;
		Loader::traverseDirectory(inputDirectory, centers, dimensions);


		// Check if centers are ok to combine
		LOGI << "Checking data consistency";
		CodebookFeatures features = validateCenters(centers);

		LOGI << "Checking configuration consistency";
		ClusteringParams params = Config::getClusteringParams();
		if (!validateClustering(features, params))
			throw std::runtime_error("Invalid clustering configuration");


		// Merge all the centers in one matrix
		LOGI << "Grouping data";
		cv::Mat data = cv::Mat::zeros(dimensions.first, dimensions.second, CV_32FC1);
		int currentRow = 0;
		for (size_t i = 0; i < centers.size(); i++)
		{
			centers[i].first.copyTo(data.rowRange(currentRow, currentRow + centers[i].first.rows));
			currentRow += centers[i].first.rows;
		}


		// Calculate the codebook from the loaded centers
		LOGI << "Calculating codebook (clustering)";
		ClusteringResults results;
		Clustering::searchClusters(data, params, results);


		// Write the codebook to disk
		LOGI << "Writing codebook";
		std::string filename = generateFilename(centers[0].first.rows,
												centers[0].first.cols,
												Config::getClusteringParams(),
												features.type,
												features.stat);
		Writer::writeCodebook(OUTPUT_DIR + filename,
							  results.centers,
							  Config::getClusteringParams(),
							  features.type,
							  features.searchRadius,
							  features.nbands,
							  features.nbins,
							  features.bidirectional);
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
