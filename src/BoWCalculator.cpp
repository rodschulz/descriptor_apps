/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <opencv2/core/core.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "Clustering.hpp"
#include "Config.hpp"
#include "Loader.hpp"
#include "Writer.hpp"

#define CONFIG_LOCATION "config/config_bow_calculator.yaml"

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
			throw std::runtime_error("Not enough exec params given\nUsage: BoWCalculator <input_directory>");
		std::string inputDirectory = _argv[1];

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

		// Read data from files
		std::cout << "Loading data from files" << std::endl;
		std::pair<int, int> dimensions(0, 0);
		std::vector<std::pair<cv::Mat, std::map<std::string, std::string> > > centers;
		Loader::traverseDirectory(inputDirectory, centers, dimensions);

		// Check if centers are ok to combine
		std::cout << "Checking consistency" << std::endl;
		int nbands = -1;
		int nbins = -1;
		bool bidirectional = false;
		for (size_t i = 0; i < centers.size(); i++)
		{
			int auxBands = boost::lexical_cast<int>(centers[i].second["bandNumber"]);
			int auxBins = (int)(boost::lexical_cast<float>(centers[i].second["patchSize"]) / boost::lexical_cast<float>(centers[i].second["sequenceBin"]));
			bool auxBidir = boost::iequals(centers[i].second["bidirectional"], "true");

			if (i == 0)
			{
				nbands = auxBands;
				nbins = auxBins;
				bidirectional = auxBidir;
			}
			else
			{
				if (nbands != auxBands)
					std::cout << "WARNING: mixing number of bands (" << nbands << " != " << auxBands << ")" << std::endl;
				if (nbins != auxBins)
					std::cout << "WARNING: mixing bins (" << nbins << " != " << auxBins << ")" << std::endl;
				if (bidirectional != auxBidir)
					std::cout << "WARNING: mixing bidirectional and no bidirectional bands" << std::endl;
			}
		}

		// Merge all the centers in one matrix
		std::cout << "Groupping data" << std::endl;
		cv::Mat data = cv::Mat::zeros(dimensions.first, dimensions.second, CV_32FC1);
		int currentRow = 0;
		for (size_t i = 0; i < centers.size(); i++)
		{
			centers[i].first.copyTo(data.rowRange(currentRow, currentRow + centers[i].first.rows));
			currentRow += centers[i].first.rows;
		}

		// Calculate the BoW from the loaded centers
		std::cout << "Calculating BoW (clustering)" << std::endl;
		ClusteringResults results;
		Clustering::searchClusters(data, Config::getClusteringParams(), results);

		// Write the BoW to disk
		std::cout << "Writing BoW" << std::endl;
		Writer::writeBoW(OUTPUT_DIR "bow.dat", results.centers, Config::getClusteringParams(), nbands, nbins, bidirectional);
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
