/**
 * Author: rodrigo
 * 2016     
 */
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <opencv2/core/core.hpp>
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
		if (system("mkdir -p " OUTPUT_FOLDER) != 0)
			throw std::runtime_error("can't create the output folder: " + workingDir + OUTPUT_FOLDER);

		// Clean the output directory
		if (system("rm -rf " OUTPUT_FOLDER "*") != 0)
			std::cout << (std::string) "WARNING: can't clean output directory: " + workingDir + OUTPUT_FOLDER << std::endl;

		// Load the configuration file
		std::cout << "Loading configuration" << std::endl;
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error((std::string) "Error reading config at " + workingDir + CONFIG_LOCATION);

		// Read data from files
		std::cout << "Loading data from files" << std::endl;
		std::pair<int, int> dimensions(0, 0);
		std::vector<cv::Mat> centers;
		Loader::traverseDirectory(inputDirectory, centers, dimensions);

		// Merge all the centers in one matrix
		std::cout << "Groupping data" << std::endl;
		cv::Mat data = cv::Mat::zeros(dimensions.first, dimensions.second, CV_32FC1);
		int currentRow = 0;
		for (size_t i = 0; i < centers.size(); i++)
		{
			centers[i].copyTo(data.rowRange(currentRow, currentRow + centers[i].rows));
			currentRow += centers[i].rows;
		}

		// Calculate the BoW from the loaded centers
		std::cout << "Calculating BoW (clustering)" << std::endl;
		ClusteringResults results;
		Clustering::searchClusters(data, Config::getClusteringParams(), results);

		// Write the BoW to disk
		std::cout << "Writing BoW" << std::endl;
		Writer::writeBoW(OUTPUT_FOLDER "bow.dat", results.centers, Config::getClusteringParams());
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
