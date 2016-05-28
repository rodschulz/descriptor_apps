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
#include "MetricFactory.hpp"
#include "Config.hpp"
#include "Utils.hpp"

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
			throw std::runtime_error("Not enough exec params given\nUsage: BoWCalculator <descriptors_directory>");

		// Clean the output directory
		if (system("rm -rf " OUTPUT_FOLDER "*") != 0)
			std::cout << (std::string) "WARNING: can't clean output directory: " + workingDir + OUTPUT_FOLDER << std::endl;

		// Load the configuration file
		std::cout << "Loading configuration" << std::endl;
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error((std::string) "Error reading config at " + workingDir + CONFIG_LOCATION);


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
