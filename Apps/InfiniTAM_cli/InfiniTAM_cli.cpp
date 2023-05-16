// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include "CLIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"

#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

void list2vec(std::string filename, std::vector<std::string> &time_list, std::vector<std::string> &color_image, std::vector<std::string> &depth_image)
{
        std::ifstream file;
        std::string line;
        file.open(filename);

        // Confirm that we can open the file
        if (!file.good())
        {
                std::cerr << "[Error] Cannot open " << filename << "\n";
                std::cerr << "[Error] Is the path correct?\n";
                exit(1);
        }

        std::vector<std::string> output;
        std::vector<double> pose_value;

        std::string path = filename.substr(0, filename.find_last_of("/")+1);
        std::cout << "Path: " << path << std::endl;

        while(file.good() && (getline(file, line)))
        {
#ifdef DEBUG
                std::cout << "[DEBUG] line: " << line << std::endl;
#endif
                output.clear();
                if (line.find("#") != std::string::npos)
                        continue;

                std::istringstream iss(line);
                std::string str;
                while (iss >> str)
                {
#ifdef DEBUG
                        std::cout << "[DEBUG] str: " << str << std::endl;
#endif
                        output.push_back(str);
                }
				time_list.push_back(output[0]);

                // color_image.push_back(output[11]);
                // depth_image.push_back(output[9]);

                // color_image.push_back(path + output[11]);
                // depth_image.push_back(path + output[9]);

                color_image.push_back(path + "../" + output[11]);
                depth_image.push_back(path + "../" + output[9]);

        }
}

int main(int argc, char** argv)
try
{
	const char *calibFile = "";
	const char *groundtruth = "";
	const char *imagesource_part1 = NULL;
	const char *imagesource_part2 = NULL;
	const char *imagesource_part3 = NULL;
    //pyh add a scene file (txt) so that it reads the file to figure out the scene name
    const char *scene_file="";
	int arg = 1;
	do {
		if (argv[arg] != NULL) calibFile = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part1 = argv[arg]; else break;
		//pyh force my way to read scene param
        ++arg;
		if (argv[arg] != NULL) scene_file = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part2 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part3 = argv[arg]; else break;
	} while (false);
    //pyh print check to see if the scene file is correct
    printf("pyh scene file %s\n", scene_file);

	if (arg == 1) {
		printf("usage: %s [<calibfile> [<imagesource>] ]\n"
		       "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
		       "  <imagesource> : either one argument to specify OpenNI device ID\n"
		       "                  or two arguments specifying rgb and depth file masks\n"
		       "\n"
		       "examples:\n"
		       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/%%04i.ppm ./Files/Teddy/%%04i.pgm\n"
		       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/associated_pose.txt\n\n", argv[0], argv[0], argv[0]);
	}

	printf("initialising ...\n");
	ITMLibSettings *internalSettings = new ITMLibSettings();

	ImageSourceEngine *imageSource;
	IMUSourceEngine *imuSource = NULL;
	printf("using calibration file: %s\n", calibFile);

	// Never run infinitam_cli with sensors since no feedback.
	if (imagesource_part2 == NULL) // Only one input pose specified. Use list generator.
	{
		std::cout << "Using ImageListPathGenerator! " << std::endl;
		groundtruth = imagesource_part1;

		std::vector<std::string> time_list;
		std::vector<std::string> color_list;
		std::vector<std::string> depth_list;

		list2vec(imagesource_part1, time_list, color_list, depth_list);

		std::cout << "size:" << color_list.size() << " " << depth_list.size() << std::endl;
		ImageListPathGenerator pathGenerator(time_list, color_list, depth_list);
		imageSource = new ImageFileReader<ImageListPathGenerator>(calibFile, pathGenerator);
	}
	else
	{
		if (imagesource_part3 == NULL) // Two input files specified. Use mask generator.
		{
			std::cout << "Using ImageMaskPathGenerator! " << std::endl;
			printf("using rgb images: %s\nusing depth images: %s\n", imagesource_part1, imagesource_part2);

			ImageMaskPathGenerator pathGenerator(imagesource_part1, imagesource_part2);
			imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFile, pathGenerator);
		}
		else // Three input files specified. Use IMU data as well.
		{
			printf("using rgb images: %s\nusing depth images: %s\nusing imu data: %s\n", imagesource_part1, imagesource_part2, imagesource_part3);
			imageSource = new RawFileReader(calibFile, imagesource_part1, imagesource_part2, Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(imagesource_part3);
		}
	}

	ITMMainEngine *mainEngine = new ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(
		internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize(), groundtruth
	);
    //pyh read file and assign scene name
    std::ifstream file(scene_file);
    std::string line;
    if(file.is_open()){
        std::getline(file,line);
        std::cout << "The first line of the file is: " << line << std::endl;
        file.close();
    }
    mainEngine->scene_name=line;
	
    CLIEngine::Instance()->Initialise(imageSource, imuSource, mainEngine, internalSettings);
    CLIEngine::Instance()->scene_name = line;
	CLIEngine::Instance()->Run();
	CLIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}
