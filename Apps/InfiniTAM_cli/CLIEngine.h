// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../InputSource/ImageSourceEngine.h"
#include "../../InputSource/IMUSourceEngine.h"
#include "../../ITMLib/Core/ITMMainEngine.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/NVTimer.h"
#include "../../ITMLib/Engines/Meshing/ITMMeshingEngineFactory.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class CLIEngine
		{
			static CLIEngine* instance;

			InputSource::ImageSourceEngine *imageSource;
			InputSource::IMUSourceEngine *imuSource;
			ITMLib::ITMLibSettings *internalSettings;
			ITMLib::ITMMainEngine *mainEngine;

			StopWatchInterface *timer_instant;
			StopWatchInterface *timer_average;

		private:
			ITMUChar4Image *inputRGBImage;
			ITMShortImage *inputRawDepthImage;
			ITMLib::ITMIMUMeasurement *inputIMUMeasurement;
			ITMUChar4Image *outImage;

			int currentFrameNo;

			struct FrequencyControl {
				unsigned freqDivisor;
				int framesSinceFreqChange;
				std::vector<unsigned> *processed;
				std::vector<double> *frequencies;
				std::vector<unsigned> *newBricks;
			};

			FrequencyControl freqControl;

			unsigned raycastingFreqDivisor;
            //pyh add mesh count
            unsigned mesh_count;
            ITMLib::ITMMesh *mesh;
		public:
			static CLIEngine* Instance(void) {
				if (instance == NULL) instance = new CLIEngine();
				return instance;
			}

			float processedTime;

			void Initialise(InputSource::ImageSourceEngine *imageSource, InputSource::IMUSourceEngine *imuSource, ITMLib::ITMMainEngine *mainEngine, ITMLib::ITMLibSettings *settings);
			void Shutdown();

			void Run();
			bool ProcessFrame();
		
            //pyh added for outputing scene dependent name
            std::string scene_name;
        
        };
	}
}
