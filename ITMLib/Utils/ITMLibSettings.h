// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "../../ORUtils/MemoryDeviceType.h"

namespace ITMLib
{
	class ITMLibSettings
	{
	public:
		/// The device used to run the DeviceAgnostic code
		typedef enum {
			DEVICE_CPU,
			DEVICE_CUDA,
			DEVICE_METAL
		} DeviceType;

		typedef enum
		{
			FAILUREMODE_RELOCALISE,
			FAILUREMODE_IGNORE,
			FAILUREMODE_STOP_INTEGRATION
		} FailureMode;

		typedef enum
		{
			SWAPPINGMODE_DISABLED,
			SWAPPINGMODE_ENABLED,
			SWAPPINGMODE_DELETE
		} SwappingMode;

		typedef enum
		{
			LIBMODE_BASIC,
			LIBMODE_BASIC_SURFELS,
			LIBMODE_LOOPCLOSURE
		} LibMode;

		/// Frequency mode for incoming images
		typedef enum
		{
			FREQMODE_NONE,
			FREQMODE_CONSTANT,
			FREQMODE_CONTROLLER
		} FreqMode;

		/// Select the type of device to use
		DeviceType deviceType;

		/// Use ICP for tracking, otherwise use file pose
		bool useICP;

		/// Do not raycast every frame
		bool useApproximateRaycast;

		/// Decouple raycasting from fusion
		bool useDecoupledRaycasting;

		/// Only project depth point (and not ray) during visibility check
		bool useApproximateDepthCheck;

		/// Use visibility list of previous frame during visibility check
		bool usePreviousVisibilityList;

		/// Apply bilateral filter to depth image before using it
		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;

		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;
		LibMode libMode;

		/// Fusion frequency mode
		FreqMode freqMode;

		/// Fusion frequency in constant frequency mode
		double fusionFreq;

		/// Maximum fusion frequency
		static constexpr double MAX_FREQ = 30.0f;

		/// Raycasting frequency in decoupled mode
		double raycastingFreq;

		//pyh parameters for PID TODO: add divisors
        double K_p;
        double K_i;
        double K_d;
        double alpha;

        const char *trackerConfig;

		/// Further, scene specific parameters such as voxel size
		ITMSceneParams sceneParams;
		ITMSurfelSceneParams surfelSceneParams;

		ITMLibSettings(void);
		virtual ~ITMLibSettings(void) {}

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&);
		ITMLibSettings& operator=(const ITMLibSettings&);

		MemoryDeviceType GetMemoryType() const;
	};
}
