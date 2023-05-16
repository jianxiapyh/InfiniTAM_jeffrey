//pyh this code takes insight from my earlier version of InfiniTAM and plugin and a later attempt by Henry, 
//main changes made:
//       - adjust for ScanNet 
//          - read from offline_scannet, thus expecting both groundtruth & images from switchboard 
//       - remove some hardcoded elements
//          -calibration location  
//       - change publisher format
//          - from raycasted image to compressed mesh format

#include "../../common/plugin.hpp"
#include "../../common/data_format.hpp"
#include "../../common/phonebook.hpp"
#include "../../common/relative_clock.hpp"
#include "../../common/threadloop.hpp"

#include "CLIEngine.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
//#include "../ORUtils/FileUtils.h"
#include<shared_mutex>
#include <iostream>
using namespace ILLIXR;
class infiniTAM : public plugin {
    public:
        infiniTAM(std::string name_, phonebook* pb_)
           : plugin{name_, pb_}
           , sb{pb->lookup_impl<switchboard>()}
           , _m_scannet_datum{sb->get_buffered_reader<groundtruth_sceneRecon_type>("ScanNet_Data")}
        {
            ////pyh for now, I just hardcode internal settings that exists in ScanNet.sh, later we might need to have a more intelligent way to get these variables
            //ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
            //internalSettings->useICP = false;
            //internalSettings->useApproximateDepthCheck = false;
            //internalSettings->usePreviousVisibilityList = false;
            //internalSettings->freqMode = ITMLib::ITMLibSettings::FreqMode::FREQMODE_CONSTANT;
            //internalSettings->fusionFreq = 30.0;
            //internalSettings->useDecoupledRaycasting = true;
            //internalSettings->raycastingFreq=1.0;

            //calib = new ITMLib::ITMRGBDCalib();
            ////pyh get from the calibration.txt from each scene
            //const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
            //std::string illixr_data = std::string{illixr_data_c_str}; 
            //const std::string calib_subpath = "calibration.txt";
            //std::ifstream calib_subpath{illixr_data + calib_subpath};
            //if(!readRGBDCalib(calib_source.c_str(), *calib)){
            //    printf("Read RGBD caliberation file failed\n");
            //}

            ////pyh checking calibration
            //std::cout<<"rgb calibration: "<< calib->intrinsics_rgb.projectionParamsSimple.all<<"\n"; 
            //std::cout<<"depth calibration: "<< calib->intrinsics_d.projectionParamsSimple.all<<"\n";
            //std::cout<<"BilaterFilter: "<<internalSettings->useBilateralFilter<<"\n";

            //mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
            //        internalSettings,
            //        *calib,
            //        calib->intrinsics_rgb.imgSize,
            //        calib->intrinsics_d.imgSize
            //        );

            ////pyh: first allocate for incoming depth & RGB image on CPU, then later copy to GPU
            ////function input parameters: dims, allocate_cpu?, allocate_gpu?
            //inputRawDepthImage = new ITMShortImage(calib->intrinsics_d.imgSize, true, false);
            //inputRGBImage = new ITMUChar4Image(calib->intrinsics_rgb.imgSize, true, false);
            //
            ////pyh: listen to cam_type, then check if corresponding pose type have arrived
            //sb->schedule<groundtruth_sceneRecon_type>(id,"ScanNet_Data",[&](switchboard::ptr<const groundtruth_sceneRecon_type> datum, std::size_t iteration_no){
            //        this->ProcessFrame(datum, iteration_num);
            //});

            //frame_count=0;

            printf("================================InfiniTAM: setup finished==========================\n");


        }
        //this needs to wait for both pose_type and image
        void ProcessFrame(switchboard::ptr<const cam_type> datum, std::size_t iteration_no){
            //if(!datum->cam0.has_value() || !datum->cam1.has_value()){
            //    printf("haven't received cam0 or cam1");
            //    ILLIXR::abort();
            //}
            //freqControl.frequencies->push_back(ITMLibSettings::MAX_FREQ / static_cast<double>(freqControl.freqDivisor));
            //bool shouldSkip = (freqControl.framesSinceFreqChange % freqControl.freqDivisor) != 0;
            //if(shouldSkip){
            //    std::cout << "=== Skipping frame ===" << frame_count << std::endl;
            //}else{
            //    std::cout << "=== Running frame ===" <<frame_count<<std::endl; 
            //    ////grab rgb and depth images
            //    //cv::Mat cur_depth{current_datum->depth.value()};
            //    //cv::Mat cur_rgb{current_datum->rgb.value()};
            //    ////grab gt position and orientation
            //    //Eigen::Vector3f cur_pos = datum->pose.position;
            //    //Eigen::Quaternionf cur_orientation = datum->pose.orientation;
            //    ////convert to InfiniTAM expected data structure
            //    //const uint16_t *depth_frame = reinterpret_cast<const uint16_t*>(cur_depth.datastart);
            //    //short *cur_depth_head = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
            //    //const Vector4u *color_frame = reinterpret_cast<const Vector4u*>(cur_rgb.datastart);
            //    //Vector4u *cur_rgb_head = inputRGBImage->GetData(MEMORYDEVICE_CPU);
            //    //std::memcpy(cur_rgb_head, color_frame, sizeof(Vector4u) *inputRGBImage->dataSize);
            //    //std::memcpy(cur_depth_head, depth_frame, sizeof(short)  * inputRawDepthImage->dataSize);
            //    //
            //    ////groundtruth mode
            //    //mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, cur_trans);
            //    //
            //    //ORcudaSafeCall(cudaThreadSynchronize());

            //}
            frame_count++;
        }
        virtual ~infiniTAM() override{

        }
    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard>  _m_sb;
        switchboard::reader<groundtruth_sceneRecon_type> _m_scannet_datum;
        
        //InfiniTAM related variables
        ITMLib::ITMRGBDCalib *calib;
        ITMLib::ITMMainEngine *mainEngine;
        ITMUChar4Image *inputRGBImage;
        ITMShortImage *inputRawDepthImage;
        ITMLib::ITMLibSettings *internalSettings;
        
        //variables for fusion frequency
        //changes:remove processed and new bricks
        struct FrequencyControl{
            unsigned freqDivisor;
            int framesSinceFreqChange;
            std::vector<double> *frequencies;
        };

        FrequencyControl FreqControl;
        unsigned raycastingFreqDivisor;
        unsigned frame_count;



}
PLUGIN_MAIN(infiniTAM)

