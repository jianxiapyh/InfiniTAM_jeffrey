//pyh this code takes insight from my earlier version of InfiniTAM and plugin and a later attempt by Henry, 
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/relative_clock.hpp"
#include "common/phonebook.hpp"

//add infinitam libraries
#include "ITMLib/Core/ITMBasicEngine.h"
#include "ITMLib/Utils/ITMLibSettings.h"
#include "ITMLib/ITMLibDefines.h"
#include "ORUtils/FileUtils.h"

//draco related libraries
#include <draco/io/ply_reader.h>
#include <draco/io/file_reader_factory.h>
#include <draco/io/file_writer_factory.h>
#include <draco/io/stdio_file_reader.h>
#include <draco/io/stdio_file_writer.h>
#include "draco/io/ply_property_writer.h"
#include "draco/io/ply_decoder.h"
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/io/file_utils.h"

#include <iostream>
#include <opencv/cv.hpp>
#include <stdio.h>
using namespace ILLIXR;
class infinitam : public plugin {
    public:
        infinitam(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_scannet_datum{sb->get_reader<scene_recon_type>("ScanNet_Data")}
        , _m_mesh{sb->get_writer<mesh_demo_type>("original_scene")}
        {
            //TODO pyh
            //1. for now, I just hardcode internal settings that exists in ScanNet.sh, later we might need to have a more intelligent way to get these variables
            internalSettings = new ITMLib::ITMLibSettings();
            internalSettings->useICP = false;
            internalSettings->useApproximateDepthCheck = false;
            internalSettings->usePreviousVisibilityList = false;
            internalSettings->freqMode = ITMLib::ITMLibSettings::FreqMode::FREQMODE_CONSTANT;
            internalSettings->fusionFreq = 10.0;
            internalSettings->useDecoupledRaycasting = true;
            internalSettings->raycastingFreq=1.0;

            calib = new ITMLib::ITMRGBDCalib();
            const char* illixr_data_c_str = std::getenv("ILLIXR_DATA"); 
            std::string illixr_data = std::string{illixr_data_c_str};
            const std::string calib_subpath = "/calibration.txt";
            std::string calib_source{illixr_data + calib_subpath};
            printf("calibration file path: %s\n", calib_source.c_str());
            if(!readRGBDCalib(calib_source.c_str(), *calib)){
                printf("Read RGBD caliberation file failed\n");
            }
            
            //pyh extract scene name
            std::size_t pos = illixr_data.find_last_of("/");
            scene_number = illixr_data.substr(pos+1);
            std::cout<<"Scene number: "<<scene_number<<std::endl;
            merge_name = "ITM-BE_" + scene_number +".obj"; 
            
            //pyh checking calibration
            std::cout<<"rgb calibration: "<< calib->intrinsics_rgb.projectionParamsSimple.all<<"\n";
            std::cout<<"rgb size: x: "<< calib->intrinsics_rgb.imgSize.x<< " y: "<<calib->intrinsics_rgb.imgSize.y <<"\n";
            std::cout<<"depth calibration: "<< calib->intrinsics_d.projectionParamsSimple.all<<"\n";
            std::cout<<"depth size: x: "<< calib->intrinsics_d.imgSize.x<< " y: "<<calib->intrinsics_d.imgSize.y <<"\n";

            //pyh comment to test plugin
            mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
                    internalSettings,
                    *calib,
                    calib->intrinsics_rgb.imgSize,
                    calib->intrinsics_d.imgSize
                    );
            
            //pyh first allocate for incoming depth & RGB image on CPU, then later copy to GPU
            //constructor paramters: dimensions, allocate_CPU? allocate_GPU?
            inputRawDepthImage = new ITMShortImage(calib->intrinsics_d.imgSize, true, false);
            inputRGBImage = new ITMUChar4Image(calib->intrinsics_rgb.imgSize, true, false);
            printf("depth dimension vector x: %d y: %d\n", inputRawDepthImage->noDims.x, inputRawDepthImage->noDims.y); 
            printf("rgb dimension vector x: %d y: %d\n", inputRGBImage->noDims.x, inputRGBImage->noDims.y); 
            
            if (internalSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
            {
                std::cout<<"device_cuda\n";
            }
            //pyh intialize frequency control
            freqControl.freqDivisor = mainEngine->GetFreqDivisor();
            freqControl.framesSinceFreqChange = 0;
            freqControl.frequencies = new std::vector<double>;
            raycastingFreqDivisor = ITMLib::ITMLibSettings::MAX_FREQ / static_cast<unsigned>(internalSettings->raycastingFreq);
            
            sb->schedule<scene_recon_type>(id,"ScanNet_Data",[&](switchboard::ptr<const scene_recon_type> datum, std::size_t){
                    this->ProcessFrame(datum);
            });

            //track how many frame InfiniTAM has processed
            frame_count=0;

            //pyh initialize mesh and raycasted image
            latest_mesh = new ITMLib::ITMMesh(MEMORYDEVICE_CUDA,0);
            reconstructedImage = new ITMUChar4Image(calib->intrinsics_d.imgSize, true, false);
            mesh=new ITMLib::ITMMesh(MEMORYDEVICE_CUDA,0);
            draco::FileReaderFactory::RegisterReader(draco::StdioFileReader::Open);
            draco::FileWriterFactory::RegisterWriter(draco::StdioFileWriter::Open);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,pos_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,tex_coords_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,normals_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,generic_quantization_bits);
            encoder.SetSpeedOptions(speed, speed);

            //iterate between different mesh compression modules
            queue_choice=0;
            printf("================================InfiniTAM: setup finished==========================\n");
        }

        void ProcessFrame(switchboard::ptr<const scene_recon_type> datum)
        {
            printf("================================InfiniTAM: frame %d received==========================\n", frame_count);
            if(!datum->depth.empty() && !datum->rgb.empty())
            {
                freqControl.frequencies->push_back(ITMLib::ITMLibSettings::MAX_FREQ / static_cast<double>(freqControl.freqDivisor));
                bool shouldSkip = (freqControl.framesSinceFreqChange % freqControl.freqDivisor) != 0;
                if(shouldSkip){
                    //std::cout << "=== Skipping frame ===" << frame_count << std::endl;
                }else{
                    //std::cout << "=== Running frame ===" <<frame_count<<std::endl;
                    //pyh: convert to transformation matrix here this saves reading a fixed file in the ITMBasicEngine
                    Eigen::Matrix3f rot = datum->pose.orientation.normalized().toRotationMatrix();
                    ORUtils::Matrix4<float> cur_trans_matrix;
                    cur_trans_matrix = {
                        rot(0, 0), rot(1, 0), rot(2, 0), 0.0f,
                        rot(0, 1), rot(1, 1), rot(2, 1), 0.0f,
                        rot(0, 2), rot(1, 2), rot(2, 2), 0.0f,
                        datum->pose.position.x(), datum->pose.position.y(), datum->pose.position.z(), 1.0f
                    };

                    // Set first pose
                    if(frame_count == 0)
                    {
                        //printf("Setting Initial Pose using pose: %f %f %f %f %f %f %f \n", datum->pose.position.x(), datum->pose.position.y(), datum->pose.position.z(), datum->pose.orientation.x(), datum->pose.orientation.y(), datum->pose.orientation.z(), datum->pose.orientation.w()); 
                        mainEngine->SetInitialPose(cur_trans_matrix);
                    }
                    else {
                        //printf("pose: %f %f %f %f %f %f %f \n", datum->pose.position.x(), datum->pose.position.y(), datum->pose.position.z(), datum->pose.orientation.x(), datum->pose.orientation.y(), datum->pose.orientation.z(), datum->pose.orientation.w()); 
                    }
                    unsigned newDivisor = mainEngine->GetFreqDivisor();
                    if (newDivisor != freqControl.freqDivisor){
                        printf("frequency change\n");
                        freqControl.freqDivisor = newDivisor;
                        freqControl.framesSinceFreqChange = 0;
                    }
                    
                    cv::Mat cur_depth{datum->depth};
                    cv::Mat cur_rgb{datum->rgb};
                    //printf("depth type %s\n",type2str(cur_depth.type()).c_str());
                    //printf("rgb type %s\n",type2str(cur_rgb.type()).c_str());

                    //cv::imwrite("depth.png",cur_depth);
                    //cv::imwrite("color.png",cur_rgb);
                    
                    //pyh converting the to the InfiniTAM expected data structure
                    const short *depth_frame = reinterpret_cast<const short*>(cur_depth.datastart);
                    short *cur_depth_head = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
                    std::memcpy(cur_depth_head, depth_frame, sizeof(short)  *inputRawDepthImage->dataSize);
                    
                    //enable color
                    const Vector4u *color_frame = reinterpret_cast<const Vector4u*>(cur_rgb.datastart);
                    Vector4u *cur_rgb_head = inputRGBImage->GetData(MEMORYDEVICE_CPU);
                    std::memcpy(cur_rgb_head, color_frame, sizeof(Vector4u) * inputRGBImage->dataSize);
                    
                    //This should be ProcessFrame
                    mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, cur_trans_matrix);
                }

                if(internalSettings->useDecoupledRaycasting)
                {
                   if ((frame_count % raycastingFreqDivisor) == 0 && frame_count!=0) 
                   {
                       std::cout<<"=== Raycast at frame ===" <<frame_count <<std::endl;
                       
                       //pyh add GetMesh Function
                       auto start = std::chrono::high_resolution_clock::now();
                       
                       mainEngine->GetMesh(mesh);
                       
                       std::string file_name = "ITM-BE_";
                       std::string merge_name = file_name + this->scene_number + "_" + std::to_string(frame_count) +".obj";
                       
                       mesh->WriteOBJ(merge_name.c_str());
                       _m_mesh.put(_m_mesh.allocate<mesh_demo_type>(mesh_demo_type{ nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, frame_count, merge_name}));
                   }
                }
                
                ORcudaSafeCall(cudaThreadSynchronize());
            }
            else{   
                if(datum->depth.empty()){ printf("depth empty\n");}
                if (datum->rgb.empty()){ printf("rgb empty\n"); }
            }
            if(datum->last_frame)
            {
                printf("reached last frame at %d\n", frame_count++);
                mainEngine->GetMesh(latest_mesh);
                latest_mesh->WriteOBJ(merge_name.c_str());
            }
            freqControl.framesSinceFreqChange++;
            frame_count++;
        }

        virtual ~infinitam() override{
            //mainEngine->SaveSceneToMesh(output_mesh_name.c_str());
        }

        //pyh some debugging function
        std::string type2str(int type)
        {
            std::string r;
            uchar depth = type & CV_MAT_DEPTH_MASK;
            uchar chans = 1 + (type >> CV_CN_SHIFT);
            switch ( depth ){
                case CV_8U:  r = "8U"; break;
                case CV_8S:  r = "8S"; break;
                case CV_16U: r = "16U"; break;
                case CV_16S: r = "16S"; break;
                case CV_32S: r = "32S"; break;
                case CV_32F: r = "32F"; break;
                case CV_64F: r = "64F"; break;
                default:     r = "User"; break;
            } 
            r += "C";
            r += (chans+'0');
            return r;
         }

    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard> sb;
        switchboard::reader<scene_recon_type> _m_scannet_datum;
        //switchboard::writer<reconstruction_type> _m_reconstruction;
        switchboard::writer<mesh_demo_type> _m_mesh;

        //InfiniTAM related variables
        ITMLib::ITMRGBDCalib *calib;
        ITMLib::ITMMainEngine *mainEngine;
        ITMUChar4Image *inputRGBImage;
        ITMShortImage *inputRawDepthImage;
        ITMLib::ITMLibSettings *internalSettings;
        ITMLib::ITMMesh *latest_mesh;
        ITMUChar4Image *reconstructedImage;
        ITMLib::ITMMainEngine::GetImageType reconstructedImageType{ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME};
        ITMLib::ITMMesh *mesh;
        
        //draco related variables
        std::unique_ptr<draco::ExpertEncoder> expert_encoder;
        draco::Encoder encoder;
        int pos_quantization_bits = 11;
        int tex_coords_quantization_bits = 10;
        bool tex_coords_deleted = false;
        int normals_quantization_bits = 8;
        bool normals_deleted = false;
        int generic_quantization_bits = 8;
        bool generic_deleted = false;
        int compression_level = 1;
        int speed = 10-compression_level;

        std::vector<char> compressed_mesh;
        std::string scene_number;
        std::string merge_name;
        //variables for fusion frequency
        struct FrequencyControl{
             unsigned freqDivisor;
             int framesSinceFreqChange;
             std::vector<double> *frequencies;
        };
        FrequencyControl freqControl;
        unsigned raycastingFreqDivisor;
        
        unsigned frame_count;
        unsigned queue_choice;
};

PLUGIN_MAIN(infinitam)
