#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "timing.h"
#include "pinhole_camera.h"
#include "warping.h"
//#include "testing_utils.h"
//#include "pinhole_camera.h"
#include "lucas_kanade_se3.h"
//#include "pinhole_camera_impl.h"
#include "decoder_network.h"
#include "display_utils.h"
#include "cu_image_proc.h"
#include "cuda_context.h"
#include "keyframe_map.h"
#include "timing.h"
#include "logutils.h"
#include "feature_detection.h"
#include "tum_io.h"
#include <VisionCore/Buffers/Image2D.hpp>


typedef Eigen::Matrix<float, 3, 1>  PointT;
typedef Eigen::Matrix<float, 2, 1>  PixelT;

struct NetworkConfig
{
    struct Cam { double fx, fy, u0, v0; };
    Cam camera;

    std::string graph_path;
    std::size_t input_width;
    std::size_t input_height;
    std::size_t pyramid_levels;
    std::size_t code_size;
    bool grayscale;
    double avg_dpt;

    bool depth_pred;

    // input tensor names
    std::string input_image_name;
    std::string input_code_name;

    // output tensor names
    std::string code_pred_name;
    std::vector<std::string> depth_est_names;
    std::vector<std::string> depth_jac_names;
    std::vector<std::string> depth_std_names;
    std::vector<std::string> depth_pred_names;
};



void CreateAndInitNetwork(const std::string& cfgpath)
{
   auto	netcfg_ = df::LoadJsonNetworkConfig(cfgpath);
   auto	network_ = std::make_shared<df::DecoderNetwork>(netcfg_);

   auto	width = netcfg_.input_width;
   auto	height = netcfg_.input_height;
   auto	codesize = netcfg_.code_size;
   auto	pyrlevels = netcfg_.pyramid_levels;
}

int main(int argc, char **argv)
{
    /*
    std::vector<std::int64_t> TensorDims({1, 20});
    std::cout << "vector of size:"<<TensorDims.size() << TensorDims.at(0)<<std::endl;
    //std::size_t size = 1;
    for (auto& d : TensorDims)
      std::cout << "Values:" << d<< std::endl;
    */
    //df::PinholeCamera<float> netcam(240.0, 249.0, 128.0,96.0, 256, 192); 
    static constexpr std::size_t CS = 32;
    std::string cfg_path;
    if(argc > 1)
    {
        std::string argv0(argv[1]);
        cfg_path = argv0;
        std::cout << "cfg_path" << cfg_path << std::endl;

    }
    const auto netcfg = df::LoadJsonNetworkConfig(cfg_path);
    df::DecoderNetwork decoder(netcfg);

  // shorthand variables
    int pyrlevels = netcfg.pyramid_levels;
    int width = netcfg.input_width;
    int height = netcfg.input_height;
    std::cout << "Pyramid Levels:" << pyrlevels <<std::endl
              << "Width:" <<  width  <<std::endl 
              << "Height:" <<  height <<std::endl; 
             
    
    //df::PinholeCamera<float> netcam(240.0, 249.0, 128.0,96.0, 256, 192); 
    df::PinholeCamera<float> netcam(netcfg.camera.fx, netcfg.camera.fy,
                                     netcfg.camera.u0, netcfg.camera.v0,
                                     width, height);
 
    auto thread_blocks = [&](int param, int val){
	return param*val/2;
    };
    std::cout << "Thread_blocks:"<<thread_blocks (10,20)<<std::endl;
    //std::cout<<thread_blocks[0] << " "<<thread_blocks[1]<<std::endl
  
    PointT point_x = PointT(10, 9,100);
    PointT point_y = PointT(10, 10,100);
    PointT point_z = PointT(9, 10,100);

    PixelT my_pixel =  netcam.Project(point_x);
    std::cout << my_pixel<<std::endl;
    std::cout << netcam.PixelValid(my_pixel, 100)<<std::endl;
    std::cout << netcam.ProjectPointJacobian(point_y) << std::endl; 
    Sophus::SE3f pose_wf = Sophus::SE3f{};
    auto zero_code = Eigen::VectorXf::Zero(CS);
    //Eigen::MatrixBase::Zero(CS);
    //Eigen::Matrix<float> matrix_code;
    //std::cout << "Pose:" << pose_wf <<"Code:" << zero_code <<std::endl;
    //std::cout <<"Code:" << matrix_code.rows() <<std::endl;
    
}

/*
 "avg_dpt": 2.0,
  "camera": {
    "fx": 240.0,
    "fy": 240.0,
    "u0": 128.0,
    "v0": 96.0
  },
*/
