/*
 * This file is part of DeepSlam.
 *
 * Copyright (C) 2020 Imperial College London
 * 
 * The use of the code within this file and all code within files that make up
 * the software that is DeepSlam is permitted for non-commercial purposes
 * only.  The full terms and conditions that apply to the code within this file
 * are detailed within the LICENSE file and at
 * <https://www.imperial.ac.uk/dyson-robotics-lab/projects/deepslam/deepslam-license>
 * unless explicitly stated. By downloading this file you agree to comply with
 * these terms.
 *
 * If you wish to use any of this code for commercial purposes then please
 * email researchcontracts.engineering@imperial.ac.uk.
 *
 */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <VisionCore/Buffers/Image2D.hpp>
#include <thread>

#include "decoder_network.h"
#include "intrinsics.h"
#include "timing.h"
#include "display_utils.h"
#include "warping.h"
#include "deepslam_options.h"
#include "deepslam.h"
#include "camera_interface_factory.h"
#include "tum_io.h"

DEFINE_string(netcfg, "../data/nets/scannet256_32.cfg", "Path to configuration file containing details of network etc");
DEFINE_string(vocabcfg, "../data/voc/small_voc.yml.gz", "Path to configuration file containing details of network etc");
DEFINE_string(imgpath, "../data/testimg/1052.jpg", "Image to run the network on");
DEFINE_string(gtpath, "", "Path to ground truth depth image");
DEFINE_int32(ntests, 1, "Number of times to run the decoding to average the time");
DEFINE_double(scale, 1, "Scale the jac");
DEFINE_double(gtscale, 1000, "Scale used to convert the gt image to meters");
DEFINE_string(incam, "", "Intrinsic parameters of the input camera image: fx,fy,u0,v0");
DEFINE_string(sourceurl, "tum:///root/tum_dataset/rgbd_dataset_freiburg1_desk", "File Reader");
DEFINE_int32(frame_limit, 20, "Frame limit");

void create_df_opts(df::DeepSlamOptions &df_opts);
using ImagePyramid = vc::RuntimeBufferPyramidManaged<float, vc::TargetHost>;
#define CS 32

template<typename T>
T split(const std::string &s, char delim=',')
{
  std::stringstream ss(s);
  std::string item;
  T elems;
  while (std::getline(ss, item, delim)){
    std::cout << item << std::endl;
    elems.push_back(std::stof(item));
  }
  return elems;
}

int main(int argc, char** argv)
{
  gflags::SetUsageMessage("Decode image test");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // create network
  std::string vocabcfg(FLAGS_vocabcfg);
  const auto netcfg = df::LoadJsonNetworkConfig(FLAGS_netcfg);
  //df::DecoderNetwork decoder(netcfg);

  // shorthand variables
  int pyrlevels = netcfg.pyramid_levels;
  int width = netcfg.input_width;
  int height = netcfg.input_height;

  // load image from disk
  cv::Mat img_orig = cv::imread(FLAGS_imgpath, cv::IMREAD_COLOR);
  CHECK(!img_orig.empty());
  int in_width = img_orig.cols;
  int in_height = img_orig.rows;

  // get the network intrinsics
  df::PinholeCamera<float> netcam;/*(netcfg.camera.fx, netcfg.camera.fy,
                                     netcfg.camera.u0, netcfg.camera.v0,
                                     width, height);*/

  // parse intrinsics if given in command line
  df::PinholeCamera<float> incam;
  if (not FLAGS_incam.empty())
  {
    auto intr = split<std::vector<float>>(FLAGS_incam, ',');
    CHECK_EQ(intr.size(), 4) << "Malformed intrinsics, should be: fx,fy,u0,v0";
    incam = df::PinholeCamera<float>(intr[0], intr[1], intr[2], intr[3], in_width, in_height);
  }

  // load in ground truth if provided
  // Slam System Init
  std::unique_ptr<df::DeepSlam<float,CS>> slam_system_ = std::make_unique<df::DeepSlam<float,CS>>();
  df::DeepSlamOptions df_opts;
  //df::DeepFactorsOptions df_opts;

  //df_opts.network_path = FLAGS_netcfg; //"../data/nets/scannet256_32.cfg";
  //std::string vocab_path = 
  df_opts.vocabulary_path = vocabcfg;//"../data/voc/small_voc.yml.gz";
  df_opts.network_path = FLAGS_netcfg;//"../data/voc/small_voc.yml.gz";
  df_opts.tracking_iterations = split<std::vector<int>>("5,5,10");
  create_df_opts(df_opts);
  //df_opts.tracking_iterations = split<std::vector<int>>("1,1,1");


  std::unique_ptr<df::drivers::CameraInterface> caminterface_ = df::drivers::CameraInterfaceFactory::Get()->GetInterfaceFromUrl(FLAGS_sourceurl);
  incam = caminterface_->GetIntrinsics(); 
  slam_system_->Init(incam, df_opts);

  //auto netcfg = slam_system_->GetNetworkConfig();
  incam.ResizeViewport(netcfg.input_width, netcfg.input_height);
  netcam = slam_system_->GetNetworkCam();


  // Bootstrap single Frame
  double live_timestamp_init;
  cv::Mat live_frame_init;
  std::mutex slam_mutex_;
  uint32_t frame_count = 0;
  
  while (caminterface_->HasMore() && frame_count < FLAGS_frame_limit )
  {
  	caminterface_->GrabFrames(live_timestamp_init, &live_frame_init);
    ++frame_count;
    slam_system_->ForceBuildKeyFrames( live_timestamp_init, live_frame_init );
  }
  slam_system_->DispKFs();
  slam_system_->PosePerKF();
  //slam_system_->CreateImgPyrAndGradients(live_timestamp_init, live_frame_init);
  //slam_system_->SaveCurrentImageAndGrad("./output");

#if 0
  caminterface_->GrabFrames(live_timestamp_init, &live_frame_init);
  slam_system_->BootstrapOneFrame(live_timestamp_init, live_frame_init);
  LOG(INFO)<< "Boot Strapping Complete..... Proceeding with next frames";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  /*std::cout << "Current KF Pose Estimate" <<std::endl
           << slam_system_->GetCameraPose().matrix()<<std::endl;
  slam_system_->SaveKeyframes("./output");*/

  
  uint32_t frame_count = 0;

  while (caminterface_->HasMore() && frame_count < FLAGS_frame_limit )
  {
     double live_timestamp_;
     cv::Mat live_frame_;
     LOG(INFO) << "Frame id:"<<frame_count <<std::endl;
     caminterface_->GrabFrames(live_timestamp_, &live_frame_);
     /*if(frame_count == 12 || frame_count == 112 || frame_count == 11 || frame_count == 111)
     {
         ++frame_count;
         continue;
     }*/
      std::lock_guard<std::mutex> guard(slam_mutex_);
      try
      {
        slam_system_->ProcessFrame(live_timestamp_, live_frame_);
      }
      catch(std::exception& e)
      {
        LOG(INFO) << "Exception in slam: " << + e.what();
        slam_system_->SavePostCrashInfo("./output");
        //break;
        //continue;
      }
      ++frame_count;

  }
#endif
  
  

  // Loop through all frames

  
}

void create_df_opts(df::DeepSlamOptions &df_opts)
{
df_opts.predict_code=false;
df_opts.debug=true;

df_opts.tracking_iterations={5,5,10};
df_opts.pho_iters={4,8,15};

df_opts.use_photometric=true;
df_opts.huber_delta=0.1;
df_opts.normalize_image=false;

df_opts.use_reprojection=true;
df_opts.rep_nfeatures=500;
df_opts.rep_scale_factor=1.2;
df_opts.rep_nlevels=9;
df_opts.rep_max_dist=50;
df_opts.rep_huber=4;
df_opts.rep_iters=15;
df_opts.rep_sigma=1.0;
df_opts.rep_ransac_maxiters=1000;
//df_opts.rep_ransac_threshold=0.000001;
df_opts.rep_ransac_threshold=0.001;

df_opts.use_geometric=false;
df_opts.geo_npoints=3000;
df_opts.geo_stochastic=false;
df_opts.geo_huber=0.1;
df_opts.geo_iters=15;

df_opts.loop_closure=true;
df_opts.loop_max_dist=0.1;
df_opts.loop_min_similarity=0.4;
//df_opts.loop_detection_freq=10;
df_opts.loop_active_window=10;
df_opts.keyframe_mode=df::DeepSlamOptions::KeyframeMode::AUTO;
df_opts.dist_threshold=0.6;
df_opts.inlier_threshold=0.7;
df_opts.frame_dist_threshold=0.03;

df_opts.connection_mode=df::MapperOptions::ConnectionMode::LASTN;
df_opts.max_back_connections=2;

df_opts.tracking_mode=df::DeepSlamOptions::TrackingMode::LAST;
df_opts.tracking_huber_delta=0.03;
df_opts.tracking_dist_threshold=3.5;
df_opts.tracking_error_threshold=0.5;

df_opts.pose_prior=0.1;
df_opts.code_prior=1;
df_opts.partial_relin_check=true;
df_opts.relinearize_threshold=0.03;



}

#if 0
void handle_image_demo()
{
  cv::Mat gt;
  if (!FLAGS_gtpath.empty())
  {
    gt = cv::imread(FLAGS_gtpath, cv::IMREAD_ANYDEPTH);
    gt.convertTo(gt, CV_32FC1, 1/FLAGS_gtscale);
  }

  // preprocess image
  cv::Mat img;
  cv::cvtColor(img_orig, img, cv::COLOR_RGB2GRAY);
  img.convertTo(img, CV_32FC1, 1/255.0);

  // optionally correct intrinsics
  if (not FLAGS_incam.empty())
  {
    LOG(INFO) << "Correcting intrinsics of the input image";
    img = ChangeIntrinsics(img, incam, netcam);
  }
  else
  {
    LOG(WARNING) << "Input intrinsics not specified in the command "
                    "line arguments, NOT correcting for intrinsics";
    cv::resize(img, img, {width, height});
  }
  cv::imwrite("transformed_img.png", img);

  // copy it to buffer
  vc::Image2DView<float, vc::TargetHost> img_cpu(img);

  // code
  Eigen::VectorXf code = Eigen::VectorXf::Zero(netcfg.code_size);
  Eigen::MatrixXf pred_code = Eigen::VectorXf::Zero(netcfg.code_size);

  // allocate buffers
  ImagePyramid pyr_prx(pyrlevels, width, height);
  ImagePyramid pyr_prx_pred(pyrlevels, width, height);
  ImagePyramid pyr_stdev(pyrlevels, width, height);
  ImagePyramid pyr_jac(pyrlevels, width, height);


  // decode the predicted code
  decoder.Decode(img_cpu, pred_code, &pyr_prx_pred, &pyr_stdev, &pyr_jac);
  std::cout << "Prediceted code:" << pred_code <<std::endl;


  // recover std from log(std)
  cv::Mat std = GetOpenCV(pyr_stdev[0]);
  cv::exp(std, std);
  std = sqrt(2) * std;

  auto cmap = cv::COLORMAP_JET;

  // display
  cv::resize(img_orig, img_orig, {width, height});
  cv::Mat prx_vis = prx_to_color(pyr_prx[0], cmap);
  std::vector<cv::Mat> vec_disp= {img_orig};
  for (uint32_t level=0;level < pyrlevels;++level) 
  {
     vec_disp.push_back(prx_to_color(pyr_prx[level], cmap));
     vec_disp.push_back(prx_to_color(pyr_prx_pred[level], cmap));
     vec_disp.push_back(prx_to_color(pyr_jac[level], cmap));
     vec_disp.push_back(prx_to_color(pyr_stdev[level], cmap));
  } 
 /*
  cv::Mat prx_pred_vis = prx_to_color(pyr_prx_pred[0], cmap);
  cv::Mat prx_pred_jac_vis = prx_to_color(pyr_jac[0], cmap);
  cv::Mat prx_pred_std_vis = prx_to_color(pyr_stdev[0], cmap);
*/
  cv::Mat std_vis = apply_colormap(5*std, cv::COLORMAP_JET);
  //std::vector<cv::Mat> array = {img_orig, prx_vis, prx_pred_vis, prx_pred_jac_vis, prx_pred_std_vis};

  // preprocess and display ground truth
  if (!gt.empty())
  {
    if (not FLAGS_incam.empty())
      gt = ChangeIntrinsics(gt, incam, netcam);
    else
      cv::resize(gt, gt, {width, height});

    // convert to prox and display
    cv::Mat gt_prx = df::DepthToProx(gt, netcfg.avg_dpt);
    vec_disp.push_back(apply_colormap(gt_prx, cmap));

    // two errors
//    array.push_back(cv::abs(gt_prx-GetOpenCV(pyr_prx[0]))/2.0);
//    array.push_back(cv::abs(gt_prx-GetOpenCV(pyr_prx_pred[0]))/2.0);
  }

  //array.push_back(std_vis);
  vec_disp.push_back(std_vis);

  //cv::namedWindow("results", cv::WINDOW_NORMAL);
  //cv::imshow("results", CreateMosaic(array, 1, FLAGS_gtpath.empty() ? 3 : 4));
  //cv::imwrite("results.png", CreateMosaic(array, 1, FLAGS_gtpath.empty() ? 3+2 : 4+2));
  cv::imwrite("results.png", CreateMosaic(vec_disp, 1, vec_disp.size()));


}
#endif
