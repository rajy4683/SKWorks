/*
 Top-Level SLAM class that includes the primary DeepSLAM elements:
	1. Mapping Modules
        2. Localization Module
        3. Loop Detection Module 
        4. Decoder Network
*/
#ifndef DF_DEEPSLAM_H_
#define DF_DEEPSLAM_H_

#include <cstddef>
#include <memory>
#include <vector>
#include <Eigen/Core>

#include "mapper.h"
#include "camera_tracker.h"
#include "pinhole_camera.h"
#include "decoder_network.h"
#include "cuda_context.h"
#include "keyframe_map.h"
#include "deepslam_options.h"
#include "loop_detector.h"
#include "feature_detection.h"

// debug temporary
#include "cu_se3aligner.h"

namespace cv { class Mat; }

namespace df
{

struct DeepSlamStatistics
{
  float inliers;
  float distance;
  float tracker_error;
  cv::Mat residual;
  std::map<std::size_t, bool> relin_info;
};

template <typename Scalar, int CS>
class DeepSlam
{
public:
  typedef Scalar ScalarT;
  typedef Sophus::SE3<Scalar> SE3T;
  typedef df::Keyframe<Scalar> KeyframeT;
  typedef typename df::Keyframe<Scalar>::IdType KeyframeId;
  //typedef typename Map<Scalar>::Ptr MapPtr;
  typedef df::Map<Scalar> MapT;
  typedef typename MapT::Ptr MapPtr;
  typedef df::Mapper<Scalar,CS> MapperT;
  typedef LoopDetector<Scalar> LoopDetectorT;
  typedef vc::RuntimeBufferPyramidManaged<Scalar, vc::TargetDeviceCUDA> ImagePyrT;
  typedef vc::RuntimeBufferPyramidManaged<Eigen::Matrix<Scalar,1,2>, vc::TargetDeviceCUDA> GradPyrT;

  // callback types
  typedef std::function<void (MapPtr)> MapCallbackT;
  typedef std::function<void (const SE3T&)> PoseCallbackT;
  typedef std::function<void (const DeepSlamStatistics&)> StatsCallbackT;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  DeepSlam();
  virtual ~DeepSlam();

  /* Do not create copies of this object */
  DeepSlam(const DeepSlam& other) = delete;

  /*
   * Logic
   */
  void Init(df::PinholeCamera<Scalar>& cam, const DeepSlamOptions& opts);
  void Reset();
  void ProcessFrame(double timestamp, const cv::Mat& frame);

  /*
   * Initialize the system with two images and optimize them
   */
  void BootstrapTwoFrames(double ts1, double ts2, const cv::Mat& img0, const cv::Mat& img1);

  /*
   * Initialize the system with a single image (decodes zero-code only)
   */
  void BootstrapOneFrame(double timestamp, const cv::Mat& img);

  void ForceKeyframe();
  void ForceFrame();

  /*
   * Getters
   */
  MapPtr GetMap() { return mapper_->GetMap(); }
  SE3T GetCameraPose() { return pose_wc_; }
  df::DecoderNetwork::NetworkConfig GetNetworkConfig() { return netcfg_; }
  DeepSlamStatistics GetStatistics() { return stats_; }
  df::PinholeCamera<Scalar> GetNetworkCam();

  /*
   * Setters
   */
  void SetMapCallback(MapCallbackT cb) { map_callback_ = cb; mapper_->SetMapCallback(map_callback_); }
  void SetPoseCallback(PoseCallbackT cb) { pose_callback_ = cb; }
  void SetStatsCallback(StatsCallbackT cb) { stats_callback_ = cb; }
  void SetOptions(DeepSlamOptions opts);

  /*
   * Notifications
   */
  void NotifyPoseObservers();
  void NotifyMapObservers();
  void NotifyStatsObservers();

  /*
   * Debugging
   */
  void SavePostCrashInfo(std::string dir);
  void SaveResults(std::string dir);
  void SaveKeyframes(std::string dir);
  /* retrieve current image and gradients */
  void SaveCurrentImageAndGrad(std::string dir);
  void CreateImgPyrAndGradients(double timestamp, const cv::Mat& frame, bool doupload=true);
  void ForceBuildKeyFrames(double timestamp, const cv::Mat& frame );
  void DispKFs();
  void PosePerKF();

private:
  void InitGpu(std::size_t device_id);
  void UploadLiveFrame(const cv::Mat& frame);
  cv::Mat PreprocessImage(const cv::Mat& frame, cv::Mat& out_color,
                          Features& features);
  SE3T TrackFrame();
  SE3T Relocalize();

  bool NewKeyframeRequired();
  bool NewFrameRequired();
  KeyframeId SelectKeyframe();
  bool CheckTrackingLost(const SE3T& pose_wc);

private:
  bool force_keyframe_;
  bool force_frame_;
  bool bootstrapped_;
  bool tracking_lost_;
  KeyframeId curr_kf_;
  KeyframeId prev_kf_;
  SE3T pose_wc_;
  DeepSlamOptions opts_;
  DeepSlamStatistics stats_; //TODO: Currently it would be last frame's stats. Need to consider mapping it with relevant frames

  std::shared_ptr<DecoderNetwork> network_;
  std::shared_ptr<CameraTracker> tracker_;
  std::unique_ptr<MapperT> mapper_;
  std::unique_ptr<LoopDetectorT> loop_detector_;
  std::unique_ptr<FeatureDetector> feature_detector_;

  df::CameraPyramid<Scalar> camera_pyr_;
  df::PinholeCamera<Scalar> orig_cam_;
  df::DecoderNetwork::NetworkConfig netcfg_;
  cv::Mat map1_, map2_; // maps from initUndistorRectifyMap

  // buffers for the live frame
  std::shared_ptr<ImagePyrT> pyr_live_img_;
  std::shared_ptr<GradPyrT> pyr_live_grad_;

  MapCallbackT map_callback_;
  PoseCallbackT pose_callback_;
  StatsCallbackT stats_callback_;

  // debug stuff
  std::vector<std::pair<int,int>> loop_links;

  struct DebugImages
  {
    cv::Mat reprojection_errors;
    cv::Mat photometric_errors;
  };
  std::list<DebugImages> debug_buffer_;

  typedef df::SE3Aligner<Scalar> SE3AlignerT;
  typename SE3AlignerT::Ptr se3aligner_;
};

} // namespace df

#endif // DF_H_
