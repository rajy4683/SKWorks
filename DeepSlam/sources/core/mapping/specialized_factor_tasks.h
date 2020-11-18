/*
 * This file is part of DeepSlam.
 *
 */
#ifndef DF_SPECIAL_FACTORS_TASKS_H_
#define DF_SPECIAL_FACTORS_TASKS_H_

#include "factor_task.h"

// forward declare these some time later
#include "keyframe.h"
#include "cu_sfmaligner.h"
#include "camera_pyramid.h"
#include "photometric_factor.h"
#include "sparse_geometric_factor.h"
#include "reprojection_factor.h"

namespace df
{
namespace work
{


/*
   A simple task to check the Factor pipeline flow
*/
class CallbackFactorTask : public FactorTask
{
public:
  typedef std::function<void()> CallbackT;
  CallbackFactorTask(CallbackT f) : finished_(false), f_(f) {}

  virtual void ManageTask(gtsam::NonlinearFactorGraph& new_factors,
                           gtsam::FactorIndices& remove_indices,
                           gtsam::Values& var_init) override {}

  virtual void UpdateIterations() override
  {
    f_();
    finished_ = true;
  }

  virtual bool Finished() const override
  {
    return finished_;
  }

  virtual std::string Name() override { return "CallbackFactorTask"; }

  bool finished_;
  CallbackT f_;
};

/*
   Task object that creates the priors for new factors.
   This object is instantiated when:
   1. During Bootstrapping of Keyframes for SLAM system
   2. As a first step during addition of any Keyframe/Frame 
*/

template <typename Scalar, int CS>
class InitPriorsTask : public FactorTask
{
public:
  typedef typename df::Keyframe<Scalar>::Ptr KeyframePtr;
  typedef typename df::Frame<Scalar>::Ptr FramePtr;

  // constructor from keyframe
  InitPriorsTask(KeyframePtr kf, Scalar code_prior);

  // constructor from keyframe with zero-pose prior
  InitPriorsTask(KeyframePtr kf, Scalar code_prior, Scalar pose_prior);

  // constructor from frame
  InitPriorsTask(FramePtr fr);

  virtual ~InitPriorsTask();

  virtual void ManageTask(gtsam::NonlinearFactorGraph& new_factors,
                           gtsam::FactorIndices& remove_indices,
                           gtsam::Values& var_init) override;

  virtual void UpdateIterations() override;

  virtual bool Finished() const override
  {
    return !first_;
  }

  virtual std::string Name() override;

private:
  bool first_;
  gtsam::Values var_init_;
  gtsam::NonlinearFactorGraph priors_;
  std::string name_;
};

// adds support for multi-scale optimization
template <typename Scalar>
class GenericFactorTask : public FactorTask
{
public:
  typedef typename df::Keyframe<Scalar>::Ptr KeyframePtr;
  typedef typename df::Frame<Scalar>::Ptr FramePtr;
  typedef std::vector<int> IterList;

  // constructor for single scale optimization
  // Used mainly for Geometric and Reprojection factor tasks
  GenericFactorTask(int iters, bool remove_after = false);

  // constructor for multi scale optimization
  // Currently used only for Photometric factor tasks
  GenericFactorTask(IterList iters, bool remove_after = false);

  virtual ~GenericFactorTask() {}

  // basic version of this func adds some factors
  // on first run and then keeps track of iterations
  virtual void ManageTask(gtsam::NonlinearFactorGraph& new_factors,
                           gtsam::FactorIndices& remove_indices,
                           gtsam::Values& var_init) override;

  // override this function to create simple
  // non pyramid levels
  virtual gtsam::NonlinearFactorGraph ConstructFactors();

  // counts iterations and descends pyramid levels
  virtual void UpdateIterations() override;

  virtual bool Finished() const override;

  virtual void SignalNoRelinearize() override;
  virtual void SignalRemove() override;
  virtual void LastFactorIndices(gtsam::FactorIndices& indices) override;

  virtual bool Involves(FramePtr ptr) const = 0;

  bool IsCoarsestLevel() const;
  bool IsNewLevelStart() const;

protected:
  bool remove_;
  bool first_;
  IterList iters_;
  IterList orig_iters_;
  int active_level_;
  gtsam::FactorIndices last_indices_;
  bool remove_after_;
};

/* 
   Task to manage a Photometric factor
*/
template <typename Scalar, int CS>
class PhotometricTask : public GenericFactorTask<Scalar>
{
public:
  typedef typename df::Keyframe<Scalar>::Ptr KeyframePtr;
  typedef typename df::Frame<Scalar>::Ptr FramePtr;
  typedef typename df::SfmAligner<Scalar,CS>::Ptr AlignerPtr;
  typedef typename df::CameraPyramid<Scalar> CamPyramid;
  typedef df::PhotometricFactor<Scalar,CS> PhotoFactor;
  typedef typename GenericFactorTask<Scalar>::IterList IterList;

  // constructor with a vector of iterations
  PhotometricTask(KeyframePtr kf, FramePtr fr, IterList iters,
                CamPyramid cam_pyr, AlignerPtr aligner, bool update_vld = false, bool remove_after=false);

  virtual ~PhotometricTask();

  gtsam::NonlinearFactorGraph ConstructFactors() override;
  virtual bool Involves(FramePtr ptr) const;
  virtual std::string Name();

private:
  KeyframePtr kf_;
  FramePtr fr_;
  df::CameraPyramid<Scalar> cam_pyr_;
  AlignerPtr aligner_;
  bool update_vld_;
};

/*
 *  Task to manage a Geometric factor
 */
template <typename Scalar, int CS>
class GeometricTask : public GenericFactorTask<Scalar>
{
public:

  typedef typename df::Frame<Scalar>::Ptr FramePtr;
  typedef typename df::Keyframe<Scalar>::Ptr KeyframePtr;
  typedef df::SparseGeometricFactor<Scalar,CS> GeoFactor;

  // constructor with single number of iterations
  GeometricTask(KeyframePtr kf0, KeyframePtr kf1,
              int iters, df::PinholeCamera<Scalar> cam,
              int num_points, float huber_delta, bool stochastic);

  virtual ~GeometricTask();

  virtual gtsam::NonlinearFactorGraph ConstructFactors() override;
  virtual bool Involves(FramePtr ptr) const override;
  virtual std::string Name() override;

private:
  KeyframePtr kf0_;
  KeyframePtr kf1_;
  df::PinholeCamera<Scalar> cam_;
  int num_points_;
  float huber_delta_;
  bool stochastic_;
};

/*
 *  Task to manage a Reprojection factor
 */
template <typename Scalar, int CS>
class ReprojectionTask : public GenericFactorTask<Scalar>
{
public:
  typedef typename df::Keyframe<Scalar>::Ptr KeyframePtr;
  typedef typename df::Frame<Scalar>::Ptr FramePtr;
  typedef df::ReprojectionFactor<Scalar,CS> RepFactor;

  // constructor with single number of iterations
  ReprojectionTask(KeyframePtr kf, FramePtr fr, int iters,
              df::PinholeCamera<Scalar> cam,
              float feature_max_dist, float huber_delta,
              float sigma, int maxiters, float threshold);

  virtual ~ReprojectionTask();

  virtual bool Finished() const override;
  virtual gtsam::NonlinearFactorGraph ConstructFactors() override;
  virtual bool Involves(FramePtr ptr) const override;
  virtual std::string Name() override;

private:
  KeyframePtr kf_;
  FramePtr fr_;
  df::PinholeCamera<Scalar> cam_;
  float max_dist_;
  float huber_delta_;
  float sigma_;
  int maxiters_;
  float threshold_;
  bool finished_ = false;
};

} // namespace work
} // namespace df

#endif // DF_SPECIAL_FACTORS_TASKS_H_
