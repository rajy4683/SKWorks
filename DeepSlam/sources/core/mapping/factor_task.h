/*
 * This file is part of DeepSlam.
 *
 *
 */
#ifndef DF_FACTOR_TASK_H_
#define DF_FACTOR_TASK_H_

#include <memory>
#include <string>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/FastVector.h>

// typedef FactorIndices so that we don't have to include ISAM2 header here
namespace gtsam
{
typedef gtsam::FastVector<size_t> FactorIndices;
}

namespace df
{
namespace work
{

/*
    This is the top-level interface class for all specialized FactorTask objects.
    Each FactorTask will consist of following:
     1. A variant of iSAM2 Factor classes that will be inserted/deleted from iSAM2 graph
     2. Iterations/Lifecycle management hooks which will be invoked by the FactorPipelineController object
*/

class FactorTask
{
public:
  typedef std::shared_ptr<FactorTask> Ptr;

  FactorTask();

  virtual ~FactorTask();

  /*
       Interface class that needs to be implemented in the child classes.
       This function is meant to construct/delete actual factors and map the relevant iterations
  */
  virtual void ManageTask(gtsam::NonlinearFactorGraph& new_factors,
                           gtsam::FastVector<size_t>& remove_indices,
                           gtsam::Values& var_init) = 0;
  /*
      Checks and reduces the iterations of the task
  */
  virtual void UpdateIterations() = 0;
  /*
      Reflects the state of the task
   */
  virtual bool Finished() const = 0;
  virtual std::string Name() = 0;
  /*
      Called when no more  factors can be relinearized. 
  */
  virtual void SignalNoRelinearize() {}
  virtual void SignalRemove() {} /// Factor can be removed
  virtual void LastFactorIndices(gtsam::FactorIndices& indices) {} // Indices received back from the iSAM2->update

  /*
     Function to add one child to the factors. The child nodes are linearized after the parents
  */
  template<class T, class... Args>
  Ptr AddChildTask(Args&&... args)
  {
    auto child = std::make_shared<T>(std::forward<Args>(args)...);
    return AddChildTask(child);
  }

  Ptr AddChildTask(Ptr child);
  Ptr RemoveChildTask();

  std::string Id() const { return id; }

private:
  Ptr child_;
  std::string id;

  static int next_id;
};

} // namespace work
} // namespace df

#endif // DF_FACTOR_TASK_H_
