/*
 * This file is part of DeepSlam code base.
 *
 */
#ifndef DF_FACTOR_PIPELINE_CTRL_H_
#define DF_FACTOR_PIPELINE_CTRL_H_

#include <list>
#include <functional>
#include <glog/logging.h>

#include "factor_task.h"
#include "specialized_factor_tasks.h"

namespace df
{
namespace work
{

/*
   Manages the iSAM2 factors pipelines
   1. It creates/constructs the factors, 
   2. Keeps track of the iterations of each factor
   3. Deletes/clears upon completion of relinearization
   Although currently not a singleton, there should be only one object of this class. 
*/
class FactorPipelineController
{
public:
	typedef FactorTask::Ptr FactorTaskPtr;

	// Adds a new factor-block into the pipeline
	// THis function has two signatures: one with rvalue reference+variable arguments and other with regular object 
	template <typename T, class... Args>
	FactorTaskPtr AddFactorTask(Args&&... args) //Adds a new factor-block into the pipeline
	{
		auto work = std::make_shared<T>(std::forward<Args>(args)...);
		return AddFactorTask(work);
	}
	// Add a new factor to the pipeline.
	FactorTaskPtr AddFactorTask(FactorTaskPtr work); // AddWork

	void ManagePipeline(gtsam::NonlinearFactorGraph& add_factors, // Bookkeeping: 
				   gtsam::FactorIndices& remove_indices,
				   gtsam::Values& var_init);
        
	void ManagePipelineWithUpdate(gtsam::NonlinearFactorGraph& add_factors, // Bookkeeping + Update done together
				   gtsam::FactorIndices& remove_indices,
				   gtsam::Values& var_init);

	void RehashIndices(gtsam::FactorIndices indices); // DistributeIndices: Redistributes index groups to respective Factor objects
	void Remove(std::function<bool (FactorTaskPtr)> f); // Removes given set of factors from the pipeline and also marks individual Factor objects for removal
	void Purge(std::function<bool (FactorTaskPtr)> f); // Erase: Directly knocks off the set of Factors from the pipeline

	void Update(); // Updates iterations for every factor that was added and removes children  if a particular factor object is marked Finished
	void SkipRelinearize(); // If iSAM2 returns with 0 (i.e no factors to relinearize), this function will reduce iteration cou

	void PrintEntries();
	bool Empty() const { return factor_obj_list_.empty(); }
	void Clear(); // Clears the factor queue, factor maps and the last_factors_ map-->Clean reset

private:
  std::list<FactorTaskPtr> factor_obj_list_; // List of all the factors in existence
  std::map<std::string, FactorTaskPtr> factor_obj_map_; // Stores the factor_obj.Id to Factor Object mapping
  std::map<std::string, int> latest_created_factors_;
};

} // namespace work
} // namespace df

#endif // DF_FACTOR_PIPELINE_CTRL_H_
