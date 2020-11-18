/*
 * This file is part of DeepSlam.
 */
#include "factor_pipeline_controller.h"

namespace df
{
namespace work
{

FactorPipelineController::FactorTaskPtr FactorPipelineController::AddFactorTask(FactorTaskPtr task)
{
  factor_obj_list_.push_back(task);
  factor_obj_map_.insert({task->Id(), task});
  return task;
}

void FactorPipelineController::ManagePipeline(gtsam::NonlinearFactorGraph &new_factors,
                              gtsam::FactorIndices &remove_indices,
                              gtsam::Values &init_variables)
{
  // factors added this iteration
  // will be used in consuming the factor indices after ISAM2::update step
  latest_created_factors_.clear();

  // Get the newly contructed factos and the removable indices.
  for (auto& task : factor_obj_list_)
  {
    gtsam::NonlinearFactorGraph add_factors;
    gtsam::FactorIndices rem_factors; // Contains the factors to be removed
    gtsam::Values init;
    task->ManageTask(add_factors, rem_factors, init);

    // keep track which task items added factors
    if (!add_factors.empty())
      latest_created_factors_.insert({task->Id(), add_factors.size()});

    // add new factors, vars, etc
    new_factors += add_factors;
    remove_indices.insert(remove_indices.end(), rem_factors.begin(), rem_factors.end());
    init_variables.insert(init);
  }
}
void FactorPipelineController::ManagePipelineWithUpdate(gtsam::NonlinearFactorGraph &new_factors,
                              gtsam::FactorIndices &remove_indices,
                              gtsam::Values &init_variables)
{
  // factors added this iteration
  // will be used in consuming the factor indices after ISAM2::update step
  latest_created_factors_.clear();
  // Get the newly contructed factos and the removable indices.
  //for (auto& task : factor_obj_list_)
  auto it = factor_obj_list_.begin();
  while (it != factor_obj_list_.end())
  {
    FactorTaskPtr task = *it;
    gtsam::NonlinearFactorGraph add_factors;
    gtsam::FactorIndices rem_factors; // Contains the factors to be removed
    gtsam::Values init;
    task->ManageTask(add_factors, rem_factors, init);

    // keep track which task items added factors
    if (!add_factors.empty())
      latest_created_factors_.insert({task->Id(), add_factors.size()});

    // add new factors, vars, etc
    new_factors += add_factors;
    remove_indices.insert(remove_indices.end(), rem_factors.begin(), rem_factors.end());
    init_variables.insert(init);

    // in case we need to remove this task
    auto this_it = it;
    it++;

    task->UpdateIterations();

    // check if the task has finished
    if (task->Finished())
    {
      LOG(INFO) << "Work " << task->Id() << " has finished";
      // add its child if it has one
      auto child = task->RemoveChildTask();
      if (child)
      {
        VLOG(1) << "Adding its child: " << child->Id();
        AddFactorTask(child);
      }
      factor_obj_map_.erase(task->Id());
      factor_obj_list_.erase(this_it);
    }
  }
}

void FactorPipelineController::RehashIndices(gtsam::FactorIndices indices)
{
  for (auto& kv : latest_created_factors_)
  {
    std::string id = kv.first;
    auto task = factor_obj_map_[id];
    int n = kv.second;

    // first N goes to id
    gtsam::FactorIndices ind(indices.begin(), indices.begin() + n);
    if (task)
      task->LastFactorIndices(ind);

    // remove
    indices.erase(indices.begin(), indices.begin() + n);
  }
  latest_created_factors_.clear();
}

void FactorPipelineController::Remove(std::function<bool (FactorTaskPtr)> f)
{
  auto it = std::remove_if(factor_obj_list_.begin(), factor_obj_list_.end(), f);
  for (auto ii = it; ii != factor_obj_list_.end(); ++ii)
    (*ii)->SignalRemove();
}

void FactorPipelineController::Purge(std::function<bool (FactorTaskPtr)> f)
{
  auto it = std::remove_if(factor_obj_list_.begin(), factor_obj_list_.end(), f);
  factor_obj_list_.erase(it, factor_obj_list_.end());
}

void FactorPipelineController::Update()
{
  VLOG(1) << "[FactorPipelineController::Update] Current task:";
  /*for (auto& task : factor_obj_list_)
    VLOG(1) << task->Name();
  */
  auto it = factor_obj_list_.begin();
  while (it != factor_obj_list_.end())
  {
    FactorTaskPtr task = *it;

    // in case we need to remove this task
    auto this_it = it;
    it++;

    task->UpdateIterations();
    // check if the task has finished
    if (task->Finished())
    {
      LOG(INFO)<< "Work:" << task->Id() << " has finished";
      // add its child if it has one
      auto child = task->RemoveChildTask();
      if (child)
      {
        LOG(INFO) << "Adding its child: " << child->Id();
        AddFactorTask(child);
      }
      factor_obj_map_.erase(task->Id());
      factor_obj_list_.erase(this_it);
    }
  }
}

void FactorPipelineController::SkipRelinearize()
{
  for (auto& task : factor_obj_list_)
    task->SignalNoRelinearize();
}

void FactorPipelineController::PrintEntries()
{
  for (auto& task : factor_obj_list_)
    LOG(INFO) << task->Name();
}

void FactorPipelineController::Clear()
{
  factor_obj_list_.clear();
  factor_obj_map_.clear();
  latest_created_factors_.clear();
}

} // namespace task
} // namespace df
