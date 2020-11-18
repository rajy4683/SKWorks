/*
 * This file is part of DeepSlam.
 *
 */
#include "factor_task.h"

namespace df
{
namespace work
{

int FactorTask::next_id = 0;

FactorTask::~FactorTask() {}

FactorTask::FactorTask()
{
  id = "[" + std::to_string(next_id++) + "]";
}

FactorTask::Ptr FactorTask::AddChildTask(Ptr child)
{
  child_ = child;
  return child;
}

FactorTask::Ptr FactorTask::RemoveChildTask()
{
  auto child = child_;
  child_.reset();
  return child;
}

} // namespace work
} // namespace df
