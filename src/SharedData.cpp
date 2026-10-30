/*
 * Author: Jan Chleboun
 * Date: 20-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#include "SharedData.h"

#include <iostream>

SharedData::SharedData() : is_new_info(false), is_end(false)
{
  std::cout << "Initialized shared data." << std::endl;
}

void SharedData::update_lns_info(LNSIterationInfo new_info)
{
  // lock  the mutex
  std::lock_guard<std::mutex> lock(mtx);
  lns_info.push_back(std::move(new_info));

  // signal that new data arrived
  is_new_info.store(true, std::memory_order_release);
}

auto SharedData::consume_lns_info() -> std::vector<LNSIterationInfo>
{
  assertm(is_new_info.load(std::memory_order_acquire), "Trying to consume paths when there are no paths.");
  // lock  the mutex
  std::lock_guard<std::mutex> lock(mtx);

  // signal that the paths have been consumed
  is_new_info.store(false, std::memory_order_release);

  // return the paths (which consumes them)
  std::vector<LNSIterationInfo> ret = std::move(lns_info);
  lns_info.clear();
  return ret;
}

