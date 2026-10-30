/*
 * Author: Jan Chleboun
 * Date: 20-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#pragma once
#include <atomic>
#include <mutex>

#include "IterInfo.h"
#include "utils.h"

class SharedData
{
public:
  std::atomic<bool> is_new_info;
  std::atomic<bool> is_end;

  SharedData();

  void update_lns_info(LNSIterationInfo new_info);

  auto consume_lns_info() -> std::vector<LNSIterationInfo>;

private:
  std::mutex                    mtx;
  std::vector<LNSIterationInfo> lns_info;
};
