/*
 * Author: Jan Chleboun
 * Date: 22-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#pragma once
#include <utility>

#include "Solver.h"
#include "utils.h"

class SIPPIterationInfo
{
public:
  const TimePoint cur_expanded;
  const double    g;
  const double    h;
  const double    h2;
  const double    h3;
  const int       generated;
  const int       expanded;
  const int       iteration_num;
  SIPPIterationInfo(const TimePoint& cur_expanded_, double g_, double h_, double h2_, double h3_, int generated_, int expanded_,
                    int iteration_num_);
};

using SIPPInfo = std::vector<SIPPIterationInfo>;

class LNSIterationInfo
{
public:
  int                   iteration_num;
  bool                  accepted;
  int                   improvement;
  std::vector<SIPPInfo> sipp_info;
  Solution              sol;
  std::vector<int>      planning_order;
  std::string           destroy_strategy;
  LNSIterationInfo(int iteration_num_, bool accepted_, int improvement_, std::vector<SIPPInfo>& sipp_info_, Solution sol_,
                   std::string destroy_strategy_)
      : iteration_num(iteration_num_),
        accepted(accepted_),
        improvement(improvement_),
        sipp_info(std::move(sipp_info_)),
        sol(std::move(sol_)),
        destroy_strategy(std::move(destroy_strategy_))
  {
    sol.convert_paths();
  }
};

using LNSInfo = std::vector<LNSIterationInfo>;
