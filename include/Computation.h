/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#pragma once
#include <atomic>
#include <memory>
#include <random>
#include <thread>

#include "Instance.h"
#include "LNS.h"
#include "SharedData.h"

class Computation
{
public:
  // constructor and destructor
  Computation(const Instance& instance_, SharedData* shared_data_, LNS_settings lns_settings_, int seed);
  ~Computation();

  void setSecondarySolver(LNS_settings secondary_lns_settings_);
  // manage the underlying thread
  void start();
  void stop();
  void join_thread();
  auto get_solution() -> const Solution&
  {
    if (running.load(std::memory_order_acquire))
    {
      throw std::runtime_error("Trying to access the solution before the solver finished.");
    }
    return secondary_solver->solution;
  }

  // the function called in the thread
  void              run();
  std::atomic<bool> running;

private:
  std::unique_ptr<LNS> solver;
  std::unique_ptr<LNS> secondary_solver;
  std::thread          comp_thread;
  const Instance&      instance;
  SharedData*          shared_data;
  int                  seed;
  std::mt19937         rnd_generator;
  LNS_settings         lns_settings;
  LNS_settings         secondary_lns_settings;
};
