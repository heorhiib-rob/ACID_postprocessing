/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "Computation.h"

#include <chrono>
#include <iostream>

#include "LNS.h"

Computation::Computation(const Instance& instance_, SharedData* shared_data_, LNS_settings lns_settings_, int seed_ = -1)
    : instance(instance_), shared_data(shared_data_), running(false), seed(seed_), lns_settings(lns_settings_), secondary_lns_settings(lns_settings_)
{
  // initialized the random generator
  if (seed < 0)
  {
    // random device to generate a random seed
    std::random_device rd;
    rnd_generator = std::mt19937(rd());
  }
  else
  {
    rnd_generator = std::mt19937(seed);
  }

  // initialize the solver
  solver = std::make_unique<LNS>(instance, rnd_generator, shared_data, lns_settings);
}

Computation::~Computation()
{
  stop();
  join_thread();
}

void Computation::setSecondarySolver(LNS_settings secondary_lns_settings_)
{
  secondary_lns_settings = secondary_lns_settings_;
  secondary_solver = std::make_unique<LNS>(instance, rnd_generator, shared_data, secondary_lns_settings);
}

void Computation::start()
{
  // start the underlying thread
  comp_thread = std::thread(&Computation::run, this);
  running.store(true, std::memory_order_release);
}

void Computation::stop()
{
  // set the stop signal
  running.store(false, std::memory_order_release);
}

void Computation::join_thread()
{
  if (comp_thread.joinable())
  {
    comp_thread.join();
  }
}

void Computation::run()
{
  // measure the time of the first phase
  Clock clock;
  clock.start();

  solver->solve();

  // the second phase gets only the remaining time
  const double first_phase_time = clock.get_current_time().first;
  secondary_lns_settings.time_limit = std::max(0.0, secondary_lns_settings.time_limit - first_phase_time);
  if (solver->solution.feasible)
  {
    std::cout << "First phase: found collision-free solution, sum of costs: " << solver->solution.sum_of_costs
              << ", makespan: " << solver->solution.makespan << std::endl;
  }
  else
  {
    std::cout << "First phase: unable to find collision-free solution." << std::endl;
  }

  //give first solver's solution to secondary solver
  Solution second_phase_initial_solution = solver->solution;
  if(second_phase_initial_solution.feasible){
    secondary_solver->set_initial_solution(second_phase_initial_solution);

    //optimize cost
    secondary_solver->solveFromInitial();
  }
  else
  {
    // find a solution from scratch
    secondary_solver->solve();
  }

  if (secondary_solver->solution.feasible)
  {
    std::cout << "Final solution has sum of costs: " << secondary_solver->solution.sum_of_costs
              << ", makespan: " << secondary_solver->solution.makespan << std::endl;
  }
  else
  {
    std::cout << "Unable to find feasible solution." << std::endl;
  }
  running.store(false, std::memory_order_release);
}
