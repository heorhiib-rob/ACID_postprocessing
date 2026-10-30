/*
 * Author: Jan Chleboun
 * Date: 06-03-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#pragma once

#include <any>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <variant>

#include "Instance.h"
#include "LNS.h"
#include "indicators/indicators.hpp"
#include "utils.h"
#define PROGRESS_BAR_WIDTH 100

using AlgorithmType = std::variant<SIPP_implementation>;

class Algorithm
{
public:
  AlgorithmType                                 type;
  LNS_settings                                  lns_settings;
  std::vector<std::pair<std::string, std::any>> parameters;

  // Variadic constructor accepting any number of parameters of any type
  template <typename... Args>
  Algorithm(AlgorithmType type_, LNS_settings lns_settings_, Args&&... args) : type(type_), lns_settings(lns_settings_)
  {
    (parameters.emplace_back(std::forward<Args>(args)), ...);
  }

  template <typename T>
  void add_parameter(const T& param)
  {
    parameters.push_back(param);
  }

  [[nodiscard]] auto get_name() const -> std::string;

  [[nodiscard]] auto get_parameters_str() const -> std::vector<std::pair<std::string, std::string>>;
};

auto get_mapf_benchmark_path() -> std::string;

auto get_scene_names(const std::string& map_name, int runs_per_map) -> std::vector<std::string>;

auto create_progress_bar(std::string name, int max_iter) -> std::unique_ptr<indicators::ProgressBar>;

auto load_instances(const std::vector<std::string>& scenes, const std::string& map_name, int agent_num)
    -> std::pair<std::vector<Instance>, std::vector<std::pair<double, double>>>;

void save_experiment(nlohmann::json& experiment_res, int agent_num, const std::string& map_name, const std::string& scene_name,
                     const std::string& experiment_name, const std::string& algorithm_name);

auto what_can_be_skipped(const std::string& experiment_name, const std::vector<std::string>& maps,
                         const std::vector<std::vector<int>>& agent_nums, int runs_per_map, const std::vector<Algorithm>& algorithms)
    -> std::vector<bool>;
