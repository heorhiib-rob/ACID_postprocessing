/*
 * Author: Jan Chleboun
 * Date: 06-03-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "experiment_utils.h"

#include "magic_enum/magic_enum.hpp"
#include "utils.h"

auto get_mapf_benchmark_path() -> std::string
{
  return std::filesystem::canonical("/proc/self/exe").parent_path().parent_path().parent_path().parent_path().string() +
         "/MAPF-benchmark";  // Linux only
}

auto get_scene_names(const std::string& map_name, int runs_per_map) -> std::vector<std::string>
{
  std::vector<std::string> scen_files;  // will be returned

  // find path to the scene directory
  std::string scen_dir_path = get_mapf_benchmark_path() + "/mapf-scen-random/scen-random/";

  // iterate over all files in the scene directory
  for (const auto& entry : std::filesystem::directory_iterator(scen_dir_path))
  {
    std::string filename = entry.path().filename().string();
    // Check if the filename starts with map name
    if (filename.rfind(map_name, 0) == 0)
    {
      // add to the scene files
      scen_files.push_back(filename);
    }
  }
  const int scen_num = std::min(static_cast<int>(scen_files.size()), runs_per_map);

  struct
  {
    auto operator()(const std::string& a, const std::string& b) const -> bool
    {
      const int a_scen_num = find_last_number(a);
      const int b_scen_num = find_last_number(b);
      return a_scen_num < b_scen_num;
    }
  } scene_compare;

  // print
  std::sort(scen_files.begin(), scen_files.end(), scene_compare);
  std::vector<std::string> result(std::make_move_iterator(scen_files.begin()), std::make_move_iterator(scen_files.begin() + scen_num));
  return result;
}

auto create_progress_bar(std::string name, int max_iter) -> std::unique_ptr<indicators::ProgressBar>
{
  return std::make_unique<indicators::ProgressBar>(
      indicators::option::BarWidth{PROGRESS_BAR_WIDTH}, indicators::option::Start{"["}, indicators::option::Fill{"■"},
      indicators::option::Lead{"■"}, indicators::option::Remainder{" "}, indicators::option::End{" ]"},
      indicators::option::ForegroundColor{indicators::Color::white}, indicators::option::ShowElapsedTime{true},
      indicators::option::MaxProgress{max_iter}, indicators::option::ShowRemainingTime{true}, indicators::option::PrefixText{name},
      indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}});
}

auto load_instances(const std::vector<std::string>& scenes, const std::string& map_name, int agent_num)
    -> std::pair<std::vector<Instance>, std::vector<std::pair<double, double>>>
{
  const std::string                      map_path = get_mapf_benchmark_path() + "/mapf-map/" + map_name + ".map";
  std::vector<std::pair<double, double>> preprocessing_times(scenes.size());
  std::vector<Instance>                  ret(scenes.size());
  Clock                                  clock;

  for (int i = 0; i < (int)scenes.size(); i++)
  {
    const std::string scene_name = scenes[i];
    // create the instance
    const std::string scene_path = get_mapf_benchmark_path() + "/mapf-scen-random/scen-random/" + scene_name;
    clock.start();
    ret[i]                 = Instance(map_path, scene_path, agent_num);
    preprocessing_times[i] = clock.end();
  }
  return {ret, preprocessing_times};
}

void save_experiment(nlohmann::json& experiment_res, int agent_num, const std::string& map_name, const std::string& scene_name,
                     const std::string& experiment_name, const std::string& algorithm_name)
{
  // choose a file name that does not exist yet
  int                      num                 = 0;
  const std::string        scene_name_stripped = scene_name.substr(map_name.size() + 1, scene_name.find('.') - map_name.size() - 1);
  static const std::string experiment_dir_path = get_base_path() + "/experiments/" + experiment_name + "/";
  std::string              result_dir_path     = experiment_dir_path + map_name + "/";
  std::string              filename = result_dir_path + experiment_name + "_" + std::to_string(agent_num) + "agents_" + map_name + "_" +
                         scene_name_stripped + "_" + algorithm_name + "_" + std::to_string(num) + ".json";
  while (std::filesystem::exists(filename))
  {
    num++;
    filename = result_dir_path + experiment_name + "_" + std::to_string(agent_num) + "agents_" + map_name + "_" + scene_name_stripped +
               "_" + algorithm_name + "_" + std::to_string(num) + ".json";
  }
  // std::cout << "Saving experiment to: " << filename << std::endl;

  std::ofstream file(filename);
  file << experiment_res.dump(4);  // Pretty print
  file.close();
}

auto what_can_be_skipped(const std::string& experiment_name, const std::vector<std::string>& maps,
                         const std::vector<std::vector<int>>& agent_nums, int runs_per_map, const std::vector<Algorithm>& algorithms)
    -> std::vector<bool>
{
  assertm(maps.size() == agent_nums.size(), "The number of maps must be the same as the number of agent numbers lists");
  std::vector<bool> skips;
  const std::string result_dir_path = get_base_path() + "/experiments/" + experiment_name + "/";

  // iterate over the maps
  for (int i = 0; i < static_cast<int>(maps.size()); i++)
  {
    const std::string& map_name = maps[i];
    const std::string  map_dir  = result_dir_path + map_name;
    // std::cout << "Map dir: " << map_dir << std::endl;
    if (!std::filesystem::exists(map_dir))
    {
      std::filesystem::create_directory(map_dir);
      // cant skip files for which the directory did not exist
      skips.resize(skips.size() + agent_nums[i].size() * runs_per_map * algorithms.size(), false);
      continue;
    }

    skips.reserve(skips.size() + agent_nums[i].size() * runs_per_map * algorithms.size());


    std::vector<std::string> scenes = get_scene_names(map_name, runs_per_map);
    assertm(!scenes.empty(), "Did not find any scenes");

    // iterate over the agent numbers
    for (int j = 0; j < (int)agent_nums[i].size(); j++)
    {
      assertm(i <= (int)agent_nums.size(), "Invalid map index");
      assertm(j <= (int)agent_nums[i].size(), "Invalid map index");
      for (int k = 0; k < runs_per_map; k++)
      {
        const int          idx        = k % (int)scenes.size();
        const std::string& scene_name = scenes[idx];
        size_t             dot_pos    = scene_name.find('.');
        if (dot_pos == std::string::npos)
        {
          throw std::runtime_error("Could not find dot in scene name: '" + scene_name + "'");
        }
        const std::string scene_name_stripped = scene_name.substr(map_name.size() + 1, dot_pos - map_name.size() - 1);

        for (const auto& algo : algorithms)
        {
          const std::string algorithm_name = algo.get_name();
          const std::string filename = map_dir + "/" + experiment_name + "_" + std::to_string(agent_nums[i][j]) + "agents_" + map_name +
                                       "_" + scene_name_stripped + "_" + algorithm_name + "_" +
                                       std::to_string((int)std::floor(k / scenes.size())) + ".json";

          assertm((int)filename.size() < 260, "Filename must be shorter than 260 characters.");
          // check whether the file exists
          if (std::filesystem::exists(filename))
          {
            skips.push_back(true);
          }
          else
          {
            skips.push_back(false);
          }
        }
      }
    }
  }
  return skips;
}


[[nodiscard]] auto Algorithm::get_name() const -> std::string
{
  // convert the enum to string
  std::string ret = std::visit([](auto&& arg) -> std::string { return std::string(magic_enum::enum_name(arg)); }, type);

  // if there are no parameters, return the string
  if (parameters.empty())
  {
    return ret;
  }

  // else add the parameters to the string representation
  for (const auto& param : parameters)
  {
    // add the parameter name
    ret += "_" + param.first + "_";

    // convert the parameter value to string
    std::string param_val_str = any_to_str(param.second);

    ret += param_val_str;
  }
  return ret;
}

[[nodiscard]] auto Algorithm::get_parameters_str() const -> std::vector<std::pair<std::string, std::string>>
{
  std::vector<std::pair<std::string, std::string>> ret(parameters.size());
  for (int i = 0; i < static_cast<int>(parameters.size()); i++)
  {
    ret[i] = std::make_pair(parameters[i].first, any_to_str(parameters[i].second));
  }
  return ret;
}

