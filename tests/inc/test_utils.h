/*
 * Author: Jan Chleboun
 * Date: 12-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#include <list>
#include <string>

#include "SafeIntervalTable.h"
#include "utils.h"

auto get_base_path_tests() -> std::string;

auto generate_random_timepointpath(int len) -> TimePointPath;

auto filter_time_intervals(std::pair<std::list<TimeInterval>::const_iterator, std::list<TimeInterval>::const_iterator> start_goal,
                           TimeInterval interval, int from, int to, SafeIntervalTable& table) -> std::vector<TimeInterval>;


auto find_path_distance_gradient(int agent_num, const Instance& instance) -> Path;
