/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include <utils.h>

#include <array>
#include <boost/functional/hash.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <nfd.hpp>

void Clock::start()
{
  // wall time
  wall_time_start = Time::now();
  // cpu time
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_time_start);
  initialized = true;
}

[[nodiscard]] auto Clock::get_current_time() const -> std::pair<double, double>
{
  assertm(initialized, "Can not retrieve time of uninitialized clock.");
  double   wall_time = ((dsec)(Time::now() - wall_time_start)).count();
  timespec ts{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  auto cpu_time =
      (static_cast<double>(ts.tv_sec - cpu_time_start.tv_sec) + static_cast<double>((ts.tv_nsec - cpu_time_start.tv_nsec)) / 1e9);
  return {wall_time, cpu_time};
}

auto Clock::end() -> std::pair<double, double>
{
  assertm(initialized, "Can not end uninitialized clock.");
  initialized        = false;
  double   wall_time = ((dsec)(Time::now() - wall_time_start)).count();
  timespec ts{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  auto cpu_time =
      (static_cast<double>(ts.tv_sec - cpu_time_start.tv_sec) + static_cast<double>((ts.tv_nsec - cpu_time_start.tv_nsec)) / 1e9);
  return {wall_time, cpu_time};
}

Node::Node(const double g_, const double h_) : g(g_), h(h_), f(g_ + h_)
{
}

auto Node::operator>(const Node& other) -> bool
{
  return f > other.f;
}

auto Node::operator<(const Node& other) -> bool
{
  return f < other.f;
}

auto Node::operator==(const Node& other) -> bool
{
  return g == other.g && h == other.h;
}


Point2d::Point2d(int x_, int y_) : x(x_), y(y_)
{
}

auto Point2d::operator+(const Point2d& other) const -> Point2d
{
  Point2d ret(x + other.x, y + other.y);
  return ret;
}

auto Point2d::operator==(const Point2d& other) const -> bool
{
  return x == other.x && y == other.y;
}

void Point2d::add(const Point2d& other)
{
  x += other.x;
  y += other.y;
}

auto Point2d::hash_point::operator()(const Point2d& p) const -> size_t
{
  std::size_t seed = 0;
  boost::hash_combine(seed, p.x);
  boost::hash_combine(seed, p.y);
  return seed;
}


Point2d::operator std::string() const
{
  return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
}

auto find_direction(const Point2d& start, const Point2d& end) -> Direction
{
  if (start.x > end.x)
  {
    return Direction::LEFT;
  }
  if (start.y > end.y)
  {
    return Direction::UP;
  }
  if (start.y < end.y)
  {
    return Direction::DOWN;
  }
  if (start.x < end.x)
  {
    return Direction::RIGHT;
  }
  return Direction::NONE;
}

auto find_direction(const int start, const int end) -> Direction
{
  if (start == end + 1)
  {
    return Direction::LEFT;
  }
  if (start == end - 1)
  {
    return Direction::RIGHT;
  }
  if (start > end)
  {
    return Direction::UP;
  }
  if (start < end)
  {
    return Direction::DOWN;
  }
  return Direction::NONE;
}

TimeInterval::TimeInterval(const int t_min_, const int t_max_) : t_min(t_min_), t_max(t_max_)
{
}

TimeInterval::TimeInterval(const int t_min_, const int t_max_, const bool col_) : t_min(t_min_), t_max(t_max_), col(col_)
{
}

auto TimeInterval::operator==(const TimeInterval& other) const -> bool
{
  return t_min == other.t_min && t_max == other.t_max;
}

auto TimeInterval::operator+(const int x) const -> TimeInterval
{
  return TimeInterval(t_min + x, t_max + x);
}

TimeInterval::operator std::string() const
{
  if(!col){
    return "(" + std::to_string(t_min) + "," + std::to_string(t_max) + ")";
  } else {
    return "(SC" + std::to_string(t_min) + "," + std::to_string(t_max) + "SC)";
  }
}

auto overlap(const TimeInterval& i1, const TimeInterval& i2) -> bool
{
  return !(i1.t_max < i2.t_min || i2.t_max < i1.t_min);
}

TimePoint::TimePoint(const int location_, const TimeInterval& interval_) : location(location_), interval(interval_)
{
}

auto TimePoint::operator==(const TimePoint& other) const -> bool
{
  return location == other.location && interval == other.interval;
}

// functions to print Point2d and TimeInterval

auto operator<<(std::ostream& os, Point2d const& p) -> std::ostream&
{
  return os << "(" << p.x << "," << p.y << ")";
}

auto operator<<(std::ostream& os, TimeInterval const& i) -> std::ostream&
{
  return os << "(" << i.t_min << "," << i.t_max << ")";
}

auto timepointpath_to_path(const TimePointPath& tp_path) -> Path
{
  Path path;
  for (auto it = tp_path.begin(); it != tp_path.end(); it++)
  {
    // for last element add the position only once, as it is obvious that the agent stays in the last position forever
    int N = 1;

    // find the start of the next time interval
    if (it != std::prev(tp_path.end()))
    {
      N = std::next(it)->interval.t_min - it->interval.t_min;
    }

    // insert N copies to the end of the vector
    path.resize(path.size() + N, it->location);
  }
  return path;
}

auto path_to_timepointpath(const Path& path) -> TimePointPath
{
  if (path.empty())
  {
    return TimePointPath();
  }

  TimePointPath tp_path;
  TimePoint     curr(path.front(), {0, -1});
  for (int it : path)
  {
    // if the new location is not the same as the last location, create new interval
    if (it != curr.location)
    {
      tp_path.push_back(curr);

      // set the new location
      curr.location       = it;
      curr.interval.t_min = curr.interval.t_max + 1;
      curr.interval.t_max = curr.interval.t_min;
    }
    else
    {
      curr.interval.t_max++;
    }
  }
  // the agent stays on the last position
  curr.interval.t_max = INT_MAX;
  tp_path.push_back(curr);
  return tp_path;
}

auto check_timepointpath_interval_no_overlap(const TimePointPath& tp_path) -> bool
{
  // paths with 0 or 1 interval can not have overlaps
  if (tp_path.size() <= 1)
  {
    return true;
  }

  // check for overlaps
  for (auto it = tp_path.begin(); it != std::prev(tp_path.end()); it++)
  {
    if (overlap(it->interval, std::next(it)->interval))
    {
      return false;
    }
  }
  return true;
}

void fix_timepointpath_interval_overlap(TimePointPath& tp_path)
{
  // paths with 0 or 1 interval can not have overlaps
  if (tp_path.size() >= 2)
  {
    // check for overlaps
    for (auto it = tp_path.begin(); it != std::prev(tp_path.end()); it++)
    {
      it->interval.t_max = std::next(it)->interval.t_min - 1;
    }
  }
}

auto is_valid_timepointpath(const TimePointPath& tp_path) -> bool
{
  // special case - empty path
  if (tp_path.empty())
  {
    return true;
  }

  // check that each interval has smaller t_min than t_max
  for (const auto& it : tp_path)
  {
    if (it.interval.t_min > it.interval.t_max)
    {
      return false;
    }
  }

  // check that the agent stays at its last position
  if (tp_path.back().interval.t_max != INT_MAX)
  {
    return false;
  }

  if ((int)tp_path.size() >= 2)
  {
    // check that each interval starts after the previous one ends
    for (auto it = tp_path.begin(); it != std::prev(tp_path.end()); it++)
    {
      if (it->interval.t_max != std::next(it)->interval.t_min - 1)
      {
        return false;
      }
    }
  }

  // check whether the locations are neighbors
  return true;
}

// Function to get the directory of the running executable
auto get_base_path() -> std::string
{
  return std::filesystem::canonical("/proc/self/exe").parent_path().parent_path().string();  // Linux only
}


auto fast_random_bool() -> bool
{
  static unsigned int num   = 0;
  static int          count = 0;
  if (count % std::numeric_limits<int>::digits == 0)
  {
    num   = std::rand();
    count = 0;
  }
  const bool ret = (bool)((num >> count) & 1);
  count++;
  return ret;
}

auto double_to_str(double num, char separating_char, int precision) -> std::string
{
  std::stringstream stream;
  stream << std::fixed << std::setprecision(precision) << num;
  std::string ret = stream.str();
  // replace . with -
  std::replace(ret.begin(), ret.end(), '.', separating_char);
  return ret;
}

auto any_to_str(const std::any& value) -> std::string
{
  // string
  if (value.type() == typeid(std::string))
  {
    return std::any_cast<std::string>(value);
  }
  // string_view
  if (value.type() == typeid(std::string_view))
  {
    return std::string(std::any_cast<std::string_view>(value));
  }
  // const char*
  if (value.type() == typeid(const char*))
  {
    return std::string(std::any_cast<const char*>(value));
  }
  // int
  if (value.type() == typeid(int))
  {
    return std::to_string(std::any_cast<int>(value));
  }
  // float
  if (value.type() == typeid(float))
  {
    return double_to_str(std::any_cast<float>(value));
  }
  // double
  if (value.type() == typeid(double))
  {
    return double_to_str(std::any_cast<double>(value));
  }
  // bool
  if (value.type() == typeid(bool))
  {
    return std::to_string(static_cast<int>(std::any_cast<bool>(value)));
  }
  // unknown type
  throw std::runtime_error("Unsupported parameter type in any to str: " + std::string(value.type().name()));
}

auto find_last_number(const std::string& str) -> int
{
  // iterate from the back
  int i = static_cast<int>(str.size()) - 1;

  while (i >= 0 && (std::isdigit(str[i]) == 0))  // skip non digits
  {
    i--;
  }

  if (i < 0)
  {
    return -1;  // No number found
  }

  int end = i;

  // Find the beginning of the number
  while (i >= 0 && (std::isdigit(str[i]) > 0))
  {
    i--;
  }

  return std::stoi(str.substr(i + 1, end - i));
}

auto location_at_time(TimePointPath& path, int time) -> int
{
  for (const auto& it : path)
  {
    if (it.interval.t_min <= time && it.interval.t_max >= time)
    {
      return it.location;
    }
  }
  return -1;
}

auto open_file_dialog(const std::string& type, const std::string& file_extensions) -> std::string
{
  // initialize nfd to ensure proper cleanup
  NFD::Guard      nfdGuard;
  NFD::UniquePath outPath;

  // prepare the filter
  nfdfilteritem_t filter[1] = {{type.c_str(), file_extensions.c_str()}};
  // show the dialog
  nfdresult_t result = NFD::OpenDialog(outPath, filter, 1);
  if (result == NFD_OKAY)
  {
    return outPath.get();
  }
  if (result != NFD_CANCEL)  // user pressed cancel
  {
    std::cout << "Error: " << NFD::GetError() << std::endl;
  }
  return "";
}
