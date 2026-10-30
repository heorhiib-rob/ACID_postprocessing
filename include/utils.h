/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#pragma once
// #include <omp.h>

#include <SFML/Graphics.hpp>
#include <any>
#include <cassert>
#include <chrono>
#include <climits>
#include <cstdint>
#include <vector>
#include <unordered_set>

// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert((void(msg), exp))

// time measuring
using dsec = std::chrono::duration<double>;
using Time = std::chrono::steady_clock;

enum class Direction
{
  NONE  = 0,
  UP    = 1,
  RIGHT = 2,
  DOWN  = 3,
  LEFT  = 4
};

enum class SIPP_implementation
{
  SIPP_mine,
  SIPP_mine_ap,
  SIPP_suboptimal,
  SIPP_suboptimal_ap,
  SIPP_mapf_lns,
};

enum class INFO_type
{
  no_info,
  visualisation,
  experiment
};


// struct omp_locks
// {
//   // constructor
//   explicit omp_locks(int num_cells);

//   // destructor
//   ~omp_locks();

//   // std::vector<omp_lock_t> locks;  // for parallelization
// };

class Clock
{
public:
  void               start();
  [[nodiscard]] auto get_current_time() const -> std::pair<double, double>;
  auto               end() -> std::pair<double, double>;

private:
  bool             initialized = false;
  Time::time_point wall_time_start;
  timespec         cpu_time_start{};
};


class Node
{
public:
  double g, h, f;
  double hcol;

  Node(double g_, double h_);
  virtual ~Node() = default;  // make Node an abstract class

  // compare
  virtual auto operator>(const Node& other) -> bool;
  virtual auto operator<(const Node& other) -> bool;
  virtual auto operator==(const Node& other) -> bool;
};

class Point2d
{
public:
  int x, y;
  Point2d(int x_, int y_);
  ~Point2d() = default;

  /**
   * @brief Add two points.
   *
   * @param other The other point to add.
   *
   * @return A new point that is the sum of the two points.
   */
  auto operator+(const Point2d& other) const -> Point2d;

  /**
   * @brief Subtract two points.
   *
   * @param other The other point to subtract.
   *
   * @return A new point that is the difference of the two points.
   */
  auto operator-(const Point2d& other) const -> Point2d;


  /**
   * @brief Check if two points are equal.
   *
   * @param other The other point to compare.
   *
   * @return True if the two points are equal, false otherwise.
   */
  auto operator==(const Point2d& other) const -> bool;


  /**
   * @brief Add a point to this point.
   *
   * @param other The other point to add.
   */
  void add(const Point2d& other);

  /**
   * @brief Operator for printing the point.
   *
   * @param os The output stream.
   * @param p The point to print.
   *
   * @return The output stream.
   */
  friend auto operator<<(std::ostream& os, Point2d const& p) -> std::ostream&;

  /**
   * @brief Hash function for the Point2d class.
   */
  struct hash_point
  {
    auto operator()(const Point2d& p) const -> size_t;
  };


  explicit operator std::string() const;
};

auto find_direction(const Point2d& start, const Point2d& end) -> enum Direction;
auto find_direction(int start, int end) -> enum Direction;

class TimeInterval
{
public:
  int t_min, t_max;
  bool col = false;
  bool edge_col = false;  // SIPPS: arriving in this interval swaps places with another agent

  TimeInterval(int t_min_, int t_max_);

  TimeInterval(int t_min_, int t_max_, bool col_);

  auto        operator==(const TimeInterval& other) const -> bool;
  auto        operator+(int x) const -> TimeInterval;
  explicit    operator std::string() const;
  friend auto operator<<(std::ostream& os, TimeInterval const& i) -> std::ostream&;
};

auto overlap(const TimeInterval& i1, const TimeInterval& i2) -> bool;

class TimePoint
{
public:
  int          location;
  TimeInterval interval;

  TimePoint(int location, const TimeInterval& interval);

  auto operator==(const TimePoint& other) const -> bool;
};

// define a path as a vectors of (int, int) tuples
using Path          = std::vector<int>;
using PointPath     = std::vector<Point2d>;
using TimePointPath = std::vector<TimePoint>;

auto timepointpath_to_path(const TimePointPath& tp_path) -> Path;
auto path_to_timepointpath(const Path& path) -> TimePointPath;
auto check_timepointpath_interval_no_overlap(const TimePointPath& tp_path) -> bool;
void fix_timepointpath_interval_overlap(TimePointPath& tp_path);
auto is_valid_timepointpath(const TimePointPath& tp_path) -> bool;


/**
 * @brief Get the path to the project directory, assuming the project is run from the build directory. This function is useful for loading
 * files from the project directory.
 *
 * @return The path to the project directory.
 */
auto get_base_path() -> std::string;

inline auto safe_increase(int num) -> int
{

  if (num == INT_MAX)
  {
    return INT_MAX;
  }
  return num + 1;
}


auto any_to_str(const std::any& value) -> std::string;
auto double_to_str(double num, char separating_char = ',', int precision = 2) -> std::string;
auto find_last_number(const std::string& str) -> int;
auto location_at_time(TimePointPath& path, int time) -> int;
auto open_file_dialog(const std::string& type, const std::string& file_extensions) -> std::string;
