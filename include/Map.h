/*
 * Author: Jan Chleboun
 * Date: 04-04-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#pragma once
#include "utils.h"

class Map
{
public:
  int                  width  = 0;
  int                  height = 0;
  bool                 loaded = false;
  std::string          type;
  std::vector<int>     free_location_to_location_vec;
  std::vector<int>     location_to_free_location_vec;
  std::vector<uint8_t> data;

  Map() = default;
  void               print() const;
  [[nodiscard]] auto find_neighbors(const Point2d& p) const -> std::vector<Point2d>;
  [[nodiscard]] auto find_neighbors(int loc) const -> std::vector<int>;
  void               load(const std::string& map_fname);

  [[nodiscard]] inline auto is_in(const Point2d& p) const -> bool
  {
    assertm(loaded, "Map not loaded.");
    return !(p.x < 0 || p.x >= width || p.y < 0 || p.y >= height);
  }

  [[nodiscard]] inline auto is_in(int loc) const -> bool
  {
    assertm(loaded, "Map not loaded.");
    return loc >= 0 && loc < width * height;
  }

  [[nodiscard]] inline auto index(const Point2d& point) const -> uint8_t
  {
    assertm(loaded, "Map not loaded.");
    assertm(is_in(point), "Trying to index a point that is not in the map.");
    return data[point.y * width + point.x];
  }

  [[nodiscard]] inline auto index(int loc) const -> uint8_t
  {
    assertm(loaded, "Map not loaded.");
    assertm(is_in(loc), "Trying to index a point that is not in the map.");
    return data[loc];
  }

  [[nodiscard]] inline auto index_free(int free_loc) const -> uint8_t
  {
    return index(free_location_to_location(free_loc));
  }

  [[nodiscard]] inline auto free_location_to_location(int free_loc) const -> int
  {
    assertm(loaded, "Map not loaded.");
    assertm(0 <= free_loc && free_loc < static_cast<int>(free_location_to_location_vec.size()), "Invalid free location index.");
    return free_location_to_location_vec[free_loc];
  }

  [[nodiscard]] inline auto location_to_free_location(int loc) const -> int
  {
    assertm(loaded, "Map not loaded.");
    assertm(index(loc) == 0, "Not a free location");
    return location_to_free_location_vec[loc];
  }

  [[nodiscard]] inline auto position_to_index(const Point2d& p) const -> int
  {
    assertm(loaded, "Map not loaded.");
    return p.y * width + p.x;
  }

  [[nodiscard]] inline auto get_num_free_cells() const -> int
  {
    assertm(loaded, "Map not loaded.");
    return static_cast<int>(free_location_to_location_vec.size());
  }

  [[nodiscard]] inline auto get_num_cells() const -> int
  {
    assertm(loaded, "Map not loaded.");
    assertm(width * height == static_cast<int>(data.size()), "Width and height inconsistent with the map data.");
    return static_cast<int>(data.size());
  }
};
