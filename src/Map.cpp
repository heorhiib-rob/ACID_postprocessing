/*
 * Author: Jan Chleboun
 * Date: 04-04-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#include "Map.h"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>

auto Map::find_neighbors(const int loc) const -> std::vector<int>
{
  assertm(loaded, "Map not loaded.");
  assertm(is_in(loc) && index(loc) == 0, "Invalid position.");
  const std::array<int, 4> neighbors_rel = {-width, -1, 1, width};
  std::vector<int>         ret;
  for (const auto& neigh_rel : neighbors_rel)
  {
    // edges - sjip the neighboring indexes which  are on different row
    if (loc % width == 0 && neigh_rel == -1)
    {
      continue;  // left edge
    }
    if (loc % width == width - 1 && neigh_rel == 1)
    {
      continue;  // right edge
    }
    const int neigh = loc + neigh_rel;
    if (is_in(neigh) && index(neigh) == 0)
    {
      ret.push_back(neigh);
    }
  }
  return ret;
}

auto Map::find_neighbors(const Point2d& pos) const -> std::vector<Point2d>
{
  assertm(loaded, "Map not loaded.");
  assertm(is_in(pos) && index(pos) == 0, "Invalid position.");
  std::vector<Point2d>         ret;
  const std::array<Point2d, 4> neighbors = {Point2d(0, 1), Point2d(1, 0), Point2d(0, -1), Point2d(-1, 0)};
  for (const auto& it : neighbors)
  {
    const Point2d neigh = pos + it;
    if (!is_in(neigh))
    {
      continue;
    }
    if (index(neigh) != 0)
    {
      continue;
    }
    ret.push_back(neigh);
  }
  return ret;
}

void Map::print() const
{
  if (!loaded)
  {
    std::cout << "Map not loaded." << std::endl;
  }
  else
  {
    std::cout << "Map size: " << width << "x" << height << std::endl;
    assertm(width * height > 0, "Invalid map size.");
    for (int i = 0; i < (int)data.size(); i++)
    {
      std::cout << (int)data[i];
      if (i % width == height - 1)
      {
        std::cout << std::endl;
      }
      else
      {
        std::cout << " ";
      }
    }
  }
}

void Map::load(const std::string& map_fname)
{
  // reset the map data
  int num_of_cols = 0;
  int num_of_rows = 0;

  // open the map file
  std::ifstream map_file(map_fname.c_str());

  // check if the open was successful
  if (!map_file.is_open())
  {
    throw std::runtime_error("Cannot open file '" + map_fname + "'");
  }

  // read the file line by line
  std::string line;
  while (getline(map_file, line))
  {
    // split the line
    std::vector<std::string> split_line;
    boost::split(split_line, line, boost::is_any_of(" "));

    // filter out invalid lines
    if (split_line.empty())
    {
      continue;
    }

    if (boost::iequals(split_line[0], "type"))
    {
      type = split_line[1];  // read type
    }
    else if (boost::iequals(split_line[0], "height"))  // read height
    {
      try
      {
        num_of_rows = stoi(split_line[1]);
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("Unable to read the number of rows.");
      }
    }
    else if (boost::iequals(split_line[0], "width"))  // read width
    {
      try
      {
        num_of_cols = stoi(split_line[1]);
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("Unable to read the number of rows.");
      }
    }
    else if (boost::iequals(split_line[0], "map"))  // read the map
    {
      while (getline(map_file, line))
      {
        for (char i : line)
        {
          // treat . as free cell
          if (i == '.')
          {
            // remember index to the free location vector
            location_to_free_location_vec.push_back(static_cast<int>(free_location_to_location_vec.size()));
            // remember index to the location
            free_location_to_location_vec.push_back(static_cast<int>(data.size()));
            data.push_back(0);
          }
          // treat @, T as obstacle
          else if (i == '@' || i == 'T')
          {
            // there is no index to the free location index
            location_to_free_location_vec.push_back(-1);
            data.push_back(1);
          }
          // report all other symbols as unknown
          else
          {
            throw std::runtime_error("Unknown symbol in map description.");
          }
        }
        // check whether the number of cells in this row is consistent with num_of_columns
        if (static_cast<int>(data.size()) % num_of_cols != 0)
        {
          throw std::runtime_error("Inconsistent number of symbols in a row in the map description.");
        }
      }
      // check whether the number of lines is consistent with num_of_rows
      if (static_cast<int>(data.size()) != num_of_cols * num_of_rows)
      {
        throw std::runtime_error("Inconsistent number of rows in the map description.");
      }
    }
  }

  if (num_of_cols <= 0)
  {
    throw std::runtime_error("Invalid number of columns.");
  }
  if (num_of_rows <= 0)
  {
    throw std::runtime_error("Invalid number of rows.");
  }

  assertm(location_to_free_location_vec.size() == data.size(), "Each location must have an index to the free location vector.");

  // close the map file
  map_file.close();

  // assign the size of the map
  height = num_of_rows;
  width  = num_of_cols;
  loaded = true;
}
