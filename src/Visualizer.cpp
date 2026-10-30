/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "Visualizer.h"

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/WindowStyle.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "IconsFontAwesome5.h"
#include "imgui-SFML.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "nfd.hpp"
#include "utils.h"

// default size of the panels (0 - 1)
#define LEFT_PANEL_SIZE_DEFAULT 0.2
#define RIGHT_PANEL_SIZE_DEFAULT 0.2
#define BOTTOM_PANEL_SIZE_DEFAULT 0.2

#define CELL_SIZE 20
#define BORDER_SIZE 1
#define GOAL_VIS_THICKNESS 3
#define LINE_WIDTH 3  // denominator of line width, so line width is 1/LINE_WIDTH * CELL_SIZE
#define MENU_BAR_SIZE 40
#define ICON_SIZE 20.0f
#define FRAME_RATE 60
#define DEFAULT_ANTIALIASING_LEVEL 8.0F
// random color generation
#define MIN_BRIGHTNESS 20
#define MAX_BRIGHTNESS 230
#define COLOR_GREY sf::Color(150, 150, 150, 255)

Visualizer::Visualizer(Instance& instance_, Computation& computation_thread_, SharedData& shared_data_, int seed_)
    : instance(instance_),
      computation_thread(computation_thread_),
      shared_data(shared_data_),
      solution_vis([this](Visualization& vis) { this->solution_visualization_update(vis); }),
      sipp_vis([this](Visualization& vis) { this->sipp_visualization_update(vis); }),
      lns_vis([this](Visualization& vis) { this->lns_visualization_update(vis); }),
      seed(seed_)
{
  clock.start();
  // initialized the random generator
  if (seed - 1 < 0)  // visualizer gets a seed with +1
  {
    // random device to generate a random seed
    std::random_device rd;
    rnd_generator = std::mt19937(rd());
  }
  else
  {
    rnd_generator = std::mt19937(seed);
  }


  // generate random colors for the agents
  for (int i = 0; i < (int)instance.get_start_locations().size(); i++)
  {
    agents_colors.push_back(generate_random_color(UCHAR_MAX, UCHAR_MAX, UCHAR_MAX, 0.25));
  }

  // create the map texture
  mapToTexture(instance.get_map_data());

  // generate the circles that represent agents and their goals
  update_agent_circles(instance.get_start_positions());
  update_goal_circles(instance.get_goal_positions());

  // generate the rectangles that represent the map
  create_cell_rectangles();

  // initialize visualization for the astar paths
  // for (int i = 0; i < (int)instance.get_optimal_paths().size(); i++)
  //{
  //  update_path(i, instance.get_optimal_paths()[i]);
  //}
  std::cout << "Visualizer initialized. " << std::endl;
}

Visualizer::~Visualizer()
{
  stop();
  join_thread();
}

void Visualizer::start()
{
  // start the underlying thread
  running    = true;
  vis_thread = std::thread(&Visualizer::run, this);
}

void Visualizer::stop()
{
  // set the stop signal
  running = false;
}

void Visualizer::join_thread()
{
  if (vis_thread.joinable())
  {
    vis_thread.join();
  }
}

void Visualizer::run()
{
  // calculate useful constants
  const int screen_width  = (int)sf::VideoMode::getDesktopMode().width;
  const int screen_height = (int)sf::VideoMode::getDesktopMode().height;

  // get the map size
  const int map_width  = instance.get_map_data().width;
  const int map_height = instance.get_map_data().height;

  // turn on antialiasing
  sf::ContextSettings c_settings;
  c_settings.antialiasingLevel = DEFAULT_ANTIALIASING_LEVEL;

  // create the sfml window
  sf::RenderWindow window(sf::VideoMode(screen_width, screen_height), "MAPF solver", sf::Style::Default, c_settings);

  // set framerate
  window.setFramerateLimit(FRAME_RATE);

  // check whether the window was created
  if (!ImGui::SFML::Init(window))
  {
    throw std::runtime_error("Unable to initialize ImGUI.");
  }

  // Activate the window for OpenGL rendering
  window.setActive();

  // create a view with the same size as the map and center in the middle of the map
  sf::View view(sf::Vector2f((float)map_width * ((float)CELL_SIZE / 2), (float)map_height * ((float)CELL_SIZE / 2)),
                sf::Vector2f((float)map_width * CELL_SIZE, (float)map_height * CELL_SIZE));

  // adjust the sml part so that it fits between the panels but the aspect ratio is preserved
  adjust_sfml_part_viewport(window, view, LEFT_PANEL_SIZE_DEFAULT, RIGHT_PANEL_SIZE_DEFAULT, BOTTOM_PANEL_SIZE_DEFAULT);

  // load the fonts
  load_fonts();

  // enable docking
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  sf::Clock deltaClock;

  // the main loop
  while (window.isOpen() && running)
  {
    // check whether new paths arrived
    if (solution_vis.is_playing)
    {
      solution_vis.next_iteration();
    }
    else if (sipp_vis.is_playing)
    {
      sipp_vis.next_iteration();
    }
    else if (lns_vis.is_playing)
    {
      lns_vis.next_iteration();
    }
    else if (shared_data.is_new_info.load(std::memory_order_acquire))
    {
      std::vector<LNSIterationInfo> new_lns_info = shared_data.consume_lns_info();
      assertm(!new_lns_info.empty(), "Received empty LNS info");
      // write to the console
      if (static_cast<int>(lns_info.size()) == 0)
      {
        // initial solution
        add_to_log("Found initial solution with cost: " + std::to_string(new_lns_info[0].sol.sum_of_costs));
      }
      // add all improvements to the log
      for (const auto& it : new_lns_info)
      {
        // skip the first one, because it is the initial solution
        if (it.iteration_num == 0)
        {
          continue;
        }
        // add the improvement to the log
        if (it.accepted)
        {
          add_to_log("Found better solution with cost: " + std::to_string(it.sol.sum_of_costs));
        }
      }
      lns_info.insert(lns_info.end(), std::make_move_iterator(new_lns_info.begin()), std::make_move_iterator(new_lns_info.end()));
      lns_vis.max_time = static_cast<int>(lns_info.size()) - 1;
      // std::cout << "New max time: " << lns_vis.max_time << std::endl;
      lns_vis.update();
    }

    // process the events
    process_events(window, view);

    // clear the window
    window.clear(sf::Color::Black);

    // update imgui
    ImGui::SFML::Update(window, deltaClock.restart());

    // create docking space so that the GUI elements can be docked
    create_docking_space();

    // show the window for debugging imgui
#ifndef NDEBUG
    // ImGui::ShowDemoWindow();
#endif

    // create the GUI elements
    create_left_panel();
    create_right_panel();
    create_bottom_panel();

    // create the menu bar
    create_menu_bar();

    // show dialog if applicable
    number_of_agents_dialog();

    // draw the map
    window.draw(map_sprite);

    // draw the cell rectangles
    draw_cell_rectangles(window);

    // draw the path visualizations
    draw_paths(window);

    // draw the agents
    draw_goals(window);
    draw_agents(window);

    // set the view
    window.setView(view);

    // Render ImGui on top
    ImGui::SFML::Render(window);

    // display
    window.display();
  }

  // close the window
  if (window.isOpen())
  {
    window.close();
  }
  ImGui::SFML::Shutdown();
  std::cout << "Visualizer thread ended" << std::endl;

  // signal to the other thread to end
  shared_data.is_end.store(true, std::memory_order_release);
}

// Render the map onto a texture
void Visualizer::mapToTexture(const Map& map_data)
{
  // initialize the texture
  sf::RenderTexture map_render_texture;
  map_render_texture.create(map_data.width * CELL_SIZE, map_data.height * CELL_SIZE);

  // clear the texture with a background color
  map_render_texture.clear(sf::Color::White);

  // Draw the grid onto the texture
  for (int y = 0; y < map_data.height; y++)
  {
    for (int x = 0; x < map_data.width; x++)
    {
      // create the cell
      sf::RectangleShape cell(sf::Vector2f(CELL_SIZE, CELL_SIZE));
      cell.setPosition((float)x * CELL_SIZE, (float)y * CELL_SIZE);
      if ((int)map_data.data[y * map_data.width + x] == 1)
      {
        // obstacles
        cell.setFillColor(sf::Color::Black);
      }
      else
      {
        // free cells
        cell.setFillColor(sf::Color::White);
      }

      // grid color and thickness
      cell.setOutlineThickness(-BORDER_SIZE);
      cell.setOutlineColor(sf::Color::Black);

      map_render_texture.draw(cell);
    }
  }

  // Finalize the rendering
  map_render_texture.display();

  // set the map texture
  map_texture = map_render_texture.getTexture();

  // turn on antialiasing (makes the lines look like they have low resolution)
  // map_texture.setSmooth(true);

  // create the map sprite
  map_sprite = sf::Sprite(map_texture);
}
void Visualizer::update_agent_circles(const std::vector<Point2d>& positions)
{
  assertm((int)positions.size() == instance.get_num_of_agents(), "Wrong number of agent positions.");

  // initialize the vector
  if (agents_circles.size() != positions.size())
  {
    agents_circles.clear();
    agents_circles.resize(positions.size(), sf::CircleShape(0));
  }

  // assign the correct circles
  for (int i = 0; i < (int)positions.size(); i++)
  {
    const int x = positions[i].x;
    const int y = positions[i].y;
    // create shape
    agents_circles[i] = sf::CircleShape((double)(CELL_SIZE - 2 * BORDER_SIZE) / 2);
    // set color
    agents_circles[i].setFillColor(agents_colors[i]);
    agents_circles[i].setOutlineThickness(0);
    // set position
    agents_circles[i].setPosition((float)x * CELL_SIZE + BORDER_SIZE, (float)y * CELL_SIZE + BORDER_SIZE);
  }
}

void Visualizer::draw_agents(sf::RenderWindow& window)
{
  assertm(agents_circles.size() > 0, "Trying to draw empty agent vector.");
  for (const auto& it : agents_circles)
  {
    // draw the shape
    window.draw(it);
  }
}

void Visualizer::update_goal_circles(const std::vector<Point2d>& positions)
{
  assertm((int)positions.size() == instance.get_num_of_agents(), "Wrong number of agent positions.");

  // initialize the vector
  if (goal_circles.size() != positions.size())
  {
    goal_circles.clear();
    goal_circles.resize(positions.size(), sf::CircleShape(0));
  }

  // assign the correct circles
  for (int i = 0; i < (int)positions.size(); i++)
  {
    const int x = positions[i].x;
    const int y = positions[i].y;

    // create shape
    goal_circles[i] = sf::CircleShape((double)(CELL_SIZE - 2 * BORDER_SIZE) / 2);

    // set color
    goal_circles[i].setOutlineColor(agents_colors[i]);
    goal_circles[i].setOutlineThickness(-GOAL_VIS_THICKNESS);
    goal_circles[i].setFillColor(sf::Color::White);  // make the inner part white
                                                     //
    // set position
    goal_circles[i].setPosition((float)x * CELL_SIZE + BORDER_SIZE, (float)y * CELL_SIZE + BORDER_SIZE);
  }
}

void Visualizer::draw_goals(sf::RenderWindow& window)
{
  assertm(goal_circles.size() > 0, "Trying to draw empty goal vector.");
  for (const auto& it : goal_circles)
  {
    // draw the shape
    window.draw(it);
  }
}

void Visualizer::update_cell_rectangles_highlights()
{
  //  set all cells to inactive
  for (auto& row : cell_rectangles_highlights)
  {
    std::fill(row.begin(), row.end(), Highlight::NONE);
  }

  // highlight the destroyed paths from previous iterations
  if (highlight_destroyed)
  {
    for (auto it : lns_info[lns_vis.time].sol.destroyed_paths)
    {
      for (auto pos : lns_info[std::max(0, lns_vis.time - 1)].sol.converted_paths[it])
      {
        const Point2d cur_pos                            = instance.location_to_position(pos);
        cell_rectangles_highlights[cur_pos.y][cur_pos.x] = Highlight::LNS_DESTROYED;
      }
    }
  }

  // find all active rectangles
  if (selected_agent != -1)
  {
    for (int i = 0; i < std::min(sipp_vis.time, (int)(*sipp_info).size()); i++)
    {
      const Point2d cur_pos                            = instance.location_to_position((*sipp_info)[i].cur_expanded.location);
      cell_rectangles_highlights[cur_pos.y][cur_pos.x] = Highlight::SIPP_EXPANDED;
      // draw only until time
      if (i == sipp_vis.time)
      {
        break;
      }
    }
  }
}

void Visualizer::create_cell_rectangles()
{
  // get the map info
  const Map& map_data = instance.get_map_data();
  const int  width    = map_data.width;
  const int  height   = map_data.height;

  // clear the previous rectangles
  cell_rectangles.clear();
  cell_rectangles_highlights.clear();

  // resize the vectors
  cell_rectangles.resize(height);
  cell_rectangles_highlights.resize(height);

  // create a rectangle shape to initialize the rectangles
  sf::RectangleShape rec(sf::Vector2f(CELL_SIZE, CELL_SIZE));
  rec.setOutlineThickness(-BORDER_SIZE);
  rec.setOutlineColor(sf::Color::Black);

  // add the rectangles
  for (int y = 0; y < height; y++)
  {
    // initialize the row of rectangles
    cell_rectangles_highlights[y].resize(width, Highlight::NONE);
    cell_rectangles[y].resize(width, rec);

    for (int x = 0; x < width; x++)
    {
      cell_rectangles[y][x].setPosition((float)x * CELL_SIZE, (float)y * CELL_SIZE);
      if (map_data.index(Point2d(x, y)) == 1)
      {
        // obstacles red, because they should not be ever drawn
        cell_rectangles[y][x].setFillColor(sf::Color::Red);
      }
      else
      {
        // free cells grey
        cell_rectangles[y][x].setFillColor(COLOR_GREY);
      }
    }
  }
}

void Visualizer::draw_cell_rectangles(sf::RenderWindow& window)
{
  assertm(static_cast<int>(cell_rectangles.size()) > 0, "No cell rectangles.");
  assertm(cell_rectangles.size() == cell_rectangles_highlights.size(), "Size of the rectangles vector and counts vector must be the same.");
  for (int i = 0; i < static_cast<int>(cell_rectangles.size()); i++)
  {
    assertm((int)cell_rectangles[i].size() > 0, "No cell rectangles.");
    for (int j = 0; j < (int)cell_rectangles[i].size(); j++)
    {
      if (cell_rectangles_highlights[i][j] == Highlight::NONE)
      {
        continue;
      }
      if (cell_rectangles_highlights[i][j] == Highlight::LNS_DESTROYED)
      {
        cell_rectangles[i][j].setFillColor(sf::Color::Red);
      }
      else if (cell_rectangles_highlights[i][j] == Highlight::SIPP_EXPANDED)
      {
        cell_rectangles[i][j].setFillColor(COLOR_GREY);
      }
      window.draw(cell_rectangles[i][j]);
    }
  }

  // draw the currently expanded
  if (selected_agent >= 0 && sipp_vis.time < (int)(*sipp_info).size())
  {
    const Point2d& cur_pos  = instance.location_to_position((*sipp_info)[sipp_vis.time].cur_expanded.location);
    curr_expanded_rectangle = cell_rectangles[cur_pos.y][cur_pos.x];
    curr_expanded_rectangle.setFillColor(sf::Color::Yellow);
    window.draw(curr_expanded_rectangle);
  }
}

void Visualizer::update_path_vis(const int agent_num, const Path& path)
{
  assertm(agent_num >= 0 && agent_num < (int)agents_colors.size() && agent_num < instance.get_num_of_agents(), "Invalid agent number.");

  // check whether the vector is big enough
  if (agent_num >= (int)agents_paths_vis.size())
  {
    agents_paths_vis.resize(agent_num + 1);
  }

  // find the color for this agent
  const sf::Color color = agents_colors[agent_num];

  // create the path visualization
  sf::VertexArray path_vis(sf::Quads);
  if (path.size() >= 2)
  {
    auto           segment_start = path.cbegin();
    auto           segment_end   = path.cbegin() + 1;
    enum Direction dir           = Direction::NONE;

    // merge contiguous segments
    for (int i = 2; i < (int)path.size() + 1; i++)
    {
      if (segment_start == segment_end || *segment_start == *segment_end)  // segment of length 0
      {
        segment_end++;
        continue;
      }
      // skip repeating points
      if (i < (int)path.size() && path[i] == *segment_end)
      {
        segment_end++;
        continue;
      }

      if (dir == Direction::NONE)
      {
        dir = find_direction(*segment_start, *segment_end);
      }

      assertm(dir != Direction::NONE, "Direction must be set.");

      // check whether segment continuous (trick with checking if i == path.size() is used to prevent out of bounds)
      if (i != (int)path.size() && dir == find_direction(*segment_end, path[i]))
      {
        // extend the end of the segment
        segment_end++;
        continue;
      }

      // find the rectangle that represents the segment
      Point2d segment_start_pt = instance.location_to_position(*segment_start);
      Point2d segment_end_pt   = instance.location_to_position(*segment_end);

      int min_x, min_y, max_x, max_y;
      min_x = std::min(segment_start_pt.x, segment_end_pt.x);
      min_y = std::min(segment_start_pt.y, segment_end_pt.y);
      max_x = std::max(segment_start_pt.x, segment_end_pt.x);
      max_y = std::max(segment_start_pt.y, segment_end_pt.y);
      assertm((max_x - min_x > 0) != (max_y - min_y > 0), "Segment must be either horizontal or vertical.");

      // calculate the corners of the rectangle
      sf::Vector2f left_upper =
          sf::Vector2f(CELL_SIZE * (min_x + 0.5 - (double)1 / (2 * LINE_WIDTH)), CELL_SIZE * (min_y + 0.5 - (double)1 / (2 * LINE_WIDTH)));
      sf::Vector2f right_upper =
          sf::Vector2f(CELL_SIZE * (max_x + 0.5 + (double)1 / (2 * LINE_WIDTH)), CELL_SIZE * (min_y + 0.5 - (double)1 / (2 * LINE_WIDTH)));
      sf::Vector2f right_lower =
          sf::Vector2f(CELL_SIZE * (max_x + 0.5 + (double)1 / (2 * LINE_WIDTH)), CELL_SIZE * (max_y + 0.5 + (double)1 / (2 * LINE_WIDTH)));
      sf::Vector2f left_lower =
          sf::Vector2f(CELL_SIZE * (min_x + 0.5 - (double)1 / (2 * LINE_WIDTH)), CELL_SIZE * (max_y + 0.5 + (double)1 / (2 * LINE_WIDTH)));

      // add the segment into the path visualization
      path_vis.append(sf::Vertex(left_upper, color));
      path_vis.append(sf::Vertex(right_upper, color));
      path_vis.append(sf::Vertex(right_lower, color));
      path_vis.append(sf::Vertex(left_lower, color));

      // start a new segment
      segment_start = segment_end;
      segment_end++;
      dir = find_direction(*segment_start, *segment_end);
    }
  }
  assertm(agent_num < (int)agents_paths_vis.size(), "Agent_num must be bigger than the size of the paths visualization vector.");
  agents_paths_vis[agent_num] = path_vis;
}

void Visualizer::update_all_path_vis(int time)
{
  for (int i = 0; i < (int)(*agents_paths).size(); i++)
  {
    // calculate the start index of the path according to the visualization time
    if (!(*agents_paths)[i].empty())
    {
      int idx = std::min((int)(*agents_paths)[i].size() - 1, time);
      update_path_vis(i, Path((*agents_paths)[i].begin() + idx, (*agents_paths)[i].end()));
    }
  }
}

void Visualizer::draw_paths(sf::RenderWindow& window)
{
  for (auto& it : agents_paths_vis)
  {
    if (it.getVertexCount() == 0)
    {
      continue;
    }
    window.draw(it);
  }
}

void Visualizer::update_all_paths(int time)
{
  // std::cout << "Updating all paths." << std::endl;
  agents_paths = &lns_info[time].sol.converted_paths;
  assertm(instance.get_num_of_agents() == (int)(*agents_paths).size(), "Number of paths should be the same as the number of agents.");

  int max_path_len = 0;
  // construct the path visualizations and find the new length of the visualization
  for (int i = 0; i < instance.get_num_of_agents(); i++)
  {
    int len = (int)(*agents_paths)[i].size() - 1;
    if (len > 0)
    {
      update_path_vis(i, (*agents_paths)[i]);
      max_path_len = std::max(max_path_len, len);
    }
  }

  // update the visualization
  solution_vis.new_data(max_path_len);
}


void Visualizer::create_docking_space(void)
{
  // get the viewport for the whole window
  ImGuiViewport* viewport = ImGui::GetMainViewport();

  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::SetNextWindowViewport(viewport->ID);

  // set the flags for the gui overlay
  ImGuiWindowFlags host_window_flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground;

  // set the style
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));

  // create the dockspace
  ImGui::Begin("DockSpace Window", nullptr, host_window_flags);
  ImGui::PopStyleVar(3);

  // Get menu bar height
  float menuBarHeight = MENU_BAR_SIZE;
  // float menuBarHeight = ImGui::GetFrameHeight(); // does not work for some reason

  // Offset dockspace position to account for menu bar
  ImGui::SetCursorPosY(menuBarHeight);

  // Create the dockspace
  ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
  ImGui::DockSpace(dockspace_id, ImVec2(0.0F, 0.0F), ImGuiDockNodeFlags_PassthruCentralNode);
  static bool initialized = false;
  if (!initialized)  // initialize the dockspace only once
  {
    initialized = true;

    // Clear old dock layout
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);

    // Adjust dock builder size based on menu bar height
    ImVec2 dockspaceSize = ImVec2(viewport->Size.x, viewport->Size.y - menuBarHeight);
    ImGui::DockBuilderSetNodeSize(dockspace_id, dockspaceSize);

    // Create dock spaces
    ImGuiID dock_main_id = dockspace_id;  // Main area

    // split bottom
    ImGuiID dock_bottom = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Down, BOTTOM_PANEL_SIZE_DEFAULT, nullptr, &dock_main_id);

    // split sides
    ImGuiID dock_left = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Left, LEFT_PANEL_SIZE_DEFAULT, nullptr, &dock_main_id);

    // calculate the right panel size taking into account the left panel
    const double right_panel_size = (float)RIGHT_PANEL_SIZE_DEFAULT / (1.0 - LEFT_PANEL_SIZE_DEFAULT);
    ImGuiID      dock_right = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Right, (float)right_panel_size, nullptr, &dock_main_id);

    // Create the dock windows
    ImGui::DockBuilderDockWindow("Left Panel", dock_left);
    ImGui::DockBuilderDockWindow("Right Panel", dock_right);
    ImGui::DockBuilderDockWindow(ICON_FA_CODE " Console", dock_bottom);

    // Finalize
    ImGui::DockBuilderFinish(dockspace_id);
  }
  ImGui::End();
}

void Visualizer::load_fonts()
{
  // Clear old fonts
  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->Clear();

  // load the default font
  io.Fonts->AddFontFromFileTTF((get_base_path() + "/fonts/Cousine-Regular.ttf").c_str(), 20.0F);
  // io.Fonts->AddFontFromFileTTF((get_base_path() + "/fonts/DroidSans.ttf").c_str(), 20.0f);

  // check, whether the load was successful
  if (!ImGui::SFML::UpdateFontTexture())
  {
    std::cout << "WARNING: Could not load the default font. " << std::endl;
    io.Fonts->AddFontDefault();
  }

  // Load FontAwesome font for icons
  static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};  // Unicode range for icons

  ImFontConfig config;
  config.MergeMode        = true;       // Merge with the default font
  config.PixelSnapH       = true;       // Align to pixels for sharp rendering
  config.GlyphMinAdvanceX = ICON_SIZE;  // make the icon monospaced

  // if (!io.Fonts->AddFontFromFileTTF((get_base_path() + "/fonts/forkawesome-webfont.ttf").c_str(), 13.0f, &config, icons_ranges))
  if (io.Fonts->AddFontFromFileTTF((get_base_path() + "/fonts/fa-solid-900.ttf").c_str(), ICON_SIZE, &config, icons_ranges) == nullptr)
  {
    std::cout << "WARNING: Unable to load icons." << std::endl;
  };

  if (!ImGui::SFML::UpdateFontTexture())
  {
    std::cout << "WARNING: Could not update the icon font. " << std::endl;
  }
}

void Visualizer::adjust_sfml_part_viewport(sf::RenderWindow& window, sf::View& view, const float left_panel_size,
                                           const float right_panel_size, const float bottom_panel_size)
{
  // calculate the map parameters
  const int    map_width        = instance.get_map_data().width;
  const int    map_height       = instance.get_map_data().height;
  const double map_aspect_ratio = (double)map_width / (double)map_height;

  // get the window parameters
  const int window_width  = window.getSize().x;
  const int window_height = window.getSize().y;

  // calculate the size of the part of the window where the sfml stuff will be rendered
  const double sfml_part_width        = window_width * (1.0 - left_panel_size - right_panel_size);
  const double sfml_part_height       = window_height * (1.0 - bottom_panel_size);
  const double sfml_part_aspect_ratio = sfml_part_width / sfml_part_height;
  double       viewport_width, viewport_height;

  // the viewport must have the same aspoect ratio as the map
  if (map_aspect_ratio < sfml_part_aspect_ratio)
  {
    // map is wider
    viewport_width  = sfml_part_width;
    viewport_height = viewport_width / map_aspect_ratio;
  }
  else
  {
    // map is taller
    viewport_height = sfml_part_height;
    viewport_width  = viewport_height * map_aspect_ratio;
  }

  // calculate the viewport size relative to the window size
  const auto  viewport_width_rel  = (float)(viewport_width / sfml_part_width);
  const auto  viewport_height_rel = (float)(viewport_height / sfml_part_height);
  const float offset_x            = (1 - viewport_width_rel) / 2;
  const float offset_y            = (1 - viewport_height_rel) / 2;
  view.setViewport(sf::FloatRect({left_panel_size + offset_x, offset_y},
                                 {(float)(viewport_width / window_width), (float)(viewport_height / window_height)}));
}

void Visualizer::process_events(sf::RenderWindow& window, sf::View& view)
{
  // poll all events from the window
  sf::Event event{};
  while (window.pollEvent(event))
  {
    ImGui::SFML::ProcessEvent(window, event);  // Send event to ImGui

    // close event
    if (event.type == sf::Event::Closed)
    {
      running = false;
    }
    // zoom event
    else if (event.type == sf::Event::MouseWheelScrolled)
    {
      if (event.mouseWheelScroll.delta > 0)
      {
        view.zoom(1 / zoom_step);  // Zoom in
        zoom_level *= 1 / zoom_step;
      }
      else
      {
        view.zoom(zoom_step);  // Zoom out
        zoom_level *= zoom_step;
      }
    }
    // mouse click
    else if (event.type == sf::Event::MouseButtonPressed)
    {
      if (event.mouseButton.button == sf::Mouse::Left)
      {
        const int          window_width  = window.getSize().x;
        const int          window_height = window.getSize().y;
        const sf::Vector2i mouse_pos     = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);
        const sf::Vector2f mouse_pos_rel =
            sf::Vector2f((float)mouse_pos.x / (float)window_width, (float)mouse_pos.y / (float)window_height);
        // check whether clicked on the map
        if (view.getViewport().contains(mouse_pos_rel))
        {
          const sf::Vector2f translated_pos = window.mapPixelToCoords(mouse_pos);
          is_moving                         = true;
          last_click_pos                    = mouse_pos;
          for (int i = 0; i < (int)agents_circles.size(); i++)
          {
            // check whether clicked on agent
            if (agents_circles[i].getGlobalBounds().contains(translated_pos))
            {
              selected_agent = i;
              update_sipp_info();
              return;
            }
          }
        }
      }
    }
    // mouse is pressed
    else if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
    {
      if (is_moving)
      {
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
        const sf::Vector2i delta     = last_click_pos - mouse_pos;
        // get viewport size
        const float viewport_width  = view.getViewport().width;
        const float viewport_height = view.getViewport().height;
        const float view_width      = view.getSize().x;
        const float view_height     = view.getSize().y;
        const int   window_width    = window.getSize().x;
        const int   window_height   = window.getSize().y;
        view.move(view_width * delta.x / ((float)window_width * viewport_width),
                  view_height * delta.y / ((float)window_height * viewport_height));
        last_click_pos = mouse_pos;
      }
    }
    else if (event.type == sf::Event::MouseButtonReleased)
    {
      is_moving = false;
    }
    // Panning with arrow keys
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
    {
      view.move(-CELL_SIZE * zoom_level, 0);
    }
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
    {
      view.move(CELL_SIZE * zoom_level, 0);
    }
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
    {
      view.move(0, -CELL_SIZE * zoom_level);
    }
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
    {
      view.move(0, CELL_SIZE * zoom_level);
    }
  }
}

void Visualizer::create_left_panel()
{
  ImGui::Begin("Left Panel", nullptr, ImGuiWindowFlags_NoResize);
  // agent info
  if (selected_agent != -1)
  {
    ImGui::Text("Agent info:");
    ImGui::Text("\tSelected agent: %d", selected_agent);
    ImGui::Text("\tCurrent position: %s", ((std::string)get_agent_position(selected_agent, solution_vis.time)).c_str());
    ImGui::Text("\tStart: %s", ((std::string)instance.get_start_positions()[selected_agent]).c_str());
    ImGui::Text("\tGoal: %s", ((std::string)instance.get_goal_positions()[selected_agent]).c_str());
    // sipp visualization
    create_visualization_playback_bar("Sipp visualization:", sipp_vis, 20);

    // sipp node info
    if (sipp_vis.time > 0)
    {
      ImGui::Text("SIPP node info:");
      const SIPPIterationInfo& cur_info = (*sipp_info)[sipp_vis.time - 1];

      ImGui::Text("\tIteration num: %d", cur_info.iteration_num);
      ImGui::Text("\tExpanded position: %s", ((std::string)instance.location_to_position(cur_info.cur_expanded.location)).c_str());
      ImGui::Text("\tExpanded interval: %s", ((std::string)cur_info.cur_expanded.interval).c_str());
      ImGui::Text("\tg: %lf", cur_info.g);
      ImGui::Text("\th: %lf", cur_info.h);
      ImGui::Text("\th2: %lf", cur_info.h2);
      ImGui::Text("\th3: %lf", cur_info.h3);
      ImGui::Text("\tf: %lf", cur_info.g + cur_info.h);
      int whose_goal = instance.whose_goal(cur_info.cur_expanded.location);
      ImGui::Text("\tWhose goal: %s", std::to_string(whose_goal).c_str());
      ImGui::Text("\tNum generated: %d", cur_info.generated);
      ImGui::Text("\tNum expanded: %d", cur_info.expanded);

      // print all already planned agents
      std::stringstream ss;
      int               i = 0;
      for (const auto& it : lns_info[lns_vis.time].sol.priorities)
      {
        if (it == selected_agent)
        {
          break;
        }
        ss << it << " ";
        i++;
      }
      float panelWidth = ImGui::GetContentRegionAvail().x;

      // Enable text wrapping at the panel's width
      ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + panelWidth);
      ImGui::Text("\tAlready planned: %s", ss.str().c_str());
      ImGui::PopTextWrapPos();
    }
  }
  ImGui::End();
}

void Visualizer::create_right_panel()
{
  ImGui::Begin("Right Panel", nullptr, ImGuiWindowFlags_NoResize);
  // Solution visualization
  create_visualization_playback_bar("Solution visualization:", solution_vis, 0);

  // LNS visualization
  create_visualization_playback_bar("LNS visualization:", lns_vis, 10);

  ImGui::Text("LNS info:");
  const LNSIterationInfo& cur_info = lns_info[lns_vis.time];

  ImGui::Text("\tIteration num: %d", cur_info.iteration_num);
  ImGui::Text("\tAccepted: %d", cur_info.accepted);
  ImGui::Text("\tImprovement: %d", cur_info.improvement);
  ImGui::Text("\tDestroy strategy: %s", cur_info.destroy_strategy.c_str());
  // create a stringstrem from all destroyed agents
  std::stringstream ss;
  for (const auto& it : cur_info.sol.destroyed_paths)
  {
    ss << it << " ";
  }
  float panelWidth = ImGui::GetContentRegionAvail().x;

  // Enable text wrapping at the panel's width
  ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + panelWidth);
  ImGui::Text("\tDestroyed: %s", ss.str().c_str());
  ImGui::PopTextWrapPos();
  if (ImGui::Checkbox("Highlight destroyed:", &highlight_destroyed))
  {
    update_cell_rectangles_highlights();
  }


  ImGui::End();
}

void Visualizer::create_visualization_playback_bar(std::string vis_name, Visualization& vis, int id)
{
  // TODO rewrite vsualizations to use this method
  ImGui::Text(vis_name.c_str());
  // Set button background color to transparent
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0F, 1.0F, 1.0F, 0.0F));

  // calculate the width of the slider label
  const float time_text_width = ImGui::CalcTextSize("Time: ").x + ImGui::GetStyle().ItemSpacing.x;

  // Get the width of the remaining space in the panel after adding the slider label
  const float panel_size      = ImGui::GetContentRegionAvail().x;
  float       available_width = panel_size - time_text_width;

  // create the slider
  ImGui::Text("Time: ");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(available_width);
  if (ImGui::SliderInt((std::string("##") + std::to_string(id++)).c_str(), &vis.time, 0,
                       vis.max_time))  // ##time_slider means that the slider name is not displayed
  {
    // slider changed
    vis.update();
  }

  // calculate the width of button array
  constexpr int num_buttons = 5;
  float         total_width = num_buttons * ImGui::CalcTextSize(ICON_FA_PLAY).x + 2 * num_buttons * ImGui::GetStyle().FramePadding.x +
                      (num_buttons - 1) * ImGui::GetStyle().ItemSpacing.x;

  // center the playback buttons
  float start_x = time_text_width + (available_width - total_width) * 0.5F;
  if (start_x + total_width > panel_size)
  {
    start_x = panel_size - total_width;
  }
  ImGui::SetCursorPosX(start_x);

  if (ImGui::Button((std::string(ICON_FA_PLAY "##xx") + std::to_string(id++)).c_str()))
  {
    vis.start();
  }
  ImGui::SameLine();

  if (ImGui::Button((std::string(ICON_FA_STOP "##xx") + std::to_string(id++)).c_str()))
  {
    vis.reset();
  }
  ImGui::SameLine();

  if (ImGui::Button((std::string(ICON_FA_STEP_BACKWARD "##xx") + std::to_string(id++)).c_str()))
  {
    vis.step_backward();
  }
  ImGui::SameLine();

  if (ImGui::Button((std::string(ICON_FA_PAUSE "##xx") + std::to_string(id++)).c_str()))
  {
    vis.pause();
  }
  ImGui::SameLine();

  if (ImGui::Button((std::string(ICON_FA_STEP_FORWARD "##xx") + std::to_string(id++)).c_str()))
  {
    vis.step_forward();
  }

  ImGui::PopStyleColor(1);
}


void Visualizer::create_bottom_panel()
{
  ImGui::Begin(ICON_FA_CODE " Console");
  for (auto& i : log)
  {
    ImGui::Text("%s", i.c_str());
  }
  ImGui::End();
}

void Visualizer::create_menu_bar()
{
  // Set style
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 10));          // Increase padding (width, height)
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 10));         // Adjust window padding
  ImGui::GetStyle().WindowBorderSize        = 2.0f;                         // Add border
  ImGui::GetStyle().Colors[ImGuiCol_Border] = ImVec4(0.1F, 0.1F, 0.1F, 1);  // border

  // create the widget
  if (ImGui::BeginMainMenuBar())
  {
    if (ImGui::BeginMenu("Instance"))
    {
      if (ImGui::MenuItem("Load map"))
      {
        std::string map_fname = open_file_dialog("Map file", "map");
        if (!map_fname.empty())
        {
          settings.map_fname = map_fname;
          add_to_log("Loaded map: " + map_fname);
          std::string scene_fname = open_file_dialog("Scene file", "scen");
          if (!scene_fname.empty())
          {
            add_to_log("Loaded scene: " + scene_fname);
            settings.scene_fname  = scene_fname;
            show_agent_num_dialog = true;
          }
          else
          {
            add_to_log("Failed to load a scene file.");
          }
        }
        else
        {
          add_to_log("Failed to load a map file.");
        }
        /*
        if (!map_path.empty() && !scene_path.empty())
        {
          std::cout << "Loaded map: " << map_path << std::endl;
          // stop computation
          shared_data.is_end.store(true, std::memory_order_release);

          // wait until computation ended
          while (!computation_thread.running.load(std::memory_order_acquire))
          {
            // sleep a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }

          // load new instance
          instance.reset();

          // reset visualization
        }*/
      }
      if (ImGui::MenuItem("Load scene"))
      {
        std::string scene_fname = open_file_dialog("Scene file", "scen");
        if (!scene_fname.empty())
        {
          add_to_log("Loaded scene: " + scene_fname);
          settings.scene_fname  = scene_fname;
          show_agent_num_dialog = true;
        }
        else
        {
          add_to_log("Failed to load a scene file.");
        }
      }
      if (ImGui::MenuItem("Change number of agents"))
      {
        show_agent_num_dialog = true;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Settings"))
    {
      if (ImGui::MenuItem("Algorithm settings"))
      {
        std::cout << "Algorithm settings." << std::endl;
      }
      ImGui::EndMenu();
    }
    // ImVec2 menu_bar_size = ImGui::GetWindowSize();

    ImGui::EndMainMenuBar();
  }
  ImGui::PopStyleVar(2);  // Pop both styles
}

auto Visualizer::get_agent_position(const int agent_num, const int time) const -> const Point2d
{
  assertm(agent_num < (int)(*agents_paths).size(), "Agent number outside of the range.");
  // if path empty, return the start position
  if ((*agents_paths)[agent_num].empty())
  {
    return instance.get_start_positions()[agent_num];
  }
  const int idx = std::min((int)(*agents_paths)[agent_num].size() - 1, time);
  return instance.location_to_position((*agents_paths)[agent_num][idx]);
}

auto Visualizer::get_agent_location(const int agent_num, const int time) const -> int
{
  assertm(agent_num < (int)(*agents_paths).size(), "Agent number outside of the range.");
  const int idx = std::min((int)(*agents_paths)[agent_num].size() - 1, time);
  return (*agents_paths)[agent_num][idx];
}

auto Visualizer::get_agent_positions(const int time) const -> std::vector<Point2d>
{
  std::vector<Point2d> agent_positions;

  // find the current position for each agent
  agent_positions.reserve((int)(*agents_paths).size());
  for (int i = 0; i < (int)(*agents_paths).size(); i++)
  {
    agent_positions.push_back(get_agent_position(i, time));
  }
  return agent_positions;
}

auto Visualizer::get_agent_locations(const int time) const -> std::vector<int>
{
  std::vector<int> agent_positions;

  // find the current position for each agent
  agent_positions.reserve((int)(*agents_paths).size());
  for (int i = 0; i < (int)(*agents_paths).size(); i++)
  {
    agent_positions.push_back(get_agent_location(i, time));
  }
  return agent_positions;
}

void Visualizer::solution_visualization_update(const Visualization& vis)
{
  update_all_path_vis(vis.time);
  update_agent_circles(get_agent_positions(vis.time));
}


void Visualizer::sipp_visualization_update(const Visualization& vis)
{
  update_cell_rectangles_highlights();
}

void Visualizer::lns_visualization_update(const Visualization& vis)
{
  // reset other visualizations
  update_all_paths(vis.time);
  solution_vis.reset();
  // update sipp info
  if (selected_agent != -1)
  {
    update_sipp_info();
    sipp_vis.reset();
  }
  else
  {
    update_cell_rectangles_highlights();
  }
}

// based on https://stackoverflow.com/questions/43044/algorithm-to-randomly-generate-an-aesthetically-pleasing-color-palette
auto Visualizer::generate_random_color(const int mix_r, const int mix_g, const int mix_b, const double mix_ratio) -> sf::Color
{
  static std::uniform_int_distribution<int> rnd_dist(0, UCHAR_MAX - 1);
  while (true)
  {
    const int r = (int)std::round((rnd_dist(rnd_generator)) * (1 - mix_ratio) + mix_r * mix_ratio);
    const int g = (int)std::round((rnd_dist(rnd_generator)) * (1 - mix_ratio) + mix_g * mix_ratio);
    const int b = (int)std::round((rnd_dist(rnd_generator)) * (1 - mix_ratio) + mix_b * mix_ratio);
    // make sure the color is not too similar to black or white
    if (MIN_BRIGHTNESS <= (r + g + b) / 3 && (r + g + b) / 3 <= MAX_BRIGHTNESS)
    {
      return sf::Color(r, g, b);
    }
  }
}

void Visualizer::update_sipp_info()
{
  if (selected_agent == -1)
  {
    sipp_info = nullptr;
  }
  else
  {
    // iterate over lns info in the reversed order, find the latest iteration before current visualization time where selected agent was
    // replanned
    int offset = static_cast<int>(lns_info.size()) - 1 - lns_vis.time;
    for (auto info = lns_info.crbegin() + offset; info != lns_info.crend(); info++)
    {
      const auto it = std::find(info->sol.destroyed_paths.begin(), info->sol.destroyed_paths.end(), selected_agent);
      if (it != info->sol.destroyed_paths.end())
      {
        int idx = it - info->sol.destroyed_paths.begin();
        assertm(idx >= 0 && idx < static_cast<int>(info->sipp_info.size()), "Invalid idx");
        sipp_info = &(info->sipp_info[idx]);
        sipp_vis.new_data((*sipp_info).size());
        return;
      }
    }
    // use the first iteration
    sipp_info = &(lns_info[0].sipp_info[selected_agent]);
    sipp_vis.new_data((*sipp_info).size());
  }
}

void Visualizer::number_of_agents_dialog()
{
  static int user_input = 0;
  if (show_agent_num_dialog)
  {
    ImGui::OpenPopup("Number of agents");
    show_agent_num_dialog = false;  // Reset the flag after opening
  }

  if (ImGui::BeginPopupModal("Number of agents", NULL, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::Text("Please enter the number of agents:");
    ImGui::InputInt("##number", &user_input);

    if (ImGui::Button("OK"))
    {
      settings.num_of_agents = user_input;
      show_agent_num_dialog  = false;
      add_to_log("Number of agents changed to: " + std::to_string(settings.num_of_agents));
      ImGui::CloseCurrentPopup();
    }

    ImGui::SameLine();

    if (ImGui::Button("Cancel"))
    {
      show_agent_num_dialog = false;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

void Visualizer::add_to_log(std::string message)
{
  auto [timestamp_wall, timestamp_cpu] = clock.get_current_time();
  log.push_back(double_to_str(timestamp_wall, '.', 3) + ":\t" + message);
}

Visualization::Visualization(std::function<void(Visualization& vis)> update_func_) : update_func(std::move(update_func_))
{
}

void Visualization::reset()
{
  is_playing        = false;
  time              = 0;
  iteration_counter = 0;
  update();
}

void Visualization::start()
{
  if (time >= max_time)
  {
    reset();
  }
  is_playing = true;
}

void Visualization::pause()
{
  is_playing = false;
}

void Visualization::update()
{
  update_func(*this);
}

void Visualization::step_forward()
{
  if (time < max_time)
  {
    time++;
    update();
  }
}


void Visualization::step_backward()
{
  if (time > 0)
  {
    time--;
    update();
  }
}

void Visualization::new_data(int new_len)
{
  // reset the visualization time
  time              = 0;
  iteration_counter = 0;
  max_time          = new_len;
}

void Visualization::next_iteration()
{
  iteration_counter++;

  // update the iteration time
  if ((int)std::round((float)iteration_counter * speed) % FRAME_RATE == 0)
  {
    iteration_counter = 0;
    time++;
    if (time >= max_time)
    {
      pause();
    }
    update();
  }
}

