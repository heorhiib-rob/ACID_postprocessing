/*
 * Author: Jan Chleboun
 * Date: 03-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

// Include the header where Point2d is declared
#include "test_utils.h"
#include "utils.h"

// a custom assertion to check whether the value is in expected range, inclusive
::testing::AssertionResult inRange(int val, int a, int b)
{
  if ((val >= a) && (val <= b))
    return ::testing::AssertionSuccess();
  else if (val < a)
    return ::testing::AssertionFailure() << val << " is less than " << a;
  else
    return ::testing::AssertionFailure() << val << " is greater than " << b;
}

// Node tests
// test, whether nodes are compared the right way
TEST(NodeComparison, GreaterThan)
{
  // ints
  EXPECT_TRUE(Node(1, 2) > Node(2, 0)) << "Wrong comparison positive.";
  EXPECT_TRUE(Node(5.3, 4.1) > Node(1.2, 7.8)) << "Wrong comparison positive.";
  EXPECT_FALSE(Node(1, 2) > Node(2, 2)) << "Wrong comparison positive.";
  EXPECT_FALSE(Node(4, 5) > Node(12, 0)) << "Wrong comparison positive.";

  // negative numbers
  EXPECT_TRUE(Node(-1, -1) > Node(-2, -2)) << "Wrong comparison negative.";
  EXPECT_TRUE(Node(-1.1, -5.6) > Node(-10.5, -1.0)) << "wrong comparison negative.";
  EXPECT_FALSE(Node(-10, -1) > Node(-1, -8)) << "wrong comparison negative.";
  EXPECT_FALSE(Node(-1, -1) > Node(-1, 0)) << "wrong comparison negative.";

  // mix
  EXPECT_TRUE(Node(1, 1) > Node(-2, -2)) << "Wrong comparison positive and negative.";
  EXPECT_TRUE(Node(1, -5) > Node(-10, 3)) << "wrong comparison positive and negative.";
  EXPECT_FALSE(Node(-10, -1) > Node(1, 8)) << "wrong comparison positive and negative.";
  EXPECT_FALSE(Node(1, -1) > Node(1, 0)) << "wrong comparison positive and negative.";

  // equal
  EXPECT_FALSE(Node(0, 0) > Node(0, 0)) << "Wrong comparison on equal.";
  EXPECT_FALSE(Node(1.2, 2.4) > Node(2.4, 1.2)) << "Wrong comparison on equal.";
  EXPECT_FALSE(Node(-3, -5) > Node(-3, -5)) << "Wrong comparison on equal.";
}

TEST(NodeComparison, LessThan)
{
  // ints
  EXPECT_FALSE(Node(1, 2) < Node(2, 0)) << "Wrong comparison positive.";
  EXPECT_FALSE(Node(5.3, 4.1) < Node(1.2, 7.8)) << "Wrong comparison positive.";
  EXPECT_TRUE(Node(1, 2) < Node(2, 2)) << "Wrong comparison positive.";
  EXPECT_TRUE(Node(4, 5) < Node(12, 0)) << "Wrong comparison positive.";

  // negative numbers
  EXPECT_FALSE(Node(-1, -1) < Node(-2, -2)) << "Wrong comparison negative.";
  EXPECT_FALSE(Node(-1.1, -5.6) < Node(-10.5, -1.0)) << "wrong comparison negative.";
  EXPECT_TRUE(Node(-10, -1) < Node(-1, -8)) << "wrong comparison negative.";
  EXPECT_TRUE(Node(-1, -1) < Node(-1, 0)) << "wrong comparison negative.";

  // mix
  EXPECT_FALSE(Node(1, 1) < Node(-2, -2)) << "Wrong comparison positive and negative.";
  EXPECT_FALSE(Node(1, -5) < Node(-10, 3)) << "wrong comparison positive and negative.";
  EXPECT_TRUE(Node(-10, -1) < Node(1, 8)) << "wrong comparison positive and negative.";
  EXPECT_TRUE(Node(1, -1) < Node(1, 0)) << "wrong comparison positive and negative.";

  // equal
  EXPECT_FALSE(Node(0, 0) < Node(0, 0)) << "Wrong comparison on equal.";
  EXPECT_FALSE(Node(1.2, 2.4) < Node(2.4, 1.2)) << "Wrong comparison on equal.";
  EXPECT_FALSE(Node(-3, -5) < Node(-3, -5)) << "Wrong comparison on equal.";
}

TEST(NodeComparison, Equal)
{
  EXPECT_TRUE(Node(0, 0) == Node(0, 0)) << "Wrong comparison on equal.";
  EXPECT_TRUE(Node(1, 2) == Node(1, 2)) << "Wrong comparison on equal.";
  EXPECT_TRUE(Node(-4, -2) == Node(-4, -2)) << "Wrong comparison on equal.";
  EXPECT_TRUE(Node(100.314, 15.0) == Node(100.314, 15.0)) << "Wrong comparison on equal.";

  EXPECT_FALSE(Node(0, 0) == Node(1, 0)) << "Wrong comparison on different.";
  EXPECT_FALSE(Node(0, 1) == Node(1, 0)) << "Wrong comparison on different.";
  EXPECT_FALSE(Node(1.1123, 13.1) == Node(111.23, 1.31)) << "Wrong comparison on different.";
  EXPECT_FALSE(Node(2, 1) == Node(1, 2)) << "Wrong comparison on different.";
}
// Point2d tests
// Basic Tests for Constructor and Public Variables
TEST(Point2dTest, Constructor)
{
  Point2d p(3, 4);
  EXPECT_EQ(p.x, 3) << "Wrong x value in constructor.";
  EXPECT_EQ(p.y, 4) << "Wrong y value in constructor.";
}

// Test Addition Operator `+`
TEST(Point2dTest, OperatorPlus)
{
  Point2d p1(1, 1);
  Point2d p2(2, 3);
  Point2d result = p1 + p2;

  EXPECT_EQ(result.x, 3) << "Wrong addition result.";
  EXPECT_EQ(result.y, 4) << "Wrong addition result.";
}

// Test Equality Operator `==`
TEST(Point2dTest, OperatorEquality)
{
  Point2d p1(2, 2);
  Point2d p2(2, 2);
  Point2d p3(3, 4);

  EXPECT_TRUE(p1 == p2) << "Points should be equal.";       // Should be equal
  EXPECT_FALSE(p1 == p3) << "Points should not be equal.";  // Should not be equal
}

// Test `add()` Method (Modifies the Object)
TEST(Point2dTest, AddMethod)
{
  Point2d p1(1, 1);
  Point2d p2(2, 3);

  p1.add(p2);

  EXPECT_EQ(p1.x, 3) << "Wrong x after addition.";
  EXPECT_EQ(p1.y, 4) << "Wrong y after addition.";
}

// Edge Case: Adding Zero
TEST(Point2dTest, AddZero)
{
  Point2d p1(5, 7);
  Point2d zero(0, 0);

  p1.add(zero);

  EXPECT_EQ(p1.x, 5) << "Wrong x after adding 0.";
  EXPECT_EQ(p1.y, 7) << "Wrong y after adding 0.";
}

// Edge Case: Adding Negative Values
TEST(Point2dTest, AddNegative)
{
  Point2d p1(5, 7);
  Point2d neg(-2, -3);

  p1.add(neg);

  EXPECT_EQ(p1.x, 3) << "Wrong x after adding a negative number.";
  EXPECT_EQ(p1.y, 4) << "Wrong y after adding a negative number.";
}

// Edge Case: Adding Large Values (Overflow Test)
TEST(Point2dTest, LargeValues)
{
  Point2d p1(1000000000, 1000000000);
  Point2d p2(1000000000, 1000000000);

  Point2d result = p1 + p2;

  EXPECT_EQ(result.x, 2000000000) << "Overflow not handled in x.";
  EXPECT_EQ(result.y, 2000000000) << "Overflow not handled in y.";
}

// MAP TESTS

// Helper function to create a map
namespace UTILS_TEST
{
Map create_test_map()
{
  Map my_map;
  my_map.width  = 3;
  my_map.height = 3;
  my_map.data   = {0, 1, 0, 0, 0, 1, 1, 0, 1};
  my_map.loaded = true;
  return my_map;
}
}  // namespace UTILS_TEST

// Test is in check
TEST(MapTest, IsIn)
{
  Map my_map = UTILS_TEST::create_test_map();
  // test Point2d format
  EXPECT_TRUE(my_map.is_in({0, 0})) << "Point should be in the map.";
  EXPECT_TRUE(my_map.is_in({2, 2})) << "Point should be in the map.";
  EXPECT_FALSE(my_map.is_in({-1, 0})) << "Point should not be in the map.";
  EXPECT_FALSE(my_map.is_in({0, 3})) << "Point should not be in the map.";
  EXPECT_FALSE(my_map.is_in({3, 3})) << "Point should not be in the map.";
  // test int format
  EXPECT_TRUE(my_map.is_in(0)) << "Point should be in the map.";
  EXPECT_TRUE(my_map.is_in(8)) << "Point should be in the map.";
  EXPECT_FALSE(my_map.is_in(-1)) << "Point should not be in the map.";
  EXPECT_FALSE(my_map.is_in(9)) << "Point should not be in the map.";
}

// Test index function
TEST(MapTest, Index)
{
  Map my_map = UTILS_TEST::create_test_map();
  // test Point2d format
  EXPECT_EQ(my_map.index({0, 0}), 0) << "Should be free cell.";
  EXPECT_EQ(my_map.index({1, 2}), 0) << "Should be free cell.";
  EXPECT_EQ(my_map.index({1, 0}), 1) << "Should be obstacle.";
  EXPECT_EQ(my_map.index({2, 2}), 1) << "Should be obstacle.";
  // test int format
  EXPECT_EQ(my_map.index(0), 0) << "Should be free cell.";
  EXPECT_EQ(my_map.index(4), 0) << "Should be free cell.";
  EXPECT_EQ(my_map.index(1), 1) << "Should be obstacle.";
  EXPECT_EQ(my_map.index(8), 1) << "Should be obstacle.";
}


// test finding the neighbours
// Test Case: Center Position
TEST(FindNeighborsTest, CenterPosition)
{
  Map my_map = UTILS_TEST::create_test_map();

  // test Point2d version
  std::vector<Point2d> neighbors = my_map.find_neighbors({1, 1});
  EXPECT_THAT(neighbors, ::testing::UnorderedElementsAre(Point2d(1, 2), Point2d(0, 1)));

  // test int version
  std::vector<int> neighbors_int = my_map.find_neighbors(4);
  EXPECT_THAT(neighbors_int, ::testing::UnorderedElementsAre(3, 7));
}

// Test Case: Corner Position (Top-Left)
TEST(FindNeighborsTest, TopLeftCorner)
{
  Map my_map = UTILS_TEST::create_test_map();

  // Point2d version
  std::vector<Point2d> neighbors = my_map.find_neighbors({0, 0});
  std::vector<Point2d> expected  = {{0, 1}};
  EXPECT_EQ(neighbors, expected);

  // int version
  std::vector<int> neighbors_int = my_map.find_neighbors(0);
  EXPECT_EQ(neighbors_int, std::vector<int>({3}));
}

// Test Case: Position (Bottom-middle)
TEST(FindNeighborsTest, BottomMiddle)
{
  Map my_map = UTILS_TEST::create_test_map();

  // Point2d version
  std::vector<Point2d> neighbors = my_map.find_neighbors({1, 2});
  std::vector<Point2d> expected  = {{1, 1}};
  EXPECT_EQ(neighbors, expected);

  // int version
  std::vector<int> neighbors_int = my_map.find_neighbors(7);
  EXPECT_EQ(neighbors_int, std::vector<int>({4}));
}

// Test Case: Left Edge
TEST(FindNeighborsTest, LeftEdge)
{
  Map my_map = UTILS_TEST::create_test_map();

  // Point2d version
  std::vector<Point2d> neighbors = my_map.find_neighbors({0, 1});
  EXPECT_THAT(neighbors, ::testing::UnorderedElementsAre(Point2d(0, 0), Point2d(1, 1)));

  // int version
  std::vector<int> neighbors_int = my_map.find_neighbors(3);
  EXPECT_THAT(neighbors_int, ::testing::UnorderedElementsAre(0, 4));
}

// Test Case: Out-of-Bounds Position
TEST(FindNeighborsTest, OutOfBounds)
{
  Map     my_map = UTILS_TEST::create_test_map();
  Point2d pos(-1, -1);

#ifdef NDEBUG
  std::vector<Point2d> neighbors     = my_map.find_neighbors(pos);
  std::vector<int>     neighbors_int = my_map.find_neighbors(-1);

  // EXPECT_TRUE(neighbors.empty());  // Should return empty vector
  // EXPECT_TRUE(neighbors_int.empty());
#else
  // expect assert death
  EXPECT_DEATH(my_map.find_neighbors({-1, -1}), "Invalid position.");
  EXPECT_DEATH(my_map.find_neighbors(-1), "Invalid position.");
#endif
}

// Test Case: Position Inside an Obstacle
#ifdef NDEBUG
TEST(FindNeighborsTest, DISABLED_InsideObstacle)
#else
TEST(FindNeighborsTest, InsideObstacle)
#endif
{
  Map my_map = UTILS_TEST::create_test_map();

  EXPECT_DEATH(my_map.find_neighbors({0, 2}), "Invalid position.");
  EXPECT_DEATH(my_map.find_neighbors(6), "Invalid position.");
}

// Test Case: Large Map Edge Case
TEST(FindNeighborsTest, LargeMapEdge)
{
  Map my_map;
  my_map.width  = 100;
  my_map.height = 100;
  my_map.data.resize(100 * 100, 0);  // All free cells
  my_map.loaded = true;

  // Check the Point2d version
  std::vector<Point2d> neighbors = my_map.find_neighbors({99, 99});
  EXPECT_THAT(neighbors, ::testing::UnorderedElementsAre(Point2d(98, 99), Point2d(99, 98)));

  // Check the int version
  std::vector<int> neighbors_int = my_map.find_neighbors(9999);
  EXPECT_THAT(neighbors_int, ::testing::UnorderedElementsAre(9899, 9998));
}

// test detection of overlap of time intervals
// Test Case: No overlap
TEST(TimeIntervalIntersectionTest, NoOverlap)
{
  TimeInterval i1(0, 1);
  TimeInterval i2(2, 3);
  EXPECT_FALSE(overlap(i1, i2)) << "Intervals should not intersect.";
}

// Test Case: Overlap
TEST(TimeIntervalIntersectionTest, Overlap)
{
  TimeInterval i1(0, 2);
  TimeInterval i2(1, 3);
  EXPECT_TRUE(overlap(i1, i2)) << "Intervals should intersect.";
  EXPECT_TRUE(overlap(i2, i1)) << "Intervals should intersect.";
}

// Test Case: One Interval Inside Another
TEST(TimeIntervalIntersectionTest, Inside)
{
  TimeInterval i1(0, 4);
  TimeInterval i2(1, 3);
  EXPECT_TRUE(overlap(i1, i2)) << "Intervals should intersect.";
  EXPECT_TRUE(overlap(i2, i1)) << "Intervals should intersect.";
}

// Test Case: One Interval Touching Another
TEST(TimeIntervalIntersectionTest, Touching)
{
  TimeInterval i1(0, 1);
  TimeInterval i2(1, 2);
  EXPECT_TRUE(overlap(i1, i2)) << "Intervals should intersect.";
  EXPECT_TRUE(overlap(i2, i1)) << "Intervals should intersect.";
}

// Test Case: same interval
TEST(TimeIntervalIntersectionTest, SameInterval)
{
  TimeInterval i1(0, 1);
  EXPECT_TRUE(overlap(i1, i1)) << "Intervals should intersect.";
}

// Test Case: intervals of length 1
TEST(TimeIntervalIntersectionTest, LengthOne)
{
  TimeInterval i1(1, 1);
  TimeInterval i2(2, 2);
  EXPECT_FALSE(overlap(i1, i2)) << "Intervals should not intersect.";
  EXPECT_FALSE(overlap(i2, i1)) << "Intervals should not intersect.";
}

// TimePointPath related tests
// test timepointpath interval overlap
TEST(test_utils, timepointpath_interval_overlap)
{
  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 10}});
  tp_path.push_back({6, {3, 6}});
  tp_path.push_back({7, {6, 6}});
  tp_path.push_back({8, {7, INT_MAX}});

  EXPECT_EQ(check_timepointpath_interval_no_overlap(tp_path), false) << "There should be overlap.";

  // fix the overlap
  fix_timepointpath_interval_overlap(tp_path);

  EXPECT_EQ(check_timepointpath_interval_no_overlap(tp_path), true) << "There should not be any overlap after the fix.";

  // check that the intervals are correct
  EXPECT_EQ(tp_path[0].interval, TimeInterval(0, 2)) << "Wrong interval.";
  EXPECT_EQ(tp_path[1].interval, TimeInterval(3, 5)) << "Wrong interval.";
  EXPECT_EQ(tp_path[2].interval, TimeInterval(6, 6)) << "Wrong interval.";
  EXPECT_EQ(tp_path[3].interval, TimeInterval(7, INT_MAX)) << "Wrong interval.";
}

// Test case: no overlap
TEST(test_utils, timepointpath_no_overlap)
{
  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 2}});
  tp_path.push_back({6, {3, 5}});
  tp_path.push_back({7, {6, 6}});
  tp_path.push_back({8, {7, INT_MAX}});

  EXPECT_EQ(check_timepointpath_interval_no_overlap(tp_path), true) << "There should not be overlap.";
}

// Test case: empty path
TEST(test_utils, timepointpath_empty)
{
  EXPECT_EQ(check_timepointpath_interval_no_overlap(TimePointPath()), true) << "There should not be overlap.";
}

// test timepointpath validity check function
TEST(test_utils, timepointpath_validity_check_simple_valid)
{
  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 2}});
  tp_path.push_back({6, {3, 5}});
  tp_path.push_back({7, {6, 6}});
  tp_path.push_back({8, {7, INT_MAX}});

  EXPECT_EQ(is_valid_timepointpath(tp_path), true) << "Path should be valid.";
}

TEST(test_utils, timepointpath_validity_check_simple_invalid_end)
{
  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 2}});
  tp_path.push_back({6, {3, 5}});
  tp_path.push_back({7, {6, 6}});
  tp_path.push_back({8, {7, 9}});

  EXPECT_EQ(is_valid_timepointpath(tp_path), false) << "Path should not be valid.";
}

TEST(test_utils, timepointpath_validity_check_simple_invalid_overlap)
{
  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 2}});
  tp_path.push_back({6, {3, 5}});
  tp_path.push_back({7, {6, 7}});
  tp_path.push_back({8, {7, INT_MAX}});

  EXPECT_EQ(is_valid_timepointpath(tp_path), false) << "Path should not be valid.";
}

TEST(test_utils, timepointpath_validity_check_simple_invalid_gap)
{
  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 1}});
  tp_path.push_back({6, {3, 5}});
  tp_path.push_back({7, {6, 7}});
  tp_path.push_back({8, {7, INT_MAX}});

  EXPECT_EQ(is_valid_timepointpath(tp_path), false) << "Path should not be valid.";
}

TEST(test_utils, timepointpath_validity_check_random)
{
  // generate random timepointpaths
  for (int i = 0; i < 5; i++)
  {
    TimePointPath tp_path = generate_random_timepointpath(100);
    EXPECT_EQ(is_valid_timepointpath(tp_path), false) << "Path should not be valid.";

    fix_timepointpath_interval_overlap(tp_path);
    EXPECT_EQ(is_valid_timepointpath(tp_path), true) << "Path should be valid.";
  }
}

// test the instance timepointpath validity check, which also check validity of locations
TEST(test_utils, timepointpath_validity_check_locations_valid)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);

  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 2}});
  tp_path.push_back({6, {3, 5}});
  tp_path.push_back({7, {6, 6}});
  tp_path.push_back({8, {7, INT_MAX}});

  EXPECT_EQ(instance->check_timepointpath_validity(tp_path), true) << "Path should be valid.";
}

TEST(test_utils, timepointpath_validity_check_locations_invalid)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 2}});
  tp_path.push_back({1, {3, INT_MAX}});

  EXPECT_EQ(instance->check_timepointpath_validity(tp_path), false) << "Path should not be valid, jump.";

  TimePointPath tp_path2;
  tp_path2.push_back({24, {0, 2}});
  tp_path2.push_back({25, {3, INT_MAX}});
  EXPECT_EQ(instance->check_timepointpath_validity(tp_path2), false) << "Path should not be valid, out of bounds.";

  TimePointPath tp_path3;
  tp_path3.push_back({4, {0, 2}});
  tp_path3.push_back({5, {3, INT_MAX}});
  EXPECT_EQ(instance->check_timepointpath_validity(tp_path3), false) << "Path should not be valid, locations not next to each other.";
}

// Test conversion from timepointpath to Path simple
TEST(test_utils, timepointpath_to_path_simple)
{
  // generate a simple timepointpath
  TimePointPath tp_path;
  tp_path.push_back({3, {0, 10}});
  tp_path.push_back({6, {3, 6}});
  tp_path.push_back({7, {6, 6}});
  tp_path.push_back({8, {7, INT_MAX}});

  // convert to path
  Path path          = timepointpath_to_path(tp_path);
  Path expected_path = {3, 3, 3, 6, 6, 6, 7, 8};

  // check that the path has the right length
  EXPECT_EQ(path.size(), expected_path.size()) << "Wrong path length.";

  // check that the path has the right time intervals
  EXPECT_EQ(path, expected_path) << "Wrong path.";
}

// Test conversion from timepointpath to Path
TEST(test_utils, timepointpath_to_path_random)
{
  // generate a random timepointpath
  const int     len     = 1000;
  TimePointPath tp_path = generate_random_timepointpath(len);

  // convert to path
  Path path = timepointpath_to_path(tp_path);

  // check that the path has the right length
  EXPECT_EQ(path.size(), len) << "Wrong path length.";

  // check that the path has the right time intervals
  for (auto it = tp_path.begin(); it != tp_path.end(); it++)
  {
    int t_max = it->interval.t_min;
    if (it != std::prev(tp_path.end()))
    {
      t_max = std::next(it)->interval.t_min - 1;
    }
    for (int t = it->interval.t_min; t <= t_max; t++)
    {
      EXPECT_EQ(path[t], it->location) << "Wrong location at time " << t;
    }
  }
}

// test conversion from path to timepointpath
TEST(test_utils, path_to_timepointpath_simple)
{
  // generate a simple path
  Path          path    = {3, 3, 3, 6, 6, 6, 7, 8};
  TimePointPath tp_path = path_to_timepointpath(path);

  // check that the path has the right length
  ASSERT_EQ(tp_path.size(), 4) << "Wrong path length.";

  // check that the path has the right time intervals
  for (int i = 0; i < (int)tp_path.size(); i++)
  {
    // check that the location is correct during the whole interval
    for (int t = tp_path[i].interval.t_min; t <= tp_path[i].interval.t_max; t++)
    {
      EXPECT_EQ(tp_path[i].location, path[t]) << "Wrong location at time " << t;
      // dont check the last interval
      if (tp_path[i].interval.t_max == INT_MAX)
      {
        break;
      }
    }
  }
}

// special case empty path
TEST(test_utils, path_to_timepointpath_empty)
{
  // generate an empty path
  Path path;

  // convert to timepointpath
  TimePointPath tp_path = path_to_timepointpath(path);

  // check that the path is empty
  EXPECT_TRUE(tp_path.empty()) << "Path should be empty.";
}

// test that path to timepointpath to path gives the same path
TEST(test_utils, path_to_timepointpath_to_path)
{
  Path path({1, 1, 1, 2, 2, 3, 2, 7, 7, 7, 7, 7, 7, 8, 8, 7, 8, 7, 8, 3, 3, 8, 13, 13, 13, 12, 11});

  // convert to timepointpath and back
  Path path2 = timepointpath_to_path(path_to_timepointpath(path));

  // check equality
  EXPECT_EQ(path, path2) << "Paths are not equal.";
}

// test timepointpath to path to timepointpath
TEST(test_utils, timepointpath_to_path_to_timepointpath)
{
  // generate a random timepointpath
  const int     len     = 10;
  TimePointPath tp_path = generate_random_timepointpath(len);
  fix_timepointpath_interval_overlap(tp_path);

  // convert to path and back
  TimePointPath tp_path2 = path_to_timepointpath(timepointpath_to_path(tp_path));

  // check equality
  EXPECT_EQ(tp_path, tp_path2) << "Paths are not equal.";
}

// Test find_direction
TEST(test_utils, DirectionRight)
{
  Point2d p1(0, 0);
  Point2d p2(1, 0);

  // Point version
  EXPECT_EQ(find_direction(p1, p2), Direction::RIGHT) << "Wrong direction.";
  // location version
  EXPECT_EQ(find_direction(0, 1), Direction::RIGHT) << "Wrong direction.";
}

TEST(test_utils, DirectionLeft)
{
  Point2d p1(1, 3);
  Point2d p2(0, 3);

  // Point version
  EXPECT_EQ(find_direction(p1, p2), Direction::LEFT) << "Wrong direction.";
  // location version
  EXPECT_EQ(find_direction(1, 0), Direction::LEFT) << "Wrong direction.";
}

TEST(test_utils, DirectionUp)
{
  Point2d p1(1, 1);
  Point2d p2(1, 0);

  // Point version
  EXPECT_EQ(find_direction(p1, p2), Direction::UP) << "Wrong direction.";
  // location version
  EXPECT_EQ(find_direction(4, 1), Direction::UP) << "Wrong direction.";
}

TEST(test_utils, DirectionDown)
{
  Point2d p1(2, 1);
  Point2d p2(2, 2);

  // Point version
  EXPECT_EQ(find_direction(p1, p2), Direction::DOWN) << "Wrong direction.";
  // location version
  EXPECT_EQ(find_direction(5, 8), Direction::DOWN) << "Wrong direction.";
}

TEST(test_utils, DirectionNone)
{
  Point2d p1(1, 1);
  Point2d p2(1, 1);

  // Point version
  EXPECT_EQ(find_direction(p1, p2), Direction::NONE) << "Wrong direction.";
  // location version
  EXPECT_EQ(find_direction(4, 4), Direction::NONE) << "Wrong direction.";
}

// test random bool generator
// TEST(test_utils, RandomBool)
// {
//   // generate 100 random bools and check, whether they are valid (just a simple test to test the  unit test environment)
//   for (int i = 0; i < 100; i++)
//   {
//     bool b = fast_random_bool();
//     EXPECT_TRUE(b == true || b == false) << "Invalid value.";
//   }
// }

// test function that retrieves the last number from string
TEST(FindLastNumberTest, BasicCases)
{
  EXPECT_EQ(find_last_number("abc 123 def 456 ghi 789"), 789);
  EXPECT_EQ(find_last_number("The number is 42"), 42);
  EXPECT_EQ(find_last_number("123 456 789"), 789);
}

TEST(FindLastNumberTest, NoNumber)
{
  EXPECT_EQ(find_last_number("Hello World"), -1);
  EXPECT_EQ(find_last_number("No digits here!"), -1);
  EXPECT_EQ(find_last_number(""), -1);
}

TEST(FindLastNumberTest, SingleNumber)
{
  EXPECT_EQ(find_last_number("42"), 42);
  EXPECT_EQ(find_last_number("Only one 99"), 99);
  EXPECT_EQ(find_last_number("123abc"), 123);
}

TEST(FindLastNumberTest, LeadingAndTrailingSpaces)
{
  EXPECT_EQ(find_last_number("   100   "), 100);
  EXPECT_EQ(find_last_number("   test 55"), 55);
}

TEST(FindLastNumberTest, SpecialCharacters)
{
  EXPECT_EQ(find_last_number("!@# $%^ 77 &*()"), 77);
  EXPECT_EQ(find_last_number("**&& 9981 !!"), 9981);
}

TEST(FindLastNumberTest, NegativeNumbers)
{
  EXPECT_EQ(find_last_number("abc -123 def -456 ghi -789"), 789);
  EXPECT_EQ(find_last_number("-42"), 42);
  EXPECT_EQ(find_last_number("Some -9 and 8"), 8);
}

TEST(FindLastNumberTest, MixedContent)
{
  EXPECT_EQ(find_last_number("a1b2c3 88 end99"), 99);
  EXPECT_EQ(find_last_number("Pi is around 3.14159"), 14159);
  EXPECT_EQ(find_last_number("x y z 1 2 3 abc 987"), 987);
}

