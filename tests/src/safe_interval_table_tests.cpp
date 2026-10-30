/*
 * Author: Jan Chleboun
 * Date: 26-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#include <gtest/gtest.h>

#include <climits>

#include "Instance.h"
#include "LNS.h"
#include "SIPP.h"
#include "SafeIntervalTable.h"
#include "test_utils.h"

/**
 * @brief Class for testing the SafeIntervalTable class
 */
class SafeIntervalTableTest : public ::testing::Test
{
protected:
  std::unique_ptr<Instance>          instance;
  std::unique_ptr<SafeIntervalTable> table;

  void SetUp() override
  {
    // load instance
    std::string base_path = get_base_path_tests();  // path to my_solver

    instance =
        std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);
    table = std::make_unique<SafeIntervalTable>(*instance);
  }

  void TearDown() override
  {
  }
};

// Constraint Table tests
// Adding a single constraint and checking if it is stored correctly
TEST_F(SafeIntervalTableTest, SingleConstraint)
{
  TimePoint tp(7, {3, 5});  // location (1,2) with time interval [3,5]
                            //
  table->add_constraint(tp);

  // test the estimate of the maximal path len
  EXPECT_EQ(table->get_max_path_len_estimate(), 13);

  // test that the first safe interval is right
  auto first_si = table->get_first_safe_interval(7);
  EXPECT_EQ(*first_si, TimeInterval(0, 2));

  // test that both the interval before and after the constraint are retrieved
  std::list<TimeInterval>::const_iterator start, end;
  std::tie(start, end) = table->get_safe_intervals(7, {0, 6});
  ASSERT_EQ(std::distance(start, end), 2) << "Expected two safe intervals after adding a constraint!";
  EXPECT_EQ(*start, TimeInterval(0, 2));
  EXPECT_EQ(*std::next(start), TimeInterval(6, INT_MAX));


  // test that no interval is retrieved when the constraint is in the way
  std::tie(start, end) = table->get_safe_intervals(7, {2, 5});
  ASSERT_EQ(std::distance(start, end), 1) << "Expected only one safe interval!";
  EXPECT_EQ(*start, TimeInterval(0, 2));
}

// Adding multiple constraints and ensuring safe intervals are computed correctly
TEST_F(SafeIntervalTableTest, MultipleSingleConstraints)
{
  table->add_constraint(TimePoint(7, {2, 4}));
  table->add_constraint(TimePoint(7, {5, 5}));
  table->add_constraint(TimePoint(7, {6, 7}));
  table->add_constraint(TimePoint(7, {10, 15}));

  // test the estimate of the maximal path len
  EXPECT_EQ(table->get_max_path_len_estimate(), 23);

  std::list<TimeInterval>::const_iterator start, end;
  std::tie(start, end) = table->get_safe_intervals(7, {0, 20});
  ASSERT_EQ(std::distance(start, end), 3) << "Expected three safe intervals after adding multiple constraints!";
  EXPECT_EQ(*start, TimeInterval(0, 1));
  EXPECT_EQ(*std::next(start), TimeInterval(8, 9));
  EXPECT_EQ(*std::next(start, 2), TimeInterval(16, INT_MAX));
}

// Removing a constraint and ensuring safe intervals update
TEST_F(SafeIntervalTableTest, RemoveConstraint)
{
  TimePoint tp(7, {3, 5});
  table->add_constraint(tp);
  table->remove_constraint(tp);

  table->update_latest_constraint_end_estimate();
  // test the estimate of the maximal path len
  EXPECT_EQ(table->get_max_path_len_estimate(), 8);

  auto first_si = table->get_first_safe_interval(7);
  EXPECT_TRUE(first_si->t_min == 0 && first_si->t_max == INT_MAX);

  std::list<TimeInterval>::const_iterator start, end;
  std::tie(start, end) = table->get_safe_intervals(7, {0, INT_MAX});
  ASSERT_EQ(std::distance(start, end), 1) << "Expected one safe interval after removing a constraint!";
  EXPECT_EQ(*start, TimeInterval(0, INT_MAX));
}

TEST_F(SafeIntervalTableTest, GetSafeIntervalEmpty)
{
  auto first_si = table->get_first_safe_interval(8);
  EXPECT_TRUE(first_si->t_min == 0 && first_si->t_max == INT_MAX) << "No constraints should mean an unlimited safe interval.";

  // check the path len estimate
  EXPECT_EQ(table->get_max_path_len_estimate(), 8);

  auto [start, end] = table->get_safe_intervals(8, {0, INT_MAX});
  ASSERT_EQ(std::distance(start, end), 1) << "Expected one safe interval for an empty position!";
  EXPECT_EQ(*start, TimeInterval(0, INT_MAX));
}


// Handling constraints dynamically (adding and removing in sequence)
TEST_F(SafeIntervalTableTest, DynamicAddRemove)
{
  table->add_constraint(TimePoint(2, {2, 4}));
  table->add_constraint(TimePoint(2, {6, 8}));
  table->remove_constraint(TimePoint(2, {2, 4}));

  // test the estimate of the maximal path len
  EXPECT_EQ(table->get_max_path_len_estimate(), 16);

  std::list<TimeInterval>::const_iterator start, end;
  std::tie(start, end) = table->get_safe_intervals(2, {0, 10});
  ASSERT_EQ(std::distance(start, end), 2) << "Expected two safe intervals after removal.";
  EXPECT_EQ(*start, TimeInterval(0, 5));
  EXPECT_EQ(*std::next(start), TimeInterval(9, INT_MAX));
}

// Constraint removal from empty table if in debug mode
#ifdef NDEBUG
TEST_F(SafeIntervalTableTest, DISABLED_RemoveFromEmptyTable)
#else
TEST_F(SafeIntervalTableTest, RemoveFromEmptyTable)
#endif
{
  TimePoint tp(3, {3, 6});
  // expect assert death
  EXPECT_DEATH(table->remove_constraint(tp), "Constraint interval can not have any overlap with safe interval.");
}

// Constraint removal when no constraint exists
#ifdef NDEBUG
TEST_F(SafeIntervalTableTest, DISABLED_RemoveNonExistentConstraint)
#else
TEST_F(SafeIntervalTableTest, RemoveNonExistentConstraint)
#endif
{
  TimePoint tp(3, {3, 6});
  // test the second method
  EXPECT_DEATH(table->remove_constraint(tp), "Constraint interval can not have any overlap with safe interval.");
}

// Test 8: Adding a constraint that is already present (should cause assert death)
#ifdef NDEBUG
TEST_F(SafeIntervalTableTest, DISABLED_AddExistingConstraint)
#else
TEST_F(SafeIntervalTableTest, AddExistingConstraint)
#endif
{
  TimePoint tp(3, {3, 6});
  table->add_constraint(tp);
  ASSERT_DEATH(table->add_constraint(tp), "Can not add an overlapping constraint.");
}

// Adding a path that forms constraints, test whether edge constraints are added
// Adding a path that forms constraints, test whether edge constraints are added
TEST_F(SafeIntervalTableTest, AddPathConstraints)
{
  std::vector<TimePoint> path1 = {TimePoint(6, {0, 2}), TimePoint(7, {3, 4}), TimePoint(8, {5, 6})};
  table->add_constraints(path1);
  std::vector<TimePoint> path2 = {TimePoint(3, {0, 5}), TimePoint(6, {6, 8}), TimePoint(7, {9, 10})};
  table->add_constraints(path2);

  // test the estimate of the maximal path len
  EXPECT_EQ(table->get_max_path_len_estimate(), 18);

  // test first edge conflict
  std::vector<TimeInterval> safe_intervals = filter_time_intervals(table->get_safe_intervals(6, {0, 3}), {0, 3}, 7, 6, *table);
  EXPECT_EQ(safe_intervals.size(), 0) << "Expected no safe interval due to an edge conflict.";

  // test going to 1, 1 from a non constrained position
  auto [start, end] = table->get_safe_intervals(6, {0, 10});
  ASSERT_EQ(std::distance(start, end), 2) << "Expected two safe intervals after adding path constraints.";
  EXPECT_EQ(*start, TimeInterval(3, 5));
  EXPECT_EQ(*std::next(start), TimeInterval(9, INT_MAX));

  // test second edge conflict
  std::tie(start, end) = table->get_safe_intervals(6, {5, 9});
  safe_intervals       = filter_time_intervals(table->get_safe_intervals(6, {5, 9}), {5, 9}, 7, 6, *table);
  EXPECT_EQ(safe_intervals.size(), 1) << "Expected only one safe interval.";
  EXPECT_EQ(safe_intervals[0], TimeInterval(5, 5));
}

// check that the edge conflict is resolved correctly when there are more constraints in the list
TEST_F(SafeIntervalTableTest, EdgeConflictMoreConstraints)
{
  std::vector<TimePoint> path1 = {TimePoint(6, {0, 1}), TimePoint(7, {2, 4}), TimePoint(6, {5, 6}), TimePoint(7, {7, 8}),
                                  TimePoint(6, {9, 10})};
  table->add_constraints(path1);

  std::vector<TimeInterval> safe_intervals = filter_time_intervals(table->get_safe_intervals(6, {0, 2}), {0, 2}, 7, 6, *table);
  EXPECT_EQ(safe_intervals.size(), 0) << "Expected no safe intervals due to edge conflict.";
}

// test edge constraint table
// add to edge constraint table
TEST_F(SafeIntervalTableTest, EdgeConstraintTableAdd)
{
  // add a constraint to the table
  table->edge_constraint_table.add(6, 7, 3);

  // check that the constraint is in the table
  EXPECT_TRUE(table->edge_constraint_table.get(6, 7, 3));

  // check that the other times are not there
  EXPECT_FALSE(table->edge_constraint_table.get(6, 7, 2));
  EXPECT_FALSE(table->edge_constraint_table.get(6, 7, 4));
}

// remove from edge constraint table
TEST_F(SafeIntervalTableTest, EdgeConstraintTableRemove)
{
  // add a constraint to the table
  table->edge_constraint_table.add(1, 2, 3);

  EXPECT_TRUE(table->edge_constraint_table.get(1, 2, 3));

  // remove the constraint
  table->edge_constraint_table.remove(1, 2, 3);

  // check that the constraint is not in the table
  EXPECT_FALSE(table->edge_constraint_table.get(1, 2, 3));
}

// add an existing edge constraint
#ifdef NDEBUG
TEST_F(SafeIntervalTableTest, DISABLED_EdgeConstraintTableAddExisting)
#else
TEST_F(SafeIntervalTableTest, EdgeConstraintTableAddExisting)
#endif
{
  // add a constraint to the table
  table->edge_constraint_table.add(1, 2, 3);

  // expect assert death
  EXPECT_DEATH(table->edge_constraint_table.add(1, 2, 3), "Edge constraint already exists.");
}

// add an invalid edge constraint
#ifdef NDEBUG
TEST_F(SafeIntervalTableTest, DISABLED_EdgeConstraintTableAddInvalid)
#else
TEST_F(SafeIntervalTableTest, EdgeConstraintTableAddInvalid)
#endif
{
  // expect assert death
  EXPECT_DEATH(table->edge_constraint_table.add(1, 1, 2), "Invalid edge constraint.");
}

