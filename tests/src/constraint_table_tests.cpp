/*
 * Author: Jan Chleboun
 * Date: 02-04-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include <gtest/gtest.h>

#include <climits>

#include "ConstraintTable.h"
#include "Instance.h"
#include "LNS.h"
#include "SIPP.h"
#include "test_utils.h"

/**
 * @brief Class for testing the ConstraintTable class
 */
class ConstraintTableTest : public ::testing::Test
{
protected:
  std::unique_ptr<Instance>        instance;
  std::unique_ptr<ConstraintTable> table;

  void SetUp() override
  {
    // load instance
    std::string base_path = get_base_path_tests();  // path to my_solver

    instance =
        std::make_unique<Instance>(base_path + "/tests/test_maps/empty_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 4);
    table = std::make_unique<ConstraintTable>(*instance);
  }

  void TearDown() override
  {
  }
};


// Constraint Table tests
// Adding a single constraint and checking if it is stored correctly
TEST_F(ConstraintTableTest, SingleConstraint)
{
  TimePoint tp(7, {3, 5});  // location (1,2) with time interval [3,5]

  table->add_constraint(tp, 0);

  // test that the visiting agents counts are right
  ASSERT_EQ(table->get_agents_counts(7).size(), 1) << "There should be one agent.";
  EXPECT_EQ(table->get_agents_counts(7).begin()->first, 0);
  EXPECT_EQ(table->get_agents_counts(7).begin()->second, 1);

  // test that there is no constraint before
  EXPECT_EQ(table->get_blocking_agent(6, 7, 2), std::make_pair(-1, -1));

  // test that the constraint is stored correctly
  EXPECT_EQ(table->get_blocking_agent(6, 7, 3), std::make_pair(0, -1));
  EXPECT_EQ(table->get_blocking_agent(6, 7, 4), std::make_pair(0, -1));
  EXPECT_EQ(table->get_blocking_agent(6, 7, 5), std::make_pair(0, -1));

  // test that there is no constraint after
  EXPECT_EQ(table->get_blocking_agent(6, 7, 6), std::make_pair(-1, -1));
}

// Adding multiple constraints and ensuring safe intervals are computed correctly
TEST_F(ConstraintTableTest, MultipleSingleConstraints)
{
  table->add_constraint(TimePoint(7, {2, 4}), 0);
  table->add_constraint(TimePoint(7, {5, 5}), 1);
  table->add_constraint(TimePoint(7, {6, 7}), 2);
  table->add_constraint(TimePoint(7, {10, 15}), 3);

  // test that the visiting agents counts are right
  ASSERT_EQ(table->get_agents_counts(7).size(), 4) << "There should be four agents.";
  for (const auto& it : table->get_agents_counts(7))
  {
    // expect that the agent is 0, 1, 2 or 3 and that the count is 1
    EXPECT_TRUE(it.first >= 0 && it.first <= 3) << "Agent should be 0, 1, 2 or 3.";
    EXPECT_EQ(it.second, 1) << "Count should be 1.";
  }

  // test that there is no constraint before
  EXPECT_EQ(table->get_blocking_agent(6, 7, 1), std::make_pair(-1, -1));

  // test that the constraints are stored correctly
  EXPECT_EQ(table->get_blocking_agent(6, 7, 2), std::make_pair(0, -1));
  EXPECT_EQ(table->get_blocking_agent(8, 7, 5), std::make_pair(1, -1));
  EXPECT_EQ(table->get_blocking_agent(6, 7, 6), std::make_pair(2, -1));
  EXPECT_EQ(table->get_blocking_agent(6, 7, 10), std::make_pair(3, -1));
  EXPECT_EQ(table->get_blocking_agent(8, 7, 15), std::make_pair(3, -1));

  // test that there is no constraint after
  EXPECT_EQ(table->get_blocking_agent(6, 7, 16), std::make_pair(-1, -1));
}

// Removing a constraint and ensuring safe intervals update
TEST_F(ConstraintTableTest, RemoveConstraint)
{
  TimePoint tp(0, {1, 2});
  table->add_constraint(tp, 0);
  EXPECT_EQ(table->get_blocking_agent(1, 0, 1), std::make_pair(0, -1));
  EXPECT_EQ(table->get_blocking_agent(3, 0, 2), std::make_pair(0, -1));
  EXPECT_EQ(table->get_agents_counts(0).size(), 1);
  table->remove_constraint(tp, 0);

  // test that the constrained is removed
  EXPECT_EQ(table->get_blocking_agent(1, 0, 1), std::make_pair(-1, -1));
  EXPECT_EQ(table->get_blocking_agent(3, 0, 2), std::make_pair(-1, -1));
  EXPECT_EQ(table->get_agents_counts(0).size(), 0);
}

// Handling constraints dynamically (adding and removing in sequence)
TEST_F(ConstraintTableTest, DynamicAddRemove)
{
  table->add_constraint(TimePoint(4, {2, 4}), 0);
  table->add_constraint(TimePoint(4, {6, 8}), 1);
  // test agent counts before remove
  EXPECT_EQ(table->get_agents_counts(4).size(), 2);

  table->remove_constraint(TimePoint(4, {2, 4}), 0);

  // test agent counts after remove
  ASSERT_EQ(table->get_agents_counts(4).size(), 1);
  EXPECT_EQ(table->get_agents_counts(4).begin()->first, 1);
  EXPECT_EQ(table->get_agents_counts(4).begin()->second, 1);

  // test that the first constraint is removed
  EXPECT_EQ(table->get_blocking_agent(3, 4, 3), std::make_pair(-1, -1));

  // test that the second constraint stays
  EXPECT_EQ(table->get_blocking_agent(3, 4, 6), std::make_pair(1, -1));
}

// Constraint removal from empty table if in debug mode
#ifdef NDEBUG
TEST_F(ConstraintTableTest, DISABLED_RemoveFromEmptyTable)
#else
TEST_F(ConstraintTableTest, RemoveFromEmptyTable)
#endif
{
  TimePoint tp(4, {3, 6});
  // expect assert death
  EXPECT_DEATH(table->remove_constraint(tp, 0), "Trying to remove interval from an empty list.");
}

// Adding a constraint that is already present (should cause assert death)
#ifdef NDEBUG
TEST_F(ConstraintTableTest, DISABLED_AddExistingConstraint)
#else
TEST_F(ConstraintTableTest, AddExistingConstraint)
#endif
{
  TimePoint tp(4, {3, 6});
  table->add_constraint(tp, 0);
  ASSERT_DEATH(table->add_constraint(tp, 1), "Cannot add overlapping constraints.");
}

// Adding a path that forms constraints, test whether edge constraints are added
TEST_F(ConstraintTableTest, AddPathConstraints)
{
  std::vector<TimePoint> path1 = {TimePoint(4, {0, 2}), TimePoint(7, {3, 4}), TimePoint(8, {5, 6})};
  table->add_constraints(path1, 1);
  std::vector<TimePoint> path2 = {TimePoint(5, {0, 5}), TimePoint(4, {6, 8}), TimePoint(7, {9, 10})};
  table->add_constraints(path2, 2);

  // test first edge conflict
  EXPECT_EQ(table->get_blocking_agent(7, 4, 2), std::make_pair(1, -1));                                      // regular conflict
  EXPECT_EQ(table->get_blocking_agent(7, 4, 3), std::make_pair(-1, 1)) << "There should be edge conflict.";  // edge conflict

  // test going to 4, from a non constrained position
  EXPECT_EQ(table->get_blocking_agent(1, 4, 3), std::make_pair(-1, -1)) << "There should be no conflict.";
  EXPECT_EQ(table->get_blocking_agent(1, 4, 9), std::make_pair(-1, -1)) << "There should be no conflict.";

  // test second edge conflict
  EXPECT_EQ(table->get_blocking_agent(7, 4, 5), std::make_pair(-1, -1));
  EXPECT_EQ(table->get_blocking_agent(7, 4, 6), std::make_pair(2, -1));  // regular conflict
  EXPECT_EQ(table->get_blocking_agent(7, 4, 9), std::make_pair(-1, 2));  // edge  conflict
}

// check that the edge conflict is resolved correctly when there are more constraints in the list
TEST_F(ConstraintTableTest, EdgeConflictMoreConstraints)
{
  std::vector<TimePoint> path1 = {TimePoint(4, {0, 1}), TimePoint(7, {2, 4}), TimePoint(4, {5, 6}), TimePoint(7, {7, 8}),
                                  TimePoint(4, {9, 10})};
  table->add_constraints(path1, 1);

  EXPECT_EQ(table->get_blocking_agent(7, 4, 2), std::make_pair(-1, 1)) << "There should be edge conflict.";
  EXPECT_EQ(table->get_blocking_agent(7, 4, 7), std::make_pair(-1, 1)) << "There should be edge conflict.";

  // test agent counts
  ASSERT_EQ(table->get_agents_counts(4).size(), 1) << "There should be one agent.";
  EXPECT_EQ(table->get_agents_counts(4).begin()->first, 1) << "There should be 3 counts for agent 1.";
  EXPECT_EQ(table->get_agents_counts(4).begin()->second, 3) << "There should be 3 counts for agent 1.";
}

// test edge constraint table
// add to edge constraint table
TEST_F(ConstraintTableTest, EdgeConstraintTableAdd)
{
  // add a constraint to the table
  table->edge_constraint_table.add(4, 7, 3, 2);

  // check that the constraint is in the table
  EXPECT_EQ(table->edge_constraint_table.get(4, 7, 3), 2);

  // check that the other times are not there
  EXPECT_EQ(table->edge_constraint_table.get(4, 7, 2), -1);
  EXPECT_EQ(table->edge_constraint_table.get(4, 7, 4), -1);
}

// remove from edge constraint table
TEST_F(ConstraintTableTest, EdgeConstraintTableRemove)
{
  // add a constraint to the table
  table->edge_constraint_table.add(1, 2, 3, 3);

  EXPECT_EQ(table->edge_constraint_table.get(1, 2, 3), 3);

  // remove the constraint
  table->edge_constraint_table.remove(1, 2, 3);

  // check that the constraint is not in the table
  EXPECT_EQ(table->edge_constraint_table.get(1, 2, 3), -1);
}

// add an existing edge constraint
#ifdef NDEBUG
TEST_F(ConstraintTableTest, DISABLED_EdgeConstraintTableAddExisting)
#else
TEST_F(ConstraintTableTest, EdgeConstraintTableAddExisting)
#endif
{
  // add a constraint to the table
  table->edge_constraint_table.add(1, 2, 3, 0);

  // expect assert death
  EXPECT_DEATH(table->edge_constraint_table.add(1, 2, 3, 1), "Edge constraint already exists.");
}

// add an invalid edge constraint
#ifdef NDEBUG
TEST_F(ConstraintTableTest, DISABLED_EdgeConstraintTableAddInvalid)
#else
TEST_F(ConstraintTableTest, EdgeConstraintTableAddInvalid)
#endif
{
  // expect assert death
  EXPECT_DEATH(table->edge_constraint_table.add(1, 1, 2, 0), "Invalid edge constraint.");
}
