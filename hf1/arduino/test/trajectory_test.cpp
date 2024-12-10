#include <gtest/gtest.h>
#include "trajectory.h"

class TestState {
public:
  TestState() : foo_(0) {}
  TestState(int foo) : foo_(foo) {}

private:
  int foo_;
};

TEST(TrajecotoryTest, TrajectoryCreatedWithIntendedCapacity) {
  Trajectory<TestState, /*Capacity=*/2> trajectory;
  EXPECT_EQ(trajectory.capacity(), 2);
}

TEST(TrajecotoryTest, TrajectoryIsCreatedWithZeroSize) {
  Trajectory<TestState, /*Capacity=*/2> trajectory;
  EXPECT_EQ(trajectory.size(), 0);
}

TEST(TrajecotoryTest, InsertKeepsWaypointsOrdered) {
  Trajectory<TestState, /*Capacity=*/10> trajectory;
  trajectory.Insert(Waypoint<TestState>(1.2, TestState()));
  trajectory.Insert(Waypoint<TestState>(0.5, TestState()));
  trajectory.Insert(Waypoint<TestState>(5.8, TestState()));
  trajectory.Insert(Waypoint<TestState>(3.4, TestState()));

  ASSERT_EQ(trajectory.size(), 4);
  EXPECT_EQ(trajectory[0].seconds(), 0.5);
  EXPECT_EQ(trajectory[1].seconds(), 1.2);
  EXPECT_EQ(trajectory[2].seconds(), 3.4);
  EXPECT_EQ(trajectory[3].seconds(), 5.8);
}

TEST(TrajecotoryTest, TrajectoryConstructorKeepsWaypointsOrdered) {
  Trajectory<TestState, /*Capacity=*/10> trajectory({
    Waypoint<TestState>(1.2, TestState()),
    Waypoint<TestState>(0.5, TestState()),
    Waypoint<TestState>(5.8, TestState()),
    Waypoint<TestState>(3.4, TestState())
  });

  ASSERT_EQ(trajectory.size(), 4);
  EXPECT_EQ(trajectory[0].seconds(), 0.5);
  EXPECT_EQ(trajectory[1].seconds(), 1.2);
  EXPECT_EQ(trajectory[2].seconds(), 3.4);
  EXPECT_EQ(trajectory[3].seconds(), 5.8);
}

TEST(TrajecotoryTest, WaypointFoundByTime) {
  Trajectory<TestState, /*Capacity=*/10> trajectory({
    Waypoint<TestState>(1.2, TestState()),
    Waypoint<TestState>(0.5, TestState()),
    Waypoint<TestState>(5.8, TestState()),
    Waypoint<TestState>(3.4, TestState())
  });

  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(0.5), 0);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(1.2), 1);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(3.4), 2);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(5.8), 3);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(0), -1);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(0.6), 0);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(3.3), 1);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(3.5), 2);
  ASSERT_EQ(trajectory.FindWaypointAtOrBeforeSeconds(6), 3);
}
