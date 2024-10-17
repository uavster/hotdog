#include <gtest/gtest.h>
#include "ring_buffer.h"

TEST(RingBufferTest, CapacityReturnsRightValue) {
  RingBuffer<int, /*kCapacity=*/6> buffer;

  EXPECT_EQ(buffer.Capacity(), 6);
}

TEST(RingBufferTest, BufferStarstWithZeroSize) {
  RingBuffer<int, /*kCapacity=*/6> buffer;

  EXPECT_EQ(buffer.Size(), 0);
}

TEST(RingBufferTest, OldestValueReturnsNullIfEmpty) {
  RingBuffer<int, /*kCapacity=*/2> buffer;

  EXPECT_EQ(buffer.OldestValue(), nullptr);
}

TEST(RingBufferTest, CommitIncreasesSize) {
  RingBuffer<int, /*kCapacity=*/6> buffer;
  buffer.Commit();
  EXPECT_EQ(buffer.Size(), 1);

  buffer.Commit();
  EXPECT_EQ(buffer.Size(), 2);
}

TEST(RingBufferTest, CommitIncreasesSizeUpToCapacity) {
  RingBuffer<int, /*kCapacity=*/3> buffer;
  buffer.Commit();
  buffer.Commit();
  buffer.Commit();
  buffer.Commit();
  EXPECT_EQ(buffer.Size(), 2);
}

TEST(RingBufferTest, OldestValueMatchesWrittenValue) {
  RingBuffer<int, /*kCapacity=*/6> buffer;
  buffer.NewValue() = 52;
  buffer.Commit();
  ASSERT_NE(buffer.OldestValue(), nullptr);

  EXPECT_EQ(*buffer.OldestValue(), 52);
}

TEST(RingBufferTest, NewValueRollsOverCapacity) {
  RingBuffer<int, /*kCapacity=*/2> buffer;
  buffer.NewValue() = 52;
  buffer.Commit();
  buffer.NewValue() = 53;
  buffer.Commit();
  ASSERT_NE(buffer.OldestValue(), nullptr);

  EXPECT_EQ(*buffer.OldestValue(), 53);
}

TEST(RingBufferTest, ConsumeReducesSize) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Commit();
  buffer.Commit();
  buffer.Consume();

  EXPECT_EQ(buffer.Size(), 1);
}

TEST(RingBufferTest, ConsumeChangesOldestValue) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.NewValue() = 52;
  buffer.Commit();
  buffer.NewValue() = 53;
  buffer.Commit();
  ASSERT_NE(buffer.OldestValue(), nullptr);
  ASSERT_EQ(*buffer.OldestValue(), 52);

  buffer.Consume();
  EXPECT_EQ(*buffer.OldestValue(), 53);
}

TEST(RingBufferTest, ReadOnEmptyBufferDies) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  
  EXPECT_DEATH(buffer.Read(), "");
}

TEST(RingBufferTest, ReadReturnsWrittenValue) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  
  EXPECT_EQ(buffer.Read(), 52);
}

TEST(RingBufferTest, ReadConsumesValue) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  ASSERT_EQ(buffer.Size(), 1);
  ASSERT_EQ(buffer.Read(), 52);
  
  EXPECT_EQ(buffer.Size(), 0);
}

TEST(RingBufferTest, OldestValueWithIndexPointsAtCorrectValueAndDoesNotConsume) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  ASSERT_NE(buffer.OldestValue(0), nullptr);
  ASSERT_NE(buffer.OldestValue(1), nullptr);
  ASSERT_NE(buffer.OldestValue(2), nullptr);

  EXPECT_EQ(*buffer.OldestValue(0), 52);
  EXPECT_EQ(*buffer.OldestValue(1), 53);
  EXPECT_EQ(*buffer.OldestValue(2), 54);
  EXPECT_EQ(buffer.Size(), 3);
}

TEST(RingBufferTest, ConsumeWithIndexExtractsMiddleValue) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  buffer.Consume(1);
  ASSERT_NE(buffer.OldestValue(0), nullptr);
  ASSERT_NE(buffer.OldestValue(1), nullptr);

  EXPECT_EQ(buffer.Size(), 2);
  EXPECT_EQ(*buffer.OldestValue(0), 52);
  EXPECT_EQ(*buffer.OldestValue(1), 54);
}

TEST(RingBufferTest, ConsumeWithIndexExtractsFirstValue) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  buffer.Consume(0);
  ASSERT_NE(buffer.OldestValue(0), nullptr);
  ASSERT_NE(buffer.OldestValue(1), nullptr);

  EXPECT_EQ(buffer.Size(), 2);
  EXPECT_EQ(*buffer.OldestValue(0), 53);
  EXPECT_EQ(*buffer.OldestValue(1), 54);
}

TEST(RingBufferTest, ConsumeWithIndexExtractsLastValue) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  buffer.Consume(2);
  ASSERT_NE(buffer.OldestValue(0), nullptr);
  ASSERT_NE(buffer.OldestValue(1), nullptr);

  EXPECT_EQ(buffer.Size(), 2);
  EXPECT_EQ(*buffer.OldestValue(0), 52);
  EXPECT_EQ(*buffer.OldestValue(1), 53);
}

TEST(RingBufferTest, WriteKeepsOrderAfterConsumeWithIndex) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  buffer.Consume(1);
  buffer.Write(27);
  ASSERT_NE(buffer.OldestValue(0), nullptr);
  ASSERT_NE(buffer.OldestValue(1), nullptr);
  ASSERT_NE(buffer.OldestValue(2), nullptr);

  EXPECT_EQ(buffer.Size(), 3);
  EXPECT_EQ(*buffer.OldestValue(0), 52);
  EXPECT_EQ(*buffer.OldestValue(1), 54);
  EXPECT_EQ(*buffer.OldestValue(2), 27);
}

TEST(RingBufferTest, SizeIsZeroAfterClear) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  ASSERT_EQ(buffer.Size(), 3);

  buffer.Clear();
  EXPECT_EQ(buffer.Size(), 0);
}

TEST(RingBuffer, SizeReturnsZeroAfterClear) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  ASSERT_EQ(buffer.Size(), 3);

  buffer.Clear();
  EXPECT_EQ(buffer.Size(), 0);
}

TEST(RingBuffer, IsFullReturnsFalseIfEmpty) {
  RingBuffer<int, /*kCapacity=*/4> buffer;

  EXPECT_FALSE(buffer.IsFull());
}

TEST(RingBuffer, IsFullReturnsFalseIfNotFull) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);

  EXPECT_FALSE(buffer.IsFull());
}

TEST(RingBuffer, IsFullReturnsTrueIfAtCapacity) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);

  EXPECT_TRUE(buffer.IsFull());
}

TEST(RingBuffer, IsFullReturnsTrueAfterRollover) {
  RingBuffer<int, /*kCapacity=*/4> buffer;
  buffer.Write(52);
  buffer.Write(53);
  buffer.Write(54);
  buffer.Write(55);

  EXPECT_TRUE(buffer.IsFull());
}
