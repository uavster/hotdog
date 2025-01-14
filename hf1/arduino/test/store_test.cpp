#include <gtest/gtest.h>
#include "store.h"

TEST(StoreTest, NoElemExistsAfterCreation) {
  Store<int, 3> store;

  EXPECT_FALSE(store.HasElement(0));
  EXPECT_FALSE(store.HasElement(1));
  EXPECT_FALSE(store.HasElement(2));
}

TEST(StoreTest, SetElemIsReturned) {
  Store<int, 3> store;
  ASSERT_FALSE(store.HasElement(0));
  ASSERT_FALSE(store.HasElement(1));
  ASSERT_FALSE(store.HasElement(2));

  store[1] = 52;

  EXPECT_FALSE(store.HasElement(0));
  ASSERT_TRUE(store[1].ok());
  EXPECT_EQ(*store[1], 52);
  EXPECT_FALSE(store.HasElement(2));
}

TEST(StoreTest, ElemDoesNotExistsAfterErase) {
  Store<int, 3> store;

  store[1] = 52;
  ASSERT_TRUE(store[1].ok());
  ASSERT_EQ(*store[1], 52);

  store.Erase(1);

  EXPECT_FALSE(store.HasElement(1));
}

TEST(StoreTest, NoElemsExistAfterEraseAll) {
  Store<int, 3> store;

  store[0] = 52;
  store[1] = 53;
  store[2] = 54;
  ASSERT_TRUE(store[0].ok());
  ASSERT_EQ(*store[0], 52);
  ASSERT_TRUE(store[1].ok());
  ASSERT_EQ(*store[1], 53);
  ASSERT_TRUE(store[2].ok());
  ASSERT_EQ(*store[2], 54);

  store.EraseAll();
  
  EXPECT_FALSE(store.HasElement(0));
  EXPECT_FALSE(store.HasElement(1));
  EXPECT_FALSE(store.HasElement(2));
}
