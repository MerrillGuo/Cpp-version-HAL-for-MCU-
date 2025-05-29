#include "test_ring_buffer.h"

#include "ring_buffer.h"
#include "unity_fixture.h"

#include <array>

using namespace hcl;

static RingBuffer<uint8_t, 8> buffer;

TEST_GROUP(TestRingBuffer);

TEST_SETUP(TestRingBuffer) {
  buffer.clear();
}

TEST_TEAR_DOWN(TestRingBuffer) {}

// 定义所有测试用例
#define TEST_CASE(name) TEST(TestRingBuffer, name)

TEST_CASE(InitialState) {
  TEST_ASSERT_TRUE(buffer.empty());
  TEST_ASSERT_FALSE(buffer.is_full());
  TEST_ASSERT_EQUAL(0, buffer.used());
  TEST_ASSERT_EQUAL(8, buffer.capacity());
  TEST_ASSERT_EQUAL(8, buffer.available());
}

TEST_CASE(SinglePushPop) {
  uint8_t data = 0x42;
  uint8_t received;

  TEST_ASSERT_TRUE(buffer.push(data));
  TEST_ASSERT_EQUAL(1, buffer.used());
  TEST_ASSERT_FALSE(buffer.empty());

  TEST_ASSERT_TRUE(buffer.pop(received));
  TEST_ASSERT_EQUAL(data, received);
  TEST_ASSERT_TRUE(buffer.empty());
}

TEST_CASE(BufferFull) {
  for (uint8_t i = 0; i < 8; i++) {
    TEST_ASSERT_TRUE(buffer.push(i));
  }

  TEST_ASSERT_TRUE(buffer.is_full());
  TEST_ASSERT_FALSE(buffer.push(0xFF));  // 应该失败
}

TEST_CASE(BulkPushPop) {
  std::array<uint8_t, 4> write_data = {1, 2, 3, 4};
  std::array<uint8_t, 4> read_data = {0};

  TEST_ASSERT_TRUE(buffer.push(std::span(write_data)));
  TEST_ASSERT_EQUAL(4, buffer.used());

  auto read_count = buffer.pop(std::span(read_data));
  TEST_ASSERT_EQUAL(4, read_count);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(write_data.data(), read_data.data(), 4);

  // 测试边界情况 - 尝试写入过大的数据
  std::array<uint8_t, 10> too_large = {0};
  TEST_ASSERT_FALSE(buffer.push(std::span(too_large)));

  // 测试部分读取
  std::array<uint8_t, 4> write_data2 = {5, 6, 7, 8};
  buffer.push(std::span(write_data2));

  std::array<uint8_t, 2> partial_read = {0};
  auto count = buffer.pop(std::span(partial_read));
  TEST_ASSERT_EQUAL(2, count);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(write_data2.data(), partial_read.data(), 2);
  TEST_ASSERT_EQUAL(2, buffer.used());  // 验证剩余数据量
}

TEST_CASE(WrapAround) {
  // 先填充6个字节
  for (uint8_t i = 0; i < 6; i++) {
    buffer.push(i);
  }

  // 读出4个字节
  std::array<uint8_t, 4> temp = {0};
  buffer.pop(std::span(temp));

  TEST_ASSERT_EQUAL(2, buffer.used());       // 验证剩余2字节
  TEST_ASSERT_EQUAL(6, buffer.available());  // 验证可用空间

  // 写入刚好填满到buffer末尾的数据
  std::array<uint8_t, 6> exact_wrap = {0xA, 0xB, 0xC, 0xD, 0xE, 0xF};
  TEST_ASSERT_TRUE(buffer.push(std::span(exact_wrap)));
  TEST_ASSERT_TRUE(buffer.is_full());

  // 验证所有数据
  std::array<uint8_t, 8> all_data = {0};
  auto count = buffer.pop(std::span(all_data));
  TEST_ASSERT_EQUAL(8, count);
  // 验证前2个字节（原有数据）
  TEST_ASSERT_EQUAL_UINT8(4, all_data[0]);
  TEST_ASSERT_EQUAL_UINT8(5, all_data[1]);
  // 验证环绕后的6个字节
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_EQUAL_UINT8(exact_wrap[i], all_data[i + 2]);
  }
}

TEST_CASE(Peek) {
  // 测试空buffer peek
  std::array<uint8_t, 1> empty_peek = {0};
  TEST_ASSERT_EQUAL(0, buffer.peek(std::span(empty_peek)));

  std::array<uint8_t, 3> write_data = {1, 2, 3};
  std::array<uint8_t, 3> peek_data = {0};

  buffer.push(std::span(write_data));

  // 第一次peek
  auto count = buffer.peek(std::span(peek_data));
  TEST_ASSERT_EQUAL(3, count);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(write_data.data(), peek_data.data(), 3);
  TEST_ASSERT_EQUAL(3, buffer.used());  // peek不应改变buffer状态

  // 再次peek，确保数据仍然存在
  peek_data.fill(0);
  count = buffer.peek(std::span(peek_data));
  TEST_ASSERT_EQUAL(3, count);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(write_data.data(), peek_data.data(), 3);

  // 测试部分peek
  std::array<uint8_t, 2> partial_peek = {0};
  count = buffer.peek(std::span(partial_peek));
  TEST_ASSERT_EQUAL(2, count);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(write_data.data(), partial_peek.data(), 2);

  // 测试环绕情况的peek
  buffer.clear();
  // 填充使tail_到达末尾
  for (uint8_t i = 0; i < 6; i++) {
    buffer.push(i);
  }
  // 读出4个，使head_在中间
  std::array<uint8_t, 4> temp = {0};
  buffer.pop(std::span(temp));
  // 写入跨越末尾的数据
  std::array<uint8_t, 3> wrap_data = {0xA, 0xB, 0xC};
  buffer.push(std::span(wrap_data));

  // peek所有数据
  std::array<uint8_t, 5> wrap_peek = {0};
  count = buffer.peek(std::span(wrap_peek));
  TEST_ASSERT_EQUAL(5, count);
  TEST_ASSERT_EQUAL_UINT8(4, wrap_peek[0]);  // 原有数据
  TEST_ASSERT_EQUAL_UINT8(5, wrap_peek[1]);
  TEST_ASSERT_EQUAL_UINT8(0xA, wrap_peek[2]);  // 环绕数据
  TEST_ASSERT_EQUAL_UINT8(0xB, wrap_peek[3]);
  TEST_ASSERT_EQUAL_UINT8(0xC, wrap_peek[4]);
}

TEST_CASE(Clear) {
  std::array<uint8_t, 3> data = {1, 2, 3};
  buffer.push(std::span(data));

  buffer.clear();
  TEST_ASSERT_TRUE(buffer.empty());
  TEST_ASSERT_EQUAL(0, buffer.used());
  TEST_ASSERT_EQUAL(8, buffer.available());
}

TEST_CASE(ErrorCases) {
  // 测试从空buffer中pop
  uint8_t data;
  TEST_ASSERT_FALSE(buffer.pop(data));
  
  // 测试空span的push/pop
  std::array<uint8_t, 0> empty_array;
  TEST_ASSERT_TRUE(buffer.push(std::span(empty_array)));
  TEST_ASSERT_EQUAL(0, buffer.pop(std::span(empty_array)));
  
  // 测试nullptr的情况
  TEST_ASSERT_EQUAL(0, buffer.peek(std::span<uint8_t>()));
}

TEST_CASE(ContinuousOperations) {
  // 反复填充和清空buffer
  for (int i = 0; i < 3; i++) {
    // 填满
    for (uint8_t j = 0; j < 8; j++) {
      TEST_ASSERT_TRUE(buffer.push(j));
    }
    TEST_ASSERT_TRUE(buffer.is_full());
    
    // 部分读出
    std::array<uint8_t, 4> temp;
    TEST_ASSERT_EQUAL(4, buffer.pop(std::span(temp)));
    
    // 再次写入
    for (uint8_t j = 0; j < 4; j++) {
      TEST_ASSERT_TRUE(buffer.push(j));
    }
    
    // 完全清空
    buffer.clear();
    TEST_ASSERT_TRUE(buffer.empty());
  }
}

TEST_CASE(BoundaryValues) {
  // 测试最大值写入
  TEST_ASSERT_TRUE(buffer.push(UINT8_MAX));
  uint8_t data;
  TEST_ASSERT_TRUE(buffer.pop(data));
  TEST_ASSERT_EQUAL(UINT8_MAX, data);
  
  // 测试在buffer即将满时的操作
  for (uint8_t i = 0; i < 7; i++) {
    TEST_ASSERT_TRUE(buffer.push(i));
  }
  TEST_ASSERT_EQUAL(1, buffer.available());
  TEST_ASSERT_TRUE(buffer.push(0));  // 最后一个空位
  TEST_ASSERT_FALSE(buffer.push(0)); // 已满
}

TEST_GROUP_RUNNER(TestRingBuffer) {
    RUN_TEST_CASE(TestRingBuffer, InitialState);
    RUN_TEST_CASE(TestRingBuffer, SinglePushPop);
    RUN_TEST_CASE(TestRingBuffer, BufferFull);
    RUN_TEST_CASE(TestRingBuffer, BulkPushPop);
    RUN_TEST_CASE(TestRingBuffer, WrapAround);
    RUN_TEST_CASE(TestRingBuffer, Peek);
    RUN_TEST_CASE(TestRingBuffer, Clear);
    RUN_TEST_CASE(TestRingBuffer, ErrorCases);
    RUN_TEST_CASE(TestRingBuffer, ContinuousOperations);
    RUN_TEST_CASE(TestRingBuffer, BoundaryValues);
}

void RunAllTestsForTestRingBuffer(void) {
    RUN_TEST_GROUP(TestRingBuffer);
}
