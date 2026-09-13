#include "core/JobSystem.hpp"
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <numeric>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

TEST(JobSystemTests, EveryIndexIsVisitedExactlyOnce) {
  JobSystem jobs(4);
  std::vector<int> visits(10000, 0);

  jobs.parallelFor(visits.size(), 64, [&visits](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      visits[i] += 1;
    }
  });

  for (const int count : visits) {
    ASSERT_EQ(count, 1);
  }
}

TEST(JobSystemTests, ChunksCoverTheRangeWithoutOverlapping) {
  JobSystem jobs(4);
  std::mutex mutex;
  std::vector<std::pair<std::size_t, std::size_t>> chunks;

  jobs.parallelFor(1000, 64, [&](std::size_t begin, std::size_t end) {
    const std::lock_guard<std::mutex> lock(mutex);
    chunks.emplace_back(begin, end);
  });

  std::sort(chunks.begin(), chunks.end());

  std::size_t expectedBegin = 0;
  for (const auto& chunk : chunks) {
    EXPECT_EQ(chunk.first, expectedBegin);
    EXPECT_LE(chunk.second - chunk.first, 64u);
    expectedBegin = chunk.second;
  }

  EXPECT_EQ(expectedBegin, 1000u);
}

TEST(JobSystemTests, ParallelForBlocksUntilEveryChunkHasFinished) {
  JobSystem jobs(4);

  for (int attempt = 0; attempt < 50; ++attempt) {
    std::atomic<int> completed{0};

    jobs.parallelFor(256, 8, [&completed](std::size_t begin, std::size_t end) {
      for (std::size_t i = begin; i < end; ++i) {
        completed.fetch_add(1, std::memory_order_relaxed);
      }
    });

    ASSERT_EQ(completed.load(std::memory_order_relaxed), 256);
  }
}

TEST(JobSystemTests, WorkIsSpreadOverMoreThanOneThread) {
  JobSystem jobs(4);
  ASSERT_EQ(jobs.threadCount(), 4u);

  std::mutex mutex;
  std::set<std::thread::id> threads;

  jobs.parallelFor(4096, 1, [&](std::size_t begin, std::size_t end) {
    volatile double sink = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
      for (int k = 0; k < 200; ++k) {
        sink = sink + 1.0;
      }
    }

    const std::lock_guard<std::mutex> lock(mutex);
    threads.insert(std::this_thread::get_id());
  });

  EXPECT_GT(threads.size(), 1u);
}

TEST(JobSystemTests, SingleThreadedPoolRunsInline) {
  JobSystem jobs(1);
  EXPECT_EQ(jobs.threadCount(), 1u);

  const std::thread::id caller = std::this_thread::get_id();
  std::thread::id observed;

  jobs.parallelFor(
      100, 1, [&observed](std::size_t, std::size_t) { observed = std::this_thread::get_id(); });

  EXPECT_EQ(observed, caller);
}

TEST(JobSystemTests, EmptyRangeRunsNothing) {
  JobSystem jobs(4);
  int calls = 0;

  jobs.parallelFor(0, 8, [&calls](std::size_t, std::size_t) { calls += 1; });

  EXPECT_EQ(calls, 0);
}

TEST(JobSystemTests, RangeSmallerThanTheGrainRunsAsOneChunk) {
  JobSystem jobs(4);
  int calls = 0;

  jobs.parallelFor(5, 64, [&calls](std::size_t begin, std::size_t end) {
    calls += 1;
    EXPECT_EQ(begin, 0u);
    EXPECT_EQ(end, 5u);
  });

  EXPECT_EQ(calls, 1);
}

TEST(JobSystemTests, ThePoolCanBeReusedManyTimes) {
  JobSystem jobs(4);
  std::vector<int> values(500, 0);

  for (int round = 0; round < 200; ++round) {
    jobs.parallelFor(values.size(), 32, [&values](std::size_t begin, std::size_t end) {
      for (std::size_t i = begin; i < end; ++i) {
        values[i] += 1;
      }
    });
  }

  for (const int value : values) {
    ASSERT_EQ(value, 200);
  }
}

namespace {
int createdThreads = 0;
std::thread failAfterOneThread(void (*entry)(void*), void* context) {
  if (createdThreads++ == 1)
    throw std::runtime_error("injected thread creation failure");
  return std::thread(entry, context);
}
} // namespace
class JobSystemTestAccess {
public:
  static void failConstruction() {
    JobSystem jobs(4, failAfterOneThread);
  }
};

TEST(JobSystemTests, PartialConstructionJoinsStartedThreadsBeforeRethrowing) {
  createdThreads = 0;
  EXPECT_THROW(JobSystemTestAccess::failConstruction(), std::runtime_error);
  EXPECT_EQ(createdThreads, 2);
}

TEST(JobSystemTests, WorkerExceptionReachesCallerAndPoolRemainsUsable) {
  JobSystem jobs(4);
  const auto caller = std::this_thread::get_id();
  EXPECT_THROW(jobs.parallelFor(10000, 1,
                                [&](std::size_t, std::size_t) {
                                  if (std::this_thread::get_id() != caller)
                                    throw std::runtime_error("worker failure");
                                  std::this_thread::sleep_for(std::chrono::microseconds(100));
                                }),
               std::runtime_error);
  std::atomic<int> visits{0};
  jobs.parallelFor(100, 1, [&](std::size_t begin, std::size_t end) {
    visits.fetch_add(static_cast<int>(end - begin));
  });
  EXPECT_EQ(visits.load(), 100);
}

TEST(JobSystemTests, RecursiveAndConcurrentSubmissionsAreRejected) {
  JobSystem jobs(4);
  EXPECT_THROW(jobs.parallelFor(100, 1,
                                [&](std::size_t, std::size_t) {
                                  jobs.parallelFor(1, 1, [](std::size_t, std::size_t) {});
                                }),
               std::logic_error);
  std::atomic<bool> entered{false}, release{false};
  std::thread submitter([&] {
    jobs.parallelFor(1, 1, [&](std::size_t, std::size_t) {
      entered.store(true);
      while (!release.load())
        std::this_thread::yield();
    });
  });
  while (!entered.load())
    std::this_thread::yield();
  EXPECT_THROW(jobs.parallelFor(1, 1, [](std::size_t, std::size_t) {}), std::logic_error);
  release.store(true);
  submitter.join();
}
