#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

class JobSystem {
public:
  explicit JobSystem(unsigned threadCount = 0);
  ~JobSystem();

  JobSystem(const JobSystem&) = delete;
  JobSystem& operator=(const JobSystem&) = delete;
  JobSystem(JobSystem&&) = delete;
  JobSystem& operator=(JobSystem&&) = delete;

  unsigned threadCount() const;

  using JobFunction = void (*)(void* context, std::size_t begin, std::size_t end);
  void parallelFor(std::size_t count, std::size_t grainSize, JobFunction function, void* context);

  template <typename Fn> void parallelFor(std::size_t count, std::size_t grainSize, Fn&& function) {

    auto callable = [&function](std::size_t begin, std::size_t end) { function(begin, end); };
    auto trampoline = [](void* context, std::size_t begin, std::size_t end) {
      (*static_cast<decltype(callable)*>(context))(begin, end);
    };
    parallelFor(count, grainSize, trampoline, &callable);
  }

private:
  friend class JobSystemTestAccess;
  using ThreadFactory = std::thread (*)(void (*)(void*), void*);
  JobSystem(unsigned threadCount, ThreadFactory factory);
  void stopAndJoin();
  void workerLoop();
  void runChunks();

  std::vector<std::thread> m_workers;

  mutable std::mutex m_mutex;
  std::condition_variable m_wake;
  std::condition_variable m_finished;

  JobFunction m_function = nullptr;
  void* m_context = nullptr;
  std::size_t m_count = 0;
  std::size_t m_grainSize = 1;
  std::size_t m_chunkCount = 0;
  std::atomic<std::size_t> m_nextChunk{0};
  std::atomic<bool> m_submitting{false};
  std::atomic<bool> m_cancelled{false};
  std::exception_ptr m_exception;

  std::size_t m_runningWorkers = 0;
  std::uint64_t m_generation = 0;
  bool m_stopping = false;
};
