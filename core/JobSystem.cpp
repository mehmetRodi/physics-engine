#include "core/JobSystem.hpp"

#include <algorithm>
#include <stdexcept>

JobSystem::JobSystem(unsigned threadCount)
    : JobSystem(threadCount,
                [](void (*entry)(void*), void* context) { return std::thread(entry, context); }) {}

JobSystem::JobSystem(unsigned threadCount, ThreadFactory factory) {
  if (threadCount == 0) {
    const unsigned hardware = std::thread::hardware_concurrency();
    threadCount = hardware == 0 ? 1u : hardware;
  }

  m_workers.reserve(threadCount - 1);
  try {
    for (unsigned i = 1; i < threadCount; ++i) {
      m_workers.push_back(
          factory([](void* context) { static_cast<JobSystem*>(context)->workerLoop(); }, this));
    }
  } catch (...) {
    stopAndJoin();
    throw;
  }
}

JobSystem::~JobSystem() {
  stopAndJoin();
}

void JobSystem::stopAndJoin() {
  {
    const std::lock_guard<std::mutex> lock(m_mutex);
    m_stopping = true;
  }

  m_wake.notify_all();

  for (std::thread& worker : m_workers) {
    worker.join();
  }
}

unsigned JobSystem::threadCount() const {
  return static_cast<unsigned>(m_workers.size()) + 1u;
}

void JobSystem::parallelFor(std::size_t count, std::size_t grainSize, JobFunction function,
                            void* context) {
  if (m_submitting.exchange(true, std::memory_order_acquire))
    throw std::logic_error("concurrent or recursive JobSystem submission");
  struct SubmissionGuard {
    std::atomic<bool>& flag;
    ~SubmissionGuard() {
      flag.store(false, std::memory_order_release);
    }
  } guard{m_submitting};
  if (count == 0) {
    return;
  }
  if (function == nullptr)
    throw std::invalid_argument("null job function");

  grainSize = std::max<std::size_t>(grainSize, 1);

  if (m_workers.empty() || count <= grainSize) {
    function(context, 0, count);
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(m_mutex);
    m_function = function;
    m_context = context;
    m_count = count;
    m_grainSize = grainSize;
    m_chunkCount = count / grainSize + (count % grainSize != 0);
    m_nextChunk.store(0, std::memory_order_relaxed);
    m_cancelled.store(false, std::memory_order_relaxed);
    m_exception = nullptr;
    m_runningWorkers = m_workers.size();
    ++m_generation;
  }

  m_wake.notify_all();

  runChunks();

  std::unique_lock<std::mutex> lock(m_mutex);
  m_finished.wait(lock, [this] { return m_runningWorkers == 0; });
  m_function = nullptr;
  m_context = nullptr;
  if (m_exception)
    std::rethrow_exception(m_exception);
}

void JobSystem::runChunks() {
  for (;;) {
    if (m_cancelled.load(std::memory_order_relaxed))
      return;
    const std::size_t chunk = m_nextChunk.fetch_add(1, std::memory_order_relaxed);

    if (chunk >= m_chunkCount) {
      return;
    }

    const std::size_t begin = chunk * m_grainSize;
    const std::size_t end = begin + std::min(m_grainSize, m_count - begin);
    try {
      m_function(m_context, begin, end);
    } catch (...) {
      const std::lock_guard<std::mutex> lock(m_mutex);
      if (!m_exception)
        m_exception = std::current_exception();
      m_cancelled.store(true, std::memory_order_relaxed);
      return;
    }
  }
}

void JobSystem::workerLoop() {
  std::uint64_t seenGeneration = 0;

  for (;;) {
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_wake.wait(lock,
                  [this, seenGeneration] { return m_stopping || m_generation != seenGeneration; });

      if (m_stopping) {
        return;
      }

      seenGeneration = m_generation;
    }

    runChunks();

    {
      const std::lock_guard<std::mutex> lock(m_mutex);
      --m_runningWorkers;
    }

    m_finished.notify_one();
  }
}
