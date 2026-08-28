#include "heap_trap.hpp"

#include <atomic>
#include <cstdlib>
#include <new>

namespace {
std::atomic<bool> g_armed{false};
std::atomic<int> g_count{0};

void bump() noexcept {
  if (g_armed.load(std::memory_order_relaxed)) {
    g_count.fetch_add(1, std::memory_order_relaxed);
  }
}
}  // namespace

namespace starcom::test {

void heapTrapArm() noexcept { g_armed.store(true, std::memory_order_relaxed); }
void heapTrapDisarm() noexcept { g_armed.store(false, std::memory_order_relaxed); }
void heapTrapReset() noexcept { g_count.store(0, std::memory_order_relaxed); }
int heapTrapCount() noexcept { return g_count.load(std::memory_order_relaxed); }

}  // namespace starcom::test

void* operator new(std::size_t n) {
  bump();
  void* p = std::malloc(n == 0 ? 1 : n);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new[](std::size_t n) { return ::operator new(n); }

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
  bump();
  return std::malloc(n == 0 ? 1 : n);
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
  return ::operator new(n, std::nothrow);
}
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }

#if defined(STARCOM_HEAP_WRAP)
extern "C" {

void* __real_malloc(std::size_t);
void* __real_calloc(std::size_t, std::size_t);
void* __real_realloc(void*, std::size_t);

void* __wrap_malloc(std::size_t n) {
  bump();
  return __real_malloc(n);
}

void* __wrap_calloc(std::size_t nmemb, std::size_t size) {
  bump();
  return __real_calloc(nmemb, size);
}

void* __wrap_realloc(void* p, std::size_t n) {
  bump();
  return __real_realloc(p, n);
}

}
#endif
