// Declares the heap-call trap for host tests.
// The count lives in heap_trap.cpp.
// Book: none.

#pragma once

namespace starcom::test {

void heapTrapArm() noexcept;
void heapTrapDisarm() noexcept;
void heapTrapReset() noexcept;
int heapTrapCount() noexcept;

}  // namespace starcom::test
