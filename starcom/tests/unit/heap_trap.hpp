// Declares the heap-call trap for host tests.
// The count lives in heap_trap.cpp.
// Book: none.

#ifndef STARCOM_TESTS_UNIT_HEAP_TRAP_H_
#define STARCOM_TESTS_UNIT_HEAP_TRAP_H_

namespace starcom::test {

void heapTrapArm() noexcept;
void heapTrapDisarm() noexcept;
void heapTrapReset() noexcept;
int heapTrapCount() noexcept;

}  // namespace starcom::test

#endif  // STARCOM_TESTS_UNIT_HEAP_TRAP_H_
