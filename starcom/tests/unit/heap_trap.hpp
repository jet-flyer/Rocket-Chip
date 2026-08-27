#pragma once

namespace starcom::test {

void heap_trap_arm() noexcept;
void heap_trap_disarm() noexcept;
void heap_trap_reset() noexcept;
int heap_trap_count() noexcept;

}  // namespace starcom::test
