// Result is expected<T, Error>.
// No state.
// Book: none.

#pragma once

#include "starcom/error.hpp"
#include <tl/expected.hpp>

// The library build uses tl::expected.
namespace starcom::ccsds {
template <typename T>
using Result = tl::expected<T, Error>;
}
