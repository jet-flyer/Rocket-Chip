// Result is expected<T, Error>.
// No state.
// Book: none.

#ifndef STARCOM_RESULT_H_
#define STARCOM_RESULT_H_

#include "starcom/error.hpp"
#include <tl/expected.hpp>

// The library build uses tl::expected.
namespace starcom::ccsds {
template <typename T>
using Result = tl::expected<T, Error>;
}

#endif  // STARCOM_RESULT_H_
