#pragma once

#include "starcom/error.hpp"

#ifdef STARCOM_USE_STD_EXPECTED
#include <expected>
namespace starcom::ccsds {
template <typename T>
using Result = std::expected<T, Error>;
}
#else
#include <tl/expected.hpp>
namespace starcom::ccsds {
template <typename T>
using Result = tl::expected<T, Error>;
}
#endif
