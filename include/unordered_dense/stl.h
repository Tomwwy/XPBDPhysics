// STL include block for the module-split build of ankerl::unordered_dense.
//
// The vendored unordered_dense.h is the version that factors its standard
// library includes out into this sibling header (it does `#include "stl.h"`
// when not built as a C++20 module). This file supplies exactly that set of
// standard headers. Kept separate so the vendored single-header file stays
// byte-for-byte upstream.
#pragma once

#include <array>            // for array
#include <cstdint>          // for uint64_t, uint32_t, uint8_t, UINT64_C
#include <cstring>          // for size_t, memcpy, memset
#include <functional>       // for equal_to, hash
#include <initializer_list> // for initializer_list
#include <iterator>         // for pair, distance
#include <limits>           // for numeric_limits
#include <memory>           // for allocator, allocator_traits, shared_ptr
#include <optional>         // for optional
#include <stdexcept>        // for out_of_range
#include <string>           // for basic_string
#include <string_view>      // for basic_string_view, hash
#include <tuple>            // for forward_as_tuple
#include <type_traits>      // for enable_if_t, declval, conditional_t, ...
#include <utility>          // for forward, exchange, pair, as_const, piece...
#include <vector>           // for vector
