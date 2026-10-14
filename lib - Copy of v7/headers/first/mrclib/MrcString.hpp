// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcString.h"

#include <cstring>
#include <string_view>

namespace mrclib {
/** Converts a MRC_String to a string_view */
constexpr std::string_view to_string_view(const struct MRC_String* str) {
    if (str && str->str) {
        return {str->str, str->len};
    } else {
        return "";
    }
}

/** Converts a string_view to a MRC_String */
constexpr MRC_String make_string(std::string_view view) {
    return MRC_String{view.data(), view.size()};
}

/** Allocates a copy of a string_view and stores the result into a MRC_String */
inline MRC_String alloc_mrc_string(std::string_view view) {
    MRC_String out;
    size_t len = view.size();
    std::memcpy(MRC_AllocateString(&out, len), view.data(), len);
    return out;
}

/** Allocates a copy of a MRC_String */
inline MRC_String copy_mrc_string(const MRC_String& str) {
    if (str.str == nullptr || str.len == 0) {
        return MRC_String{nullptr, 0};
    }
    return alloc_mrc_string(to_string_view(&str));
}
}  // namespace mrclib
