// Android: OpenVINS only uses boost::filesystem::{exists, remove, path,
// create_directories}; map them to C++17 std::filesystem (no compiled Boost on Android).
#pragma once
#include <filesystem>
namespace boost {
namespace filesystem = std::filesystem;
}
