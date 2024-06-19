//
// Created by waxz on 5/19/24.
//

#ifndef LIBROSCPP_FS_HPP
#define LIBROSCPP_FS_HPP


#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
#include "filesystem.hpp"
//#include "ghc/filesystem.hpp"

namespace fs = ghc::filesystem;
#endif

#endif //LIBROSCPP_FS_HPP
