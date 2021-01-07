// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include "../../../../../../../../../../../../../../../AppData/Local/Android/Sdk/ndk/21.0.6113669/toolchains/llvm/prebuilt/windows-x86_64/sysroot/usr/include/c++/v1/vector"

namespace wpi {

// Binary insortion into vector; std::log(n) efficiency.
template <typename T>
typename std::vector<T>::iterator insert_sorted(std::vector<T>& vec,
                                                T const& item) {
  return vec.insert(std::upper_bound(vec.begin(), vec.end(), item), item);
}
}  // namespace wpi
