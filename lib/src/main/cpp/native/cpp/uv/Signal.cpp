// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/uv/Signal.h"

#include "wpi/uv/Loop.h"

namespace wpi::uv {

std::shared_ptr<Signal> Signal::Create(Loop& loop) {
  auto h = std::make_shared<Signal>(private_init{});
  int err = uv_signal_init(loop.GetRaw(), h->GetRaw());
  if (err < 0) {
    loop.ReportError(err);
    return nullptr;
  }
  h->Keep();
  return h;
}

void Signal::Start(int signum) {
  Invoke(
      &uv_signal_start, GetRaw(),
      [](uv_signal_t* handle, int signum) {
        Signal& h = *static_cast<Signal*>(handle->data);
        h.signal(signum);
      },
      signum);
}

}  // namespace wpi::uv
