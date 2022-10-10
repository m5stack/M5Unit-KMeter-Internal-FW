//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license. See LICENSE file in the project root for full license information.

#pragma once

// #define DEBUG 1
#pragma GCC optimize ("O3")

#include <cstdint>

namespace command_processor
{
  void setup(void);
  void loop(void);

  bool addData(std::uint8_t value);
  void closeData(void);
  void prepareTxData(void);

  enum proc_t
  {
    proc_main,
    proc_spi,
    proc_i2c,
    proc_max,
  };

  void setActive(proc_t proc);
  void setDeactive(proc_t proc);
}
