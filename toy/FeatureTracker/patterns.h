/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include "usings.h"

namespace toy {

// clang-format off
struct Pattern {
  //          00  01  02  03
  //
  //      04  05  06  07  08  09
  //
  //  10  11  12  13  14  15  16  17
  //
  //  18  19  20  21  22  23  24  25
  //
  //  26  27  28  29  30  31  32  33
  //
  //  34  35  36  37  38  39  40  41
  //
  //      42  43  44  45  46  47
  //
  //          48  49  50  51
  //
  // -----> x
  // |
  // |
  // y

  static constexpr float pattern_raw[][2] = {
                          {-3, -7}, {-1, -7}, {1, -7},  {3, -7},

                {-5, -5}, {-3, -5}, {-1, -5}, {1, -5},  {3, -5}, {5, -5},

      {-7, -3}, {-5, -3}, {-3, -3}, {-1, -3}, {1, -3}, {3, -3},  {5, -3},  {7, -3},

      {-7, -1}, {-5, -1}, {-3, -1}, {-1, -1}, {1, -1}, {3, -1},  {5, -1},  {7, -1},

      {-7, 1},  {-5, 1},  {-3, 1},  {-1, 1},  {1, 1},  {3, 1},   {5, 1},   {7, 1},

      {-7, 3},  {-5, 3},  {-3, 3},  {-1, 3},  {1, 3},  {3, 3},   {5, 3},   {7, 3},

                {-5, 5},  {-3, 5},  {-1, 5},  {1, 5},   {3, 5},  {5, 5},

                          {-3, 7},  {-1, 7},  {1, 7},   {3, 7}
  };

  static constexpr size_t PATTERN_SIZE = sizeof(pattern_raw) / (2 * sizeof(float));
  // clang-format on

  static const Eigen::Matrix2Pf& pattern() {
    static const Eigen::Matrix2Pf
      pattern = 0.5
                * Eigen::Map<const Eigen::Matrix<float, 2, PATTERN_SIZE>>(
                  (const float*)pattern_raw);
    return pattern;
  }
};

}  //namespace toy
