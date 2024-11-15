/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, James Jackson  BYU MAGICC Lab, Provo UT
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "common.h"

#include <cmath>
#include <cstdio>

TEST(TurboMath, fastAlt)
{
  // no out of bounds check it makes no sense to return zero altitude.
  // EXPECT_EQ(turbomath::alt(69681), 0.0);
  // EXPECT_EQ(turbomath::alt(10700), 0.0);

  // all valid int values
  float trueResult = 0.0;
  for (int i = 69682; i < 106597; i++) {
    trueResult = static_cast<float>((1.0 - pow(static_cast<float>(i) / ISA_PRESSURE, ISA_EXPONENT))
                                    * ISA_SCALE_FACTOR);
    EXPECT_NEAR(turbomath::alt(i), trueResult, .15);
    // arbitrarily chose <= .15m since fast_alt isn't accurate enough for EXPECT_CLOSE,
    // but being within .15 meters of the correct result seems pretty good to me
  }
}
