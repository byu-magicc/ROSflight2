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

#ifndef TURBOMATH_TURBOMATH_H
#define TURBOMATH_TURBOMATH_H

#include <cstdint>
#include <math.h>

namespace turbomath
{
#define ISA_PRESSURE (101325.0) // Pa
#define ISA_EXPONENT (0.190326730028458)
#define ISA_SCALE_FACTOR (44318.1386038261) //m

float fsign(float y);

// Used for calculating altitude in m from atmospheric pressure in Pa
float alt(float x);

float inv_sqrt(float x);

union float_converter_t
{
  float fvalue;
  int32_t ivalue;
};

class Vector
{
public:
  float x;
  float y;
  float z;

  Vector();
  Vector(float x_, float y_, float z_);

  float norm() const;
  float sqrd_norm() const;
  Vector & normalize();
  Vector normalized() const;

  float dot(const Vector & v) const;
  Vector cross(const Vector & v) const;

  Vector operator*(float s) const;
  Vector operator/(float s) const;
  Vector & operator*=(float s);
  Vector & operator/=(float s);
  Vector operator+(const Vector & v) const;
  Vector operator-(const Vector & v) const;
  Vector & operator+=(const Vector & v);
  Vector & operator-=(const Vector & v);
};

inline Vector operator*(float s, const Vector & v) { return v * s; }
inline Vector operator/(float s, const Vector & v) { return v / s; }

class Quaternion
{
public:
  float w;
  float x;
  float y;
  float z;

  Quaternion();
  Quaternion(float w_, float x_, float y_, float z_);
  Quaternion(const Vector & u, const Vector & v);
  Quaternion(float roll, float pitch, float yaw);

  Vector rotate(const Vector & v) const;
  Quaternion & normalize();
  Quaternion inverse() const;
  Quaternion & invert();
  Quaternion & from_two_unit_vectors(const Vector & u, const Vector & v);
  Quaternion & from_RPY(float roll, float pitch, float yaw);
  void get_RPY(float * roll, float * pitch, float * yaw) const;

  Vector operator*(const Vector & v) const;
  Quaternion operator*(const Quaternion & q) const;
  Quaternion & operator*=(const Quaternion & q);
  Vector boxminus(const Quaternion & q) const;
  static Vector log(const Quaternion & q)
  {
    Vector v{q.x, q.y, q.z};
    float norm_v = v.norm();

    Vector out;
    if (norm_v < 1e-8) {
      out.x = out.y = out.z = 0.0;
    } else {
      out = 2.0 * atan2(norm_v, q.w) * v / norm_v;
    }
    return out;
  }

  Vector operator-(const Quaternion & q) const { return boxminus(q); }
};

} // namespace turbomath

#endif // TURBOMATH_TURBOMATH_H
