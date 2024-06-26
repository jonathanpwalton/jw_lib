//  The MIT License (MIT)

//  Copyright (c) 2024 Jonathan Walton

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#ifndef JW_MATH_HPP_
#define JW_MATH_HPP_

#include <cstdint>
#include <cstdio>
#include <cmath>

namespace jw
{

  using u8 = uint8_t;
  using i8 = int8_t;
  using u16 = uint16_t;
  using i16 = int16_t;
  using u32 = uint32_t;
  using i32 = int32_t;
  using u64 = uint64_t;
  using i64 = int64_t;
  using f32 = float;
  using f64 = double;

  const f32 DEGREES_TO_RADIANS = acosf(-1) / 180.0F;

  inline f32 radians(f32 degrees)
  {
    return degrees * DEGREES_TO_RADIANS;
  }

  struct vec2
  {
    f32 x, y;
    vec2(f32 s) : x(s), y(s) {}
    vec2(f32 x, f32 y) : x(x), y(y) {}

    void print(bool print_type = true, FILE* output = stdout) const
    {
      if (print_type)
        fprintf(output, "vec2\n");
      fprintf(output, "--          --\n| %.4e |\n| %.4e |\n--          --\n", x, y);
    }

    f32 dot(const vec2 &b) const
    {
      return x * b.x + y * b.y;
    }

    f32 length_squared() const
    {
      return dot(*this);
    }

    f32 length() const
    {
      return sqrtf(length_squared());
    }

    vec2 &normalize()
    {
      f32 l = length();
      x /= l;
      y /= l;
      return *this;
    }

    vec2 normalized()
    {
      return vec2(*this).normalize();
    }

    vec2 operator+(f32 s) const
    {
      return vec2(x + s, y + s);
    }

    vec2 operator-(f32 s) const
    {
      return vec2(x - s, y - s);
    }

    vec2 operator*(f32 s) const
    {
      return vec2(x * s, y * s);
    }

    vec2 operator/(f32 s) const
    {
      return vec2(x / s, y / s);
    }

    vec2 operator+(const vec2 &b) const
    {
      return vec2(x + b.x, y + b.y);
    }

    vec2 operator-(const vec2 &b) const
    {
      return vec2(x - b.x, y - b.y);
    }

    vec2 &operator+=(f32 s)
    {
      x += s;
      y += s;
      return *this;
    }

    vec2 &operator-=(f32 s)
    {
      x -= s;
      y -= s;
      return *this;
    }

    vec2 &operator*=(f32 s)
    {
      x *= s;
      y *= s;
      return *this;
    }

    vec2 &operator/=(f32 s)
    {
      x /= s;
      y /= s;
      return *this;
    }

    vec2 &operator+=(const vec2 &b)
    {
      return *this = *this + b;
    }

    vec2 &operator-=(const vec2 &b)
    {
      return *this = *this - b;
    }
  };

  struct vec3
  {
    f32 x, y, z;
    vec3(f32 s) : x(s), y(s), z(s) {}
    vec3(f32 x, f32 y, f32 z) : x(x), y(y), z(z) {}
    vec3(vec2 v, f32 z = 0.0F) : x(v.x), y(v.y), z(z) {}

    void print(bool print_type = true, FILE* output = stdout) const
    {
      if (print_type)
        fprintf(output, "vec3\n");
      fprintf(output, "--          --\n| %.4e |\n| %.4e |\n| %.4e |\n--          --\n", x, y, z);
    }

    f32 dot(const vec3 &b) const
    {
      return x * b.x + y * b.y + z * b.z;
    }

    f32 length_squared() const
    {
      return dot(*this);
    }

    f32 length() const
    {
      return sqrtf(length_squared());
    }

    vec3 &normalize()
    {
      f32 l = length();
      x /= l;
      y /= l;
      z /= l;
      return *this;
    }

    vec3 normalized()
    {
      return vec3(*this).normalize();
    }

    vec3 cross(const vec3 &b)
    {
      return vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }

    vec3 operator+(f32 s) const
    {
      return vec3(x + s, y + s, z + s);
    }

    vec3 operator-(f32 s) const
    {
      return vec3(x - s, y - s, z - s);
    }

    vec3 operator*(f32 s) const
    {
      return vec3(x * s, y * s, z * s);
    }

    vec3 operator/(f32 s) const
    {
      return vec3(x / s, y / s, z / s);
    }

    vec3 operator+(const vec3 &b) const
    {
      return vec3(x + b.x, y + b.y, z + b.z);
    }

    vec3 operator-(const vec3 &b) const
    {
      return vec3(x - b.x, y - b.y, y - b.y);
    }

    vec3 &operator+=(f32 s)
    {
      x += s;
      y += s;
      z += s;
      return *this;
    }

    vec3 &operator-=(f32 s)
    {
      x -= s;
      y -= s;
      z -= s;
      return *this;
    }

    vec3 &operator*=(f32 s)
    {
      x *= s;
      y *= s;
      z *= s;
      return *this;
    }

    vec3 &operator/=(f32 s)
    {
      x /= s;
      y /= s;
      z /= s;
      return *this;
    }

    vec3 &operator+=(const vec3 &b)
    {
      return *this = *this + b;
    }

    vec3 &operator-=(const vec3 &b)
    {
      return *this = *this - b;
    }
  };

  struct vec4
  {
    f32 x, y, z, w;
    vec4(f32 s) : x(s), y(s), z(s), w(s) {}
    vec4(f32 x, f32 y, f32 z, f32 w) : x(x), y(y), z(z), w(w) {}
    vec4(vec3 v, f32 w = 0.0F) : x(v.x), y(v.y), z(v.z), w(w) {}

    void print(bool print_type = true, FILE* output = stdout) const
    {
      if (print_type)
        fprintf(output, "vec4\n");
      fprintf(output, "--          --\n| %.4e |\n| %.4e |\n| %.4e |\n| %.4e |\n--          --\n", x, y, z, w);
    }

    f32 dot(const vec4 &b) const
    {
      return x * b.x + y * b.y + z * b.z + w * b.w;
    }

    f32 length_squared() const
    {
      return dot(*this);
    }

    f32 length() const
    {
      return sqrtf(length_squared());
    }

    vec4 &normalize()
    {
      f32 l = length();
      x /= l;
      y /= l;
      z /= l;
      w /= l;
      return *this;
    }

    vec4 normalized()
    {
      return vec4(*this).normalize();
    }

    vec4 operator+(f32 s) const
    {
      return vec4(x + s, y + s, z + s, w + s);
    }

    vec4 operator-(f32 s) const
    {
      return vec4(x - s, y - s, z - s, w - s);
    }

    vec4 operator*(f32 s) const
    {
      return vec4(x * s, y * s, z * s, w * s);
    }

    vec4 operator/(f32 s) const
    {
      return vec4(x / s, y / s, z / s, w / s);
    }

    vec4 operator+(const vec4 &b) const
    {
      return vec4(x + b.x, y + b.y, z + b.z, w + b.w);
    }

    vec4 operator-(const vec4 &b) const
    {
      return vec4(x - b.x, y - b.y, y - b.y, w - b.w);
    }

    vec4 &operator+=(f32 s)
    {
      x += s;
      y += s;
      z += s;
      w += s;
      return *this;
    }

    vec4 &operator-=(f32 s)
    {
      x -= s;
      y -= s;
      z -= s;
      w -= s;
      return *this;
    }

    vec4 &operator*=(f32 s)
    {
      x *= s;
      y *= s;
      z *= s;
      w *= s;
      return *this;
    }

    vec4 &operator/=(f32 s)
    {
      x /= s;
      y /= s;
      z /= s;
      w /= s;
      return *this;
    }

    vec4 &operator+=(const vec4 &b)
    {
      return *this = *this + b;
    }

    vec4 &operator-=(const vec4 &b)
    {
      return *this = *this - b;
    }
  };

  struct quat
  {
    f32 x, y, z, w;
    quat(f32 x, f32 y, f32 z, f32 w) : x(x), y(y), z(z), w(w) {}
    quat(vec3 axis, f32 angle) : x(axis.x * sinf(angle / 2)), y(axis.y * sinf(angle / 2)), z(axis.z * sinf(angle / 2)), w(cosf(angle / 2)) {}
  };

  struct mat4
  {
    f32 m00 = 0, m01 = 0, m02 = 0, m03 = 0,
        m10 = 0, m11 = 0, m12 = 0, m13 = 0,
        m20 = 0, m21 = 0, m22 = 0, m23 = 0,
        m30 = 0, m31 = 0, m32 = 0, m33 = 0;
    mat4(f32 s = 1.0F) : m00(s), m11(s), m22(s), m33(s) {}
    mat4(const quat &q)
    {
      f32 x = q.x, y = q.y, z = q.z, w = q.w;
      f32 xx = x * x, xy = x * y, xz = x * z, xw = x * w;
      f32 yy = y * y, yz = y * z, yw = y * w;
      f32 zz = z * z, zw = z * w;

      m00 = 1 - 2 * (yy + zz);
      m10 = 2 * (xy - zw);
      m20 = 2 * (xz + yw);

      m01 = 2 * (xy + zw);
      m11 = 1 - 2 * (xx + zz);
      m21 = 2 * (yz - xw);

      m02 = 2 * (xz - yw);
      m12 = 2 * (yz + xw);
      m22 = 1 - 2 * (xx + yy);

      m30 = m31 = m32 = m03 = m13 = m23 = 0;
      m33 = 1;
    }

    void print(bool print_type = true, FILE* output = stdout) const
    {
      if (print_type)
        fprintf(output, "mat4\n");

      fprintf(output, "--                                               --\n");
      fprintf(output, "| %+.4e %+.4e %+.4e %+.4e |\n", m00, m10, m20, m30);
      fprintf(output, "| %+.4e %+.4e %+.4e %+.4e |\n", m01, m11, m21, m31);
      fprintf(output, "| %+.4e %+.4e %+.4e %+.4e |\n", m02, m12, m22, m32);
      fprintf(output, "| %+.4e %+.4e %+.4e %+.4e |\n", m03, m13, m23, m33);
      fprintf(output, "--                                               --\n");
    }

    f32 *data()
    {
      return &m00;
    }

    const f32 *data() const
    {
      return &m00;
    }

    mat4 &translate(const vec3 &xyz)
    {
      mat4 t;
      t.m30 = xyz.x;
      t.m31 = xyz.y;
      t.m32 = xyz.z;
      return *this = *this * t;
    }

    mat4 translated(const vec3 &xyz) const
    {
      mat4 r = *this;
      return r.translate(xyz);
    }

    mat4 &scale(const vec3 &s)
    {
      mat4 t;
      t.m00 = s.x;
      t.m11 = s.y;
      t.m22 = s.z;
      return *this = *this * t;
    }

    mat4 scaled(const vec3 &s) const
    {
      mat4 r = *this;
      return r.scale(s);
    }

    mat4 &rotate(const vec3 &axis, f32 angle)
    {
      mat4 t(quat(axis, angle));
      return *this = *this * t;
    }

    mat4 rotated(const vec3 &axis, f32 angle) const
    {
      mat4 r = *this;
      return r.rotate(axis, angle);
    }

    mat4 &rotate(const quat &q)
    {
      return *this = *this * mat4(q);
    }

    mat4 rotated(const quat &q) const
    {
      mat4 r = *this;
      return r.rotate(q);
    }

    mat4 operator*(const mat4 &b) const
    {
      mat4 r;

      r.m00 = m00 * b.m00 + m10 * b.m01 + m20 * b.m02 + m30 * b.m03;
      r.m01 = m01 * b.m00 + m11 * b.m01 + m21 * b.m02 + m31 * b.m03;
      r.m02 = m02 * b.m00 + m12 * b.m01 + m22 * b.m02 + m32 * b.m03;
      r.m03 = m03 * b.m00 + m13 * b.m01 + m23 * b.m02 + m33 * b.m03;

      r.m10 = m00 * b.m10 + m10 * b.m11 + m20 * b.m12 + m30 * b.m13;
      r.m11 = m01 * b.m10 + m11 * b.m11 + m21 * b.m12 + m31 * b.m13;
      r.m12 = m02 * b.m10 + m12 * b.m11 + m22 * b.m12 + m32 * b.m13;
      r.m13 = m03 * b.m10 + m13 * b.m11 + m23 * b.m12 + m33 * b.m13;

      r.m20 = m00 * b.m20 + m10 * b.m21 + m20 * b.m22 + m30 * b.m23;
      r.m21 = m01 * b.m20 + m11 * b.m21 + m21 * b.m22 + m31 * b.m23;
      r.m22 = m02 * b.m20 + m12 * b.m21 + m22 * b.m22 + m32 * b.m23;
      r.m23 = m03 * b.m20 + m13 * b.m21 + m23 * b.m22 + m33 * b.m23;

      r.m30 = m00 * b.m30 + m10 * b.m31 + m20 * b.m32 + m30 * b.m33;
      r.m31 = m01 * b.m30 + m11 * b.m31 + m21 * b.m32 + m31 * b.m33;
      r.m32 = m02 * b.m30 + m12 * b.m31 + m22 * b.m32 + m32 * b.m33;
      r.m33 = m03 * b.m30 + m13 * b.m31 + m23 * b.m32 + m33 * b.m33;

      return r;
    }

    vec4 operator*(const vec4 &b)
    {
      return vec4(
          m00 * b.x + m10 * b.y + m20 * b.z + m30 * b.w,
          m01 * b.x + m11 * b.y + m21 * b.y + m31 * b.w,
          m02 * b.x + m12 * b.y + m22 * b.z + m32 * b.w,
          m03 * b.x + m13 * b.y + m23 * b.z + m33 * b.w);
    }

    mat4 &operator*=(const mat4 &b)
    {
      return *this = *this * b;
    }

    static mat4 perspective(f32 fovy, f32 ar, f32 n, f32 f)
    {
      mat4 result;

      f32 ht = tanf(fovy / 2.0F);
      f32 t = n * ht;
      f32 r = t * ar;

      result.m00 = n / r;
      result.m11 = n / t;
      result.m22 = -(f + n) / (f - n);
      result.m23 = -1.0F;
      result.m32 = -(2.0F * n * f) / (f - n);
      result.m33 = 0.0F;

      return result;
    }
  };

}

#endif
