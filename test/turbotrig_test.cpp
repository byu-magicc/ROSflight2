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

turbomath::Vector random_vectors[25] = {
  turbomath::Vector(-0.0376278050814, 0.471775699711, -0.336572370974),
  turbomath::Vector(0.842139998851, -0.113277302409, -0.435361598132),
  turbomath::Vector(0.402876930871, -0.998517068538, 0.956603957591),
  turbomath::Vector(0.366004030077, -0.966554559399, 0.236455814495),
  turbomath::Vector(0.170963581611, -0.892193316086, -0.360102936987),
  turbomath::Vector(-0.675191763273, -0.794118513048, 0.561367212903),
  turbomath::Vector(-0.0299477253533, 0.0938177650483, 0.525814272724),
  turbomath::Vector(-0.676191678521, -0.0780862208203, -0.272955681219),
  turbomath::Vector(-0.435749833209, -0.673810649938, -0.896559097382),
  turbomath::Vector(0.709083915552, -0.135067363969, -0.385492450532),
  turbomath::Vector(-0.38728558039, -0.502219301225, 0.323557018529),
  turbomath::Vector(-0.186870345154, 0.554827454101, 0.921567682061),
  turbomath::Vector(-0.142106787605, -0.764876359963, 0.00303689980819),
  turbomath::Vector(-0.677798963582, -0.664595954482, 0.339274533414),
  turbomath::Vector(-0.700464041114, 0.325731535871, -0.621492014391),
  turbomath::Vector(-0.604865828708, 0.270639620454, 0.188624833185),
  turbomath::Vector(0.464205180183, -0.461504601245, -0.578708441515),
  turbomath::Vector(0.498899172115, -0.582342366402, -0.694758083436),
  turbomath::Vector(0.0710544604541, -0.63603887083, -0.521799692437),
  turbomath::Vector(-0.372025413205, 0.83531212357, 0.232484576742),
  turbomath::Vector(0.790872496361, -0.89600683592, 0.783984438621),
  turbomath::Vector(0.236462609786, -0.636362560394, 0.203951290805),
  turbomath::Vector(0.831924307534, -0.482532468579, 0.0600026189612),
  turbomath::Vector(0.0562194856302, -0.605799189029, -0.556494338297),
  turbomath::Vector(-0.85014432598, 0.0632157037573, 0.0272188414114)};

turbomath::Quaternion random_quaternions[25] = {
  turbomath::Quaternion(0.10377420365, -0.583993115868, -0.731526280531, -0.0530049846003),
  turbomath::Quaternion(-0.00228103177408, -0.506936771567, 0.976002181169, 0.90368722061),
  turbomath::Quaternion(-0.280191704748, 0.141235897077, 0.770363502952, 0.306427689307),
  turbomath::Quaternion(0.964538929753, -0.849755381903, 0.36374459234, 0.694507794584),
  turbomath::Quaternion(0.0176390041681, -0.960155080148, 0.340078582124, -0.119639355159),
  turbomath::Quaternion(-0.213139865459, -0.91618752978, -0.192746623826, -0.761937711418),
  turbomath::Quaternion(-0.491440057128, -0.468120646081, -0.0682240789086, -0.779728041272),
  turbomath::Quaternion(0.00414757516987, -0.980357614738, 0.243315557667, 0.487816606638),
  turbomath::Quaternion(-0.593742280674, 0.245648066311, 0.682367014935, -0.0659175648814),
  turbomath::Quaternion(-0.322464011587, 0.706588950729, -0.966024250287, -0.50354344519),
  turbomath::Quaternion(-0.537023302971, -0.496355850419, -0.326843736039, 0.456606813517),
  turbomath::Quaternion(-0.581585485434, 0.225766708322, -0.121402082687, 0.160333514827),
  turbomath::Quaternion(-0.422711480811, 0.894994456476, 0.392582496229, 0.0035135659771),
  turbomath::Quaternion(0.326380783544, 0.551413227108, 0.89489801397, 0.87883243747),
  turbomath::Quaternion(0.83500683695, -0.263875030319, -0.1783391105, 0.453727091163),
  turbomath::Quaternion(-0.30389938019, -0.0744317276089, -0.436917072268, 0.907173926266),
  turbomath::Quaternion(-0.320066655494, -0.349065285706, 0.0336903431161, 0.573906603454),
  turbomath::Quaternion(-0.103624452083, -0.82874783662, -0.635967208274, 0.562138574765),
  turbomath::Quaternion(0.90735669209, -0.611711092446, 0.732474120503, 0.866697480004),
  turbomath::Quaternion(0.626137839218, 0.41320663394, -0.821473642241, -0.344696411875),
  turbomath::Quaternion(0.266650461152, -0.784707647527, 0.324347257562, -0.724904312141),
  turbomath::Quaternion(0.964177603528, -0.378173605577, 0.767349174766, 0.560290218637),
  turbomath::Quaternion(0.0812716046369, 0.745067180353, -0.476875959113, -0.245887902551),
  turbomath::Quaternion(-0.177027678376, 0.214558558928, -0.992910369554, 0.592964390132),
  turbomath::Quaternion(0.0979109306209, 0.121890109199, 0.126418158551, 0.242200145606)};

TEST(TurboMath, fastAlt)
{
  // no out of bounds check it makes no sense to return zero altitude.
  // EXPECT_EQ(turbomath::alt(69681), 0.0);
  // EXPECT_EQ(turbomath::alt(10700), 0.0);

  // all valid int values
  float trueResult = 0.0;
  for (int i = 69682; i < 106597; i++) {
    trueResult =
      static_cast<float>((1.0 - pow(static_cast<float>(i) /ISA_PRESSURE, ISA_EXPONENT)) * ISA_SCALE_FACTOR);
    EXPECT_NEAR(turbomath::alt(i), trueResult, .15);
    // arbitrarily chose <= .15m since fast_alt isn't accurate enough for EXPECT_CLOSE,
    // but being within .15 meters of the correct result seems pretty good to me
  }
}

TEST(TurboMath, Vector)
{
  for (int i = 0; i < 24; i++) {
    turbomath::Vector vec1 = random_vectors[i];
    turbomath::Vector vec2 = random_vectors[i + 1];
    Eigen::Vector3f eig2, eig1;
    eig1 << vec1.x, vec1.y, vec1.z;
    eig2 << vec2.x, vec2.y, vec2.z;

    // Test data type
    EXPECT_VEC3_SUPERCLOSE(vec1, eig1);
    EXPECT_VEC3_SUPERCLOSE(vec2, eig2);

    // Test norming operations
    EXPECT_VEC3_SUPERCLOSE(vec1.normalized(), eig1.normalized());
    EXPECT_SUPERCLOSE(vec1.norm(), eig1.norm());
    EXPECT_SUPERCLOSE(vec1.sqrd_norm(), eig1.squaredNorm());
    turbomath::Vector vec3 = vec1;
    Eigen::Vector3f eig3 = eig1;

    vec3.normalize();
    eig3.normalize();
    EXPECT_VEC3_SUPERCLOSE(vec3, eig3);
    EXPECT_SUPERCLOSE(vec3.norm(), 1.0);

    // Test add, subtract and multiply
    EXPECT_VEC3_SUPERCLOSE((vec1 + vec2), (eig1 + eig2));
    EXPECT_VEC3_SUPERCLOSE((vec1 - vec2), (eig1 - eig2));

    EXPECT_VEC3_SUPERCLOSE((vec1 * 5.0f), (eig1 * 5.0f));
    EXPECT_VEC3_SUPERCLOSE((vec1 * -10.0f), (eig1 * -10.0f));

    vec1 *= -3.0f;
    EXPECT_VEC3_SUPERCLOSE(vec1, (eig1 * -3.0f));
    eig1 *= -3.0f;
    EXPECT_VEC3_SUPERCLOSE(vec1, eig1);

    EXPECT_VEC3_SUPERCLOSE((vec1 / -10.0f), (eig1 / -10.0f));
    vec1 /= -3.0f;
    EXPECT_VEC3_SUPERCLOSE(vec1, (eig1 / -3.0f));
    eig1 /= -3.0f;

    // Test Vector Dot Product
    EXPECT_SUPERCLOSE(vec1.dot(vec2), eig1.transpose() * eig2);

    // Test Vector Cross Product
    EXPECT_VEC3_SUPERCLOSE(vec1.cross(vec2), eig1.cross(eig2));
  }
}

TEST(TurboMath, Quaternion)
{
  for (int i = 0; i < 24; i++) {
    turbomath::Quaternion quat1 = random_quaternions[i].normalize();
    turbomath::Quaternion quat2 = random_quaternions[i + 1].normalize();

    Eigen::Quaternionf eig1(quat1.w, quat1.x, quat1.y, quat1.z);
    Eigen::Quaternionf eig2(quat2.w, quat2.x, quat2.y, quat2.z);

    EXPECT_QUAT_SUPERCLOSE(quat1, eig1);
    EXPECT_QUAT_SUPERCLOSE(quat2, eig2);

    // Check normalization
    EXPECT_SUPERCLOSE(quat1.x * quat1.x + quat1.y * quat1.y + quat1.z * quat1.z + quat1.w * quat1.w,
                      1.0f);
    EXPECT_SUPERCLOSE(quat2.x * quat2.x + quat2.y * quat2.y + quat2.z * quat2.z + quat2.w * quat2.w,
                      1.0f);

    // Test Quaternion Operators
    ASSERT_QUAT_SUPERCLOSE((quat1 * quat2), (eig2 * eig1));
    ASSERT_QUAT_SUPERCLOSE(quat1.inverse(), eig1.inverse());

    // Test Quaternion Rotate
    turbomath::Vector vec1 = random_vectors[i];
    Eigen::Vector3f veig1;
    veig1 << vec1.x, vec1.y, vec1.z;

    // Test rotate_vector by rotating vector to new frame
    turbomath::Vector vec2 = quat1.rotate(vec1);
    Eigen::Vector3f veig2 = veig1.transpose() * eig1.toRotationMatrix();
    EXPECT_VEC3_SUPERCLOSE(vec2, veig2); // compare with eigen

    // And rotate back
    turbomath::Vector vec3 = quat1.inverse() * vec2;
    EXPECT_VEC3_SUPERCLOSE(vec3, veig1);

    // Convert to and from euler angles
    float p, t, s;
    quat1.get_RPY(&p, &t, &s);

    turbomath::Quaternion quat3(p, t, s);
    Eigen::Vector3f ihat(1, 0, 0);
    Eigen::Vector3f jhat(0, 1, 0);
    Eigen::Vector3f khat(0, 0, 1);

    Eigen::Quaternionf eig3 =
      Eigen::AngleAxisf(s, khat) * Eigen::AngleAxisf(t, jhat) * Eigen::AngleAxisf(p, ihat);

    EXPECT_QUAT_SUPERCLOSE(quat3, eig3);
    EXPECT_SUPERCLOSE(quat1.w, quat3.w);
    EXPECT_SUPERCLOSE(quat1.x, quat3.x);
    EXPECT_SUPERCLOSE(quat1.y, quat3.y);
    EXPECT_SUPERCLOSE(quat1.z, quat3.z);
  }
}

TEST(TurboMath, QuatFromTwoVectors)
{
  // Test the "quat_from_two_vectors"
  turbomath::Vector vec1(1.0f, 0.0f, 0.0f);
  turbomath::Quaternion quat1(1.0f / sqrt(2.0f), 0.0f, 0.0f, 1.0f / sqrt(2.0f));
  turbomath::Vector vec2 = quat1.rotate(vec1);
  turbomath::Quaternion quat_test;
  quat_test.from_two_unit_vectors(vec2, vec1);

  ASSERT_TURBOQUAT_SUPERCLOSE(quat1, quat_test);

  // A bunch of random (off-axes tests)
  for (int i = 0; i < 25; i++) {
    turbomath::Vector vec3 = random_vectors[i].normalize();
    turbomath::Quaternion quat2 = random_quaternions[i].normalize();

    // Manually rotate this vector with the quaternion
    turbomath::Vector vec4 = quat2.rotate(vec3);

    // Extract the "shortest-arc" rotation
    quat_test.from_two_unit_vectors(vec4, vec3);

    // Try rotating that same vector with this new quaternion
    turbomath::Vector vec5 = quat_test.rotate(vec3);

    // Make sure that the vectors are the same
    ASSERT_SUPERCLOSE(vec5.x, vec4.x);
    ASSERT_SUPERCLOSE(vec5.y, vec4.y);
    ASSERT_SUPERCLOSE(vec5.z, vec4.z);
  }
}
