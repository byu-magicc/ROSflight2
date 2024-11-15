#include "common.h"
#include "mavlink.h"
#include "test_board.h"

#include "rosflight.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <fstream>

// #define DEBUG

using namespace rosflight_firmware;
using namespace Eigen;

class EstimatorTest : public ::testing::Test
{
public:
  testBoard board;
  Mavlink mavlink;
  ROSflight rf;

  std::ofstream file_;

  Vector3d gravity;
  Quaterniond q_;

  double x_freq_, y_freq_, z_freq_;
  double x_amp_, y_amp_, z_amp_;
  double tmax_;
  double x_gyro_bias_;
  double y_gyro_bias_;
  double z_gyro_bias_;
  double t_, dt_;
  int oversampling_factor_;
  int ext_att_update_rate_;
  int ext_att_count_;

  EstimatorTest()
      : mavlink(board)
      , rf(board, mavlink)
  {}

  void SetUp() override
  {
    gravity.x() = 0.0;
    gravity.y() = 0.0;
    gravity.z() = -9.80665;
    q_.w() = 1;
    q_.x() = 0;
    q_.y() = 0;
    q_.z() = 0;

    x_freq_ = 10.0;
    y_freq_ = 5.0;
    z_freq_ = 0.3;
    x_amp_ = 1.0;
    y_amp_ = 0.75;
    z_amp_ = 1.0;
    dt_ = 0.001;
    tmax_ = 30.0;
    t_ = 0.0;
    x_gyro_bias_ = 0.0;
    y_gyro_bias_ = 0.0;
    z_gyro_bias_ = 0.0;
    oversampling_factor_ = 10;

    ext_att_update_rate_ = 0;
    ext_att_count_ = 0;

    rf.init();
  }

  void initFile(const std::string & filename) { file_.open(filename); }

  double run()
  {
    float acc[3], gyro[3];

    double max_error = 0.0;
    t_ = 0.0;
    while (t_ < tmax_) {
      for (int i = 0; i < oversampling_factor_; i++) {
        Vector3d w;
        w[0] = x_amp_ * sin(x_freq_ / (2.0 * M_PI) * t_);
        w[1] = y_amp_ * sin(y_freq_ / (2.0 * M_PI) * t_);
        w[2] = z_amp_ * sin(z_freq_ / (2.0 * M_PI) * t_);
        integrate(q_, w, dt_ / double(oversampling_factor_));
        t_ += dt_ / double(oversampling_factor_);
      }

      simulateIMU(acc, gyro);
      extAttUpdate();
      board.set_imu(acc, gyro, t_ * 1e6);
      board.set_time(t_ * 1e6);
      rf.run();
      Vector3d err = computeError();

      double err_norm = err.norm();
      if (std::abs(err_norm - 2.0 * M_PI) < err_norm) {
        err_norm = std::abs(err_norm - 2.0 * M_PI);
      }
      max_error = (err_norm > max_error) ? err_norm : max_error;
    }
    return max_error;
  }

  void integrate(Quaterniond & q, const Vector3d & _w, double dt)
  {
    Vector3d w = _w * dt;

    Quaterniond w_exp;
    double w_norm = w.norm();
    if (w_norm > 1e-4) {
      w_exp.w() = std::cos(w_norm / 2.0);
      double scale = std::sin(w_norm / 2.0) / w_norm;
      w_exp.x() = scale * w(0);
      w_exp.y() = scale * w(1);
      w_exp.z() = scale * w(2);
      w_exp.normalize();
    } else {
      w_exp.w() = 1.0;
      w_exp.x() = w(0) / 2.0;
      w_exp.y() = w(1) / 2.0;
      w_exp.z() = w(2) / 2.0;
      w_exp.normalize();
    }
    q = q * w_exp;
    q.coeffs() *= sign(q.w());
  }

  void simulateIMU(float * acc, float * gyro)
  {
    Vector3d y_acc = q_.inverse() * gravity;
    acc[0] = y_acc.x();
    acc[1] = y_acc.y();
    acc[2] = y_acc.z();

    // Create gyro measurement
    gyro[0] = x_amp_ * sin(x_freq_ / (2.0 * M_PI) * t_) + x_gyro_bias_;
    gyro[1] = y_amp_ * sin(y_freq_ / (2.0 * M_PI) * t_) + y_gyro_bias_;
    gyro[2] = z_amp_ * sin(z_freq_ / (2.0 * M_PI) * t_) + z_gyro_bias_;
  }

  void extAttUpdate()
  {
    if (ext_att_update_rate_ && ++ext_att_count_ >= ext_att_update_rate_) {
      ext_att_count_ = 0;
      turbomath::Quaternion q_ext;
      q_ext.w = q_.w();
      q_ext.x = q_.x();
      q_ext.y = q_.y();
      q_ext.z = q_.z();

      rf.estimator_.set_external_attitude_update(q_ext);
    }
  }

  double sign(double x) { return x < 0 ? -1 : 1; }

  Vector3d computeError()
  {
    Quaterniond rf_quat(rf.estimator_.state().attitude.w, rf.estimator_.state().attitude.x,
                        rf.estimator_.state().attitude.y, rf.estimator_.state().attitude.z);

    Quaterniond err = q_ * rf_quat.inverse();
    Vector3d v(err.x(), err.y(), err.z());
    float w = err.w();
    double norm_v = v.norm();

    if (t_ > 20.698) {
      int debug = 1;
      (void) debug;
    }

    Vector3d log_err;
    if (norm_v > 1e-4) {
      log_err = 2.0 * std::atan(norm_v / w) * v / norm_v;
    } else {
      log_err = 2.0 * sign(w) * v;
    }

    if (file_.is_open()) {
      file_.write(reinterpret_cast<char *>(&t_), sizeof(t_));
      file_.write(reinterpret_cast<char *>(&q_), sizeof(double) * 4);
      file_.write(reinterpret_cast<const char *>(&rf_quat), sizeof(double) * 4);
      file_.write(reinterpret_cast<char *>(log_err.data()), sizeof(double) * 3);
      file_.write(reinterpret_cast<char *>(eulerError().data()), sizeof(double) * 3);
      file_.write(reinterpret_cast<const char *>(&rf.estimator_.bias().x), sizeof(float) * 3);
    }

    return log_err;
  }

  Vector3d eulerError()
  {
    Vector3d rpy = getTrueRPY();

    Vector3d err;
    err(0) = rpy(0) - rf.estimator_.state().roll;
    err(1) = rpy(1) - rf.estimator_.state().pitch;
    err(2) = rpy(2) - rf.estimator_.state().yaw;

    return err;
  }

  double biasError()
  {
    double xerr = x_gyro_bias_ - rf.estimator_.bias().x;
    double yerr = y_gyro_bias_ - rf.estimator_.bias().y;
    double zerr = z_gyro_bias_ - rf.estimator_.bias().z;
    return std::sqrt(xerr * xerr + yerr * yerr + zerr * zerr);
  }

  Vector3d getTrueRPY()
  {
    Vector3d rpy;
    rpy(0) = std::atan2(2.0f * (q_.w() * q_.x() + q_.y() * q_.z()),
                        1.0f - 2.0f * (q_.x() * q_.x() + q_.y() * q_.y()));
    rpy(1) = std::asin(2.0f * (q_.w() * q_.y() - q_.z() * q_.x()));
    rpy(2) = std::atan2(2.0f * (q_.w() * q_.z() + q_.x() * q_.y()),
                        1.0f - 2.0f * (q_.y() * q_.y() + q_.z() * q_.z()));
    return rpy;
  }
};

TEST_F(EstimatorTest, LinearGyro)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, false);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);

#ifdef DEBUG
  initFile("linearGyro.bin");
#endif
  double error = run();
  EXPECT_LE(error, 1e-3);

#ifdef DEBUG
  std::cout << "error = " << error << std::endl;
#endif
}

TEST_F(EstimatorTest, QuadraticGyro)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, false);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);

#ifdef DEBUG
  initFile("quadGyro.bin");
#endif
  double error = run();
  EXPECT_LE(error, 2e-4);

#ifdef DEBUG
  std::cout << "error = " << error << std::endl;
#endif
}

TEST_F(EstimatorTest, MatrixExp)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);

#ifdef DEBUG
  initFile("expInt.bin");
#endif
  double error = run();
  EXPECT_LE(error, 3e-3);

#ifdef DEBUG
  std::cout << "error = " << error << std::endl;
#endif
}

TEST_F(EstimatorTest, MatrixExpQuadInt)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);

#ifdef DEBUG
  initFile("expQuadInt.bin");
#endif
  double error = run();
  EXPECT_LE(error, 2e-3);

#ifdef DEBUG
  std::cout << "error = " << error << std::endl;
#endif
}

TEST_F(EstimatorTest, Accel)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);

#ifdef DEBUG
  initFile("acc.bin");
#endif
  double error = run();
  EXPECT_LE(error, 1e-2);

#ifdef DEBUG
  std::cout << "error = " << error << std::endl;
#endif
}

TEST_F(EstimatorTest, EstimateStateAccel)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);
  rf.params_.set_param_int(PARAM_INIT_TIME, 0.0f);

  turbomath::Quaternion q_tweaked;
  q_tweaked.from_RPY(0.2, 0.1, 0.0);
  q_.w() = q_tweaked.w;
  q_.x() = q_tweaked.x;
  q_.y() = q_tweaked.y;
  q_.z() = q_tweaked.z;

  x_freq_ = 0.0;
  y_freq_ = 0.0;
  z_freq_ = 0.0;
  x_amp_ = 0.0;
  y_amp_ = 0.0;
  z_amp_ = 0.0;

  tmax_ = 150.0;
  x_gyro_bias_ = 0.00;
  y_gyro_bias_ = 0.00;

  oversampling_factor_ = 1;

#ifdef DEBUG
  initFile("estState.bin");
#endif
  run();

  // only take error of roll and pitch because yaw is unobservable
  double rp_err = eulerError().head<2>().norm();
  EXPECT_LE(rp_err, 1e-3);
  EXPECT_LE(biasError(), 1e-3);
#ifdef DEBUG
  std::cout << "rp_err = " << rp_err << std::endl;
  std::cout << "biasError = " << biasError() << std::endl;
#endif
}

TEST_F(EstimatorTest, EstimateBiasAccel)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);
  rf.params_.set_param_int(PARAM_INIT_TIME, 0.0f);

  turbomath::Quaternion q_tweaked;
  q_tweaked.from_RPY(0.2, 0.1, 0.0);
  q_.w() = q_tweaked.w;
  q_.x() = q_tweaked.x;
  q_.y() = q_tweaked.y;
  q_.z() = q_tweaked.z;

  x_freq_ = 0.0;
  y_freq_ = 0.0;
  z_freq_ = 0.0;
  x_amp_ = 0.0;
  y_amp_ = 0.0;
  z_amp_ = 0.0;

  tmax_ = 150.0;
  x_gyro_bias_ = 0.01;
  y_gyro_bias_ = -0.03;
  z_gyro_bias_ = 0.00;

  oversampling_factor_ = 1;

#ifdef DEBUG
  initFile("estBias.bin");
#endif
  run();

  // only take error of roll and pitch because yaw is unobservable
  double rp_err = eulerError().head<2>().norm();
  EXPECT_LE(rp_err, 3e-3);
  EXPECT_LE(biasError(), 2e-3);
#ifdef DEBUG
  std::cout << "rp_err = " << rp_err << std::endl;
  std::cout << "biasError = " << biasError() << std::endl;
#endif
}

TEST_F(EstimatorTest, StaticExtAtt)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);
  rf.params_.set_param_int(PARAM_INIT_TIME, 0.0f);

  turbomath::Quaternion q_tweaked;
  q_tweaked.from_RPY(0.2, 0.1, 0.0);
  q_.w() = q_tweaked.w;
  q_.x() = q_tweaked.x;
  q_.y() = q_tweaked.y;
  q_.z() = q_tweaked.z;

  x_freq_ = 0.0;
  y_freq_ = 0.0;
  z_freq_ = 0.0;
  x_amp_ = 0.0;
  y_amp_ = 0.0;
  z_amp_ = 0.0;

  tmax_ = 150.0;
  x_gyro_bias_ = 0.01;
  y_gyro_bias_ = -0.03;
  z_gyro_bias_ = 0.01;

  oversampling_factor_ = 1;

  ext_att_update_rate_ = 3;

#ifdef DEBUG
  initFile("estStateExtAtt.bin");
#endif
  run();

  double error = computeError().norm();
  EXPECT_LE(error, 5e-3);
  EXPECT_LE(biasError(), 2e-2);
#ifdef DEBUG
  std::cout << "stateError = " << error << std::endl;
  std::cout << "biasError = " << biasError() << std::endl;
#endif
}

// This test is fixed by #357
TEST_F(EstimatorTest, MovingExtAtt)
{
  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_XY_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_Z_ALPHA, 0);
  rf.params_.set_param_int(PARAM_INIT_TIME, 0.0f);

  turbomath::Quaternion q_tweaked;
  q_tweaked.from_RPY(0.2, 0.1, 0.0);
  q_.w() = q_tweaked.w;
  q_.x() = q_tweaked.x;
  q_.y() = q_tweaked.y;
  q_.z() = q_tweaked.z;

  x_freq_ = 2.0;
  y_freq_ = 3.0;
  z_freq_ = 0.5;
  x_amp_ = 0.1;
  y_amp_ = 0.2;
  z_amp_ = -0.1;

  tmax_ = 150.0;
  x_gyro_bias_ = 0.01;
  y_gyro_bias_ = -0.03;
  z_gyro_bias_ = 0.01;

  oversampling_factor_ = 1;

  ext_att_update_rate_ = 3;

#ifdef DEBUG
  initFile("movingExtAtt.bin");
#endif
  run();

  double error = computeError().norm();
  EXPECT_LE(error, 4e-3);
  EXPECT_LE(biasError(), 2e-2);
#ifdef DEBUG
  std::cout << "stateError = " << error << std::endl;
  std::cout << "biasError = " << biasError() << std::endl;
#endif
}
