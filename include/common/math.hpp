#ifndef MATH_FUNCTIONS
#define MATH_FUNCTIONS

#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <iostream>


namespace Math {
class MathFunc {

 public:

  template<typename T>
  static inline T anglemod(T a) {
    return wrapAngle((a + M_PI)) - M_PI;
  }

  template<typename T>
  static inline T wrapAngle(T a) {
    double twopi = 2.0 * M_PI;
    return a - twopi * fastfloor(a / twopi);
  }

  template<typename T>
  static inline int fastfloor(T a) {
    int i = int(a);
    if (i > a) i--;
    return i;
  }

  template<typename T>
  static inline int getSign(T val) {
    return (T(0) < val) - (val < T(0));
  }

  template<typename T>
  static inline Eigen::Matrix<T, 3, 3> skewM(Eigen::Matrix<T, 3, 1> &vec) {
    Eigen::Matrix<T, 3, 3> mat;
    mat << 0.0, -vec(2), vec(1),
        vec(2), 0.0, -vec(0),
        -vec(1), vec(0), 0.0;
    return mat;
  }

  template<typename T>
  static inline Eigen::Matrix<T, 3, 3> expM(Eigen::Matrix<T, 3, 1> &vec) {
    Eigen::Matrix<T, 3, 3> rot;
    double angle = vec.norm();
    if (angle < 1e-10) return Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 1> axis = vec / angle;
    Eigen::Matrix<T, 3, 3> vecSkew = skewM(axis);
    rot = Eigen::Matrix<T, 3, 3>::Identity() + sin(angle) * vecSkew + (1.0 - cos(angle)) * vecSkew * vecSkew;
    return rot;
  }

  template<typename T>
  static inline Eigen::Matrix<T, 3, 3> expM(double angle, Eigen::Matrix<T, 3, 1> &axis) {
    Eigen::Matrix<T, 3, 3> rot;
    Eigen::Matrix3d vecSkew = skewM(axis);
    rot = Eigen::Matrix<T, 3, 3>::Identity() + sin(angle) * vecSkew + (1.0 - cos(angle)) * vecSkew * vecSkew;
    return rot;
  }

  template<typename Derived, typename Dtype>
  static inline void limitAbsoluteValue(Eigen::MatrixBase<Derived> &matrix, Dtype threshold) {
    int numberOfElements = matrix.rows() * matrix.cols();
    for (int rowID = 0; rowID < matrix.rows(); rowID++)
      for (int colID = 0; colID < matrix.cols(); colID++)
        if (abs(matrix(rowID, colID)) > threshold)
          matrix(rowID, colID) = getSign(Dtype(1.0)) * threshold;
  }
  
  template<typename T>
  static inline Eigen::Matrix<T, 4, 1> EulertoQuat(double roll, double pitch,  double yaw)
  {
    Eigen::Matrix<T, 4, 1> q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q[0] = t0 * t2 * t4 + t1 * t3 * t5;
    q[1] = t0 * t3 * t4 - t1 * t2 * t5;
    q[2] = t0 * t2 * t5 + t1 * t3 * t4;
    q[3] = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
  }

  template<typename T>
  static inline void QuattoEuler(const Eigen::Matrix<T, 4, 1>& q, double& roll, double& pitch, double& yaw)
  {
    double ysqr = q[2] * q[2];

    // roll (x-axis rotation)
    double t0 = +2.0 * (q[0] * q[1] + q[2] * q[3]);
    double t1 = +1.0 - 2.0 * (q[1] * q[1] + ysqr);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q[0] * q[2] - q[3] * q[1]);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q[0] * q[3] + q[1] * q[2]);
    double t4 = +1.0 - 2.0 * (ysqr + q[3] * q[3]);
    yaw = std::atan2(t3, t4);
  }

  template<typename T>
  static inline Eigen::Matrix<T, 3, 3> quatToRotMat(Eigen::Matrix<T, 4, 1> &q) {
    Eigen::Matrix<T, 3, 3> R;
    R << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
        2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2),
        q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2),
        2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return R;
  }

  template<typename T>
  static inline Eigen::Matrix<T, 4, 1> rotMatToQuat(Eigen::Matrix<T, 3, 3> &R) {
    Eigen::Matrix<T, 4, 1> quat;
    double tr = R.trace();
    if (tr > 0.0) {
      double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
      quat(0) = 0.25 * S;
      quat(1) = (R(2, 1) - R(1, 2)) / S;
      quat(2) = (R(0, 2) - R(2, 0)) / S;
      quat(3) = (R(1, 0) - R(0, 1)) / S;
    } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
      double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
      quat(0) = (R(2, 1) - R(1, 2)) / S;
      quat(1) = 0.25 * S;
      quat(2) = (R(0, 1) + R(1, 0)) / S;
      quat(3) = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
      double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
      quat(0) = (R(0, 2) - R(2, 0)) / S;
      quat(1) = (R(0, 1) + R(1, 0)) / S;
      quat(2) = 0.25 * S;
      quat(3) = (R(1, 2) + R(2, 1)) / S;
    } else {
      double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
      quat(0) = (R(1, 0) - R(0, 1)) / S;
      quat(1) = (R(0, 2) + R(2, 0)) / S;
      quat(2) = (R(1, 2) + R(2, 1)) / S;
      quat(3) = 0.25 * S;
    }
    return quat;
  }


  template<typename T>
  static inline Eigen::Matrix<T, 4, 1> quatMultiplication(Eigen::Matrix<T, 4, 1> &q, Eigen::Matrix<T, 4, 1> &p) {
    Eigen::Matrix<T, 4, 1> quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
        p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
        p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
  }

  template<typename T>
  static inline Eigen::Matrix<T, 4, 1> boxplusB_Frame(Eigen::Matrix<T, 4, 1> &quat, Eigen::Matrix<T, 3, 1> &rotation) {
    Eigen::Matrix<T, 4, 1> quat2;
    double norm = rotation.norm();
    double halfNorm = 0.5 * norm;
    double sinHalfNorm = sin(halfNorm);
    quat2 << cos(halfNorm), sinHalfNorm * rotation(0) / norm, sinHalfNorm * rotation(1) / norm, sinHalfNorm
        * rotation(2) / norm;
    return quatMultiplication(quat, quat2);
  }

  template<typename T>
  static inline Eigen::Matrix<T, 4, 1> boxplusI_Frame(Eigen::Matrix<T, 4, 1> &quat, Eigen::Matrix<T, 3, 1> &rotation) {
    Eigen::Matrix<T, 3, 3> rotmat = expM(rotation);
    Eigen::Matrix<T, 4, 1> quat2 = rotMatToQuat(rotmat);
    return quatMultiplication(quat2, quat);
  }

  template<typename T>
  static inline void normalizeQuat(Eigen::Matrix<T, 4, 1> &q) {
    double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] = q[0] / norm;
    q[1] = q[1] / norm;
    q[2] = q[2] / norm;
    q[3] = q[3] / norm;
  }

  template<typename T>
  static inline Eigen::Matrix<T, 4, 1> angleAxisToQuat(double angle, Eigen::Matrix<T, 3, 1> &axis) {
    Eigen::Matrix<T, 4, 1> quat;
    quat << cos(angle / 2.0), axis * sin(angle / 2.0);
    return quat;
  }

  template<typename T>
  static inline Eigen::Matrix<T, 4, 1> rotateQuatByAngleAxis(Eigen::Matrix<T, 4, 1> &q, double angle, Eigen::Matrix<T, 3, 1> &axis) {

//  Eigen::Matrix<T, 4, 1> quat;
//  quat = quatMultiplication(angleEigen::Matrix<T, 3, 1>ToQuat(angle, axis), q);
//  return quat;

    Eigen::Matrix<T, 3, 1> vector = angle * axis;
    return boxplusI_Frame(q, vector);
  }

  template<typename Dtype>
  static inline Dtype standardDev(Eigen::Matrix<Dtype, 1, -1> &samples) {
    Eigen::Matrix<Dtype, 1, -1> centered = samples.array() - samples.mean();
    return std::sqrt(centered.array().square().sum() / (samples.cols() - 1));
  }

  template<typename Dtype>
  static inline void normalize(Eigen::Matrix<Dtype, -1, 1> &samples) {
    Eigen::Matrix<Dtype, -1, 1> centered = samples.array() - samples.mean();
    Dtype std = std::sqrt(centered.array().square().sum() / (samples.cols() - 1));
    samples = centered / std;
  }

  template<typename Dtype>
  static inline void normalize(Eigen::Matrix<Dtype, 1, -1> &samples) {
    Eigen::Matrix<Dtype, 1, -1> centered = samples.array() - samples.mean();
    Dtype std = std::sqrt(centered.array().square().sum() / (samples.cols() - 1));
    samples = centered / std;
  }

  template<typename Dtype>
  static inline void normalize(std::vector<Dtype> &samples) {
    Eigen::Matrix<Dtype, 1, -1> sampleEigen(1, samples.size());
    memcpy(sampleEigen.data(), &samples[0], sizeof(Dtype) * samples.size());
    normalize(sampleEigen);
    memcpy(&samples[0], sampleEigen.data(), sizeof(Dtype) * samples.size());
  }

  /// gives you the smaller angle difference
  template<typename T>
  static inline T angleDiff(T a, T b) {
    return M_PI - std::fabs(std::fmod(std::fabs(a - b), 2.0 * M_PI) - M_PI);
  }

  /// gives you the smaller angle difference, positive if a is more clockwise
  template<typename T>
  static inline T angleDiffSigned(T a, T b) {
    double diff = std::fmod(a - b, 2.0 * M_PI);
    if (diff > 0) {
      return (diff > M_PI) ? diff - 2.0 * M_PI : diff;
    } else {
      return (-diff > M_PI) ? diff + 2.0 * M_PI : diff;
    }
  }

  /// keeping track of the indices. if you do not care about index, use std::sort
  /// assending order
  template<typename Dtype, typename IndType>
  static inline void sort ( std::vector<Dtype>& value, std::vector<IndType>& indx){
    unsigned i, j,  flag = 1;    // set flag to 1 to start first pass
    Dtype temp; // holding variable
    IndType temp2;
    unsigned numLength = value.size( );
    for(i = 1; (i <= numLength) && flag; i++)
    {
      flag = 0;
      for (j=0; j < (numLength -1); j++)
      {
        if (value[j+1] < value[j])      // descending order simply changes to >
        {
          temp = value[j];             // swap elements
          temp2 = indx[j];
          value[j] = value[j+1];
          indx[j] = indx[j+1];
          value[j+1] = temp;
          indx[j+1] = temp2;
          flag = 1;               // indicates that a swap occurred.
        }
      }
    }
  }

  /// keeping track of the indices. if you do not care about index, use std::sort
  /// assending order
  template<typename Dtype, typename IndType>
  static inline void sort ( Eigen::Matrix<Dtype, 1, -1>& value, Eigen::Matrix<IndType, 1, -1>& indx){
    unsigned i, j,  flag = 1;    // set flag to 1 to start first pass
    Dtype temp; // holding variable
    IndType temp2;
    unsigned numLength = value.size( );
    for(i = 1; (i <= numLength) && flag; i++)
    {
      flag = 0;
      for (j=0; j < (numLength -1); j++)
      {
        if (value[j+1] < value[j])      // descending order simply changes to >
        {
          temp = value[j];             // swap elements
          temp2 = indx[j];
          value[j] = value[j+1];
          indx[j] = indx[j+1];
          value[j+1] = temp;
          indx[j+1] = temp2;
          flag = 1;               // indicates that a swap occurred.
        }
      }
    }
  }

};

}

#endif //math functions