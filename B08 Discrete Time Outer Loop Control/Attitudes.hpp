
#ifndef ATTITUDES_HPP
#define ATTITUDES_HPP

#include <eigen3/Eigen/Dense>
using namespace Eigen;

typedef Matrix3d DCM; /* Container to hold a DCM */
// typedef Quaterniond Quat; /* Container to hold a quaternion */

/**
 * @class EulAxisAngle
 *
 * This is a container holding an Euler Axis/Angle attitude representation
 */
class EulAxisAngle
{
public:
  EulAxisAngle() = default;
  EulAxisAngle(double x, double i, double j, double k) : angle(x), axis(i, j, k)
  {
  }
  EulAxisAngle(double x, Vector3d v) : angle(x), axis(v) {}
  double angle;
  Vector3d axis;
};
/**
 * @class Quat
 *
 * This is a container holding a quaternion
 */
class Quat
{
public:
  Quat() = default;
  Quat(double scaler, Vector3d vec) : s(scaler), v(vec) {}
  Quat(double scaler, double i, double j, double k) : s(scaler), v(i, j, k) {}
  double s;
  Vector3d v;
};

/**
 * @class Attitudes
 *
 * This class handles conversions back and forth between a
 * Direction Cosine Matrix, Quaternion, Eigen Axis/Angle,
 * and Euler Angles as well as various helper functions used
 * in these coordinate frames.
 */
class Attitudes
{
public:
  /**
   * @brief Calculates Euler Axis/Angle from a Direction Cosine Matrix
   *
   * @param a DCM from which to calculate axis and angle
   * @return Euler Axis/Angle representation of input DCM
   */
  static EulAxisAngle dcm2eAxisAngle(DCM &a);
  /**
   * @brief Calculates a Quaternion from a Direction Cosine Matrix
   *
   * @param a DCM from which to calculate quaternion
   * @return Quaternion representation of input DCM
   */
  static Quat dcm2quat(DCM &a);
  /**
   * @brief Calculates Euler Angles from a Direction Cosine Matrix
   *
   * @param a DCM from which to calculate Euler angles
   * @return Euler Angles in rotation order of Z, Y, X
   */
  static Vector3d dcm2zyx(DCM &a);
  /**
   * @brief Calculates Direction Cosine Matrix from an Euler Axis/Angle
   *
   * @param eTheta Euler axis/angle from which to calculate DCM
   * @return DCM representation of input Euler axis/angle
   */
  static DCM eAxisAngle2dcm(EulAxisAngle &eTheta);
  /**
   * @brief Calculates Quaternion from an Euler Axis/Angle
   *
   * @param eTheta Euler axis/angle from which to calculate quaternion
   * @return Quaternion representation of input Euler axis/angle
   */
  static Quat eAxisAngle2quat(EulAxisAngle &eTheta);
  /**
   * @brief Calculates a Direction Cosine Matrix from Euler Angles
   *
   * @param sequence the order of the angles (x is 0, y is 1, z is 2
   *  for example {0,2,0} specifies an X-Z-X rotation sequence)
   * @param angles the Euler angles from which to calculate a DCM
   * @return DCM representation of input Euler angles in the given sequence
   */
  static DCM euler2dcm(Vector3d &sequence, Vector3d angles);
  /**
   * @brief Calculates Euler Axis/Angle from Quaternion
   *
   * @param q Quaternion from which to calculate axis and angle
   * @return Euler Axis/Angle representation of input quaternion
   */
  static EulAxisAngle quat2eAxisAngle(Quat &q);
  /**
   * @brief Performs a Quaternion Product operation (Cross Notation: (X))
   *
   * @param q1 quaternion on left side of quaternion product (X) operator
   * @param q2 quaternion on right side of quaternion product (X) operator
   * @return Product of the two quaternions (Cross Notation)
   */
  static Quat quatProdX(Quat &q1, Quat &q2);
  /**
   * @brief Normalizes a Quaternion
   *
   * @param q Quaternion to normalize
   */
  static void quatUnit(Quat *q);
  /**
   * @brief Calculates DCM
   *
   * @param q Quaternion from which to calculate Direction Cosine Matrix
   * @return DCM representation of input quaternion
   */
  static DCM quat2dcm(Quat &q);
  /**
   * @brief Calculates a quaternion conjugate
   *
   * @param q Quaternion to find conjugate of
   * @return Quaternion conjugate of input
   */
  static Quat quatConjugate(Quat &q);
};

#endif /* ATTITUDES_HPP */
