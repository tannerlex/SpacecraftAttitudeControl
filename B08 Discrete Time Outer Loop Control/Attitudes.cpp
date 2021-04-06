/* Formulas used in these calculations are from: Markley, F. Landis, and John L.
 * Crassidis. Fundamentals of spacecraft attitude determination and control. New
 * York, NY, USA:: Springer New York, 2014. */

#include "./Attitudes.hpp"
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

static const double EPSILON = (1+std::numeric_limits<double>::epsilon())*std::numeric_limits<double>::epsilon();

static Matrix3d xProdMatrix(Vector3d &v)
/* xProdMatrix is a helper function that generates a 3x3 matrix from a vector
 * which, when LEFT multiplied to another vector, would result in the cross
 * product of the vectors */
{
  Matrix3d x(Matrix3d::Zero());
  x(0, 1) = -v(2);
  x(0, 2) = v(1);
  x(1, 0) = v(2);
  x(1, 2) = -v(0);
  x(2, 0) = -v(1);
  x(2, 1) = v(0);
  return x;
}

static int largestColMag(Matrix3d &m)
/* largestColMag is a helper function used by dcm2eAxisAngle. It determines the
 * column of the greatest magnitude in a 3x3 matrix. */
{
  int col = 0; /* initialize to the first column */

  /* if the next column has greater norm than the last */
  if (m.col(col).norm() < m.col(1).norm())
  {
    col = 1; /* assign this column as having the greatest magnitude */
  }
  /* if the next column has greater norm than the largest so far */
  if (m.col(col).norm() < m.col(2).norm())
  {
    col = 2; /* assign this column as having the greatest magnitude */
  }
  return col;
}

EulAxisAngle Attitudes::dcm2eAxisAngle(DCM &a)
{
  /* Using Equations 2.113, 2.114, and 2.115 in above referenced text */
  EulAxisAngle eTheta;
  double theta = acos((a(0, 0) + a(1, 1) + a(2, 2) - 1.0) / 2.0);
  Vector3d e(0, 0, 0); /* initialize to a zero vector */
  if (theta > __DBL_MIN__ || theta < -__DBL_MIN__) /*(theta != 0)*/
  {
    if (theta < (M_PI + EPSILON) &&
        theta > (M_PI - EPSILON)) /*theta == pi*/
    {
      Matrix3d t = (a + Matrix3d::Identity()) / 2.0;
      int lgstCol = largestColMag(t);
      double normFactor = sqrt(t(lgstCol, lgstCol));
      e = t.col(lgstCol) / normFactor;
    }
    else /* theta != M_PI */
    {
      Vector3d vec(a(1, 2) - a(2, 1), a(2, 0) - a(0, 2), a(0, 1) - a(1, 0));
      e = 1 / (2 * sin(theta)) * vec;
    }
  }
  eTheta.angle = theta;
  eTheta.axis = e;
  return eTheta;
}

Quat Attitudes::dcm2quat(DCM &a)
{
  /* implemented using class member function calls by 1st converting to Euler
   * Axis/Angle and then to quaternion */
  EulAxisAngle eTheta = dcm2eAxisAngle(a);
  return eAxisAngle2quat(eTheta);
}

Vector3d Attitudes::dcm2zyx(DCM &a)
{
  /* Using Table B.2 from above referenced text */
  double z = atan2(a(0, 1), a(0, 0));
  double y = -asin(a(0, 2));
  double x = atan2(a(1, 2), a(2, 2));
  Vector3d vec(z, y, x);
  return vec;
}

DCM Attitudes::eAxisAngle2dcm(EulAxisAngle &eTheta)
{
  /* Using Equation 2.109 in above referenced text */
  return (Matrix3d::Identity() - sin(eTheta.angle) * xProdMatrix(eTheta.axis) +
          (1.0 - cos(eTheta.angle)) * xProdMatrix(eTheta.axis) *
              xProdMatrix(eTheta.axis));
}

Quat Attitudes::eAxisAngle2quat(EulAxisAngle &eTheta)
{
  /* Using Equation 2.124 in above referenced text */
  Quat q;                        /* output quaternion (not initialized) */
  q.s = cos(eTheta.angle / 2.0); /* assign the scaler portion */
  q.v = sin(eTheta.angle / 2.0) * eTheta.axis;
  return q;
}

static DCM xyzDCM(int e, double angle)
{
  /* xyzDCM is a helper function which generates a DCM for rotating by the angle
   * about the primary axis 'e', where e = 0 for the x axis, e = 1 for the y
   * axis, and e = 2 for the z axis. */
  DCM a;
  if (e == 0)
  {
    a(0, 0) = 1;
    a(0, 1) = 0;
    a(0, 2) = 0;
    a(1, 0) = 0;
    a(1, 1) = cos(angle);
    a(1, 2) = sin(angle);
    a(2, 0) = 0;
    a(2, 1) = -sin(angle);
    a(2, 2) = cos(angle);
  }
  else if (e == 1)
  {
    a(0, 0) = cos(angle);
    a(0, 1) = 0;
    a(0, 2) = -sin(angle);
    a(1, 0) = 0;
    a(1, 1) = 1;
    a(1, 2) = 0;
    a(2, 0) = sin(angle);
    a(2, 1) = 0;
    a(2, 2) = cos(angle);
  }
  else if (e == 2)
  {
    a(0, 0) = cos(angle);
    a(0, 1) = sin(angle);
    a(0, 2) = 0;
    a(1, 0) = -sin(angle);
    a(1, 1) = cos(angle);
    a(1, 2) = 0;
    a(2, 0) = 0;
    a(2, 1) = 0;
    a(2, 2) = 1;
  }
  return a;
}

DCM Attitudes::euler2dcm(Vector3d &sequence, Vector3d angles)
{
  DCM a0(xyzDCM(sequence(0), angles(0)));
  DCM a1(xyzDCM(sequence(1), angles(1)));
  DCM a2(xyzDCM(sequence(2), angles(2)));
  return a2 * a1 * a0;
}

EulAxisAngle Attitudes::quat2eAxisAngle(Quat &q)
{
  /* Using Equation 2.124 in above referenced text */
  double theta = 2 * acos(q.s);
  Vector3d e(1, 0, 0); /* initialize to a unit vector */
  if (theta > __DBL_MIN__ || theta < -__DBL_MIN__) /*(theta != 0)*/
  {
    e = q.v / sin(theta / 2.0);
    if (theta > M_PI)
    {
      theta = 2 * M_PI - theta;
      e *= -1.0;
    }
  }
  EulAxisAngle eout(theta, e);
  return eout;
}

DCM Attitudes::quat2dcm(Quat &q)
{
  /* Using Equation 2.125 in above referenced text */
  quatUnit(&q);
  DCM a((pow(q.s, 2.0) - pow(q.v.norm(), 2.0)) * DCM::Identity()
        -2.0 * q.s * xProdMatrix(q.v)
        +2.0 * q.v * q.v.transpose());

  return a;
}

Quat Attitudes::quatProdX(Quat &q1, Quat &q2)
{
  /* Using Equation 2.82a in above referenced text */
  Quat qout;
  qout.s = q1.s * q2.s - q1.v.transpose() * q2.v;
  qout.v = q1.s * q2.v + q2.s * q1.v - q1.v.cross(q2.v);
  return qout;
}

void Attitudes::quatUnit(Quat *q)
{
  Vector4d vec(q->s, q->v(0), q->v(1), q->v(2));
  vec.normalize();
  q->s = vec(0);
  q->v(0) = vec(1);
  q->v(1) = vec(2);
  q->v(2) = vec(3);
}

Quat Attitudes::quatConjugate(Quat &qin)
{
  /* Using Equation 2.91 in above referenced text */
  Quat qout(qin.s, -1 * qin.v);
  return qout;
}
