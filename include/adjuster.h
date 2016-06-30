// The MIT License (MIT)

// Copyright (c) 2016 Miquel Massot Campos

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


#ifndef ADJUSTER_H
#define ADJUSTER_H

#include "camera.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

typedef pcl::PointXYZRGB       Point;
typedef pcl::PointCloud<Point> PointCloud;

class Adjuster {
public:
  Adjuster();
  ~Adjuster();
  void adjust(std::vector<Camera>& cameras)
  void reset();

protected:
  ceres::Problem* problem_;

  // Mutex to control the access to the adjuster.
  boost::mutex mutex_adjuster_;
};  // class

struct ReprojectionError {
  ReprojectionError(const Eigen::Vector3d observed,
                    const Eigen::Vector3d worldPoint) {
    observed_x = observed.x() / observed.z();
    observed_y = observed.y() / observed.z();

    X = worldPoint.x();
    Y = worldPoint.y();
    Z = worldPoint.z();
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const Eigen::Vector3d observed,
                                     const Eigen::Vector3d worldPoint) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>(new ReprojectionError(observed, worldPoint)));
  }

  template <typename T>
  bool operator()(const T* const camera_rotation,
                  const T* const camera_translation,
                        T*       residuals) const {
    // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
    Eigen::Matrix<T,3,1> point;
    point << T(X), T(Y), T(Z);

    // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
    Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(camera_rotation);

    // Rotate the point using Eigen rotations
    Eigen::Matrix<T,3,1> p = q * point;

    // Map T* to Eigen Vector3 with correct Scalar type
    Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1>>(camera_translation);
    p += t;

    // Cast CCS 3D point to plane at z = 1
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // The error is the difference between the predicted and observed position.
    residuals[0] = xp - T(observed_x);
    residuals[1] = yp - T(observed_y);

    return true;
  }

  double observed_x;
  double observed_y;
  double X;
  double Y;
  double Z;
};

#endif // ADJUSTER_H
