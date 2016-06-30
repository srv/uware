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

#ifndef CAMERA_H
#define CAMERA_H

class FeaturePoint {
  FeaturePoint(const Eigen::Vector3d& ip, const Eigen::Vector3d& wp) : image_point(ip), world_point(wp)
  Eigen::Vector3d image_point;
  Eigen::Vector3d world_point;
};

class Pose {
 public:
  Eigen::Matrix4d T() {
    Eigen::Matrix4d T;
    T = q * t;
    return T;
  }

  void T(const Eigen::Matrix4d& T) {
    q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    t = Eigen::Vector3d(T.block<3, 1>(0, 3));
  }
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
};

class Camera {
 public:
  Camera(int identifier) : id(identifier) {
    q = Eigen::Quaterniond(1, 0, 0, 0);
    t = Eigen::Vector3d(0, 0, 0);
  }

  Eigen::Matrix3d K() {
    Eigen::Matrix3d K;
    K(0, 0) = fx; K(1, 1) = fy; K(2, 2) = 1.0;
    K(0, 2) = cx; K(1, 2) = cy;
    return K;
  }

  void K(const Eigen::Matrix3d& K) {
    Eigen::Matrix3d K;
    fx = K(0, 0); fy = K(1, 1);
    cx = K(0, 2); cy = K(1, 2);
  }

  void addCorrespondence(const Eigen::Vector3d& ip, const Eigen::Vector3d& wp) {
    features.push_back(FeaturePoint(ip, wp));
  }

  double fx, fy, cx, cy;
  Pose pose;
  std::vector<FeaturePoint> features;
  int id;
};

class StereoCamera {
 public:
  Camera left;
  Camera right;
  Pose pose;
};

#endif // CAMERA_H