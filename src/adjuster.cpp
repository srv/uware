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

#include "ceres_extensions.h"
#include "adjuster.h"
#include <iostream>

/**
 * @brief Default class constructor.
 */
Adjuster::Adjuster() : problem_(0) {
  problem_ = new ceres::Problem();
}

/**
 * @brief Default destructor.
 */
Adjuster::~Adjuster() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  delete problem_;
}

/**
 * @brief Restarts the Ceres problem
 */
void Adjuster::reset() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  if (problem_) {
      delete problem_;
  }
  problem_ = new ceres::Problem();
}

void Adjuster::adjust(std::vector<Camera>& cameras) {
  // Use my LocalParameterization
  ceres::LocalParameterization* quaternion_parameterization(new ceres_ext::EigenQuaternionParameterization());
  for (auto cam = cameras.begin(); cam != cameras.end(); cam++) {
    for (auto fpt = cam->features.begin(); fpt != cam->features.end(); fpt++) {
      ceres::CostFunction* cost_function = ReprojectionError::Create(fpt->image_point, fpt->world_point);
      problem_->AddResidualBlock(cost_function, NULL, cam->q.coeffs().data(), cam->t.data());
    }
    // Apply the parameterization
    problem_->SetParameterization(cam->q.coeffs().data(), quaternion_parameterization);
  }
  // Set a few options
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_, &summary);

  std::cout << "Final report:\n" << summary.FullReport();
}

