/*
 * @file RelativePoseEM.h
 * @brief use EM to optimize relative pose hypothesis
 * @author Jing Dong
 * @date July 8, 2014
 */

#pragma once

#include <joint_mapping/Hypothesis.h>
#include <joint_mapping/Settings.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Vector.h>

#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactor.h>
#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactorEM.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>

#include <vector>

namespace comap {

// perform EM optimization to given clustered initial poses, output hypothesis (not merge hypothesis)
std::vector<Hypothesis> performRelativePoseEM(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const std::vector<PoseD>& initial,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const RelativePoseEMSetting& setting);

}   // namespace comap
