#ifndef _UNARY_FACTOR_H__
#define _UNARY_FACTOR_H__
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

class GpsFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2>
{
    public:

        GpsFactor(gtsam::Key poseKey, const gtsam::Pose2 m, gtsam::SharedNoiseModel model);
    
        gtsam::Vector evaluateError(const gtsam::Pose2 &p , boost::optional<gtsam::Matrix&> H =boost::none) const;

    
    private:
        double mx_, my_, mtheta_;
    
};













#endif