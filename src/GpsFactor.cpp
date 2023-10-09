#include "include/GpsFactor.h"
GpsFactor::GpsFactor(gtsam::Key poseKey, const gtsam::Pose2 m, gtsam::SharedNoiseModel model) :
            gtsam::NoiseModelFactor1<gtsam::Pose2>(model, poseKey), mx_(m.x()), my_(m.y()), mtheta_(m.theta()) {};
    



gtsam::Vector GpsFactor::evaluateError(const gtsam::Pose2 &p , boost::optional<gtsam::Matrix&> H) const
            {
                // Jocabian H is identity matrix33, obviously, if we write the error/state
                if (H) *H = (gtsam::Matrix33() << 1.0, 0.0, 0.0, 
                                                0.0, 1.0, 0.0,
                                                0.0, 0.0, 1.0).finished();

                return (gtsam::Vector3() << p.x() - mx_, p.y() - my_, p.theta() - mtheta_).finished();
            }
