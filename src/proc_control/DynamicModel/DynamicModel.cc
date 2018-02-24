//
// Created by olavoie on 2/21/18.
//

#include "DynamicModel.h"

namespace proc_control {

    DynamicModel::DynamicModel(): dynamicModelParameters_(nullptr), parametersManager_(nullptr) {

        dynamicModelParameters_ = std::make_shared<DynamicModelParameters>();
        parametersManager_      = std::make_shared<DynamicModelManager>("dynamic_model_param", dynamicModelParameters_);

        DampingMatrix_     = Eigen::MatrixXd::Zero(CARTESIAN_SPACE, CARTESIAN_SPACE);
        GravityVector_     = Eigen::VectorXd::Zero(CARTESIAN_SPACE);
    }

    Eigen::VectorXd DynamicModel::ComputeDynamicModel(Eigen::VectorXd &velocity, Eigen::Vector3d &orientation, Eigen::VectorXd &acceleration) {

        Eigen::VectorXd cartesianThrust = Eigen::VectorXd::Zero(CARTESIAN_SPACE);
        ComputeDampingMatrix(velocity);
        ComputeGravityVector(orientation);

        cartesianThrust = dynamicModelParameters_->massMatrix * acceleration + DampingMatrix_ * velocity;
        cartesianThrust += GravityVector_;



        return cartesianThrust;

    }

    // drag = 0.5 * rho * s^2 * A * Cd
    void DynamicModel::ComputeDampingMatrix(Eigen::VectorXd &velocity) {

        Eigen::VectorXd drag = Eigen::VectorXd::Zero(CARTESIAN_SPACE);

        drag = velocity.array() * velocity.array() * dynamicModelParameters_->auvSurface.array() * dynamicModelParameters_->dragCoefficient.array();

        for(int i = 0; i < CARTESIAN_SPACE; i++){

            DampingMatrix_(i, i) = 0.5 * dynamicModelParameters_->waterDensity * drag[i];
        }

        std::cout << "Damping :\n" << DampingMatrix_ << std::endl;

    }

    void DynamicModel::ComputeGravityVector(Eigen::Vector3d &orientation) {

        Eigen::VectorXd buoyancy_center = dynamicModelParameters_->buoyancyCenter;
        double buoyancy = dynamicModelParameters_->auvBuoyancy;
        double resulting_force = buoyancy - dynamicModelParameters_->auvWeight;

        GravityVector_[X] = resulting_force  * sin(orientation[Y]);
        GravityVector_[Y] = -resulting_force * sin(orientation[X]) * cos(orientation[Y]);
        GravityVector_[Z] = -resulting_force * cos(orientation[X]) * cos(orientation[Y]);

        GravityVector_[ROLL]  = buoyancy  * cos(orientation[Y]) * (buoyancy_center[Z] * sin(orientation[X]) - buoyancy_center[Y] * cos(orientation[X]));
        GravityVector_[PITCH] = buoyancy  * (buoyancy_center[X] * cos(orientation[X]) * cos(orientation[Y]) - buoyancy_center[Z] * sin(orientation[Y]));
        GravityVector_[YAW]   = -buoyancy * (buoyancy_center[X] * sin(orientation[X]) * cos(orientation[Y]) - buoyancy_center[Y] * sin(orientation[Y]));
    }
}
