//
// Created by olavoie on 2/24/18.
//

#ifndef PROC_CONTROL_DYNAMICMODELPARAMETERS_H
#define PROC_CONTROL_DYNAMICMODELPARAMETERS_H

#include <eigen3/Eigen/Eigen>

class DynamicModelParameters
{
public:

    const int CARTESIAN_SPACE = 6;
    const double GRAVITY = 9.81;

    DynamicModelParameters(){ Init(); }
    ~DynamicModelParameters() = default;

    inline void Init();
    inline void ComputeMassMatrix();
    inline void ComputeGravityBuoyancy();


    Eigen::VectorXd dragCoefficient;
    Eigen::Vector3d buoyancyCenter;
    Eigen::VectorXd auvSurface;
    Eigen::Matrix3d inertiaTensor;
    Eigen::MatrixXd addMassTensor;
    Eigen::MatrixXd massInertiaMatrix;
    Eigen::MatrixXd massMatrix;
    double waterDensity, auvMass, auvVolume, auvWeight, auvBuoyancy;

};

inline void DynamicModelParameters::Init() {
    dragCoefficient = Eigen::VectorXd::Zero(CARTESIAN_SPACE);
    buoyancyCenter  = Eigen::Vector3d::Zero();
    auvSurface      = Eigen::VectorXd::Zero(CARTESIAN_SPACE);
    inertiaTensor   = Eigen::Matrix3d::Zero();
    addMassTensor   = Eigen::MatrixXd::Zero(CARTESIAN_SPACE, CARTESIAN_SPACE);
    waterDensity    = 0.0;
    auvMass         = 0.0;
    auvVolume       = 0.0;
}

inline void DynamicModelParameters::ComputeMassMatrix() {
    addMassTensor(0, 0) = waterDensity * auvVolume;
    addMassTensor(1, 1) = waterDensity * auvVolume;
    addMassTensor(2, 2) = waterDensity * auvVolume;
    addMassTensor(3, 3) = waterDensity * auvVolume;
    addMassTensor(4, 4) = waterDensity * auvVolume;
    addMassTensor(5, 5) = waterDensity * auvVolume;

    massInertiaMatrix(0, 0) = auvMass;
    massInertiaMatrix(1, 1) = auvMass;
    massInertiaMatrix(2, 2) = auvMass;
    massInertiaMatrix(3, 3) = inertiaTensor(0, 0);
    massInertiaMatrix(4, 4) = inertiaTensor(1, 1);
    massInertiaMatrix(5, 5) = inertiaTensor(2, 2);

    massMatrix = massInertiaMatrix + addMassTensor;
}

inline void DynamicModelParameters::ComputeGravityBuoyancy() {
    auvWeight = auvMass * GRAVITY;
    auvBuoyancy = waterDensity * auvVolume;
}




#endif //PROC_CONTROL_DYNAMICMODELPARAMETERS_H
