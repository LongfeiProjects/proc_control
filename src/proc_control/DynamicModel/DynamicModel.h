//
// Created by olavoie on 2/21/18.
//

#ifndef PROC_CONTROL_DYNAMICMODEL_H
#define PROC_CONTROL_DYNAMICMODEL_H

#include <eigen3/Eigen/Eigen>
#include <memory>

#include "proc_control/DynamicModel/DynamicModelParameters.h"
#include "proc_control/DynamicModel/Manager/DynamicModelManager.h"

namespace proc_control{

    class DynamicModel {
    public:

        const int CARTESIAN_SPACE = 6;

        DynamicModel();
        ~DynamicModel();

        Eigen::VectorXd ComputeDynamicModel(Eigen::VectorXd &velocity, Eigen::VectorXd &position, Eigen::VectorXd &acceleration);

    private:

        void ComputeDampingMatrix(Eigen::VectorXd &velocity);
        void ComputeGravityVector(Eigen::VectorXd &position);

        Eigen::MatrixXd DampingMatrix_;
        Eigen::VectorXd GravityVector_;
        std::shared_ptr<DynamicModelParameters> dynamicModelParameters_;
        std::shared_ptr<DynamicModelManager> parametersManager_;

    };

}



#endif //PROC_CONTROL_DYNAMICMODEL_H
