//
// Created by olavoie on 2/23/18.
//

#ifndef PROC_CONTROL_DYNAMICPARAMETERS_H
#define PROC_CONTROL_DYNAMICPARAMETERS_H

#include <eigen3/Eigen/Eigen>

#include "proc_control/property.h"
#include "proc_control/config/config_manager.h"
#include "proc_control/DynamicParametersConfig.h"


namespace proc_control{

    class DynamicModelParameters : public ConfigManager<DynamicParametersConfig> {
    public:

        DynamicModelParameters(const std::string &axe_name);
        ~DynamicModelParameters();

        void OnDynamicReconfigureChange(const DynamicParametersConfig &config ) override;

        void WriteConfigFile( const DynamicParametersConfig &config ) override;

        void ReadConfigFile( DynamicParametersConfig &config ) override;

        Eigen::VectorXd drag_coefficient_;
        Eigen::Vector3d buoyancy_center_;
        Eigen::Vector3d

    private:

        std::string file_path_;


    };

}



#endif //PROC_CONTROL_DYNAMICPARAMETERS_H
