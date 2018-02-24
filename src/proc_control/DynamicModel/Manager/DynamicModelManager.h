//
// Created by olavoie on 2/23/18.
//

#ifndef PROC_CONTROL_DYNAMICPARAMETERS_H
#define PROC_CONTROL_DYNAMICPARAMETERS_H

#include "proc_control/property.h"
#include "proc_control/config/config_manager.h"
#include "proc_control/DynamicParametersConfig.h"
#include "proc_control/DynamicModel/DynamicModelParameters.h"


namespace proc_control{

    class DynamicModelManager : public ConfigManager<DynamicParametersConfig> {
    public:

        DynamicModelManager(const std::string &axe_name, std::shared_ptr<DynamicModelParameters> &dynamicModelParameters);
        ~DynamicModelManager() = default;

        void OnDynamicReconfigureChange(const DynamicParametersConfig &config ) override;

        void WriteConfigFile( const DynamicParametersConfig &config ) override;

        void ReadConfigFile( DynamicParametersConfig &config ) override;

    private:

        std::string file_path_;
        std::shared_ptr<DynamicModelParameters> dynamicModelParameters_;


    };

}



#endif //PROC_CONTROL_DYNAMICPARAMETERS_H
