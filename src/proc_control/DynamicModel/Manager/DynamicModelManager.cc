//
// Created by olavoie on 2/23/18.
//

#include <fstream>
#include "DynamicModelParameters.h"


namespace proc_control{


    DynamicModelParameters::DynamicModelParameters(const std::string &name): ConfigManager(name) {

        file_path_ = kConfigPath + "/DynamicModel_parameters/Parameters" + kConfigExt;
        Init();
    }



}