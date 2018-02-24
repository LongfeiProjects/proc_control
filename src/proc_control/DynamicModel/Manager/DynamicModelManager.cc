//
// Created by olavoie on 2/23/18.
//

#include <fstream>
#include "DynamicModelManager.h"


namespace proc_control{


    DynamicModelManager::DynamicModelManager(const std::string &name, std::shared_ptr<DynamicModelParameters> &dynamicModelParameters):
            ConfigManager(name), dynamicModelParameters_(dynamicModelParameters)
    {

        file_path_ = kConfigPath + "/DynamicModel_parameters/Parameters" + kConfigExt;
        Init();
    }


    void DynamicModelManager::OnDynamicReconfigureChange(const DynamicParametersConfig &config )
    {

        dynamicModelParameters_->waterDensity           = config.Water_Density;
        dynamicModelParameters_->auvMass                = config.Auv_Mass;
        dynamicModelParameters_->auvVolume              = config.Auv_volume;
        dynamicModelParameters_->auvSurface[X]          = config.X_surface;
        dynamicModelParameters_->auvSurface[Y]          = config.Y_surface;
        dynamicModelParameters_->auvSurface[Z]          = config.Z_surface;
        dynamicModelParameters_->auvSurface[ROLL]       = config.ROLL_surface;
        dynamicModelParameters_->auvSurface[PITCH]      = config.PITCH_surface;
        dynamicModelParameters_->auvSurface[YAW]        = config.YAW_surface;

        dynamicModelParameters_->inertiaTensor(0, 0)    = config.ixx;
        dynamicModelParameters_->inertiaTensor(1, 1)    = config.iyy;
        dynamicModelParameters_->inertiaTensor(2, 2)    = config.izz;

        dynamicModelParameters_->buoyancyCenter[X]      = config.Buoyancy_center_x;
        dynamicModelParameters_->buoyancyCenter[Y]      = config.Buoyancy_center_y;
        dynamicModelParameters_->buoyancyCenter[Z]      = config.Buoyancy_center_z;

        dynamicModelParameters_->dragCoefficient[X]     = config.cd_x;
        dynamicModelParameters_->dragCoefficient[Y]     = config.cd_y;
        dynamicModelParameters_->dragCoefficient[Z]     = config.cd_z;
        dynamicModelParameters_->dragCoefficient[ROLL]  = config.cd_roll;
        dynamicModelParameters_->dragCoefficient[PITCH] = config.cd_pitch;
        dynamicModelParameters_->dragCoefficient[YAW]   = config.cd_yaw;

        dynamicModelParameters_->ComputeMassMatrix();
        dynamicModelParameters_->ComputeGravityBuoyancy();

    }

    void DynamicModelManager::WriteConfigFile(const DynamicParametersConfig &config)
    {

        YAML::Emitter out;

        out << YAML::BeginMap;
        out << YAML::Key << "Water_Density";
        out << YAML::Value << dynamicModelParameters_->waterDensity;
        out << YAML::Key << "Auv_Mass";
        out << YAML::Value << dynamicModelParameters_->auvMass;
        out << YAML::Key << "Auv_volume";
        out << YAML::Value << dynamicModelParameters_->auvVolume;
        out << YAML::Key << "X_surface";
        out << YAML::Value << dynamicModelParameters_->auvSurface[X];
        out << YAML::Key << "Y_surface";
        out << YAML::Value << dynamicModelParameters_->auvSurface[Y];
        out << YAML::Key << "Z_surface";
        out << YAML::Value << dynamicModelParameters_->auvSurface[Z];
        out << YAML::Key << "ROLL_surface";
        out << YAML::Value << dynamicModelParameters_->auvSurface[ROLL];
        out << YAML::Key << "PITCH_surface";
        out << YAML::Value << dynamicModelParameters_->auvSurface[PITCH];
        out << YAML::Key << "YAW_surface";
        out << YAML::Value << dynamicModelParameters_->auvSurface[YAW];

        out << YAML::Key << "ixx";
        out << YAML::Value << dynamicModelParameters_->inertiaTensor(0, 0);
        out << YAML::Key << "iyy";
        out << YAML::Value << dynamicModelParameters_->inertiaTensor(1, 1);
        out << YAML::Key << "izz";
        out << YAML::Value << dynamicModelParameters_->inertiaTensor(2, 2);

        out << YAML::Key << "Buoyancy_center_x";
        out << YAML::Value << dynamicModelParameters_->buoyancyCenter[X];
        out << YAML::Key << "Buoyancy_center_y";
        out << YAML::Value << dynamicModelParameters_->buoyancyCenter[Y];
        out << YAML::Key << "Buoyancy_center_z";
        out << YAML::Value << dynamicModelParameters_->buoyancyCenter[Z];

        out << YAML::Key << "cd_x";
        out << YAML::Value << dynamicModelParameters_->dragCoefficient[X];
        out << YAML::Key << "cd_y";
        out << YAML::Value << dynamicModelParameters_->dragCoefficient[Y];
        out << YAML::Key << "cd_z";
        out << YAML::Value << dynamicModelParameters_->dragCoefficient[Z];
        out << YAML::Key << "cd_roll";
        out << YAML::Value << dynamicModelParameters_->dragCoefficient[ROLL];
        out << YAML::Key << "cd_pitch";
        out << YAML::Value << dynamicModelParameters_->dragCoefficient[PITCH];
        out << YAML::Key << "cd_yaw";
        out << YAML::Value << dynamicModelParameters_->dragCoefficient[YAW];

        std::ofstream fout(file_path_);
        fout << out.c_str();

    }

    void DynamicModelManager::ReadConfigFile(DynamicParametersConfig &config)
    {

        YAML::Node node = YAML::LoadFile(file_path_);

        if (node["Water_Density"])
            dynamicModelParameters_->waterDensity = node["Water_Density"].as<double>();

        if (node["Auv_Mass"])
            dynamicModelParameters_->auvMass = node["Auv_Mass"].as<double>();

        if (node["Auv_volume"])
            dynamicModelParameters_->auvVolume = node["Auv_volume"].as<double>();

        if (node["X_surface"])
            dynamicModelParameters_->auvSurface[X] = node["X_surface"].as<double>();

        if (node["Y_surface"])
            dynamicModelParameters_->auvSurface[Y] = node["Y_surface"].as<double>();

        if (node["Z_surface"])
            dynamicModelParameters_->auvSurface[Z] = node["Z_surface"].as<double>();

        if (node["ROLL_surface"])
            dynamicModelParameters_->auvSurface[ROLL] = node["ROLL_surface"].as<double>();

        if (node["PITCH_surface"])
            dynamicModelParameters_->auvSurface[PITCH] = node["PITCH_surface"].as<double>();

        if (node["YAW_surface"])
            dynamicModelParameters_->auvSurface[YAW] = node["YAW_surface"].as<double>();

        config.Water_Density = dynamicModelParameters_->waterDensity;
        config.Auv_Mass      = dynamicModelParameters_->auvMass;
        config.Auv_volume    = dynamicModelParameters_->auvVolume;
        config.X_surface     = dynamicModelParameters_->auvSurface[X];
        config.Y_surface     = dynamicModelParameters_->auvSurface[Y];
        config.Z_surface     = dynamicModelParameters_->auvSurface[Z];
        config.ROLL_surface  = dynamicModelParameters_->auvSurface[ROLL];
        config.PITCH_surface = dynamicModelParameters_->auvSurface[PITCH];
        config.YAW_surface   = dynamicModelParameters_->auvSurface[YAW];

        config.ixx = dynamicModelParameters_->inertiaTensor(0, 0);
        config.iyy = dynamicModelParameters_->inertiaTensor(1, 1);
        config.izz = dynamicModelParameters_->inertiaTensor(2, 2);

        config.Buoyancy_center_x = dynamicModelParameters_->buoyancyCenter[X];
        config.Buoyancy_center_y = dynamicModelParameters_->buoyancyCenter[Y];
        config.Buoyancy_center_z = dynamicModelParameters_->buoyancyCenter[Z];

        config.cd_x     = dynamicModelParameters_->dragCoefficient[X];
        config.cd_y     = dynamicModelParameters_->dragCoefficient[Y];
        config.cd_z     = dynamicModelParameters_->dragCoefficient[Z];
        config.cd_roll  = dynamicModelParameters_->dragCoefficient[ROLL];
        config.cd_pitch = dynamicModelParameters_->dragCoefficient[PITCH];
        config.cd_yaw   = dynamicModelParameters_->dragCoefficient[YAW];

        dynamicModelParameters_->ComputeMassMatrix();
        dynamicModelParameters_->ComputeGravityBuoyancy();

    }



}