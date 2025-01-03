#pragma once
#include <rclcpp/rclcpp.hpp>
#include "uas/UASState.h"
#include "uas/UAS.h"
#include <Eigen/Dense>

class UASPhase
{   
    public:

        // constructors:
        UASPhase(): desiredAltitude_(-10.0f), kpZ_(0.1f) { }

        // fields:
        std::string phaseName_;
        float desiredAltitude_;
        float kpZ_;
        //localization fields
        float alpha_, rcp_, dx_, dy_,sx_, sy_, d_, theta_, rGround_, xn_, yn_, xc_, yc_, zc_, centerX_, centerY_;
        Eigen::Vector3d r_CameraRelRGV_, r_BodyRelRGV_ ,r_DroneENURelRGV_, r_DroneESDRelRGV_;
        Eigen::Matrix3d R1_, R2_, R3_, REnu_;

        // methods:
        /**
         * @brief Primary function of all UAS mission phases. Tells the UAS where to go.
         *
         */
        virtual UASState generateDesiredState(const UASState& uasState);
        //virtual UASState generateDesiredState(const CVImg& rgvCVData, const UASState& uasState);
        //virtual UASState generateDesiredState(const CVImg& rgv1CVData, const CVImg& rgv2CVData, const UASState& uasState);
        
        /**
         * @brief Determines distance between the UAS and a waypoint.
         */
        double distance(const UASState& s1, const UASState& s2);
        //RGVState localize(const Camera& camera, const CVImg& cvImg, const UAS& uas, RGV& rgv);
};