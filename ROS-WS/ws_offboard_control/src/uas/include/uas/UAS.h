#pragma once
#include "UASState.h"
#include <vector>
// #include "uas_helpers/Camera.h"


class UAS {
    public:
        // deafult constructor
        UAS();

        // parameterized constructor
        UAS(UASState state);

        //fields
        UASState state_;

        // //vector of cameras onboard UAS
        // std::vector<Camera> cameras_;

        // // methods
        // void addCamera(const Camera& camera);

};