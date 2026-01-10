#pragma once

#include "RadioClass.hpp"

class Model : public RadioClass
{
private:

public:
    Model(RadioData& newRadioData) : RadioClass(newRadioData){}
    void doFunction();
};

void Model::doFunction()
{

}
