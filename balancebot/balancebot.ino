#include "mpufusion.h"

MPUFusion *mpufusion;
FusionDataType *fd;

void setup(){
    Serial.begin(115200);
    mpufusion = new MPUFusion();
}

void loop(){
    fd = mpufusion->getFusionData();
    mpufusion->printKalmanAngles();
    mpufusion->printComplementaryAngles();
    Serial.println("");
}