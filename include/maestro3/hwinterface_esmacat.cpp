/*
 * hwinterface_esmacat.cpp
 *
 *  Created on: Jul 1, 2015
 *  Revised on: Jun 27, 2018
 *      Author: Ashish.Lab
 */

#include "hwinterface_esmacat.h"

HWInterface_Esmacat::HWInterface_Esmacat()
{
    printf("Constructing HWInterface_Esmacat...\n");
//    CRIO_sensor_raw_array = new int32_t[this->GetArraySizeOfSensorData()];
//    CRIO_analog_output_array = new int32_t[this->GetArraySizeOfAnalogOutput()];
    exo_joint_angle_sensor_rad_array = new double[NUMBER_OF_JOINT_ANGLE_SENSORS];
    motor_angle_tick_array = new double[NUMBER_OF_DRIVERS];
    motor_angle_rad_array = new double[NUMBER_OF_DRIVERS];
    motor_set_value_tick_array = new double[NUMBER_OF_DRIVERS];
    motor_set_value_rad_array = new double[NUMBER_OF_DRIVERS];
    exo_joint_angle_sensor_volts_array = new double[NUMBER_OF_JOINT_ANGLE_SENSORS];

    // Set homing parameters

    homingComplete = false;
    homeSensorArray = new bool[NUMBER_OF_DRIVERS];
    hCycleCount = 0;
    for(int i=0;i<NUMBER_OF_DRIVERS;i++){
        hCycleOffset[i]=0;
    }


    //**** FROM CALIBRATION PROGRAM ****//

//    Angle Sensor 0 Scale: 0.660452
//    Angle Sensor 1 Scale: -1.14688
//    Angle Sensor 2 Scale: 0.68944
//    Angle Sensor 3 Scale: 0.672941
//    Angle Sensor 4 Scale: 0.800454
//    Angle Sensor 5 Scale: 0.698098
//    Angle Sensor 6 Scale: 0.686341
//    Angle Sensor 7 Scale: 0.714635


    // Exo Sensor Scaling Factor
    exo_joint_angle_volts_to_rad_scale[0]=0.740;//CMC FE        //-0.0006722366;//-0.0006672864;//-0.000691737;//
    exo_joint_angle_volts_to_rad_scale[1]=-1.202;//CMC AA        //0.0014208922;//0.0013888562;//0.0013442844;//0.001438458;
    exo_joint_angle_volts_to_rad_scale[2]=1;//CMC-MCP       //0.0008403437;//-0.0007519231;//-0.0007035436;//-0.000938543;
    exo_joint_angle_volts_to_rad_scale[3]=0.652;//MCP FE        //0.0023595266;//0.00073352; //0.0020216169
    exo_joint_angle_volts_to_rad_scale[4]=1;//MCP-IP        //-0.0010196082;//-0.001038644;//-0.001067854;
    exo_joint_angle_volts_to_rad_scale[5]=0.672;//IP FE         //0.001648293;
    exo_joint_angle_volts_to_rad_scale[6]=0.755;//IN MCP FE     //-0.0011490829;
    exo_joint_angle_volts_to_rad_scale[7]=1;//IN MCP AA     //-0.0005012157;//-0.0009573988;//-0.000701414; // IF MCP FE
    exo_joint_angle_volts_to_rad_scale[8]=1;//IN MCP-PIP    //-0.001252176;
    exo_joint_angle_volts_to_rad_scale[9]=0.618;//IN PIP FE     //-0.0007747513;//-0.000862999;//-0.000825955; // IF PIP
    exo_joint_angle_volts_to_rad_scale[10]=1;//IN DIP       //0.000661115;
    exo_joint_angle_volts_to_rad_scale[11]=0.540;//MI MCP FE    //0.002111285;
    exo_joint_angle_volts_to_rad_scale[12]=1;//MI MCP AA    //-0.000797239;
    exo_joint_angle_volts_to_rad_scale[13]=1;//MI MCP-PIP   //0.000765364;
    exo_joint_angle_volts_to_rad_scale[14]=0.620;//MI PIP FE    //-0.000841061;
    exo_joint_angle_volts_to_rad_scale[15]=1;//MI DIP       //0.00072143;

    /* Set default offset based on backlash estimates (estimates in radians) from motor position testing */

    exo_joint_angle_volts_offset[0]=0.045/exo_joint_angle_volts_to_rad_scale[0];//CMC FE
    exo_joint_angle_volts_offset[1]=0.045/exo_joint_angle_volts_to_rad_scale[1];//CMC AA
    exo_joint_angle_volts_offset[2]=0/exo_joint_angle_volts_to_rad_scale[2];//CMC-MCP
    exo_joint_angle_volts_offset[3]=-0.015/exo_joint_angle_volts_to_rad_scale[3];//MCP FE
    exo_joint_angle_volts_offset[4]=0/exo_joint_angle_volts_to_rad_scale[4];//MCP-IP
    exo_joint_angle_volts_offset[5]=0.041/exo_joint_angle_volts_to_rad_scale[5];//IP FE 0.017737
    exo_joint_angle_volts_offset[6]=0.005/exo_joint_angle_volts_to_rad_scale[6];//IN MCP FE
    exo_joint_angle_volts_offset[7]=0/exo_joint_angle_volts_to_rad_scale[7];//IN MCP AA
    exo_joint_angle_volts_offset[8]=0/exo_joint_angle_volts_to_rad_scale[8];//IN MCP-PIP
    exo_joint_angle_volts_offset[9]=0.0353/exo_joint_angle_volts_to_rad_scale[9];//IN PIP FE
    exo_joint_angle_volts_offset[10]=0/exo_joint_angle_volts_to_rad_scale[10];//IN DIP
    exo_joint_angle_volts_offset[11]=0.010/exo_joint_angle_volts_to_rad_scale[11];//MI MCP FE 0.050601
    exo_joint_angle_volts_offset[12]=0/exo_joint_angle_volts_to_rad_scale[12];//MI MCP AA
    exo_joint_angle_volts_offset[13]=0/exo_joint_angle_volts_to_rad_scale[13];//MI MCP-PIP
    exo_joint_angle_volts_offset[14]=0.020/exo_joint_angle_volts_to_rad_scale[14];//MI PIP FE 0.112891/2
    exo_joint_angle_volts_offset[15]=0/exo_joint_angle_volts_to_rad_scale[15];//MI DIP

//    // Paria's additions (new calibration values)

//    // Exo Sensor Scaling Factor
//    exo_joint_angle_volts_to_rad_scale[0]=0.740;//CMC FE        //-0.0006722366;//-0.0006672864;//-0.000691737;//
//    exo_joint_angle_volts_to_rad_scale[1]=-1.202;//CMC AA        //0.0014208922;//0.0013888562;//0.0013442844;//0.001438458;
//    exo_joint_angle_volts_to_rad_scale[2]=1;//CMC-MCP       //0.0008403437;//-0.0007519231;//-0.0007035436;//-0.000938543;
//    exo_joint_angle_volts_to_rad_scale[3]=0.652;//MCP FE        //0.0023595266;//0.00073352; //0.0020216169
//    exo_joint_angle_volts_to_rad_scale[4]=1;//MCP-IP        //-0.0010196082;//-0.001038644;//-0.001067854;
//    exo_joint_angle_volts_to_rad_scale[5]=0.672;//IP FE         //0.001648293;

//    exo_joint_angle_volts_to_rad_scale[6]=-0.73885;// *new val good               0.755;//IN MCP FE     //-0.0011490829;
//    exo_joint_angle_volts_to_rad_scale[7]=1.34;//*new val good                    1;//IN MCP AA     //-0.0005012157;//-0.0009573988;//-0.000701414; // IF MCP FE
//    exo_joint_angle_volts_to_rad_scale[8]=0.7091;//*new val good                  1;//IN MCP-PIP    //-0.001252176;
//    exo_joint_angle_volts_to_rad_scale[9]=0.618;//IN PIP FE?? maxed out     //-0.0007747513;//-0.000862999;//-0.000825955; // IF PIP
//    exo_joint_angle_volts_to_rad_scale[10]=1;//IN DIP?? maxed out       //0.000661115;

//    exo_joint_angle_volts_to_rad_scale[11]=-0.7035;//* new val good                0.540;//MI MCP FE    //0.002111285;
//    exo_joint_angle_volts_to_rad_scale[12]=0.8393;//* new val good                 1;//MI MCP AA    //-0.000797239;
//    exo_joint_angle_volts_to_rad_scale[13]=-0.08693; //**new val bad               1;//MI MCP-PIP   //0.000765364;
//    exo_joint_angle_volts_to_rad_scale[14]=-0.653; //*new val good                 0.620;//MI PIP FE    //-0.000841061;
//    exo_joint_angle_volts_to_rad_scale[15]=0.9183; //**new val bad                 1;//MI DIP       //0.00072143;

//    /* Set default offset based on backlash estimates (estimates in radians) from motor position testing */

//    exo_joint_angle_volts_offset[0]=0.045/exo_joint_angle_volts_to_rad_scale[0];//CMC FE
//    exo_joint_angle_volts_offset[1]=0.045/exo_joint_angle_volts_to_rad_scale[1];//CMC AA
//    exo_joint_angle_volts_offset[2]=0/exo_joint_angle_volts_to_rad_scale[2];//CMC-MCP
//    exo_joint_angle_volts_offset[3]=-0.015/exo_joint_angle_volts_to_rad_scale[3];//MCP FE
//    exo_joint_angle_volts_offset[4]=0/exo_joint_angle_volts_to_rad_scale[4];//MCP-IP
//    exo_joint_angle_volts_offset[5]=0.041/exo_joint_angle_volts_to_rad_scale[5];//IP FE 0.017737

//    exo_joint_angle_volts_offset[6]=2.4372;//* new val good                       0.005/exo_joint_angle_volts_to_rad_scale[6];//IN MCP FE
//    exo_joint_angle_volts_offset[7]=-2.963;//* new val good                       0/exo_joint_angle_volts_to_rad_scale[7];//IN MCP AA
//    exo_joint_angle_volts_offset[8]=-0.5744;//* new val good                      0/exo_joint_angle_volts_to_rad_scale[8];//IN MCP-PIP
//    exo_joint_angle_volts_offset[9]=??? 0.0353/exo_joint_angle_volts_to_rad_scale[9];//IN PIP FE
//    exo_joint_angle_volts_offset[10]=??? 0/exo_joint_angle_volts_to_rad_scale[10];//IN DIP

//    exo_joint_angle_volts_offset[11]=1.711;//* new val good                   0.010/exo_joint_angle_volts_to_rad_scale[11];//MI MCP FE 0.050601
//    exo_joint_angle_volts_offset[12]=-2.304;//* new val good                  0/exo_joint_angle_volts_to_rad_scale[12];//MI MCP AA
//    exo_joint_angle_volts_offset[13]=-0.08693;//** new val bad                0/exo_joint_angle_volts_to_rad_scale[13];//MI MCP-PIP
//    exo_joint_angle_volts_offset[14]=-0.653;//* new value good                0.020/exo_joint_angle_volts_to_rad_scale[14];//MI PIP FE 0.112891/2
//    exo_joint_angle_volts_offset[15]=-0.2228;//**new val bad                  0/exo_joint_angle_volts_to_rad_scale[15];//MI DIP






    for(int i=0;i<NUMBER_OF_DRIVERS;i++)
    {
        motor_angle_tick_to_rad_scale[i] = 2*M_PI/(4*256.0*66); // for maxon combination 620241 (256 tick encoder, 66:1 gearbox)
        motor_angle_tick_offset[i] = 0.0;
    }

    // Set motor angle default offsets based on desired home position distance from limit switch (Set home positions here)//

    motor_angle_tick_offset[0] = (M_PI/8.0)/motor_angle_tick_to_rad_scale[0]; // Center ROM Offset: M_PI/14.0
    motor_angle_tick_offset[1] = (M_PI/32.0)/motor_angle_tick_to_rad_scale[1];
    motor_angle_tick_offset[2] = (M_PI/8.0)/motor_angle_tick_to_rad_scale[2]; // Center ROM Offset: M_PI/-16.0
    motor_angle_tick_offset[3] = (M_PI/32.0)/motor_angle_tick_to_rad_scale[3];
    motor_angle_tick_offset[4] = (M_PI/8.0)/motor_angle_tick_to_rad_scale[4];
    motor_angle_tick_offset[5] = (M_PI/6.0)/motor_angle_tick_to_rad_scale[5]; // Center ROM Offset: M_PI/16.0
    motor_angle_tick_offset[6] = (-M_PI/16.0)/motor_angle_tick_to_rad_scale[6];
    // paria's modification (changing the value to pos or negative didn't help)
    motor_angle_tick_offset[7] = (0)/motor_angle_tick_to_rad_scale[7]; // Center ROM Offset: -M_PI/6.0



    printf("HWInterface_Esmacat Constructed!!!\n");
}

HWInterface_Esmacat::~HWInterface_Esmacat(){
    cout << "deleting HWInterface_Esmacat" << endl;
    delete exo_joint_angle_sensor_rad_array;
    delete motor_angle_tick_array;
    delete motor_angle_rad_array;
    delete motor_set_value_tick_array;
    delete motor_set_value_rad_array;
    delete exo_joint_angle_sensor_volts_array;
}



bool HWInterface_Esmacat::ScanAllSensorData(){

        // assign scanned values to variables

        getAnalogSensorData(exo_joint_angle_sensor_volts_array);
        getEncoderData(motor_angle_tick_array);


    return false; //true


}

void HWInterface_Esmacat::CalibrateAllSensorData(){
        for(int i=0;i<NUMBER_OF_JOINT_ANGLE_SENSORS;i++){
                exo_joint_angle_sensor_rad_array[i] = (exo_joint_angle_sensor_volts_array[i]-exo_joint_angle_volts_offset[i])*exo_joint_angle_volts_to_rad_scale[i];
        }

//        //Paria's additions   (new calibrations)
//        for(int i=0;i<NUMBER_OF_JOINT_ANGLE_SENSORS;i++){
//            //double check this equation
//                exo_joint_angle_sensor_rad_array[i] = exo_joint_angle_volts_to_rad_scale[i]*(exo_joint_angle_sensor_volts_array[i]) + exo_joint_angle_volts_offset[i];
//        }



        for(int i=0;i<NUMBER_OF_DRIVERS;i++){
                motor_angle_rad_array[i] = (motor_angle_tick_array[i]-motor_angle_tick_offset[i])*motor_angle_tick_to_rad_scale[i];
//		printf("motor_angle_rad_array[%d] %lf, %lf \n",i,motor_angle_rad_array[i],motor_angle_tick_to_rad_scale[i]);
//		printf("motor_angle_tick_array[%d] %lf \n",i,motor_angle_tick_array[i]);
        }
        //	printf("\n");
}

bool HWInterface_Esmacat::FlushAllOutputData(){
        // convert double to int32_t
//        ConvertOutputUnit();
        setMotorPositions(motor_set_value_tick_array);
//        cout<<"Motor Set Array: "<<"\t"
//           <<motor_set_value_tick_array[4]<<"\t"
//           <<motor_set_value_tick_array[5]<<"\t"
//           <<motor_set_value_tick_array[6]<<"\t"
//           <<motor_set_value_tick_array[7]<<endl;

        // if you need to add more on analog output other than motor, add here
        // ...
        //

//	printf("motor_set_value_mV_array[6]: %lf", motor_set_value_mV_array[6]);

        return true;
}

void HWInterface_Esmacat::ConvertOutputUnit(){
        for(int i=0;i<NUMBER_OF_DRIVERS;i++){
                motor_set_value_tick_array[i] = motor_set_value_rad_array[i] *(1/motor_angle_tick_to_rad_scale[i]) + motor_angle_tick_offset[i];

                //		motor_set_value_mV_array[i] = motor_set_value_mV_array[i] * 1.0112-4.653; // this is to calibrate (DAC2ADC error)
//                motor_set_value_mV_array[i] = motor_set_value_mV_array[i] * 1.0036 + 6.62; // this is to calibrate (DAC2ADC error)
                //		motor_set_value_mV_array[i] = motor_set_value_mV_array[i] * 1.0102-4.653; // this is to calibrate (DAC2ADC error)
                //		printf ("motor_motor_set_value_mV_array %d %lf\n", i, motor_set_value_mV_array[i] );
        }

//	printf("motor_set_value_mV_array[6] in conversion: %lf", motor_set_value_mV_array[6]);
}



bool HWInterface_Esmacat::homeMotors(){

    // TODO: Set max retraction angle

    getDigitalSensorData(homeSensorArray);

    if(hCycleCount==0){ //Set states for first cycle

        cout<<endl;
        cout<<"Homing Motors..."<<endl;
        for(int i =0; i<NUMBER_OF_DRIVERS; i++){ //set the initial state of each motor
            if(homeSensorArray[i]==1){ //Limit switch not pressed
                motorState[i] = advanceToSensor;
            }
            else{ //Limit switch is pressed
                motorState[i] = retractOffSensor;
            }
        }
    }

    else{ //normal cycle operation
        getEncoderData(motor_angle_tick_array);

        for(int i=0;i<NUMBER_OF_DRIVERS;i++){

            switch(motorState[i]){
                case retractOffSensor:{
                    motor_set_value_tick_array[i] = -1*motorAdvanceVelocity*hCycleCount/motor_angle_tick_to_rad_scale[i];
                    if(homeSensorArray[i]==1){ // Limit switch is released
                        motorState[i] = advanceToSensor;
                        hCycleOffset[i] = hCycleCount;
                    }
                    break;
                }
                case advanceToSensor:{
                    motor_set_value_tick_array[i] = motorAdvanceVelocity*(hCycleCount-hCycleOffset[i])/motor_angle_tick_to_rad_scale[i];
                    if(homeSensorArray[i]==0){ // Limit switch is pressed
                        motorState[i] = homing;
                        motor_angle_tick_offset[i] = motor_angle_tick_array[i]+ motor_angle_tick_offset[i]; //Add home offset to default offset  //(M_PI/8.0)/motor_angle_tick_to_rad_scale[i];
                    }
                    break;
                }
                case homing:{
                    motor_set_value_tick_array[i] = motor_angle_tick_offset[i];
                    if(abs(motor_angle_tick_array[i]-motor_angle_tick_offset[i])<10){
                        motorState[i] = homed;
                    }
                }
                case homed:{
                    motor_set_value_tick_array[i] = motor_angle_tick_offset[i];
                    break;
                }
            }
        }

        setMotorPositions(motor_set_value_tick_array);
    }

    hCycleCount++;

//    if(hCycleCount%40 ==1){
//        for(int i=0;i<NUMBER_OF_DRIVERS;i++)
//            cout<<"Motor "<<i<<" State: "<<motorState[i]<<"\t"<<"Set: "<<motor_set_value_tick_array[i]<<"\t"<<"Enc: "<<motor_angle_tick_array[i]<<endl;
//        cout<<endl;
//    }
    homingComplete = (motorState[0]==homed && motorState[1]==homed && motorState[2]==homed && motorState[3]==homed &&
            motorState[4]==homed && motorState[5]==homed && motorState[6]==homed && motorState[7]==homed);

    if(homingComplete){
        getAnalogSensorData(exo_joint_angle_sensor_volts_array);
        for(int i=0;i<NUMBER_OF_JOINT_ANGLE_SENSORS;i++){
            exo_joint_angle_volts_offset[i] = exo_joint_angle_sensor_volts_array[i]+exo_joint_angle_volts_offset[i]; //add offset due to homing to default offset due to backlash
        }
    }
    return homingComplete;

}
    //paria's addition to not home motor 8
bool HWInterface_Esmacat::homeMotors_but_eight(){

    // TODO: Set max retraction angle

    getDigitalSensorData(homeSensorArray);

    if(hCycleCount==0){ //Set states for first cycle

        cout<<endl;
        cout<<"Homing Motors..."<<endl;
        for(int i =0; i<NUMBER_OF_DRIVERS-1; i++){ //set the initial state of each motor
            if(homeSensorArray[i]==1){ //Limit switch not pressed
                motorState[i] = advanceToSensor;
            }
            else{ //Limit switch is pressed
                motorState[i] = retractOffSensor;
            }
        }
    }

    else{ //normal cycle operation
        getEncoderData(motor_angle_tick_array);

        for(int i=0;i<NUMBER_OF_DRIVERS-1;i++){

            switch(motorState[i]){
                case retractOffSensor:{
                    motor_set_value_tick_array[i] = -1*motorAdvanceVelocity*hCycleCount/motor_angle_tick_to_rad_scale[i];
                    if(homeSensorArray[i]==1){ // Limit switch is released
                        motorState[i] = advanceToSensor;
                        hCycleOffset[i] = hCycleCount;
                    }
                    break;
                }
                case advanceToSensor:{
                    motor_set_value_tick_array[i] = motorAdvanceVelocity*(hCycleCount-hCycleOffset[i])/motor_angle_tick_to_rad_scale[i];
                    if(homeSensorArray[i]==0){ // Limit switch is pressed
                        motorState[i] = homing;
                        motor_angle_tick_offset[i] = motor_angle_tick_array[i]+ motor_angle_tick_offset[i]; //Add home offset to default offset  //(M_PI/8.0)/motor_angle_tick_to_rad_scale[i];
                    }
                    break;
                }
                case homing:{
                    motor_set_value_tick_array[i] = motor_angle_tick_offset[i];
                    if(abs(motor_angle_tick_array[i]-motor_angle_tick_offset[i])<10){
                        motorState[i] = homed;
                    }
                }
                case homed:{
                    motor_set_value_tick_array[i] = motor_angle_tick_offset[i];
                    break;
                }
            }
        }

        setMotorPositions(motor_set_value_tick_array);
    }

    hCycleCount++;

//    if(hCycleCount%40 ==1){
//        for(int i=0;i<NUMBER_OF_DRIVERS;i++)
//            cout<<"Motor "<<i<<" State: "<<motorState[i]<<"\t"<<"Set: "<<motor_set_value_tick_array[i]<<"\t"<<"Enc: "<<motor_angle_tick_array[i]<<endl;
//        cout<<endl;
//    }
//    homingComplete = (motorState[0]==homed && motorState[1]==homed && motorState[2]==homed && motorState[3]==homed &&
//            motorState[4]==homed && motorState[5]==homed && motorState[6]==homed && motorState[7]==homed);
    homingComplete = (motorState[0]==homed && motorState[1]==homed && motorState[2]==homed && motorState[3]==homed &&
            motorState[4]==homed && motorState[5]==homed && motorState[6]==homed);

    if(homingComplete){
        getAnalogSensorData(exo_joint_angle_sensor_volts_array);
        for(int i=0;i<NUMBER_OF_JOINT_ANGLE_SENSORS-1;i++){
            exo_joint_angle_volts_offset[i] = exo_joint_angle_sensor_volts_array[i]+exo_joint_angle_volts_offset[i]; //add offset due to homing to default offset due to backlash
        }
    }
    return homingComplete;

}


bool HWInterface_Esmacat::calibrateSEAJointSensors(){

}

bool HWInterface_Esmacat::calibrateObserverJointSensors(){
    //TODO: Add code to calibrate scale and offset values of non-sea sensors
}
