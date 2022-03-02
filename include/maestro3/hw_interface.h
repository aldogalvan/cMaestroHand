#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#include <stdio.h>
class HW_Interface {

protected:
        double* exo_joint_angle_sensor_rad_array;
        double* motor_angle_tick_array;
        double* motor_angle_rad_array;
        double* motor_set_value_tick_array;
        double* motor_set_value_rad_array;
        double* exo_joint_angle_sensor_volts_array;
public:
        HW_Interface();
        virtual ~HW_Interface();
        void UpdateAllSensorData();
        virtual bool ScanAllSensorData()=0;		// executed at once a loop
        virtual void CalibrateAllSensorData()=0;		// executed at once a loop
        void UpdateAllOutputData();
        virtual void ConvertOutputUnit()=0;
        virtual bool FlushAllOutputData()=0;		// executed at once a loop
        double* GetExoJointAngleSensorRadArray(){
//		printf("exo_joint_angle_sensor_rad_array[0]: %lf\n", exo_joint_angle_sensor_rad_array[0]);
                return exo_joint_angle_sensor_rad_array;
        }
//        double* exo_joint_angle_sensor_volts_array;
        double* GetMotorAngleTickArray(){return motor_angle_tick_array;};
        double* GetMotorAngleRadArray(){return motor_angle_rad_array;};
        double* GetMotorSetValueRadArray(){return motor_set_value_rad_array;};
//	void SetMotorSetValuemVArray(double* a,int size_of_a);
        void SetMotorSetValueDegArray(double* a,int size_of_a);

       //Paria's addition for getting joint angle sensor voltage values
        double* GetExoJointAngleSensorVoltArray(){
//		printf("exo_joint_angle_sensor_rad_array[0]: %lf\n", exo_joint_angle_sensor_rad_array[0]);
                return exo_joint_angle_sensor_volts_array;
        }
};

#endif // HW_INTERFACE_H

