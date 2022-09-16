#ifndef MOTOR_CONTROLLER
#define MOTOR_CONTROLLER

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <string>
#include <vector>


class MotorController {
private:
    int baud_rate;
    std::vector<int> dxl_ids;
    std::string port_name;
    DynamixelWorkbench dxl_wb;

public:
    MotorController();
    void init(int baud_rate_, std::vector<int>& dxl_ids_, std::string& port_name_);
    ~MotorController();

    bool setProfileVelocity(int profile_velocity);
    bool setProfileAcceleration(int profile_acceleration);

    bool setGoalPosition(const std::vector<int>& positions);
    bool resetPosition();

    void enable_torque();
    void disable_torque();

};


#endif