#include "motor_controller.h"
// #include "../include/platform_manipulator/motor_controller.h"

MotorController::MotorController() {}

void MotorController::init(int baud_rate_, std::vector<int>& dxl_ids_, std::string& port_name_) {
    baud_rate = baud_rate_;
    dxl_ids = dxl_ids_;
    port_name = port_name_;

    const char *log;
    bool result = false;
    uint16_t model_number = 0;

    result = dxl_wb.init(port_name.c_str(), baud_rate, &log);
    if (result == false)
    {
        printf("%s\n", log);
        printf("Failed to init\n");
    }
    else
        printf("Succeed to init(%d)\n", baud_rate);

    for (int i = 0; i < 3; ++i) {
        result = dxl_wb.ping(dxl_ids[i], &model_number, &log);
        if (result == false)
        {
            printf("%s\n", log);
            printf("Failed to ping\n");
        }
        else
        {
            printf("Succeed to ping\n");
            printf("id : %d, model_number : %d\n", dxl_ids[i], model_number);
        }
        result = dxl_wb.setExtendedPositionControlMode(dxl_ids[i], &log);
        if (result == false)
        {
            printf("%s\n", log);
            printf("Failed to change Position Control Mode\n");
        }
        else
        {

            printf("Succeed to change Position Control Mode\n");
        }
    }
}

bool MotorController::setProfileVelocity(int profile_velocity) {
    const char *log;
    bool result = false;
    
    for (int i = 0; i < 3; ++i) {
        result = result & dxl_wb.itemWrite(dxl_ids[i], "Profile_Velocity", profile_velocity, &log);
    }

    return result;
}

bool MotorController::setProfileAcceleration(int profile_acceleration) {
    const char *log;
    bool result = false;
    
    for (int i = 0; i < 3; ++i) {
        result = result & dxl_wb.itemWrite(dxl_ids[i], "Profile_Acceleration", profile_acceleration, &log);
    }

    return result;
}

bool MotorController::setGoalPosition(const std::vector<int>& positions) {
    bool result = false;
    const char *log;

    for (int i = 0; i < 3; ++i) {
        int pos = double(positions[i]) / 360 * 4096;
        result = result & dxl_wb.goalPosition(dxl_ids[i], pos, &log);
    }

    return result;
}

bool MotorController::resetPosition() {
    bool result = false;
    const char *log;

    for (int i = 0; i < 3; ++i) {
        result = result & dxl_wb.goalPosition(dxl_ids[i], 0, &log);
    }

    return result;
}

void MotorController::enable_torque() {
    bool result = false;
    const char *log;

    for (int i = 0; i < 3; ++i) {
        result = dxl_wb.torqueOn(dxl_ids[i], &log);
    }
}

void MotorController::disable_torque() {
    bool result = false;
    const char *log;

    for (int i = 0; i < 3; ++i) {
        result = dxl_wb.torqueOff(dxl_ids[i], &log);
    }
}

MotorController::~MotorController() {
    disable_torque();
}