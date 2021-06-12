#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "DobotDll.h"

/*
 * Cmd timeout
 */
#include "magician_hardware/SetCmdTimeout.h"

bool SetCmdTimeoutService(magician_hardware::SetCmdTimeout::Request &req, magician_hardware::SetCmdTimeout::Response &res)
{
    res.result = SetCmdTimeout(req.timeout);

    return true;
}

void InitCmdTimeoutServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetCmdTimeout", SetCmdTimeoutService);
    serverVec.push_back(server);
}

/*
 * Device information
 */
#include "magician_hardware/GetDeviceSN.h"
#include "magician_hardware/SetDeviceName.h"
#include "magician_hardware/GetDeviceName.h"
#include "magician_hardware/GetDeviceVersion.h"

bool GetDeviceSNService(magician_hardware::GetDeviceSN::Request &req, magician_hardware::GetDeviceSN::Response &res)
{
    char deviceSN[256];

    res.result = GetDeviceSN(deviceSN, sizeof(deviceSN));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceSN;
        res.deviceSN.data = ss.str();
    }

    return true;
}

bool SetDeviceNameService(magician_hardware::SetDeviceName::Request &req, magician_hardware::SetDeviceName::Response &res)
{
    res.result = SetDeviceName(req.deviceName.data.c_str());

    return true;
}

bool GetDeviceNameService(magician_hardware::GetDeviceName::Request &req, magician_hardware::GetDeviceName::Response &res)
{
    char deviceName[256];

    res.result = GetDeviceName(deviceName, sizeof(deviceName));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceName;
        res.deviceName.data = ss.str();
    }

    return true;
}

bool GetDeviceVersionService(magician_hardware::GetDeviceVersion::Request &req, magician_hardware::GetDeviceVersion::Response &res)
{
    uint8_t majorVersion, minorVersion, revision;

    res.result = GetDeviceVersion(&majorVersion, &minorVersion, &revision);
    if (res.result == DobotCommunicate_NoError) {
        res.majorVersion = majorVersion;
        res.minorVersion = minorVersion;
        res.revision = revision;
    }

    return true;
}

void InitDeviceInfoServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/GetDeviceSN", GetDeviceSNService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetDeviceName", SetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetDeviceName", GetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetDeviceVersion", GetDeviceVersionService);
    serverVec.push_back(server);
}

/*
 * Pose
 */
#include "magician_hardware/GetPose.h"

bool GetPoseService(magician_hardware::GetPose::Request &req, magician_hardware::GetPose::Response &res)
{
    Pose pose;

    res.result = GetPose(&pose);
    if (res.result == DobotCommunicate_NoError) {
        res.x = pose.x;
        res.y = pose.y;
        res.z = pose.z;
        res.r = pose.r;
        for (int i = 0; i < 4; i++) {
            res.jointAngle.push_back(pose.jointAngle[i]);
        }
    }

    return true;
}

void InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/GetPose", GetPoseService);
    serverVec.push_back(server);
}

/*
 * Alarms
 */
#include "magician_hardware/GetAlarmsState.h"
#include "magician_hardware/ClearAllAlarmsState.h"

bool GetAlarmsStateService(magician_hardware::GetAlarmsState::Request &req, magician_hardware::GetAlarmsState::Response &res)
{
    uint8_t alarmsState[128];
    uint32_t len;

    res.result = GetAlarmsState(alarmsState, &len, sizeof(alarmsState));
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < len; i++) {
            res.alarmsState.push_back(alarmsState[i]);
        }
    }

    return true;
}

bool ClearAllAlarmsStateService(magician_hardware::ClearAllAlarmsState::Request &req, magician_hardware::ClearAllAlarmsState::Response &res)
{
    res.result = ClearAllAlarmsState();

    return true;
}

void InitAlarmsServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/GetAlarmsState", GetAlarmsStateService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/ClearAllAlarmsState", ClearAllAlarmsStateService);
    serverVec.push_back(server);
}

/*
 * HOME
 */
#include "magician_hardware/SetHOMEParams.h"
#include "magician_hardware/GetHOMEParams.h"
#include "magician_hardware/SetHOMECmd.h"

bool SetHOMEParamsService(magician_hardware::SetHOMEParams::Request &req, magician_hardware::SetHOMEParams::Response &res)
{
    HOMEParams params;
    uint64_t queuedCmdIndex;

    params.x = req.x;
    params.y = req.y;
    params.z = req.z;
    params.r = req.r;

    res.result = SetHOMEParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetHOMEParamsService(magician_hardware::GetHOMEParams::Request &req, magician_hardware::GetHOMEParams::Response &res)
{
    HOMEParams params;

    res.result = GetHOMEParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.x = params.x;
        res.y = params.y;
        res.z = params.z;
        res.r = params.r;
    }

    return true;
}

bool SetHOMECmdService(magician_hardware::SetHOMECmd::Request &req, magician_hardware::SetHOMECmd::Response &res)
{
    HOMECmd cmd;
    uint64_t queuedCmdIndex;

    res.result = SetHOMECmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitHOMEServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetHOMEParams", SetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetHOMEParams", GetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetHOMECmd", SetHOMECmdService);
    serverVec.push_back(server);
}

/*
 * End effector
 */
#include "magician_hardware/SetEndEffectorParams.h"
#include "magician_hardware/GetEndEffectorParams.h"
#include "magician_hardware/SetEndEffectorLaser.h"
#include "magician_hardware/GetEndEffectorLaser.h"
#include "magician_hardware/SetEndEffectorSuctionCup.h"
#include "magician_hardware/GetEndEffectorSuctionCup.h"
#include "magician_hardware/SetEndEffectorGripper.h"
#include "magician_hardware/GetEndEffectorGripper.h"

bool SetEndEffectorParamsService(magician_hardware::SetEndEffectorParams::Request &req, magician_hardware::SetEndEffectorParams::Response &res)
{
    EndEffectorParams params;
    uint64_t queuedCmdIndex;

    params.xBias = req.xBias;
    params.yBias = req.yBias;
    params.zBias = req.zBias;

    res.result = SetEndEffectorParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorParamsService(magician_hardware::GetEndEffectorParams::Request &req, magician_hardware::GetEndEffectorParams::Response &res)
{
    EndEffectorParams params;

    res.result = GetEndEffectorParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xBias = params.xBias;
        res.yBias = params.yBias;
        res.zBias = params.zBias;
    }

    return true;
}

bool SetEndEffectorLaserService(magician_hardware::SetEndEffectorLaser::Request &req, magician_hardware::SetEndEffectorLaser::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorLaser(req.enableCtrl, req.on, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorLaserService(magician_hardware::GetEndEffectorLaser::Request &req, magician_hardware::GetEndEffectorLaser::Response &res)
{
    bool enableCtrl, on;

    res.result = GetEndEffectorLaser(&enableCtrl, &on);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.on = on;
    }

    return true;
}

bool SetEndEffectorSuctionCupService(magician_hardware::SetEndEffectorSuctionCup::Request &req, magician_hardware::SetEndEffectorSuctionCup::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorSuctionCup(req.enableCtrl, req.suck, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorSuctionCupService(magician_hardware::GetEndEffectorSuctionCup::Request &req, magician_hardware::GetEndEffectorSuctionCup::Response &res)
{
    bool enableCtrl, suck;

    res.result = GetEndEffectorLaser(&enableCtrl, &suck);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.suck = suck;
    }

    return true;
}

bool SetEndEffectorGripperService(magician_hardware::SetEndEffectorGripper::Request &req, magician_hardware::SetEndEffectorGripper::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorGripper(req.enableCtrl, req.grip, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorGripperService(magician_hardware::GetEndEffectorGripper::Request &req, magician_hardware::GetEndEffectorGripper::Response &res)
{
    bool enableCtrl, grip;

    res.result = GetEndEffectorLaser(&enableCtrl, &grip);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.grip = grip;
    }

    return true;
}

void InitEndEffectorServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetEndEffectorParams", SetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorParams", GetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEndEffectorLaser", SetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorLaser", GetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEndEffectorSuctionCup", SetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorSuctionCup", GetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEndEffectorGripper", SetEndEffectorGripperService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorGripper", GetEndEffectorGripperService);
    serverVec.push_back(server);
}

/*
 * JOG
 */
#include "magician_hardware/SetJOGJointParams.h"
#include "magician_hardware/GetJOGJointParams.h"
#include "magician_hardware/SetJOGCoordinateParams.h"
#include "magician_hardware/GetJOGCoordinateParams.h"
#include "magician_hardware/SetJOGCommonParams.h"
#include "magician_hardware/GetJOGCommonParams.h"
#include "magician_hardware/SetJOGCmd.h"

bool SetJOGJointParamsService(magician_hardware::SetJOGJointParams::Request &req, magician_hardware::SetJOGJointParams::Response &res)
{
    JOGJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGJointParamsService(magician_hardware::GetJOGJointParams::Request &req, magician_hardware::GetJOGJointParams::Response &res)
{
    JOGJointParams params;

    res.result = GetJOGJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCoordinateParamsService(magician_hardware::SetJOGCoordinateParams::Request &req, magician_hardware::SetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCoordinateParamsService(magician_hardware::GetJOGCoordinateParams::Request &req, magician_hardware::GetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;

    res.result = GetJOGCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCommonParamsService(magician_hardware::SetJOGCommonParams::Request &req, magician_hardware::SetJOGCommonParams::Response &res)
{
    JOGCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetJOGCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCommonParamsService(magician_hardware::GetJOGCommonParams::Request &req, magician_hardware::GetJOGCommonParams::Response &res)
{
    JOGCommonParams params;

    res.result = GetJOGCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool SetJOGCmdService(magician_hardware::SetJOGCmd::Request &req, magician_hardware::SetJOGCmd::Response &res)
{
    JOGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.isJoint = req.isJoint;
    cmd.cmd = req.cmd;
    res.result = SetJOGCmd(&cmd, false, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitJOGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetJOGJointParams", SetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetJOGJointParams", GetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetJOGCoordinateParams", SetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetJOGCoordinateParams", GetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetJOGCommonParams", SetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetJOGCommonParams", GetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetJOGCmd", SetJOGCmdService);
    serverVec.push_back(server);
}

/*
 * PTP
 */
#include "magician_hardware/SetPTPJointParams.h"
#include "magician_hardware/GetPTPJointParams.h"
#include "magician_hardware/SetPTPCoordinateParams.h"
#include "magician_hardware/GetPTPCoordinateParams.h"
#include "magician_hardware/SetPTPJumpParams.h"
#include "magician_hardware/GetPTPJumpParams.h"
#include "magician_hardware/SetPTPCommonParams.h"
#include "magician_hardware/GetPTPCommonParams.h"
#include "magician_hardware/SetPTPCmd.h"

bool SetPTPJointParamsService(magician_hardware::SetPTPJointParams::Request &req, magician_hardware::SetPTPJointParams::Response &res)
{
    PTPJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetPTPJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPJointParamsService(magician_hardware::GetPTPJointParams::Request &req, magician_hardware::GetPTPJointParams::Response &res)
{
    PTPJointParams params;

    res.result = GetPTPJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetPTPCoordinateParamsService(magician_hardware::SetPTPCoordinateParams::Request &req, magician_hardware::SetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetPTPCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPCoordinateParamsService(magician_hardware::GetPTPCoordinateParams::Request &req, magician_hardware::GetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;

    res.result = GetPTPCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool SetPTPJumpParamsService(magician_hardware::SetPTPJumpParams::Request &req, magician_hardware::SetPTPJumpParams::Response &res)
{
    PTPJumpParams params;
    uint64_t queuedCmdIndex;

    params.jumpHeight = req.jumpHeight;
    params.zLimit = req.zLimit;
    res.result = SetPTPJumpParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPJumpParamsService(magician_hardware::GetPTPJumpParams::Request &req, magician_hardware::GetPTPJumpParams::Response &res)
{
    PTPJumpParams params;

    res.result = GetPTPJumpParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.jumpHeight = params.jumpHeight;
        res.zLimit = params.zLimit;
    }

    return true;
}

bool SetPTPCommonParamsService(magician_hardware::SetPTPCommonParams::Request &req, magician_hardware::SetPTPCommonParams::Response &res)
{
    PTPCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetPTPCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPCommonParamsService(magician_hardware::GetPTPCommonParams::Request &req, magician_hardware::GetPTPCommonParams::Response &res)
{
    PTPCommonParams params;

    res.result = GetPTPCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool SetPTPCmdService(magician_hardware::SetPTPCmd::Request &req, magician_hardware::SetPTPCmd::Response &res)
{
    PTPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = req.ptpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.r = req.r;
    res.result = SetPTPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitPTPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetPTPJointParams", SetPTPJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPJointParams", GetPTPJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPCoordinateParams", SetPTPCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPCoordinateParams", GetPTPCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPJumpParams", SetPTPJumpParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPJumpParams", GetPTPJumpParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPCommonParams", SetPTPCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPCommonParams", GetPTPCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPCmd", SetPTPCmdService);
    serverVec.push_back(server);
}

/*
 * CP
 */
#include "magician_hardware/SetCPParams.h"
#include "magician_hardware/GetCPParams.h"
#include "magician_hardware/SetCPCmd.h"

bool SetCPParamsService(magician_hardware::SetCPParams::Request &req, magician_hardware::SetCPParams::Response &res)
{
    CPParams params;
    uint64_t queuedCmdIndex;

    params.planAcc = req.planAcc;
    params.juncitionVel = req.junctionVel;
    params.acc = req.acc;
    params.realTimeTrack = req.realTimeTrack;
    res.result = SetCPParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetCPParamsService(magician_hardware::GetCPParams::Request &req, magician_hardware::GetCPParams::Response &res)
{
    CPParams params;

    res.result = GetCPParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.planAcc = params.planAcc;
        res.junctionVel = params.juncitionVel;
        res.acc = params.acc;
        res.realTimeTrack = params.realTimeTrack;
    }

    return true;
}

bool SetCPCmdService(magician_hardware::SetCPCmd::Request &req, magician_hardware::SetCPCmd::Response &res)
{
    CPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cpMode = req.cpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.velocity = req.velocity;

    res.result = SetCPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitCPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetCPParams", SetCPParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetCPParams", GetCPParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetCPCmd", SetCPCmdService);
    serverVec.push_back(server);
}

/*
 * ARC
 */
#include "magician_hardware/SetARCParams.h"
#include "magician_hardware/GetARCParams.h"
#include "magician_hardware/SetARCCmd.h"

bool SetARCParamsService(magician_hardware::SetARCParams::Request &req, magician_hardware::SetARCParams::Response &res)
{
    ARCParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetARCParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetARCParamsService(magician_hardware::GetARCParams::Request &req, magician_hardware::GetARCParams::Response &res)
{
    ARCParams params;

    res.result = GetARCParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool SetARCCmdService(magician_hardware::SetARCCmd::Request &req, magician_hardware::SetARCCmd::Response &res)
{
    ARCCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cirPoint.x = req.x1;
    cmd.cirPoint.y = req.y1;
    cmd.cirPoint.z = req.z1;
    cmd.cirPoint.r = req.r1;
    cmd.toPoint.x = req.x2;
    cmd.toPoint.y = req.y2;
    cmd.toPoint.z = req.z2;
    cmd.toPoint.r = req.r2;

    res.result = SetARCCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitARCServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetARCParams", SetARCParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetARCParams", GetARCParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetARCCmd", SetARCCmdService);
    serverVec.push_back(server);
}

/*
 * WAIT
 */
#include "magician_hardware/SetWAITCmd.h"

bool SetWAITCmdService(magician_hardware::SetWAITCmd::Request &req, magician_hardware::SetWAITCmd::Response &res)
{
    WAITCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.timeout = req.timeout;
    res.result = SetWAITCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitWAITServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetWAITCmd", SetWAITCmdService);
    serverVec.push_back(server);
}

/*
 * TRIG
 */
#include "magician_hardware/SetTRIGCmd.h"

bool SetTRIGCmdService(magician_hardware::SetTRIGCmd::Request &req, magician_hardware::SetTRIGCmd::Response &res)
{
    TRIGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.address = req.address;
    cmd.mode = req.mode;
    cmd.condition = req.condition;
    cmd.threshold = req.threshold;
    res.result = SetTRIGCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitTRIGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetTRIGCmd", SetTRIGCmdService);
    serverVec.push_back(server);
}

/*
 * EIO
 */
#include "magician_hardware/SetIOMultiplexing.h"
#include "magician_hardware/GetIOMultiplexing.h"
#include "magician_hardware/SetIODO.h"
#include "magician_hardware/GetIODO.h"
#include "magician_hardware/SetIOPWM.h"
#include "magician_hardware/GetIOPWM.h"
#include "magician_hardware/GetIODI.h"
#include "magician_hardware/GetIOADC.h"
#include "magician_hardware/SetEMotor.h"
#include "magician_hardware/SetInfraredSensor.h"
#include "magician_hardware/GetInfraredSensor.h"
#include "magician_hardware/SetColorSensor.h"
#include "magician_hardware/GetColorSensor.h"

bool SetIOMultiplexingService(magician_hardware::SetIOMultiplexing::Request &req, magician_hardware::SetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;
    uint64_t queuedCmdIndex;

    ioMultiplexing.address = req.address;
    ioMultiplexing.multiplex = req.multiplex;
    res.result = SetIOMultiplexing(&ioMultiplexing, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIOMultiplexingService(magician_hardware::GetIOMultiplexing::Request &req, magician_hardware::GetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;

    ioMultiplexing.address = req.address;
    res.result = GetIOMultiplexing(&ioMultiplexing);
    if (res.result == DobotCommunicate_NoError) {
        res.multiplex = ioMultiplexing.multiplex;
    }

    return true;
}

bool SetIODOService(magician_hardware::SetIODO::Request &req, magician_hardware::SetIODO::Response &res)
{
    IODO ioDO;
    uint64_t queuedCmdIndex;

    ioDO.address = req.address;
    ioDO.level = req.level;
    res.result = SetIODO(&ioDO, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIODOService(magician_hardware::GetIODO::Request &req, magician_hardware::GetIODO::Response &res)
{
    IODO ioDO;

    ioDO.address = req.address;
    res.result = GetIODO(&ioDO);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDO.level;
    }

    return true;
}

bool SetIOPWMService(magician_hardware::SetIOPWM::Request &req, magician_hardware::SetIOPWM::Response &res)
{
    IOPWM ioPWM;
    uint64_t queuedCmdIndex;

    ioPWM.address = req.address;
    ioPWM.frequency = req.frequency;
    ioPWM.dutyCycle = req.dutyCycle;
    res.result = SetIOPWM(&ioPWM, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIOPWMService(magician_hardware::GetIOPWM::Request &req, magician_hardware::GetIOPWM::Response &res)
{
    IOPWM ioPWM;

    ioPWM.address = req.address;
    res.result = GetIOPWM(&ioPWM);
    if (res.result == DobotCommunicate_NoError) {
        res.frequency = ioPWM.frequency;
        res.dutyCycle = ioPWM.dutyCycle;
    }

    return true;
}

bool GetIODIService(magician_hardware::GetIODI::Request &req, magician_hardware::GetIODI::Response &res)
{
    IODI ioDI;

    ioDI.address = req.address;
    res.result = GetIODI(&ioDI);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDI.level;
    }

    return true;
}

bool GetIOADCService(magician_hardware::GetIOADC::Request &req, magician_hardware::GetIOADC::Response &res)
{
    IOADC ioADC;

    ioADC.address = req.address;
    res.result = GetIOADC(&ioADC);
    if (res.result == DobotCommunicate_NoError) {
        res.value = ioADC.value;
    }

    return true;
}

bool SetEMotorService(magician_hardware::SetEMotor::Request &req, magician_hardware::SetEMotor::Response &res)
{
    EMotor eMotor;
    uint64_t queuedCmdIndex;

    eMotor.index = req.index;
    eMotor.isEnabled = req.isEnabled;
    eMotor.speed = req.speed;
    res.result = SetEMotor(&eMotor, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}


bool SetInfraredSensorService(magician_hardware::SetInfraredSensor::Request &req, magician_hardware::SetInfraredSensor::Response &res)
{
    InfraredPort infraredPort = InfraredPort(req.infraredPort);
    res.result = SetInfraredSensor(req.enableCtrl, infraredPort);

    return true;
}

bool GetInfraredSensorService(magician_hardware::GetInfraredSensor::Request &req, magician_hardware::GetInfraredSensor::Response &res)
{
    uint8_t value;
    InfraredPort infraredPort = InfraredPort(req.infraredPort);
    res.result = GetInfraredSensor(infraredPort, &value);
    if (res.result == DobotCommunicate_NoError) {
        res.value = value;
    }

    return true;
}

bool SetColorSensorService(magician_hardware::SetColorSensor::Request &req, magician_hardware::SetColorSensor::Response &res)
{
    ColorPort colorPort = ColorPort(req.colorPort);
    res.result = SetColorSensor(req.enableCtrl, colorPort);

    return true;
}

bool GetColorSensorService(magician_hardware::GetColorSensor::Request &req, magician_hardware::GetColorSensor::Response &res)
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    res.result = GetColorSensor(&r, &g, &b);
    if (res.result == DobotCommunicate_NoError) {
        res.r = r;
        res.g = g;
        res.b = b;
    }

    return true;
}

void InitEIOServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetIOMultiplexing", SetIOMultiplexingService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIOMultiplexing", GetIOMultiplexingService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetIODO", SetIODOService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIODO", GetIODOService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetIOPWM", SetIOPWMService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIOPWM", GetIOPWMService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIODI", GetIODIService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIOADC", GetIOADCService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEMotor", SetEMotorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetInfraredSensor", SetInfraredSensorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetInfraredSensor", GetInfraredSensorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetColorSensor", SetColorSensorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetColorSensor", GetColorSensorService);
    serverVec.push_back(server);
}

/*
 * Queued command control
 */
#include "magician_hardware/SetQueuedCmdStartExec.h"
#include "magician_hardware/SetQueuedCmdStopExec.h"
#include "magician_hardware/SetQueuedCmdForceStopExec.h"
#include "magician_hardware/SetQueuedCmdClear.h"

bool SetQueuedCmdStartExecService(magician_hardware::SetQueuedCmdStartExec::Request &req, magician_hardware::SetQueuedCmdStartExec::Response &res)
{
    res.result = SetQueuedCmdStartExec();

    return true;
}

bool SetQueuedCmdStopExecService(magician_hardware::SetQueuedCmdStopExec::Request &req, magician_hardware::SetQueuedCmdStopExec::Response &res)
{
    res.result = SetQueuedCmdStopExec();

    return true;
}

bool SetQueuedCmdForceStopExecService(magician_hardware::SetQueuedCmdForceStopExec::Request &req, magician_hardware::SetQueuedCmdForceStopExec::Response &res)
{
    res.result = SetQueuedCmdForceStopExec();

    return true;
}

bool SetQueuedCmdClearService(magician_hardware::SetQueuedCmdClear::Request &req, magician_hardware::SetQueuedCmdClear::Response &res)
{
    res.result = SetQueuedCmdClear();

    return true;
}

void InitQueuedCmdServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetQueuedCmdStartExec", SetQueuedCmdStartExecService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetQueuedCmdStopExec", SetQueuedCmdStopExecService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetQueuedCmdForceStopExec", SetQueuedCmdForceStopExecService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetQueuedCmdClear", SetQueuedCmdClearService);
    serverVec.push_back(server);
}


int start_topic_srv_server(ros::NodeHandle &n)
{


    std::vector<ros::ServiceServer> serverVec;

    //InitCmdTimeoutServices(n, serverVec);
    //InitDeviceInfoServices(n, serverVec);
    //InitPoseServices(n, serverVec);
    //InitAlarmsServices(n, serverVec);
    //InitHOMEServices(n, serverVec);
    //InitEndEffectorServices(n, serverVec);
    //InitJOGServices(n, serverVec);
    //InitPTPServices(n, serverVec);
    //InitCPServices(n, serverVec);
    //InitARCServices(n, serverVec);
    //InitWAITServices(n, serverVec);
    //InitTRIGServices(n, serverVec);
    InitEIOServices(n, serverVec);
    //InitQueuedCmdServices(n, serverVec);

    ROS_INFO("Dobot service running...");
    ros::spin();
    ROS_INFO("Dobot service exiting...");


    return 0;
}

