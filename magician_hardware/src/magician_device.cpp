/*
Created on Thurs June 19 16:42 2019

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2019, Dobot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu


#include <magician_hardware/magician_device.h>

namespace magician_hardware {

MagicianDevice::MagicianDevice(unsigned long motor_num, std::vector<int> pulse_signs): local_nh_("~"), motor_num_(motor_num)
{
    joint_bases_.resize(motor_num_);
    joint_offsets_.resize(motor_num_);
    pulse_angles_.resize(motor_num_);

    for(size_t i=0; i<joint_bases_.size(); i++)
    {
        joint_bases_[i]=0;
        joint_offsets_[i]=0;
        pulse_angles_[i]=0;
    }

    pulse_signs_=pulse_signs;
}

MagicianDevice::~MagicianDevice()
{

}

bool MagicianDevice::InitPose()
{
    if(motor_num_>4 || pulse_signs_.size()!=motor_num_)
    {
        return false;
    }

    SetHHTTrigMode(TriggeredOnKeyReleased);
    SetHHTTrigOutputEnabled(true);

    Pose pose;
    int get_pose_times=0;
    int result=DobotCommunicate_InvalidParams;

    while (result!=DobotCommunicate_NoError) {
        result=GetPose(&pose);
        std::cout<<"InitPose"<<std::endl;

        get_pose_times++;
        if(get_pose_times>5)
        {
            return false;
        }
    }

    for(size_t i=0; i<joint_bases_.size(); i++)
    {
        joint_bases_[i]=pose.jointAngle[i]*RAD_PER_DEGREE;
        joint_offsets_[i]=0;
    }

    return true;
}

bool MagicianDevice::ResetPose(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp, std::vector<double> &joint_values)
{
    if(!req.data)
    {
        resp.message="Request's data is false";
        resp.success=false;
        return true;
    }

    Pose pose;
    int result=GetPose(&pose);
    std::cout<<"ResetPose"<<std::endl;
    if(result!=DobotCommunicate_NoError)
    {
        resp.message="Getting current pose failed";
        resp.success=false;
        return true;
    }

    bool pose_changed=false;
    double offset=0;
    for(size_t i=0; i<joint_bases_.size(); i++)
    {
        offset+=fabs(joint_bases_[i]-pose.jointAngle[i]*RAD_PER_DEGREE);
    }

    if(offset>0.1*RAD_PER_DEGREE)
    {
        pose_changed=true;
    }

    if(!pose_changed)
    {
        resp.message="Pose doesn't change";
        resp.success=false;
        return true;
    }

    for(size_t i=0; i<joint_bases_.size(); i++)
    {
        joint_bases_[i]=pose.jointAngle[i]*RAD_PER_DEGREE;
        joint_offsets_[i]=0;
    }

    joint_values=joint_bases_;

    resp.message="Resetting pose succeed";
    resp.success=true;
    return true;
}

bool MagicianDevice::ReadPose(std::vector<double> &joint_values)
{
    Pose pose;
    int result=GetPose(&pose);
#if 0
      std::cout<<"ReadPose"<<std::endl;
      std::cout<<"x: "<<pose.x<<std::endl;
      std::cout<<"y: "<<pose.y<<std::endl;
      std::cout<<"z: "<<pose.z<<std::endl;
      std::cout<<"r: "<<pose.r<<std::endl;
      std::cout<<"J1: "<<pose.jointAngle[0]<<std::endl;
      std::cout<<"J2: "<<pose.jointAngle[1]<<std::endl;
      std::cout<<"J3: "<<pose.jointAngle[2]<<std::endl;
      std::cout<<"J3: "<<pose.jointAngle[3]<<std::endl;
#endif

    bool pose_changed=true;  // chagend by gerard
    if(result==DobotCommunicate_NoError)
    {
        double offset=0;
        for(size_t i=0; i<joint_bases_.size(); i++)
        {
            offset+=fabs(joint_bases_[i]-pose.jointAngle[i]*RAD_PER_DEGREE);
        }

        if(offset>0.1*RAD_PER_DEGREE)
        {
            pose_changed=true;
        }
    }

    if(pose_changed)
    {
        for(size_t i=0; i<joint_bases_.size(); i++)
        {
            joint_bases_[i]=pose.jointAngle[i]*RAD_PER_DEGREE;
            joint_offsets_[i]=0;
        }
    }


//    bool isTriggered;
//    GetHHTTrigOutput(&isTriggered);

    joint_values.resize(motor_num_);
    for(size_t i=0; i<joint_bases_.size(); i++)
    {
        joint_values[i]=joint_bases_[i]+joint_offsets_[i];
    }

    return true;
}

bool MagicianDevice::WritePose(const std::vector<double> &joint_cmds)
{
    assert(joint_cmds.size()==motor_num_);

    std::vector<double> pulse_angles=joint_cmds;
    std::vector<double> pulses;
    pulses.resize(6);

#if 1
    std::cout<<"WritePose"<<std::endl;
#endif
      
    for (size_t i=0; i<pulse_angles.size(); i++)
    {
#if 1
        std::cout<<"joint_cmds "<< i << ": " << pulse_angles[i]<<std::endl;
#endif
        pulse_angles[i]-=joint_offsets_[i]+joint_bases_[i];
        pulses[i]=round(pulse_angles[i]*PULSE_PER_RAD*pulse_signs_[i]);
    }

    for(size_t i=pulse_angles.size(); i<pulses.size(); i++)
    {
        pulses[i]=0;
    }

    PluseCmd cmd;
    cmd.j1=pulses[0];
    cmd.j2=pulses[1];
    cmd.j3=pulses[2];
    cmd.j4=pulses[3];
    cmd.e1=pulses[4];
    cmd.e2=pulses[5];

    uint64_t index;

    int send_pulse_times=0;
    int result=DobotCommunicate_InvalidParams;
#if 0
    if(0){ // dit is nog niet de juiste methode
        PTPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = req.ptpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.r = req.r;
    int result = SetPTPCmd(&cmd, true, &queuedCmdIndex);
    if (result == DobotCommunicate_NoError) {
        //queuedCmdIndex = queuedCmdIndex;
    }
    }
#endif
    while(result!= DobotCommunicate_NoError && send_pulse_times<1) {
        result=SendPluse(&cmd, false, &index);
        send_pulse_times++;
    }

    if(result==DobotConnect_NoError)
    {
        for (size_t i=0; i<joint_offsets_.size(); i++)
        {
            pulse_angles_[i]=pulses[i]*RAD_PER_PULSE*pulse_signs_[i];
            joint_offsets_[i]+=pulse_angles_[i];
        }
        return true;
    }
    else
    {
        return false;
    }
}

void MagicianDevice::GetPulseAngle(std::vector<double> &pulse_angles)
{
    pulse_angles=pulse_angles_;
}

}
