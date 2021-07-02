#include <magician_hardware/magician_hardware_interface.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <magician_hardware/magician_ros_services.h>


typedef struct{
    controller_manager::ControllerManager *manager;
    MagicianHWInterface *interface;
}ArgsForThread;

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

static boost::mutex reset_pose_mutex;
static boost::mutex stop_update_mutex;
static bool reseting_pose;
static bool stopping_update;

void *update_loop(void *threadarg)
{
    ArgsForThread *arg=(ArgsForThread *)threadarg;
    controller_manager::ControllerManager *manager=arg->manager;
    MagicianHWInterface *interface=arg->interface;
    ros::Duration d(0.016);
    //ros::Duration d(0.032); // changed by gerard
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);

    //time for checking overrun
    struct timespec before;
    double overrun_time;
    unsigned long int pass_cnt = 0;
    unsigned long int overrun_time_cnt = 0;


    while(ros::ok())
    {
        ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
        
        interface->read(this_moment, d);
        manager->update(this_moment, d);
        interface->write(this_moment, d);

        timespecInc(tick, d.nsec);
        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec)/1e+9) -  (tick.tv_sec + double(tick.tv_nsec)/1e+9);
        pass_cnt++;
        if(overrun_time > 0.0)
        {
            overrun_time_cnt++;
            tick.tv_sec=before.tv_sec;
            tick.tv_nsec=before.tv_nsec;
            //std::cout<<"overrun_time: "<<overrun_time<<std::endl;
        }
        if((pass_cnt % 10) == 0){
          std::cout<<"."<< std::flush;
          if((pass_cnt % 100) == 0){
            std::cout<<std::endl;
            std::cout<<"Pass : " << pass_cnt <<std::endl;
            std::cout<<"Overrun : " << overrun_time_cnt <<std::endl;
            std::cout<<std::endl;
          }
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, nullptr);
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, nullptr, nullptr);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot magicain not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            return -3;
        break;
        default:
        break;
    }
    #define BUFFER_LEN  32
    uint32_t maxLen = BUFFER_LEN;
    char deviceName[BUFFER_LEN];
    char deviceSN[BUFFER_LEN];
    uint8_t majorVersion, minorVersion, revision;
    
    //SetDeviceName("Avans Dobot Magician");
    
    GetDeviceName(deviceName, maxLen);
    GetDeviceSN(deviceSN, maxLen);
    GetDeviceVersion(&majorVersion, &minorVersion, &revision);

    std::cout<<"Connected to: " << deviceName << std::endl;
    std::cout<<"Serial number: " << deviceSN << std::endl;
    std::cout<<"Version: " << (int)majorVersion << "." << (int)minorVersion <<  std::endl;
    std::cout<<"Revision: " << (int)revision << std::endl;
    

    bool doHomeing = false;
    if (argc > 2)
      if( std::strcmp( argv[2], "doHomeing" ) == 0 )doHomeing = true;
    
    SetQueuedCmdClear();
    SetQueuedCmdStartExec();

    if(doHomeing){
      
      HOMECmd cmd;
      uint64_t queuedCmdIndex;

      ROS_INFO("Homeing Robot...");
      result = SetHOMECmd(&cmd, true, &queuedCmdIndex);
      if (result) {
              ROS_ERROR("Unable to home robot");
              return -4;
      }    
      
      uint64_t executedCmdIndex = 0;
      
      while(executedCmdIndex < queuedCmdIndex){
        sleep(1);
        GetQueuedCmdCurrentIndex(&executedCmdIndex);
      }
      

      ROS_INFO("Homeing Ready");
    }
    {
        PTPJointParams ptpJointParams;

    #define DEFAULT_velocity 500
    #define DEFAULT_acceleration 500
//    #define DEFAULT_velocity 5.58 //rad/s range 0 .. 5.58 rad/s 
//    #define DEFAULT_acceleration 2 //rad/s2 range 0.. ???? rad/s2

    
      ptpJointParams.velocity[0] = DEFAULT_velocity / RAD_PER_DEGREE;
      ptpJointParams.velocity[1] = DEFAULT_velocity / RAD_PER_DEGREE;
      ptpJointParams.velocity[2] = DEFAULT_velocity / RAD_PER_DEGREE; 
      ptpJointParams.velocity[3] = DEFAULT_velocity / RAD_PER_DEGREE;
      ptpJointParams.acceleration[0]= DEFAULT_acceleration / RAD_PER_DEGREE; 
      ptpJointParams.acceleration[1]= DEFAULT_acceleration / RAD_PER_DEGREE; 
      ptpJointParams.acceleration[2]= DEFAULT_acceleration / RAD_PER_DEGREE; 
      ptpJointParams.acceleration[3]= DEFAULT_acceleration / RAD_PER_DEGREE; 

      ROS_INFO("Setting PTP Joint Parameters...");

      uint64_t queuedCmdIndex;
          
      result = SetPTPJointParams(&ptpJointParams, true, &queuedCmdIndex);
      if(result){
          ROS_ERROR("Unable to set joint parameters");//, result : " << result <<std::endl;
      }

      uint64_t executedCmdIndex = 0;
        
      GetQueuedCmdCurrentIndex(&executedCmdIndex);
      while(executedCmdIndex < queuedCmdIndex){
          sleep(1);
          GetQueuedCmdCurrentIndex(&executedCmdIndex); 
          }

    }
    
    
    
    
    
    //SetQueuedCmdStopExec(); 
    
    
    
    ros::init(argc, argv, "magician_hardware_node", ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    MagicianHWInterface magician_hw_interface(nh);
    controller_manager::ControllerManager ctlr_manager(&magician_hw_interface);


    magician_hw_interface.init();
    
    ros::NodeHandle magician_control;
    start_magician_ros_services(magician_control);

    reseting_pose=false;
    stopping_update=false;

    pthread_t tid;
    ArgsForThread *thread_arg=new ArgsForThread();
    thread_arg->manager=&ctlr_manager;
    thread_arg->interface=&magician_hw_interface;

    pthread_create(&tid, nullptr, update_loop, thread_arg);

    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    stop_magician_ros_services(magician_control);
}
