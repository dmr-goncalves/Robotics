//
// Created by francisco on 10-03-2016.
//

#include "tr5_communication.h"

Tr5_communication::Tr5_communication(void)
{
    serialConn.setBaudrate(9600);
    serialConn.setPort("/dev/ttyUSB0");
    serialConn.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
    try {
        serialConn.open();
        ROS_INFO("Opened Serial Port");
    }
    catch (serial::IOException e)
    {
        ROS_INFO("Cenas");
    }


}


Tr5_communication::Tr5_communication(std::string port, u_int32_t baudrate, int timeout)
{

    serialConn.setBaudrate(baudrate);
    serialConn.setPort(port);
    serialConn.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);
    serialConn.setFlowcontrol(serial::flowcontrol_none);
    serialConn.setParity(serial::parity_none);

    try {
        serialConn.open();
        ROS_INFO("Opened Serial Port");
    }
    catch (serial::IOException e)
    {
        ROS_INFO("Cenas");
    }

    //Init all the necessary vectors
    std::string arm_name_ = "tr5";
    // Populate joints in this arm
    joint_names_.push_back(arm_name_+"shoulder_pan_joint");
    joint_names_.push_back(arm_name_+"shoulder_lift_joint");
    joint_names_.push_back(arm_name_+"elbow_joint");
    joint_names_.push_back(arm_name_+"wrist_1_joint");
    joint_names_.push_back(arm_name_+"wrist_2_joint");
    joint_names_.push_back(arm_name_+"gripper_1_joint");
    joint_names_.push_back(arm_name_+"gripper_2_joint");

    n_dof_ = joint_names_.size();

    // Resize vectors
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);
    joint_velocity_command_.resize(n_dof_);
    joint_step_command_.resize(n_dof_);

    jointCommand_.position.resize(n_dof_);
    jointCommand_.velocity.resize(n_dof_);
    jointCommand_.name.resize(n_dof_);

    jointStatus_.position.resize(n_dof_);
    jointStatus_.velocity.resize(n_dof_);
    jointStatus_.name.resize(n_dof_);

    for (std::size_t i = 0; i < n_dof_; ++i)
    {
        joint_position_[i] = 0.0;
        joint_velocity_[i] = 0.0;
        joint_effort_[i] = 0.0;
        joint_position_command_[i] = 0.0;
        joint_effort_command_[i] = 0.0;
        joint_velocity_command_[i] = 0.0;
    }

    jointStatus_.name = joint_names_;

}


Tr5_communication::~Tr5_communication(void)
{

}

double Tr5_communication::convStepsToAngle(int pos, int axis){
    switch(axis)
    {
        case 1: return (pos*160.0/256.0);
        case 2:
        case 3: return (pos*100.0/256.0);
        case 4:
        case 5: return (pos*200.0/256.0);
        case 6: return (pos*60.0/256.0);
    }

    return 0;
}


int Tr5_communication::convAngleToSteps(double Angle, int axis){
    switch(axis)
    {
        case 1:	return (static_cast <int> ( std::max(1.0, ( 255 - (Angle + 1.396263402) * 255.0 / 2.792526803) ))); // Angle is inverted
        case 2: return (static_cast <int> ( std::max(1.0, ( 255 - (Angle + 0.5235987756) * 255.0 / 1.74533) ))); // Lack protection for 0 and 256
        case 3: return (static_cast <int> ( std::max(1.0, (-  (Angle + 0.0)  * 255.0 /1.74533) )));
        case 4:
        case 5: return (static_cast <int> ( std::max(1.0, ((Angle + 1.745329252) * 255.0 / 3.490658504) ))); // Lack protection for 0 and 256
        case 6: return (static_cast <int> ( std::max(1.0,(Angle*256.0/1.0472))));
    }

    return 0;
}


int Tr5_communication::degrees_to_steps(double degrees, int axis)
{
    double stepF;
    int step;

    switch (axis)
    {
        case 1:
            degrees = degrees - 80;
            stepF = degrees / -0.62745098039215686274509803921569;
            step = (int)(stepF + 0.5);
            break;
        case 2:
            degrees = 66.6666666666666666666666666666666 - degrees;
            stepF = degrees / 0.3921568627450980392156862745098;
            step = (int)(stepF + 0.5);
            break;
        case 3:
            degrees = 0 - degrees;
            stepF = degrees / 0.3921568627450980392156862745098;
            step = (int)(stepF + 0.5);
            break;
        case 4:
            degrees = 100 - degrees;
            stepF = degrees / 0.78431372549019607843137254901961;
            step = (int)(stepF + 0.5);
            break;
        case 5:
            degrees = 0 - degrees;
            //degrees = 200 - degrees;
            stepF = degrees / 0.78431372549019607843137254901961;
            step = (int)(stepF + 0.5);
            break;
        default:
            break;
    }
    if (step > 255){
        return -1;
    }
    else return step;
}


void Tr5_communication::moveAxis(int axis, int steps)
{
//    char command[10]={0x8+axis-1,steps,3,0};
//    std::cout << "Sending Command to Robot to move axis "<< axis << " Steps " << steps << std::endl;
//    std::cout << "Sending number of bytes " << serialConn.write(command) << std::endl;
//    std::cout << "Sending number of bytes " << serialConn.write(command) << std::endl;
//
//    usleep(5000);
//    //robot will respond with 15
//    std::string  buffer1;
//    size_t  t = 2;
//    std::cout << "robot responded with number of bytes" <<  serialConn.read(buffer1, t) << std::endl;
//    std::cout << "The received buffer is 1 " <<  buffer1 << std::endl;


    char command1[20] = {  0x08 + axis - 1, steps, 3, '\n', '\r' };
    std::cout << "Sending Command to Robot to move axis "<< axis << " Steps " << steps << std::endl;
    std::cout << "Sending number of bytes " << serialConn.write(command1) << std::endl;
    std::cout << "Sending number of bytes " << serialConn.write(command1) << std::endl;

    usleep(5000);
    //robot will respond with 15
    std::string  buffer1;
    size_t  t = 2;
    std::cout << "robot responded with number of bytes" <<  serialConn.read(buffer1, t) << std::endl;
    std::cout << "The received buffer is " <<  buffer1 << std::endl;

}

void Tr5_communication::moveMultipleAxis(std::vector<int> steps)
{

    for (int i = 0; i < steps.size() - 1; i++) {
        std::cout << " Command " << steps[i] << std::endl;
    }
    char command[]={0xF,steps[0],steps[1],steps[2],steps[3],steps[4],steps[5],3, '\n', '\r' };
    std::cout << " Command " << command << std::endl;
    std::cout << "Sending number of bytes " << serialConn.write(command) << std::endl;
    std::cout << "Sending number of bytes " << serialConn.write(command) << std::endl;
    usleep(5000);
    //robot will respond with 15
    std::string  buffer1;
    size_t  t = 2;
    std::cout << "robot responded with number of bytes" <<  serialConn.read(buffer1, t) << std::endl;
    std::cout << "The received buffer is 2" <<  buffer1 << std::endl;
}

int Tr5_communication::mm_to_steps(double distance)
{
    int steps = 0;
    double stepsf;

    distance = distance - 60;
    stepsf = distance / -0.2352941176470588;
    steps = (int)(stepsf + 0.5);

    return steps;
}

void Tr5_communication::moveAxisSpeed(int axis, int steps, int time)
{
//    int tam;
//    char Buff[20];
//    char comando[] = {0x78+axis-1,steps, time, 3,0};
//    Cp->Enviar(comando,5,tam);
//    printf("\n%s...", Cp->GetMensagem() );
//    Sleep(50);
//    Cp->EsperarRecepcao();
//    Cp->Receber(Buff,2,tam);
//    printf("\n%s...", Cp->GetMensagem() );


}

void Tr5_communication::moveMulAxisSpeed(std::vector<int> steps, int* time)
{
    int tam;
    char Buff[20];
    char command[]={0x7F,steps[0],steps[1],steps[2],steps[3],steps[4],steps[5],
                    time[0], time[1], time[2], time[3], time[4], time[5], 3,0};
    std::cout << "Sending number of bytes " << serialConn.write(command) << std::endl;
    std::cout << "Sending number of bytes " << serialConn.write(command) << std::endl;

    usleep(5000);
    //robot will respond with 15
    std::string  buffer1;
    size_t  t = 2;
    std::cout << "robot responded with number of bytes" <<  serialConn.read(buffer1, t) << std::endl;
    std::cout << "The received buffer is 3" <<  buffer1 << std::endl;
}

bool Tr5_communication::Initialize()
{
//    char command[]={32, 1, '\n', '\r' };
//    char command[]={'  ', 1, '\r' };
    std::cout << "Initializing the communication with the TR5 bytes written " << serialConn.write("  ") << std::endl;
    std::cout << "Initializing the communication with the TR5 bytes written " << serialConn.write("  ") << std::endl;
    usleep(500);
    std::string buff;
    std::cout << "The received buffer is:  " <<  serialConn.read(buff,1) << std::endl;

    std::cout << "Buffer-> " << buff << std::endl;
    return true;
}


void Tr5_communication::moveAxisJointState(sensor_msgs::JointState commandMsg){


    std::cout << " Position Size " << commandMsg.position.size() << " Joint command size " << joint_step_command_.size() <<std::endl;
    if (commandMsg.position.size() > 0 && joint_step_command_.size() > 0 ) {
        for (int i = 0; i < commandMsg.position.size() ; i++) {
            joint_step_command_[i] = convAngleToSteps(commandMsg.position[i], i + 1);
           // std::cout << " Pos in rads: " << commandMsg.position[i] << " / Command in steps " <<  joint_step_command_[i] << std::endl;
        }
    //    if (commandMsg.velocity.at(0) == 0) {
            moveAxis(6, joint_step_command_[5]);
//            moveMultipleAxis(joint_step_command_);
            std::cout << " Moving Multiple Axis " << std::endl;
    //    }
    //    else
    //        std::cout << " Speed command not yet implemented " << std::endl;
    }

}

void Tr5_communication::close()
{

}


int Tr5_communication::axisPosition(int axis)
{
    char command[] = {0x40+axis-1,3};

    std::cout << "Getting axis position " << serialConn.write(command) << std::endl;
    usleep(5000);
    std::string  buffer1;
    size_t  t = 2;
    std::cout << "robot responded with number of bytes" <<  serialConn.read(buffer1, t) << std::endl;
    std::cout << "The received buffer is 5" <<  buffer1 << std::endl;

}

sensor_msgs::JointState Tr5_communication::multipleAxisPosition()
{

    sensor_msgs::JointState joint_message;
    char command[] = {0x47,3};
    int* vecPos = new int[6];

    std::cout << "Getting axis position " << serialConn.write(command) << std::endl;
    usleep(5000);
    std::string  buffer1;
    size_t  t = 2;
    std::cout << "robot responded with number of bytes" <<  serialConn.read(buffer1, t) << std::endl;
    std::cout << "The received buffer is 6" <<  buffer1 << std::endl;


    for(int i = 0 ; i < 6; i++)
    {
        vecPos[i] = (int)(buffer1[(i+1)]);
        jointStatus_.position[i] = convStepsToAngle(i+1, (int)(buffer1[(i+1)]) );
       // std::cout << " Pos " << vecPos[i] << std::endl;
    }
    return jointStatus_; //@todo - need to put a setter and getter for this
}
