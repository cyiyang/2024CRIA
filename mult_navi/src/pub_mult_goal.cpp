#include "tf/tf.h"
#include "cmath"
#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8MultiArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "image/int_str.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <cstring>
#include "char_recognizer/Letter_RecognizationMsg.h"
#include "pill_recognizer/Pill_RecognizationMsg.h"

using namespace std;

int state[4][4] = { {1, 4, 5, 0},
                    {2, 4, 6, 0},
                    {3, 4, 7, 0}};
bool next_pub = false, restore = false;     //到下一个点是否成功  标志位表示没有正确到达位置（卡住了）
uint i=0, j=0;  //状态下标
int server_mode; //识别模式
bool next_reponse = false;
string Pill_type[5] = {
    "fish_oil",
    "single_color",
    "oval",
    "double_color",
    "circle"
};

struct dispatch
{
    int n = 0;
    int key_var[4][2];
}state_message;


//生成调度
struct dispatch create_dispatch(string letter)
{
    struct dispatch dispatch;
    int len = letter.size() / 2;
    ROS_INFO("生成调度");
    for(int i = 0; i < len; i++)
    {
        if(letter[i*2] != 'N')//N对应为识别到的空框  不进行操作
        {
          dispatch.key_var[dispatch.n][0] = letter[i * 2] - 'A' + 1 ;  //药 A1 B2 C3 N4
          dispatch.key_var[dispatch.n][1] = letter[i * 2 + 1] - '0' -1; //地点 顺序返回1234表示四个框
          dispatch.n++;
        }
        else 
        {
          dispatch.key_var[dispatch.n][0] = 0;
          dispatch.key_var[dispatch.n][1] = letter[i * 2 + 1] - '0'; //地点        
        }

    }
    return dispatch;
}


void callback1(const move_base_msgs::MoveBaseActionResult::ConstPtr& callback1) //订阅导航结果
{
    if(callback1->status.status == 3)
    {   
        ROS_INFO("成功到达目标点");
        next_pub = true;
    }
    if(callback1->status.status == 4)
    {   
        ROS_WARN("未到达目标点,后退后重新进入");
        restore = true;
    }
}
// void callback2(const std_msgs::Int8MultiArray::ConstPtr& new_state)
// {
//     int x = 0, y = 0,z = 0;
//     for(x;x<3;x++)
//         for(y;y<3;y++)
//         {
//             state[x][y] = new_state->data[z];
//             z++;
//         }
// }

void playsound(int num)
{

    switch(num)
    {       
        case 1:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/peiyao/peiyaoA.wav");break;
        case 2:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/peiyao/peiyaoB.wav");break;
        case 3:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/peiyao/peiyaoC.wav");break;

        case 5:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/quyao/quyao1.wav");break;
        case 6:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/quyao/quyao2.wav");break;
        case 7:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/quyao/quyao3.wav");break;
        case 8:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/quyao/quyao4.wav");break;

        case 9:      system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/yaowan/yuganyou.wav");break;
        case 10:     system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/yaowan/dansejiaonang.wav");break;
        case 11:     system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/yaowan/tuoyuanyaowan.wav");break;
        case 12:     system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/yaowan/suangsejiaonang.wav");break;
        case 13:     system("echo ncut1234 | sudo -S chrt -r 1 aplay /home/EPRobot/catkin_ws/src/mult_navi/sound_lib/yaowan/yuanxingyaowan.wav");break;

    default:
        ROS_INFO("playsound function error!");
    }
}


// 回调函数，处理接收到的消息
// void stringCallback(const std_msgs::String::ConstPtr& msg)
void stringCallback(std::string res)
{
    // 打印接收到的字符串消息
    struct dispatch dispatch;
    std::string temp;
    // temp = res.letter_string;
    temp=res;
    ROS_INFO("接收到的消息是：%s",temp.c_str());
    if(temp.size() != 0)
    {   
        dispatch = create_dispatch(temp);
        ROS_INFO("结果为: [%s]", temp.c_str());
        state_message = dispatch;
        //next_reponse = true;
        //ros::shutdown();
    }
}

//导航
#define to_goal(point) \
    position[point].header.stamp = ros::Time::now();\
    pub_pose.publish(position[point]);\
    while(!next_pub && ros::ok())\
    {ros::spinOnce(); if(restore) {restore = false;pub_restore.publish(vel);ros::Duration(2).sleep();vel.linear.x = 0;pub_restore.publish(vel);vel.linear.x = -0.2;next_pub = true;}}\
    next_pub = false


int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    
    bool start = true;
    ros::init(argc, argv, "pub_pose");
    ros::NodeHandle nh;
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);   //发布move_base导航位姿
    ros::Publisher pub_restore = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);   //恢复
    ros::Subscriber sub_navi_result = nh.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result",1,callback1);  //订阅导航结果
     
    ros::ServiceClient string_sub = nh.serviceClient<char_recognizer::Letter_RecognizationMsg>("/letter_channel");
    ros::ServiceClient pill_pub   = nh.serviceClient<pill_recognizer::Pill_RecognizationMsg>("/pill_channel");

    //初始化导航点位姿
    geometry_msgs::PoseStamped position[9];
    tf2::Quaternion qtn[9];
    qtn[0].setRPY(0, 0, 0); qtn[4].setRPY(0, 0, M_PI);
    qtn[1].setRPY(0, 0, M_PI / 2); qtn[2].setRPY(0, 0, M_PI / 2); qtn[3].setRPY(0, 0, M_PI / 2);
    qtn[5].setRPY(0, 0, -M_PI / 2); qtn[6].setRPY(0, 0, -M_PI / 2); 
    qtn[7].setRPY(0, 0, -M_PI / 2);qtn[8].setRPY(0, 0, -M_PI / 2);
    
    for(int i = 0; i<9; i++)
    {
        position[i].header.frame_id = "map";
        position[i].pose.orientation.x = qtn[i].getX();
        position[i].pose.orientation.y = qtn[i].getY();
        position[i].pose.orientation.z = qtn[i].getZ();
        position[i].pose.orientation.w = qtn[i].getW();
        position[i].pose.position.z = 0;
    }
    //坐标点                            1pix =内测5cm
    position[0].pose.position.x = -0.01; position[0].pose.position.y = -0.2; //起点                  
    position[1].pose.position.x = 0.40;  position[1].pose.position.y = 2.00;//A
    position[2].pose.position.x = 1.27;  position[2].pose.position.y = 2.55;//B
    position[3].pose.position.x = 1.25;  position[3].pose.position.y = 1.60;//C
    position[4].pose.position.x = -0.87; position[4].pose.position.y = 3.65;//识别点
    position[5].pose.position.x = -1.85; position[5].pose.position.y = 2.09;//1
    position[6].pose.position.x = -1.13; position[6].pose.position.y = 1.62;//2
    position[7].pose.position.x =-2.00;  position[7].pose.position.y = 1.37;//3
    position[8].pose.position.x = -1.18; position[8].pose.position.y = 0.70;//4
    
    //初始化后退速度
    geometry_msgs::Twist vel;
    vel.angular.x = 0;vel.angular.y = 0;vel.angular.z = 0;
    vel.linear.x = -0.2;vel.linear.y = 0;vel.linear.z = 0;
    ROS_INFO("1");
    // ros::Rate r(1);
    ros::Duration(1).sleep();
    ROS_INFO("2");
    ROS_INFO("3");
    ROS_INFO("4");
    int temp_goal;
    ROS_INFO("rosok");

    int flag = 1;//recognize or not  (0 1s)
    while(ros::ok())
    {   
        ROS_INFO("rosok");
        //每一轮次开始进行字母识别

        if(flag == 1)
        {
            ros::service::waitForService("/letter_channel");
            char_recognizer::Letter_RecognizationMsg letter_srv;

            letter_srv.request.signal_letter = true;
            ROS_INFO("开始识别--->");
            // 
            if( string_sub.call(letter_srv) )
            {        
                if( string_sub.waitForExistence(ros::Duration(5)))//等待5s服务响应
                {   
                    //ROS_INFO("the request of call %d",letter_srv.request.signal_letter);
                    //ROS_INFO("[mult_goal]Received the response of letter:%s",letter_srv.response.letter_string.c_str());

                    stringCallback(letter_srv.response.letter_string);
                    ROS_INFO("识别成功");
                    ROS_INFO("Received the response of letter:%s",letter_srv.response.letter_string.c_str());
                    flag = 0; 
                }
                else//识别失败
                {
                    //后退
                    pub_restore.publish(vel);
                    vel.linear.x = -0.2;
                    //回到识别点
                    ROS_INFO("识别失败回到起始点");
                    to_goal(0);  
                }
           }
        }
    
        // if(next_reponse == false)
        //     continue;

        // if(get_dispatch.call(get_server))
        // {
        //     state_message =  create_dispatch(get_server);
        //     if(get_server.response.str.length() != 0) //结果为空
        //     {
        //         state_message =  create_dispatch(get_server);
        //         ROS_INFO("识别成功");
        //     }
        //     else//识别失败
        //     {
        //         //后退
        //         pub_restore.publish(vel);
        //         ros::Duration(2).sleep();
        //         vel.linear.x = 0;
        //         pub_restore.publish(vel);
        //         vel.linear.x = -0.2;
        //         //回到识别点
        //         ROS_INFO("识别失败回到起始点");
        //         to_goal(0);
        //     }  
        // }
       //测试直接赋值字母代码
        // ROS_INFO("state_message:%d",state_message.n);
        // ROS_INFO("开始配药");

        //执行state
        for(int i =0; i < state_message.n; i++)
        {
            //取药
            ROS_INFO("开始取药");
            temp_goal = state_message.key_var[i][0];

            if(temp_goal == 0) 
                continue;  //如果识别为空框 就进行下一轮
            
            to_goal(temp_goal);
            ROS_INFO("到达药点ABC");
            playsound(temp_goal);
            // r.sleep();
            ros::Duration(1).sleep();
            //数字点
            ROS_INFO("去药丸识别点");
            to_goal(4);
            //每一轮次进行数字识别
            ROS_INFO("到达药丸识别点");


            pill_recognizer::Pill_RecognizationMsg pill_srv;
            pill_srv.request.pill2see = true;//选择识别模式为 药丸识别
            if(pill_pub.call( pill_srv) )
            {   
                int pill_type = pill_srv.response.pilltype + 9;//药丸发送01234 在此处+9使其对应 playsopund中index

                // string index = Pill_type(pill_type)
                ROS_INFO("检测到药丸信息：%d",pill_type);
                playsound(pill_type);
            }
            else
            {
                ROS_ERROR("Errro occurred: Failed to call service string_sub");
            }

            //送药
            ROS_INFO("开始送药");
            temp_goal = state_message.key_var[i][1] + 5;
            to_goal(temp_goal);
            playsound(temp_goal);
            // r.sleep();
            ros::Duration(1).sleep();
            //起点
            ROS_INFO("回到起点");
            to_goal(0);
        }
        //清空调度消息
        state_message.n = 0;
        // next_reponse = false;
        flag = 1;
    }
  return 0;
}
