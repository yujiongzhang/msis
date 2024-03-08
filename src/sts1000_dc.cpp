#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "define_sts1000.h"

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>
#include<arpa/inet.h>
#include <cmath>


class sts1000_dc : public rclcpp::Node
{
private:
    // 声明话题发布者
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sts1000_raw_publisher_;

    ANSHEAD_STS1000 anshead;//机械扫描声纳帧头

    float scanRange;
public:
    sts1000_dc(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        scanRange = SCANRANGE;
        sts1000_raw_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("sts1000_raw", 10);


        // 1. 创建通信的套接字
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(fd == -1)
        {
            perror("socket");
            exit(0);
        }
        
        // 初始化服务器地址信息
        struct sockaddr_in seraddr;
        seraddr.sin_family = AF_INET;
        seraddr.sin_port = htons(5555);    // 大端
        inet_pton(AF_INET, "192.168.1.7", &seraddr.sin_addr.s_addr);

        char buf[2048];

        struct sockaddr_in cliaddr;
        cliaddr.sin_family = AF_INET;
        cliaddr.sin_addr.s_addr = INADDR_ANY;
        cliaddr.sin_port = htons(8888);
        // inet_pton(AF_INET, "127.0.0.1", &seraddr.sin_addr.s_addr);
        int ret = bind(fd, (struct sockaddr*)&cliaddr, sizeof(cliaddr));//客户端可以绑定为固定端口也可以不
        if(ret == -1)
        {
            perror("bind");
            exit(0);
        }
        
        // int len = sizeof(cliaddr);
        // int num = 0;
        // 2. 通信
        while(rclcpp::ok())
        {
            // 接收数据
            memset(buf, 0, sizeof(buf));
            int recvlen = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);

            if(recvlen % ANSPKTSIZE == 0)
            {
                memcpy(&anshead, buf, sizeof(ANSHEAD_STS1000));
                // recvlen -= ANSPKTSIZE;
                if( anshead.ucHeader == 0xfe && anshead.ucID == 0x01 && anshead.ucRollHi == 0x00 && anshead.ucRollLo == 0x00 )
                {

                    int data_len = static_cast<uint>(((anshead.ucDataHi&0x7f)<<7)|(anshead.ucDataLo&0x7f));
                    int glb_nAngle = static_cast<uint>((((anshead.ucAngleHi&0x7f)<<7)|(anshead.ucAngleLo&0x7f))/80);

                    
                    RCLCPP_INFO(this->get_logger(), "data_len: %d, glb_nAngle: %d",data_len, glb_nAngle);
                    

                    // 创建消息
                    sensor_msgs::msg::LaserScan sts1000_ping;
                    sts1000_ping.header.stamp = rclcpp::Clock().now();
                    sts1000_ping.header.frame_id = "sts1000";
                    sts1000_ping.angle_min = glb_nAngle /200.0 * 2* M_PI;//弧度
                    sts1000_ping.angle_max = glb_nAngle /200.0 * 2* M_PI;
                    sts1000_ping.angle_increment = 0.0;
                    sts1000_ping.time_increment = 0.0;
                    sts1000_ping.scan_time = 0.0;
                    sts1000_ping.range_min = 0.0;
                    sts1000_ping.range_max = scanRange;
                    for (int i = 0; i < 500; ++i) {
                        sts1000_ping.ranges.push_back( 0.02+0.02*i);
                        sts1000_ping.intensities.push_back(buf[ sizeof(ANSHEAD_STS1000)+ i]);
                    }
                    sts1000_raw_publisher_->publish(sts1000_ping);


                }
            }
        }

        close(fd);
    }




};


int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<sts1000_dc>("sts1000_dc");
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
}
