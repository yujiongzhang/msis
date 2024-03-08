#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "cv_bridge/cv_bridge.h"

#include "algorithm_sts1000.h"
#include "define_sts1000.h"
#include "user_math_lib.h"

#include <fstream>
#include <cmath>

#include <opencv2/opencv.hpp>


class sts1000_process : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    sts1000_process(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        image1.create(cv::Size(IMAGEWIDTH, IMAGEHEIGHT), CV_8UC3);
        image1.setTo(cv::Scalar(0, 0, 0)); // 将图像填充为黑色
        image1_center = cv::Point2f((image1.cols - 1)/2.0 , (image1.rows - 1)/2.0);
        scanRange = SCANRANGE;

        calDelta();
        InitColortable();

        memset(&currentPose, 0 ,sizeof(currentPose));
        memset(&lastPose,0,sizeof(lastPose));

        sts1000_raw_subscribe_ = this->create_subscription<sensor_msgs::msg::LaserScan>("sts1000_raw", 10, std::bind(&sts1000_process::sts1000_callback, this, std::placeholders::_1));
        pose_subscribe_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&sts1000_process::pose_callback, this, std::placeholders::_1));
        sts1000_image_publisher_ =  this->create_publisher<sensor_msgs::msg::Image>("sts1000_image", 10);

        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&sts1000_process::timer_callback, this));

    }

private:
    // 声明
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sts1000_raw_subscribe_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sts1000_image_publisher_;
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;


    uint m_dwDeltaX[SCANLINEMAX];// 半径增加 x轴增加多少
    uint m_dwDeltaY[SCANLINEMAX];
    uint m_mycolorTable[256];//颜色表

    cv::Mat image1;//
    cv::Point2f image1_center;
    float scanRange;
    cv::Vec3b* m_pBits;
    Pose_zyj currentPose;
    Pose_zyj lastPose;


    void timer_callback(){

        sensor_msgs::msg::Image image1_ros;
        std_msgs::msg::Header _header;
        cv_bridge::CvImage _cv_bridge;
        _header.stamp = this->get_clock() -> now();
        _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, image1);
        _cv_bridge.toImageMsg(image1_ros);

        sts1000_image_publisher_->publish(image1_ros);
        
    }


    // 收到 LaserScan 数据的回调函数
    void sts1000_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        //提取出数据
        std::vector<float> intensities = msg->intensities;
            // 创建一个大小为 intensities.size() 的 uchar* 数组
        uchar* intensitiesData = new uchar[intensities.size()];
        RCLCPP_INFO(this->get_logger(), "intensities.size():%ld ",intensities.size());

        double current_angle = msg->angle_min / M_PI * 180; // degree

        // 将 intensities 数据转换为 uchar* 类型
        for (size_t i = 0; i < intensities.size(); ++i) {
            intensitiesData[i] = static_cast<uchar>(intensities[i]); // 这里使用 static_cast 进行数据类型转换
        }

        

        //画图
        Scan(current_angle, intensitiesData, intensities.size(), SCAN_STEP);

        // sensor_msgs::msg::Image image1_ros;
        // std_msgs::msg::Header _header;
        // cv_bridge::CvImage _cv_bridge;
        // _header.stamp = this->get_clock() -> now();
        // _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, image1);
        // _cv_bridge.toImageMsg(image1_ros);

        // sts1000_image_publisher_->publish(image1_ros);

        delete[] intensitiesData;

    }



    // 收到话题数据的回调函数
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        //读取数据
        currentPose.x = msg->position.x;
        currentPose.y = msg->position.y;
        currentPose.z = msg->position.z;
        float q[4] = {static_cast<float>(msg->orientation.w), static_cast<float>(msg->orientation.x), \
                        static_cast<float>(msg->orientation.y), static_cast<float>(msg->orientation.z)};
        Conversion_Quaternion_to_Euler(q, &(currentPose.yaw), &(currentPose.pitch),&(currentPose.roll));

        RCLCPP_INFO(this->get_logger(), "last X %f   Y %f ",lastPose.x,lastPose.y);
        RCLCPP_INFO(this->get_logger(), "currentPose X %f   Y %f ",currentPose.x,currentPose.y);
        
        //去除误差
        
        //1 rov在水平面的坐标变化
        if(fabs(currentPose.x - lastPose.x) > POSE_X_RESOLUTION || fabs(currentPose.y - lastPose.y) > POSE_Y_RESOLUTION ){
            
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

            float delta_x = currentPose.x - lastPose.x;
            float delta_y = currentPose.y - lastPose.y;

            float delta_x_pixel = delta_x / SCANRANGE * IMAGESCANRANGE;
            float delta_y_pixel = delta_y / SCANRANGE * IMAGESCANRANGE;
            float warp_values[] = { 1.0, 0.0, delta_x_pixel, 0.0, 1.0, delta_y_pixel };
            cv::Mat translation_matrix = cv::Mat(2, 3, CV_32F, warp_values);
            cv::warpAffine(image1, image1, translation_matrix, image1.size());


            lastPose.x = currentPose.x;
            lastPose.y = currentPose.y;

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            RCLCPP_INFO(this->get_logger(),"Time taken: %f seconds" , duration.count());
        }
        //2、航向角变化
        if(fabs(currentPose.yaw - lastPose.yaw) > POSE_YAW_RESOLUTION){
            float delta_yaw = currentPose.yaw - lastPose.yaw;

            cv::Mat rotation_matix = getRotationMatrix2D(image1_center, delta_yaw, 1.0);
            cv::warpAffine(image1, image1, rotation_matix, image1.size());


            lastPose.yaw = currentPose.yaw;
        }

        //3、俯仰角、横滚角变化

        //4、rov深度变化



    }

    //不同角度半径增加1时 x,y坐标变化值
    void calDelta()
    {
        double Q = -90;
        double dQ;

        dQ = 360./(double)SCANLINEMAX;
        for( uint i=0;i<SCANLINEMAX;i++ )
        {
            double dx = cos( Q*M_PI/180. );
            double dy = sin( Q*M_PI/180. );

            m_dwDeltaX[i] = (uint32_t)(dx*65536+0.5);
            m_dwDeltaY[i] = (uint32_t)(dy*65536+0.5);
            Q += dQ;
        }
    }

    //从bin文件中读取设置好的颜色表
    void InitColortable()
    {
        // QFile file(":/sts1000resources/ctable.bin");
        // file.open(QIODevice::ReadOnly);
        // uchar data[768]; // 256 * 3 = 768

        // file.read((char*)data,768);

        // for(int i=0;i<256;i++)
        // {
        //     uint a =0xff000000+((data[i*3]<<16)+(data[i*3+1]<<8)+(data[i*3+2]));
        //     *(m_mycolorTable+i)=a;
        // }
        // file.close();

        std::ifstream file("/home/zyj/ctable.bin");
        if (file.is_open()) {
            uchar data[768];
            file.read((char*)data, 256 * 3);

            for (int i = 0; i < 256; i++) {
                uint32_t a = 0xff000000 + ((data[i*3] << 16) + (data[i*3+1] << 8) + data[i*3+2]);
                *(m_mycolorTable+i)=a;
                // 将a保存到你的m_mycolorTable数组中，这里使用cout做示例
                // std::cout << a << std::endl;
            }
            file.close();
        } else {
            RCLCPP_INFO(this->get_logger(), "Failed to open file");
        }


    }

    // 绘制扫描图像的函数
    void Scan(double angle, uchar *buf, uint len, double step){
        int index;
        int color_index;
        int r;
        int g;
        int b;

        while( angle>=360.0 ) angle-=360.0;

        int scans = (int)(SCANPERANGLE*step)+1;

        int idx = (int)(angle*SCANLINEMAX/360.0 - scans/2);

        if( idx<0 ) idx += SCANLINEMAX;

        if( len>IMAGESCANRANGE ) {len = IMAGESCANRANGE;}

        for( int i=0;i<scans;i++ )
        {
            uint32_t x1 = ORIGINALPX<<16; //增加数据精度，将数值扩大后再处理，最后缩回原来大小
            uint32_t y1 = ORIGINALPY<<16;

            for( uint j=0;j<len;j++ )
            {
                x1 += m_dwDeltaX[idx];
                y1 += m_dwDeltaY[idx];

                uint x = (uint)0x0000ffff&(x1>>16);
                uint y = (uint)0x0000ffff&(y1>>16);

                if( x>=IMAGEWIDTH || y>= IMAGEHEIGHT ) continue;
    //            if( x<0||y<0 ) continue;

                index = (y) * IMAGEWIDTH + x;


                //此处颜色对应表的有点问题，需要有王老师的代码后更改
                color_index = buf[j]*2;
                if(color_index < 258){
                    r = (m_mycolorTable[color_index]>>16)&0x000000ff;
                    g = (m_mycolorTable[color_index]>>8)&0x000000ff;
                    b = (m_mycolorTable[color_index])&0x000000ff;
                }
                else{
                    r = (m_mycolorTable[254]>>16)&0x000000ff;
                    g = (m_mycolorTable[254]>>8)&0x000000ff;
                    b = (m_mycolorTable[254])&0x000000ff;
                }

                // *(m_pBits+index) = qRgb(r,g,b);
               m_pBits = image1.ptr<cv::Vec3b>(y);
               m_pBits[x][0] = b;
               m_pBits[x][1] = g;
               m_pBits[x][2] = r;
            }
            idx++;
            //qDebug()<<idx<<endl;
            if( (uint)idx>=SCANLINEMAX ) idx = 0;
        }

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<sts1000_process>("sts1000_process");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

