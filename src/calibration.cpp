#include "camera_3dlidar_calibration/calibration.h"

calibration::calibration(ros::NodeHandle& nh)
{
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration::lidar_callBack, this);
    object_sub = nh.subscribe("person", 1, &calibration::object_callBack, this);

    this->do_cali();
}

void calibration::lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PCLPointCloud2 cloud_intermediate; // 포인트 클라우드 데이터 저장

    // 포인터가 가리키는 포인트 클라우드 데이터를 cloud_intermediate에 저장
    pcl_conversions::toPCL(*msg, cloud_intermediate);
    // cloud_intermediate에 저장된 PCL 포인트 클라우드 데이터를 cloud 객체로 변환
    pcl::fromPCLPointCloud2(cloud_intermediate, cloud);
    // std::vector<cv::Point3f>로 변환
    std::vector<cv::Point3f> objectPoints;

    // 포인트 변환
    for (size_t i = 0; i < cloud.size(); ++i) {
        objectPoints.emplace_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        intensity.emplace_back(cloud.points[i].intensity);
    }
    lidar_points = objectPoints;
}


void calibration::object_callBack(const sensor_msgs::Image::ConstPtr& msg)
{
    frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
    
    this->projection(frame);
    
    cv::imshow("Projection Image", frame);
    if (cv::waitKey(1) == 27) exit(-1);
}

cv::Mat_<double> calibration::computeRotationMatrix(double roll, double pitch, double yaw) {
    // 각도를 라디안으로 변환
    roll *= M_PI / 180.0;
    pitch *= M_PI / 180.0;
    yaw *= M_PI / 180.0;

    // 회전 행렬 계산
    cv::Mat_<double> R_x(3, 3);
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    cv::Mat_<double> R_y(3, 3);
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    cv::Mat_<double> R_z(3, 3);
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // 전체 회전 행렬
    return R_z * R_y * R_x;
}


void calibration::do_cali()
{
    // 카메라 내부 매트릭스, 회전 벡터, 변환 벡터 정의
    cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                            0,fy, cy,
                                            0, 0, 1);

    // 라이다 -> 카메라 좌표계로 일치시키는 행렬
    rvec = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1);

    rvec = rvec * this->computeRotationMatrix(90, -90, 0);

    tvec = (cv::Mat_<double>(3, 1) << lidar_x - camera_x, lidar_y - camera_y, lidar_z - camera_z); 
    distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // 왜곡 없음

    std::cout << "cameraMatrix: \n" << cameraMatrix << "\n\n";
    std::cout << "rvec: \n" << rvec << "\n\n";
    std::cout << "tvec: \n" << tvec << "\n\n";
    std::cout << "distCoeffs: \n" << distCoeffs << "\n\n";
}


void calibration::projection(cv::Mat frame)
{
    try{
        // lidar_points가 존재했을때 실행
        if(lidar_points.size())
        {
            // 이미지 포인트를 저장할 벡터
            std::vector<cv::Point2f> imagePoints;
            
            // 3D 포인트를 2D 이미지 평면으로 투영
            cv::projectPoints(lidar_points, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
            
            for (size_t i=0; i < imagePoints.size(); i++)
            {
                const auto& imagePoint = imagePoints[i];
                int x = static_cast<int>(imagePoint.x); // X 좌표
                int y = static_cast<int>(imagePoint.y); // Y 좌표

                cv::circle(frame, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1); // 점 찍기
            }
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Calibration ERROR! %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    ROS_INFO("Calibration on!");
    calibration cb(nh);

    ros::spin();

    return 0;
}