#include "camera_3dlidar_calibration/LiDAR_pre.h"

//lidar 각도 pitch 10도

// 생성자
LiDAR_pre::LiDAR_pre(ros::NodeHandle& nh)
{
    ROS_INFO("Lidar_pre on!");
    
    point_sub_ = nh.subscribe("/velodyne_points", 1, &LiDAR_pre::cloud_callBack, this);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_pre", 1);
}

Eigen::Matrix3d LiDAR_pre::computeRotationMatrix(double roll, double pitch, double yaw) {
    // 각도를 라디안으로 변환
    roll *= M_PI / 180.0;
    pitch *= M_PI / 180.0;
    yaw *= M_PI / 180.0;

    // 회전 행렬 계산
    Eigen::Matrix3d  R_x;
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    Eigen::Matrix3d  R_y;
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d  R_z;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // 전체 회전 행렬
    return R_z * R_y * R_x;
}

// Pointer -> PointCloud2로 변경하는 함수
void LiDAR_pre::Pub2Sensor(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointer, ros::Time t)
{
    pcl::PCLPointCloud2 pcl_pc_filtered;

    pcl::toPCLPointCloud2(*pcl_pointer, pcl_pc_filtered);

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(pcl_pc_filtered, output);

    output.header.frame_id = "map";
    output.header.stamp = t;//ros::Time::now();

    pub.publish(output);
}

// 포인트 클라우드를 ROS 메시지로 변환하여 발행
void LiDAR_pre::Pub2Sensor(pcl::PointCloud<pcl::PointXYZI> pc2, ros::Time t)
{
    pcl::PCLPointCloud2 pcl_pc_filtered;

    pcl::toPCLPointCloud2(pc2, pcl_pc_filtered);

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(pcl_pc_filtered, output);

    output.header.frame_id = "map";
    output.header.stamp = t;//ros::Time::now();

    pub.publish(output);
}


void LiDAR_pre::cloud_callBack(const sensor_msgs::PointCloud2& msg)
{
    pcl::fromROSMsg(msg, cloud_data); // ROS 메시지를 PCL 포인트 클라우드로 변환

    // ROS_INFO("Lidar callback");

    // 포인트 클라우드 변환
    for (auto& point : cloud_data.points) {
        Eigen::Vector3d p(point.x, point.y, point.z);
        p = computeRotationMatrix(0, 0, 0) * p; // 회전 변환(roll, pitch, yaw) 적용
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
    }
    
   this->roi(); // 관심 영역 설정
   this->voxel(); // Down sampling(voxel) 실행
//    this->outlier(); // outlier 제거
   this->ransac(); // ransac 실행
   this->dbscan(EPSILON, MIN_POINTS);

   Pub2Sensor(cloud_data, msg.header.stamp);
}



void LiDAR_pre::roi()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_data_p_ = cloud_data.makeShared();

    pcl::PassThrough<pcl::PointXYZI> pass;

    // Apply Passthrough Filter
    pass.setInputCloud(raw_data_p_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.7, 50);   // 상하거리
    pass.filter(*raw_data_p_);

    pass.setInputCloud(raw_data_p_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0, 10);  // 앞뒤거리
    pass.filter(*raw_data_p_);

    pass.setInputCloud(raw_data_p_);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10.0, 10.0); //좌우거리
    pass.filter(*raw_data_p_);

    cloud_data = *raw_data_p_; // 필터링된 데이터를 cloud_data에 저장
}

void LiDAR_pre::voxel()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_data_p_ = cloud_data.makeShared();

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(roi_data_p_);              //입력
    sor.setLeafSize(0.1f, 0.1f, 0.1f); //leaf size  1cm 
    sor.filter(*roi_data_p_);          //출력 

    cloud_data = *roi_data_p_; // Voxel 데이터를 cloud_data에 저장
}

void LiDAR_pre::outlier()
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlier_p_ = cloud_data.makeShared();

    // 1. Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(outlier_p_);            //입력 
    sor.setMeanK(200);                    //분석시 고려한 이웃 점 수
    sor.setStddevMulThresh(0.2);         //Outlier로 처리할 거리 정보 
    sor.filter(*cloud_filtered);         // 필터 적용

    cloud_data = *cloud_filtered;
}


void LiDAR_pre::ransac()
{
    // 평면 포인트와 비평면 포인트를 저장하기 위한 PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>),
										inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_data_p_ = cloud_data.makeShared();   // Pointer

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); // 평면 계수를 저장하는 포인터
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());    // 평면 모델에 적합한 포인트 인덱스를 저장하기 위한 포인터


	// 오프젝트 생성 Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZI> seg;   // SAC(Sample Consensus) 세그멘테이션 객체 생성
	seg.setOptimizeCoefficients(true);        // 모델 계수의 정제를 활성화
	seg.setModelType(pcl::SACMODEL_PLANE);    // 모델 타입은 평면
	seg.setMethodType(pcl::SAC_RANSAC);       // RANSAC 방법 사용
	seg.setMaxIterations(3000);               // 최대 실행 수
	seg.setDistanceThreshold(0.15);          // 최대 거리
	seg.setInputCloud(voxel_data_p_);        //입력 클라우드
	seg.segment(*inliers, *coefficients);    // 인라이어 인덱스와 모델 계수를 계산

	pcl::copyPointCloud<pcl::PointXYZI>(*voxel_data_p_, *inliers, *inlierPoints);

	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(voxel_data_p_);
	extract.setIndices(inliers);
	extract.setNegative(true); //false
	extract.filter(*inlierPoints_neg);

    cloud_data = *inlierPoints_neg;
}

// 포인트 간의 유클리드 거리 계산 함수
double LiDAR_pre::euclideanDistance(const pcl::PointXYZI& point1, const pcl::PointXYZI& point2) {
    return std::sqrt(std::pow(point1.x - point2.x, 2) +
                     std::pow(point1.y - point2.y, 2) +
                     std::pow(point1.z - point2.z, 2));
}

// DBSCAN 알고리즘
void LiDAR_pre::dbscan(double epsilon, int minPoints) {
    int numPoints = cloud_data.size();
    std::vector<bool> visited(numPoints, false);    // 방문 여부 체크
    std::vector<int> cluster(numPoints, -1);        // 클러스터 번호, -1은 노이즈
    int clusterID = 0;

    for (int i = 0; i < numPoints; ++i) {
        if (visited[i]) continue;

        visited[i] = true;
        std::vector<int> neighbors;

        // 이웃 포인트 찾기
        for (int j = 0; j < numPoints; ++j) {
            if (euclideanDistance(cloud_data.points[i], cloud_data.points[j]) <= epsilon) {
                neighbors.push_back(j);
            }
        }

        // 이웃 포인트가 충분하지 않으면 노이즈로 간주
        if (neighbors.size() < minPoints) {
            cluster[i] = -1;  // 노이즈
        } else {
            // 새로운 클러스터 생성
            clusterID++;
            cluster[i] = clusterID;

            // 이웃 포인트들 처리
            for (int k = 0; k < neighbors.size(); ++k) {
                int neighborIndex = neighbors[k];
                if (!visited[neighborIndex]) {
                    visited[neighborIndex] = true;

                    std::vector<int> neighborNeighbors;
                    // 새로운 이웃 포인트들을 찾음
                    for (int j = 0; j < numPoints; ++j) {
                        if (euclideanDistance(cloud_data.points[neighborIndex], cloud_data.points[j]) <= epsilon) {
                            neighborNeighbors.push_back(j);
                        }
                    }

                    if (neighborNeighbors.size() >= minPoints) {
                        neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
                    }
                }

                if (cluster[neighborIndex] == -1) {
                    cluster[neighborIndex] = clusterID;
                }
            }
        }
    }

    for(int i=0; i<cluster.size(); i++){
        cloud_data.points[i].intensity = cluster.at(i);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lidar_Pre");
    ros::NodeHandle nh;

    LiDAR_pre lp(nh);

    ros::spin();

    return 0;
}