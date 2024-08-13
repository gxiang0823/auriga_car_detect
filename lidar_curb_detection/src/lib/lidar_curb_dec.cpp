/*
 * @Description:
 * @Author: speedzjy
 * @Date: 2022-04-27 11:55:01
 */

#include "lidar_curb_detection/lidar_curb_dec.hpp"
#include <road_curb_msgs/road_curb.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <vector>
#include <cmath>


struct CurbPoint{
    double x;
    double y;
    //double z;
};
struct self_position{
    double x;
    double y;
    double yaw;
    double x_for;
    double y_for;
    double yaw_for;
};
self_position self_positions;

double k1_temp = 0;
double b1_temp = 0;
double k1_temp_2 = 0;
double b1_temp_2 = 0;
int count_empty = 0;
// 计算平均值
CurbPoint calculateMean(const std::vector<CurbPoint>& points) {
    CurbPoint mean = {0.0, 0.0}; // 初始化平均值为 (0, 0)
    for (const auto& point : points) {
        mean.x += point.x;
        mean.y += point.y;
        // 如果有 z 成员，也需要加上
        // mean.z += point.z;
    }
    mean.x /= points.size();
    mean.y /= points.size();
    // mean.z /= points.size(); // 如果有 z 成员，也需要除以点的数量
    return mean;
}

// 计算标准差
double calculateStdDev(const std::vector<CurbPoint>& points, const CurbPoint& mean) {
    double sum = 0.0;
    for (const auto& point : points) {
        double dx = point.x - mean.x;
        double dy = point.y - mean.y;
        // 如果有 z 成员，也需要计算
        // double dz = point.z - mean.z;
        sum += dx * dx + dy * dy;
        // 如果有 z 成员，也需要加上 dz * dz
    }
    return std::sqrt(sum / points.size());
}
// 函数：线性拟合
void Least_square_method(const std::vector<CurbPoint>& P,double *K0,double *b0)
{
    int N = P.size();
	int i=0;
	double K=0,b=0,A=0,B=0,C=0,D=0;
	for(i=0;i<N;i++)
	{
		A+=P[i].x*P[i].y;
		B+=P[i].x;
		C+=P[i].y;
		D+=P[i].x*P[i].x;
	}
	K=(N*A-B*C)/(N*D-B*B);
	b=C/N-K*B/N;
	/*将计算得到的直线参数通过指针传递到函数外部*/
	*K0=K;
	*b0=b;	
}
//增加代码：边缘点云数据处理
std::array<float,4> processPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    // 在这个函数中，你可以对接收到的点云数据进行任何处理
    std::array<float,4> curb_fitting_result = {0.0f, 0.0f, 0.0f ,0.0f};
    //计算点云中点的数量
    size_t numPoints = cloud.size();
    //std::cout << "Number of points in the point cloud: " << numPoints << std::endl;
    
    //对边缘点云进行提取和预处理
   std::vector<CurbPoint> CurbPoints_1;
   std::vector<CurbPoint> CurbPoints;
   //std::vector<std::vector<float>> CurbPoints;
    for (const auto& point : cloud)
    {
        // 对每个点进行处理
        
        if (((point.x>-2.5&&point.x<-1.0)||point.x>0.5)&&((point.y<0.8&&point.y>-3.5))&&
        (point.x*point.x+point.y*point.y)>4)
        //if (((point.x>0)&&((point.y<0.8&&point.y>-3.5)))&&
        //(point.x*point.x+point.y*point.y)>4)
        {
            CurbPoint curbpoints_temp;
            curbpoints_temp.x = point.x;
            curbpoints_temp.y = point.y;
            CurbPoints_1.push_back(curbpoints_temp);
            //std::cout << "curpoint: (" << curbpoints_temp.x << ", " << curbpoints_temp.y << ")" << std::endl;
        }

    }
    //y方向均值处理
    CurbPoint mean = calculateMean(CurbPoints_1);
    double stdDev = calculateStdDev(CurbPoints_1, mean);
    const double threshold = 2 * stdDev;

    for (const auto& point : CurbPoints_1) {
        double dx = abs(point.x - mean.x);
        double dy = abs(point.y - mean.y);

        // 如果有 z 成员，也需要计算 dz
        // double dz = point.z - mean.z;
        double distanceSquared = dx * dx + dy * dy;
        // 如果有 z 成员，也需要加上 dz * dz

        if ((distanceSquared <= threshold * threshold)&&(dy<0.5)) {
            // 如果点到均值的距离不超过阈值，将其保留
            CurbPoints.push_back(point);
        }
    }
    //对边缘点云在xy平面进行拟合   
    cout << "CurbPoints: " <<CurbPoints.size()<< endl;
    // 拟合直线的参数
    double k, b;
    k = 0;
    b = 0;
    if (CurbPoints.size()>=4)
    {
    // 进行线性拟合
    Least_square_method(CurbPoints, &k, &b);
    //cout << "Fitted line equation: " << "y = " << k << "x + " << b << endl;
    }
    if(k>2.0)k = 2.0;

    if(k<-2.0)k = -2.0;
    curb_fitting_result[0] = k;
    curb_fitting_result[1] = b;
    return curb_fitting_result;
}

namespace CurbDectection {

LidarCurbDectection::LidarCurbDectection()
    : complete_points(new PointCloudType), boundary_points(CloudPtrList(2)) {

    //订阅全局坐标
  subTFstamped_ = nh_.subscribe("/tf", 10, &LidarCurbDectection::transformCallback, this);

  subPointCloud_ = nh_.subscribe(
      subpointCloudTopic, 10, &LidarCurbDectection::pointCloudCallback, this);


  pubCompleteCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/complete_cloud", 10);
  pubGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 10);
  pubNoGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/no_ground_cloud", 10);
  pubFeatureCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/feature_cloud", 10);
  pubCurbCloudLeft_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/curb_cloud_left", 10);
  pubCurbCloudRight_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/curb_cloud_right", 10);
  pubMarker_ =
      nh_.advertise<visualization_msgs::Marker>("/visual_marker", 10);
  //曲线拟合结果发布
  curb_fitting_ =
      nh_.advertise<road_curb_msgs::road_curb>("/road_shoulder_line", 10);
  marker_pub_ = 
      nh_.advertise<visualization_msgs::Marker>("/visual_line", 10);

  for (size_t i = 0; i < boundary_points.size(); ++i) {
    boundary_points[i] = boost::make_shared<PointCloudType>();
  }
}

LidarCurbDectection::~LidarCurbDectection() {}

//处理tf坐标
void LidarCurbDectection::transformCallback(
    const tf2_msgs::TFMessage::ConstPtr& tf_msg){
      double x_n,y_n,yaw_n;
    for (const auto& transform : tf_msg->transforms)
{
    // Convert quaternion to Euler angles
    tf2::Quaternion quat;
    tf2::convert(transform.transform.rotation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw_n);

    // Convert angles to degrees if needed
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    x_n = transform.transform.translation.x;
    y_n = transform.transform.translation.y;
    //std::cout << "Transform " << transform.header.frame_id << " to " << transform.child_frame_id << ":" << std::endl;
}
    self_positions.x = x_n;
    self_positions.y = y_n;
    self_positions.yaw = yaw_n;

}

void LidarCurbDectection::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr) {

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  PointCloudType::Ptr tmp_points(new PointCloudType);
  pcl::fromROSMsg(*in_cloud_ptr, *tmp_points);
  queue_complete_points.push_back(*tmp_points);

  complete_points->clear();
  for (size_t i = 0; i < boundary_points.size(); ++i) {
    boundary_points[i]->clear();
  }

  // 读取点云
  PointCloudType::Ptr completeCloud(new PointCloudType);
  if (!queue_complete_points.empty()) {
    *completeCloud = queue_complete_points.front();
    queue_complete_points.pop_front();
  }

  //计算点云laserID, 并将ID存为intensity
  CloudMapper mapper_laserID(cmMsg);
  PointCloudType::Ptr completeCloudMapper(new PointCloudType);
  mapper_laserID.processByOri(completeCloud, completeCloudMapper);
  AINFO << "raw points number: " << completeCloudMapper->points.size() << endl;

  // 地面提取
  // 论文中选取范围: Z x Y x X , [−3, 1] x [−40, 40] x [−70, 70]
  // 代码实际选取: Y x X , [−30, 30] x [−40, 40]
  // 使用 pcl 库进行平面特征点提取
  PointCloudType::Ptr ground_points(new PointCloudType);
  PointCloudType::Ptr ground_points_no(new PointCloudType); //非地面点

  GroundSegmentation ground(completeCloudMapper, gsMsg);
  ground.groundfilter(ground_points, ground_points_no);
  AINFO << "ground_points number: " << ground_points->points.size() << endl;
  AINFO << "no_ground_points number: " << ground_points_no->points.size()
        << endl;

  //根据之前计算的Intensity对地面点云进行mapper
  CloudMapper mapper2(cmMsg);
  scanIndices scanIDindices;
  PointCloudType::Ptr ground_points_mapper(new PointCloudType);
  mapper2.processByIntensity(ground_points, ground_points_mapper,
                             scanIDindices);

  //特征点提取
  pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  FeaturePoints curbExtract(ground_points_mapper, scanIDindices, fpMsg);
  curbExtract.extractFeatures(featurePoints);
  AINFO << "feature points number: " << featurePoints->points.size() << endl;

  //高斯过程提取
  BoundaryPoints refinePoints(*featurePoints, cmMsg, bpMsg);
  // 道路中心线
  MarkerList line_list(2);
  refinePoints.process(ground_points_no, boundary_points, line_list);

  *complete_points = *completeCloud;

  // 计时
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  AINFO << "compute this frame time cost = " << time_used.count()
        << " seconds. " << endl;

  sensor_msgs::PointCloud2 tmp_rosCloud;
  std::string in_cloud_frame_id = in_cloud_ptr->header.frame_id;

  //  完整点云
  pcl::toROSMsg(*complete_points, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  //pubCompleteCloud_.publish(tmp_rosCloud);

  // 地面点
  pcl::toROSMsg(*ground_points, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubGroundCloud_.publish(tmp_rosCloud);

  // 非地面点array
  pcl::toROSMsg(*ground_points_no, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  //pubNoGroundCloud_.publish(tmp_rosCloud);

  // 特征点
  pcl::toROSMsg(*featurePoints, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  //pubFeatureCloud_.publish(tmp_rosCloud);

  // 左边缘点
  pcl::toROSMsg(*(boundary_points[0]), tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubCurbCloudLeft_.publish(tmp_rosCloud);

  // 右边缘点
  pcl::toROSMsg(*(boundary_points[1]), tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubCurbCloudRight_.publish(tmp_rosCloud);
   
 
  //对右边缘点进行提取
  std::array<float,4> curb_fitting_result1 = processPointCloud(*(boundary_points[0]));
  std::array<float,4> curb_fitting_result2 = processPointCloud(*(boundary_points[1]));
  std::array<float,4> curb_fitting_result;
  if (abs(curb_fitting_result2[1])<0.0002)
  {
    curb_fitting_result = curb_fitting_result1;
  }
  else
  {
    curb_fitting_result = curb_fitting_result2;
  }

  road_curb_msgs::road_curb curb_fitting;
  curb_fitting.header.seq = 1;
  curb_fitting.header.stamp = ros::Time::now();
  curb_fitting.header.frame_id = in_cloud_frame_id;
  int flag1,flag2,curb_flag;
  double k1,b1,x,y,yaw;
  if (abs(curb_fitting_result[1])<0.002)
  {
    count_empty += 1;
    if(count_empty>=15)curb_flag = 1;
    else curb_flag = 2;
  }
  else
  {
    count_empty  = 0;
    curb_flag = 2;
  }
  double d_x,d_y,d_yaw;
  double line_a1,line_b1,line_c1,line_a2,line_b2,line_c2;
  double k1_new,b1_new;
  if ((abs(curb_fitting_result[1])<0.02)&&(count_empty<=250))
  {
    k1 = (k1_temp+k1_temp_2)/2;
    b1 = (b1_temp+b1_temp_2)/2;
   
    d_x = self_positions.x - self_positions.x_for;
    d_y = -(self_positions.y - self_positions.y_for);
    d_yaw = self_positions.yaw - self_positions.yaw_for;
     
    line_a1 = k1;
    line_b1 = -1;
    line_c1 = b1;
    
    line_a2 = line_a1 * cos(d_yaw) + line_b1 * sin(d_yaw);
    line_b2 = -line_a1 * sin(d_yaw) + line_b1 * cos(d_yaw);
    line_c2 = line_a1 * d_x + line_b1 * d_y + line_c1;
    
    ROS_WARN("k1:%f",k1);
    ROS_WARN("b1:%f",b1);
    if (abs(line_b2)>0.0001)
    {
    k1_new = -line_a2/line_b2;
    b1_new = -line_c2/line_b2;
    }
    else
    {
    k1_new = k1;
    b1_new = b1;
    }

    if(abs(k1_new-k1)>0.25)
    {
      if(k1_new-k1>0)k1 = k1+0.25;
      if(k1_new-k1<0)k1 = k1-0.25;
    }
    else
    {
        k1 = k1_new;
    }
    if(abs(b1_new-b1)>0.4)
    {
      if(b1_new-b1>0)b1 = b1+0.4;
      if(b1_new-b1<0)b1 = b1-0.4;
    }
    else
    {
        b1 = b1_new;
    }

    // k1 = k1_new;
    // b1 = b1_new;

    // ROS_WARN("b2:%f",line_b2);
    // ROS_WARN("k1_new:%f",k1);
    // ROS_WARN("b1_new:%f",b1);
    
  }
  else
  {
    double diff_k = 0;
    diff_k = abs(curb_fitting_result[0]-k1_temp);
    double diff_b = 0;
    diff_b = abs(curb_fitting_result[1]-b1_temp);

    if(abs(curb_fitting_result[0]-k1_temp)>0.15)
    {
      if(curb_fitting_result[0]-k1_temp>0)k1 = (k1_temp+k1_temp_2)/2+0.15;flag1 = 1;
      if(curb_fitting_result[0]-k1_temp<0)k1 = (k1_temp+k1_temp_2)/2-0.15;flag1 = 2;
      k1_temp_2 = k1_temp;
      k1_temp = k1;
      }
    else
    {
        k1 = (curb_fitting_result[0]+k1_temp+k1_temp_2)/3;
        k1_temp_2 = k1_temp;
        k1_temp = curb_fitting_result[0];
        flag1 = 0;
    }

    if(abs(curb_fitting_result[1]-b1_temp)>0.30)
    {
      if(curb_fitting_result[1]-b1_temp>0)b1 = (b1_temp+b1_temp_2)/2+0.25;flag2 = 3;
      if(curb_fitting_result[1]-b1_temp<0)b1 = (b1_temp+b1_temp_2)/2-0.25;flag2 = 4;
      b1_temp_2 = b1_temp;
      b1_temp = b1;
      }
    else
    {
        b1 = (curb_fitting_result[1]+b1_temp+b1_temp_2)/3;
        b1_temp_2 = b1_temp;
        b1_temp = curb_fitting_result[1];
        flag2 = 0;
    }
    
    
    //更新位置
    self_positions.x_for = self_positions.x;
    self_positions.y_for = self_positions.y;
    self_positions.yaw_for = self_positions.yaw;
    d_x = 0; 
    d_y = 0;
    d_yaw = 0;
  }

  // cout << "x:" << self_positions.x<<" x_for:"<<self_positions.x_for<<" dx:"<< d_x << endl;
  // cout << "y:" << self_positions.y<<" y_for:"<<self_positions.y_for<<" dy:"<< d_y << endl;
  // cout << "yaw:" << self_positions.yaw<<" yaw_for:"<<self_positions.yaw_for<<" theta:"<< d_yaw << endl;
  // cout << "k1:" << k1 << "  k1_temp:" << k1_temp << endl;
  // cout << "b1:" << b1 << "  b1_temp:" << b1_temp << endl;
  // cout <<"count:"<<count_empty<<"\n"<< endl;
  curb_fitting.a_3 = k1;
  curb_fitting.a_2 = b1;
  curb_fitting.a_1 = curb_flag;
  curb_fitting_.publish(curb_fitting);


    visualization_msgs::Marker line_strip;

     // 设置Marker消息的属性
    line_strip.header.frame_id = in_cloud_frame_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1; // 线的宽度
    line_strip.color.r = 1; // 红色
    line_strip.color.a = 1.0; // 不透明
    
    // 添加直线上的点
    geometry_msgs::Point p;
    p.x = 1;
    p.y = k1+b1;
    p.z = 0.0;
    line_strip.points.push_back(p);

    p.x = 2;
    p.y = 2*k1+b1;
    p.z = 0.0;
    line_strip.points.push_back(p);

    p.x = 3;
    p.y = 3*k1+b1;
    p.z = 0.0;
    line_strip.points.push_back(p);

    p.x = 4;
    p.y = 4*k1+b1;
    p.z = 0.0;
    line_strip.points.push_back(p);

    p.x = 5;
    p.y = 5*k1+b1;
    p.z = 0.0;
    line_strip.points.push_back(p);
    // 发布Marker消息
    if (abs(b1)>0.02)
    {
      marker_pub_.publish(line_strip);
    }
    else
    {
    visualization_msgs::Marker line_empty;
     // 设置Marker消息的属性
    line_empty.header.frame_id = in_cloud_frame_id;
    line_empty.header.stamp = ros::Time::now();
    line_empty.ns = "lines";
    line_empty.action = visualization_msgs::Marker::ADD;
    line_empty.pose.orientation.w = 1.0;
    line_empty.id = 0;
    line_empty.type = visualization_msgs::Marker::LINE_STRIP;
    line_empty.scale.x = 0.1; // 线的宽度
    line_empty.color.r = 1.0;
    line_empty.color.g = 0;
    line_empty.color.b = 0;
    line_empty.color.a = 1.0; // 不透明 
 
    
    
    marker_pub_.publish(line_empty);
    }

    
  // 道路中心线
  for (size_t i = 0; i < line_list.size(); ++i) {
    line_list[i].header.frame_id = in_cloud_frame_id;
    pubMarker_.publish(line_list[i]);
  }
}

} // namespace CurbDectection

