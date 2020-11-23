#include <openpose_ros_msgs/HumanDepthList.h>
#include <openpose_ros_msgs/HumanDepth.h>
#include <openpose_ros_msgs/PointWithDepth.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/HumanDepthList.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>

#define width  640
#define height 480
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, openpose_ros_msgs::OpenPoseHumanList> MySyncPolicy;

class PublishNode
{
private:
  ros::NodeHandle nh_;
  
  ros::Publisher openpose_human_list_depth_pub_;
  
  message_filters::Subscriber<sensor_msgs::PointCloud2>* cloud_sub_;
  message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList>* humanlist_sub_;
  message_filters::Synchronizer<MySyncPolicy>* sync_;
 
public:
  PublishNode();
  ~PublishNode();
  void getRealPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr  temp_cloud, 
				 openpose_ros_msgs::PointWithDepth& body_point_with_depth,
				 const openpose_ros_msgs::PointWithProb& body_point_with_prob);
  void processImage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const openpose_ros_msgs::OpenPoseHumanListConstPtr& human_msg);
};


PublishNode::PublishNode()
{
   std::string cloud_topic;
   std::string humanlist_topic;
   std::string output_topic;
   nh_.param("humanlist_topic",humanlist_topic,std::string("/openpose_ros/human_list"));
   nh_.param("cloud_topic", cloud_topic, std::string("/camera/depth_registered/points"));
   nh_.param("output_topic", output_topic, std::string("/openpose_ros/human_depth_list"));
   openpose_human_list_depth_pub_ = nh_.advertise<openpose_ros_msgs::HumanDepthList>(output_topic,10);
   cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic, 1);
     
   humanlist_sub_ = new message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList>(nh_, humanlist_topic ,1) ;
   
   sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *cloud_sub_, *humanlist_sub_);
   
   sync_->registerCallback(boost::bind(&PublishNode::processImage ,this ,_1, _2));
}

PublishNode::~PublishNode()
{
    delete cloud_sub_;
    delete humanlist_sub_;
    delete sync_;
}
double Average(std::vector<double> v)
{
    double total = 0.0;
    double size  = 0.0;
    for (int n = 0; n < v.size(); n++){total += v[n];size++;}

    return total/size;

}

void PublishNode::getRealPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr  temp_cloud, 
				 openpose_ros_msgs::PointWithDepth& body_point_with_depth,
				 const openpose_ros_msgs::PointWithProb& body_point_with_prob)
{
    body_point_with_depth.prob = body_point_with_prob.prob;
      
    if(body_point_with_depth.prob == 0||(body_point_with_prob.x == 0||body_point_with_prob.x == 0))
    {
        body_point_with_depth.x = 0;
        body_point_with_depth.y = 0;
        body_point_with_depth.depth = 0;
    }
    else
    {
        unsigned long int x_pixel =(unsigned long int) body_point_with_prob.x;
        unsigned long int y_pixel =(unsigned long int) body_point_with_prob.y;

        std::vector<unsigned long int> indices;
        int index=0;
        int rows = 3;
        int columns = 3;
	
        for (int i = -(rows-1)/2 ; i <= (rows-1)/2; i++)
        {
            for (int j = -(columns-1)/2; j <= (columns-1)/2; j++)
            {
                index = width*(y_pixel + i) + x_pixel + j+ 1;
                indices.push_back(index);
            }
        }

        std::vector<double> possible_x;
        std::vector<double> possible_y;
        std::vector<double> possible_z;

        for(int n=0; n < indices.size(); n++)
        {
            if (not std::isnan(temp_cloud->points[indices[n]].x) && not std::isnan(temp_cloud->points[indices[n]].y) && not std::isnan(temp_cloud->points[indices[n]].z))
            {
              possible_x.push_back(temp_cloud->points[indices[n]].x);
              possible_y.push_back(temp_cloud->points[indices[n]].y);
              possible_z.push_back(temp_cloud->points[indices[n]].z);
            }


        }
        if (possible_x.size() == 0 || possible_y.size() == 0 || possible_z.size() == 0)
        {
            body_point_with_depth.x=0;
            body_point_with_depth.y=0;
            body_point_with_depth.depth=0;
        }
        else
        {
             body_point_with_depth.x= Average(possible_x);
             body_point_with_depth.y = Average(possible_y);
             body_point_with_depth.depth = Average(possible_z);
        }
	
    }
}



void PublishNode::processImage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const openpose_ros_msgs::OpenPoseHumanListConstPtr& human_msg)
{
 
    openpose_ros_msgs::HumanDepthList human_depth_list_msg;
    human_depth_list_msg.header.stamp = ros::Time::now();
    human_depth_list_msg.num_humans = human_msg->num_humans;
    std::vector<openpose_ros_msgs::HumanDepth> human_depth_list(human_msg->num_humans);
       
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *temp_cloud);

    for(auto person =0 ;person <human_msg->num_humans;person++)
    {
	const openpose_ros_msgs::OpenPoseHuman& human = human_msg->human_list.at(person) ;
	openpose_ros_msgs::HumanDepth human_depth;

	for(auto bodyPart = 0;bodyPart <25;bodyPart++)
	{
	    const openpose_ros_msgs::PointWithProb& body_point_with_prob = human.body_key_points_with_prob.at(bodyPart);
	    openpose_ros_msgs::PointWithDepth body_point_with_depth;
	    getRealPoint(temp_cloud,body_point_with_depth,body_point_with_prob);
	    human_depth.body_key_points_with_depth.at(bodyPart) = body_point_with_depth;
	}
	human_depth.num_body_key_points_with_non_zero_depth = human.num_body_key_points_with_non_zero_prob;
	human_depth_list.at(person) = human_depth;
    }
    human_depth_list_msg.human_depth_list = human_depth_list;	
    
    openpose_human_list_depth_pub_.publish(human_depth_list_msg);

}

/*
void saveDepthMsg(const openpose_ros_msgs::HumanDepthListConstPtr& msg)
{
  
}
*/

int main(int argc ,char * argv[])
{     
     ros::init(argc,argv,"openpose_ros_depth"); 
     
     PublishNode pub;    
     ros::spin();
     return 0;
}


