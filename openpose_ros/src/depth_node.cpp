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
#include <cmath>

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
  bool get_neareast_point(const pcl::PointCloud<pcl::PointXYZ>::Ptr  temp_cloud,
                          const openpose_ros_msgs::PointWithProb& last_point_10_pix,
                          const openpose_ros_msgs::PointWithDepth& last_point_10,
                          openpose_ros_msgs::PointWithProb& point_10_pix,
                          openpose_ros_msgs::PointWithDepth& point_10);
  float distance(const openpose_ros_msgs::PointWithProb& p1,
                 const openpose_ros_msgs::PointWithProb& p2);

  void swap_point_pix(openpose_ros_msgs::PointWithProb& p1,
                      openpose_ros_msgs::PointWithProb& p2);

  void swap_depth_point(openpose_ros_msgs::PointWithDepth& p1,
                        openpose_ros_msgs::PointWithDepth& p2);

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

///
/// \brief get_neareast_point
/// \return
///  input last_point_10_pix point_10_pix last_point_10 point_10 temp_cloud
///  output:the result of whether we get the data.
bool PublishNode::get_neareast_point(const pcl::PointCloud<pcl::PointXYZ>::Ptr  temp_cloud,
                        const openpose_ros_msgs::PointWithProb& last_point_10_pix,
                        const openpose_ros_msgs::PointWithDepth& last_point_10,
                        openpose_ros_msgs::PointWithProb& point_10_pix,
                        openpose_ros_msgs::PointWithDepth& point_10)
{
    openpose_ros_msgs::PointWithProb p[9];
    p[0] = last_point_10_pix;
    p[1] = last_point_10_pix;
    p[2] = last_point_10_pix;
    p[3] = last_point_10_pix;
    p[4] = last_point_10_pix;
    p[5] = last_point_10_pix;
    p[6] = last_point_10_pix;
    p[7] = last_point_10_pix;
    p[8] = last_point_10_pix;

    p[1].x = p[1].x +16;
    p[2].y = p[2].y +16;
    p[3].x = p[3].x -16;
    p[4].y = p[4].y -16;
    p[5].x = p[5].x +8;
    p[6].y = p[6].y +8;
    p[7].x = p[7].x -8;
    p[8].y = p[8].y -8;

    openpose_ros_msgs::PointWithDepth d[9];
    getRealPoint(temp_cloud,d[0],p[0]);
    getRealPoint(temp_cloud,d[1],p[1]);
    getRealPoint(temp_cloud,d[2],p[2]);
    getRealPoint(temp_cloud,d[3],p[3]);
    getRealPoint(temp_cloud,d[4],p[4]);
    getRealPoint(temp_cloud,d[5],p[5]);
    getRealPoint(temp_cloud,d[6],p[6]);
    getRealPoint(temp_cloud,d[7],p[7]);
    getRealPoint(temp_cloud,d[8],p[8]);


    float neareast_depth = abs(d[0].depth-last_point_10.depth);
    int neareast_num = 0;

    for(int i = 1; i<9;i++)
    {
      if(d[i].depth!=0&&abs(d[i].depth-last_point_10.depth)<neareast_depth)
      {
        neareast_num = i;
        neareast_depth = abs(d[i].depth-last_point_10.depth);
      }
    }

    if(neareast_depth<0.1)
    {
      point_10_pix = p[neareast_num];
      point_10 = d[neareast_num];
      return true;
    }
    else
      return false;

}

float PublishNode::distance(const openpose_ros_msgs::PointWithProb &p1,
                            const openpose_ros_msgs::PointWithProb &p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

void PublishNode::swap_point_pix(openpose_ros_msgs::PointWithProb& p1,
                    openpose_ros_msgs::PointWithProb& p2)
{
    openpose_ros_msgs::PointWithProb temp = p1;
    p1 = p2;
    p2 = temp;
}

void PublishNode::swap_depth_point(openpose_ros_msgs::PointWithDepth& p1,
                      openpose_ros_msgs::PointWithDepth& p2)
{
    openpose_ros_msgs::PointWithDepth temp = p1;
    p1 = p2;
    p2 = temp;
}
/*
 *
 *     <node pkg="openpose_ros" type="openpose_ros_depth_node" name="openpose_ros_depth_node" output="screen">
    </node >
*/

void PublishNode::getRealPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr  temp_cloud, 
				 openpose_ros_msgs::PointWithDepth& body_point_with_depth,
				 const openpose_ros_msgs::PointWithProb& body_point_with_prob)
{
    body_point_with_depth.prob = body_point_with_prob.prob;
      
    if(body_point_with_depth.prob <= 0.1)
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

/*
    <node pkg="openpose_ros" type="openpose_ros_depth_node" name="openpose_ros_depth_node" output="screen">
    </node >
*/

void PublishNode::processImage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const openpose_ros_msgs::OpenPoseHumanListConstPtr& human_msg)
{
    static bool part_10_out = false;
    static openpose_ros_msgs::PointWithProb last_point_10_pix;
    static openpose_ros_msgs::PointWithProb last_point_13_pix;

    static openpose_ros_msgs::PointWithDepth last_point_10;
    static openpose_ros_msgs::PointWithDepth last_point_13;

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
        //because the part 10 is our right knee , we address the importance of the date when the robot track the target
        //so we should process the date by some method
        //step 1, judge whether the part 10 is in the field of the camera, we need a var to record the state
        openpose_ros_msgs::PointWithDepth point_10 = human_depth.body_key_points_with_depth.at(10);
        openpose_ros_msgs::PointWithDepth point_13 = human_depth.body_key_points_with_depth.at(13);

        openpose_ros_msgs::PointWithProb point_10_pix = human.body_key_points_with_prob.at(10);
        openpose_ros_msgs::PointWithProb point_13_pix = human.body_key_points_with_prob.at(13);

        if(part_10_out == false)
        {

            if(point_10.x > 0.45||point_10.x < -0.45)
            {
                part_10_out = true;
            }
        }
        else
        {
            if(point_10.depth != 0&&point_10.x < 0.45&&point_10.x > -0.45)
            {
                part_10_out = false;
            }
        }

        // step 2 judge whether the data change to 0 or change a lot. judge the x;
        // we need a function to get the distance of the last point 10 to now point 10;
        if(part_10_out == false)
        {
            if(point_10.depth == 0 || distance(point_10_pix, last_point_10_pix) > 20)
            {

                // step 3, if the data change to 0, it means that the openpose could'nt detect the point 10
                // then we can detect the last_point_10's rounds. so, we just detect 5 points.
                // we need a function to get the neareast point.
                if(point_10.depth == 0)
                {

                      get_neareast_point(temp_cloud,last_point_10_pix,last_point_10,point_10_pix,point_10);
                }
                else
                {
                     if(distance(point_10_pix, last_point_13_pix) < 10 && distance(point_13_pix, last_point_10_pix) < 10)
                     {
                        std::cout<<"emmmmmmm"<<std::endl;
                        std::cout <<" now 10 x is " <<point_10_pix.x<<"  10 y is" <<point_10_pix.y<< std::endl;
                        std::cout <<" last 13 x is " <<last_point_13_pix.x<<"  13 y is" <<last_point_13_pix.y<< std::endl;

                        std::cout <<" now 13 x is " <<point_13_pix.x<<"  13 y is" <<point_13_pix.y<< std::endl;
                        std::cout <<" last 10 x is " <<last_point_10_pix.x<<"  10 y is" <<last_point_10_pix.y<< std::endl;

                        swap_point_pix(point_10_pix,point_13_pix);
                        swap_depth_point(point_10,point_13);
                        std::cout<<"hahahaha"<<std::endl;
                        std::cout <<" now 10 x is " <<point_10_pix.x<<"  10 y is" <<point_10_pix.y<< std::endl;
                        std::cout <<" last 13 x is " <<last_point_13_pix.x<<"  13 y is" <<last_point_13_pix.y<< std::endl;

                        std::cout <<" now 13 x is " <<point_13_pix.x<<"  13 y is" <<point_13_pix.y<< std::endl;
                        std::cout <<" last 10 x is " <<last_point_10_pix.x<<"  10 y is" <<last_point_10_pix.y<< std::endl;

                     }
                }
                if(point_10.depth ==0)
                {
                    point_10 = last_point_10;
                    last_point_10_pix = point_10_pix;
                }
            }
        }

        if(point_10.depth ==0)
        {
            std::cout<<part_10_out<<std::endl;
            std::cout<<last_point_10.x<<"  "<<last_point_10.depth<<std::endl;
        }
        human_depth.body_key_points_with_depth.at(10) = point_10;
        human_depth.body_key_points_with_depth.at(13) = point_13;

        // last step, we record the last position of point_10 and point_13's pix locate to help us judge whether the date is right
        last_point_10_pix = point_10_pix;
        last_point_13_pix = point_13_pix;
        last_point_10     = point_10;
        last_point_13     = point_13;

        //std::cout <<"x is " <<point_10.x<<"the state of right knee is" <<part_10_out<< std::endl;

        human_depth.num_body_key_points_with_non_zero_depth = human.num_body_key_points_with_non_zero_prob;
        human_depth_list.at(person) = human_depth;
    }
    human_depth_list_msg.human_depth_list = human_depth_list;
    
    openpose_human_list_depth_pub_.publish(human_depth_list_msg);

}


int main(int argc ,char * argv[])
{     
     ros::init(argc,argv,"openpose_ros_depth"); 
     
     PublishNode pub;    
     ros::spin();
     return 0;
}


