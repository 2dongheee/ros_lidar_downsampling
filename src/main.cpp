#include "utility.h"

class Process{

private:
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher pubDownsampleCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr downSamplingCloud;

    std_msgs::Header cloudHeader;

public:
    Process():
        nh("~"){
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/hesai/pandar", 1, &Process::cloudHandler, this);
        pubDownsampleCloud = nh.advertise<sensor_msgs::PointCloud2> ("/hesai/pandar/downsampled_cloud", 1);

        allocateMemory();
        resetParameters();
    }
    ~Process(){}

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
        downSamplingCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    void resetParameters(){
        laserCloudIn->clear();
        downSamplingCloud->clear();
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        clock_t start = clock();
        copyPointCloud(laserCloudMsg);
        donwSampling();
        publishResult();
        resetParameters();
    }

     void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    }

    void donwSampling(){
        std::cout << "Input Points: " << laserCloudIn->points.size () << " (" << pcl::getFieldsList (*laserCloudIn) <<")"<< std::endl;
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(laserCloudIn);
        vg.setLeafSize(filterRes_X, filterRes_Y, filterRes_Z); 
        vg.filter(*downSamplingCloud);
        std::cout << "Output Points: " << downSamplingCloud->points.size () << " (" << pcl::getFieldsList (*downSamplingCloud) <<")"<< std::endl;
    }

    void publishResult(){
        sensor_msgs::PointCloud2 laserCloudTemp;

        if (pubDownsampleCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*downSamplingCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "Pandar64";
            pubDownsampleCloud.publish(laserCloudTemp);
        }
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "Pandar64");
    Process P;
    ros::spin();
    return 0;
}
