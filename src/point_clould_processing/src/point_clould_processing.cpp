#include "point_clould_processing.hpp"

PointCloudProcessing::PointCloudProcessing(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    std::cout<<"from Constructor \n";

    // storing the values in the member variable
    pnh.getParam("no_of_clouds", no_of_clouds_);
    pnh.getParam("event", event_);
    
    std::cout<<"Number of point cloud to be processed: "<<no_of_clouds_
                <<" | For event: "<<event_<<"\n";

    // Setting up the parameters
    nh_ = nh;
    has_point_cloud_ = false;
    pkg_path_ = ros::package::getPath("point_clould_processing");

    // Defining the publisher and subscribers
    sub_cloud_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudProcessing::rsCloudCallback, this);
    pub_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("/point_clould_processing/point_cloud_", 1);  
}

PointCloudProcessing::~PointCloudProcessing()
{
    std::cout<<"from Distructor \n";
}

void PointCloudProcessing::runOnce()
{
    std::cout<<"From Runonce \n";
    
    if(event_ != "before" and event_ != "after")
    {
        std::cout<<"\n Give valid arguments(before->wo glare / after->w glare) for event \n";
        std::cout<<"\n\n*****************************************\n Shutting down the node as input is invalid \n*****************************************\n\n\n";
        ros::shutdown();
    }
    
    // if else loop to check weather we have required number of point clouds to start post processing
    if(has_point_cloud_)
    {
        std::cout<<"Calling the function to publish the point cloud \n";

        // Point cloud conversion form RealSense cloud -> PCL 2 cloud -> PCL cloud
        convertToPCL();

        // Storing the .csv file of every cloud
        storeInCSV();

        // Generating the average point cloud
        generatePointCloudTensor();

        // Store the mean, stddev, nan for image
        storeMeanStddevNan();
        std::cout<<"\n\n ******* stored the mean, nan, std cloud ******** \n";

                        std::cout<<"\n\n************************************\n stopping the node \n************************************\n\n\n";
                ros::shutdown();

    }
    else
    {
        std::cout<<"Number of point clouds received: "<<point_cloud_count_<<" \n";
    }   
}

void PointCloudProcessing::rsCloudCallback(const sensor_msgs::PointCloud2 & msg)
{
    /* Storing the point clouds in the vector and then once we have required 
    number of point clouds it will change the value in flag variable*/
    if(rs_pt_cloud_vec_.size()<no_of_clouds_)
    {
        rs_pt_cloud_vec_.push_back(msg);
        point_cloud_count_++;
    }
    else
    {
        has_point_cloud_ = true;
        std::cout<<"Received the required number of point cloulds now its processing time \n";
    }
}

void PointCloudProcessing::convertToPCL()
{
    std::cout<<"\n ***** Starting Point cloud conversion at: "<<ros::Time::now()<<" ***** \n";

    // Iterating over the vector to conver and store the point clouds in different formats
    for(int i=0; i<rs_pt_cloud_vec_.size(); i++)
    {
        rs_pt_cloud_ = rs_pt_cloud_vec_[i];
        pcl_conversions::toPCL(rs_pt_cloud_,pcl_pt_cloud2_);
        pcl::fromPCLPointCloud2(pcl_pt_cloud2_, pcl_pt_cloud_);
        
        pcl_pt_cloud_vec_.push_back(pcl_pt_cloud_);
    }
    std::cout<<"\n >>> Conversion complete at: "<<ros::Time::now()<<" \n";
    std::cout<<"\n clearing the rs_pt_cloud vector "<<rs_pt_cloud_vec_.size()<<"\n";
    rs_pt_cloud_vec_.clear();
}
void PointCloudProcessing::storeInCSV()
{
    // Iterating over point cloud and saving the cloud in .csv files
    for(int i=0; i<pcl_pt_cloud_vec_.size(); i++)
    {
        std::cout<<"Saving the point cloud on the harddrive \n";

        // .csv file creating
        csv_file_path_ = pkg_path_ + "/data/" + event_ + "_CSV_file_for_cloud_" + std::to_string(i) + ".csv";  
        myfile_csv_.open(csv_file_path_, std::ios::out | std::ios::app);
        if(!myfile_csv_)
        {
            std::cout<<".csv file crearion failed for point cloud no \n";
        }
        else
        {
            for (int j=0; j< pcl_pt_cloud_vec_[i].points.size(); j++)
            {   
                myfile_csv_<<pcl_pt_cloud_vec_[i].points[j].x << ","<<pcl_pt_cloud_vec_[i].points[j].y << ","<<pcl_pt_cloud_vec_[i].points[j].z << " \n"; 
            }
        }           
        myfile_csv_.close(); 
    }    
}


void PointCloudProcessing::generatePointCloudTensor()
{

    double distance, sum, mean, sq_sum, stdev;
    int no_of_nan;

    vec_distance.clear(); 
    for_image_mean.clear(); 
    for_image_std.clear(); 
    for_image_nan.clear();

    for (int i=0; i < pcl_pt_cloud_vec_[0].points.size(); i++)
    {
        vec_distance.clear(); no_of_nan = 0;
        for(int j=0; j<pcl_pt_cloud_vec_.size(); j++)
        {
            if(std::isnan(pcl_pt_cloud_vec_[j].points[i].x) or std::isnan(pcl_pt_cloud_vec_[j].points[i].y) or std::isnan(pcl_pt_cloud_vec_[j].points[i].z))
            {
                no_of_nan++;
            }
            else
            {
                vec_distance.push_back(std::sqrt(pcl_pt_cloud_vec_[j].points[i].x*pcl_pt_cloud_vec_[j].points[i].x 
                                            + pcl_pt_cloud_vec_[j].points[i].y*pcl_pt_cloud_vec_[j].points[i].y 
                                            + pcl_pt_cloud_vec_[j].points[i].z*pcl_pt_cloud_vec_[j].points[i].z));   
            }          
        }

        sum = std::accumulate(vec_distance.begin(), vec_distance.end(), 0.0);
        mean = sum / vec_distance.size();

        sq_sum = std::inner_product(vec_distance.begin(), vec_distance.end(), vec_distance.begin(), 0.0);
        stdev = std::sqrt(sq_sum / vec_distance.size() - mean * mean);

        for_image_mean.push_back(mean);
        for_image_std.push_back(stdev);
        for_image_nan.push_back(no_of_nan);
    }
}

void PointCloudProcessing::storeMeanStddevNan()
{
    csv_file_path_ = pkg_path_ + "/data/" + event_ + "mean_cloud.csv";  
    myfile_csv_.open(csv_file_path_, std::ios::out | std::ios::app);
    if(!myfile_csv_)
    {
        std::cout<<".csv file crearion failed for mean cloud\n";
    }
    else
    {
        for (int j=0; j< for_image_mean.size(); j++)
        {   
            myfile_csv_<< for_image_mean[j] << ","; 
        }
        myfile_csv_<<"\n";
    }           
    myfile_csv_.close(); 

    csv_file_path_ = pkg_path_ + "/data/" + event_ + "std_cloud.csv";  
    myfile_csv_.open(csv_file_path_, std::ios::out | std::ios::app);
    if(!myfile_csv_)
    {
        std::cout<<".csv file crearion failed for mean cloud\n";
    }
    else
    {
        for (int j=0; j< for_image_std.size(); j++)
        {   
            myfile_csv_<< for_image_std[j] << ","; 
        }
        myfile_csv_<<"\n";
    }           
    myfile_csv_.close(); 

    csv_file_path_ = pkg_path_ + "/data/" + event_ + "Nan_cloud.csv";  
    myfile_csv_.open(csv_file_path_, std::ios::out | std::ios::app);
    if(!myfile_csv_)
    {
        std::cout<<".csv file crearion failed for mean cloud\n";
    }
    else
    {
        for (int j=0; j< for_image_nan.size(); j++)
        {   
            myfile_csv_<< for_image_nan[j] << ","; 
        }
        myfile_csv_<<"\n";
    }           
    myfile_csv_.close(); 
}