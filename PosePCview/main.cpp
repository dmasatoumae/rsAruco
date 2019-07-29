#include <librealsense2/rs.hpp>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <tuple>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

#include <thread>
#include <mutex>

#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
int save_flag=0;
int cluster_flag=0;
Eigen::Matrix4d read_matrix(std::string filename)
{
    using namespace std;
    ifstream file(filename);

    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d hmatrix = Eigen::Matrix4d::Identity();
    if(file.is_open())
    {
        double vecmatrix[10][10];
        for(int i = 0; i < 8; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
            file >> vecmatrix[i][j];
            file.get();
            }

        }
        //std::cout<<vecmatrix<<std::endl;

        matrix <<
        vecmatrix[0][0],vecmatrix[0][1],vecmatrix[0][2],vecmatrix[0][3],
        vecmatrix[1][0],vecmatrix[1][1],vecmatrix[1][2],vecmatrix[1][3],
        vecmatrix[2][0],vecmatrix[2][1],vecmatrix[2][2],vecmatrix[2][3],
        vecmatrix[3][0],vecmatrix[3][1],vecmatrix[3][2],vecmatrix[3][3];

        hmatrix <<
        vecmatrix[4][0],vecmatrix[4][1],vecmatrix[4][2],vecmatrix[4][3],
        vecmatrix[5][0],vecmatrix[5][1],vecmatrix[5][2],vecmatrix[5][3],
        vecmatrix[6][0],vecmatrix[6][1],vecmatrix[6][2],vecmatrix[6][3],
        vecmatrix[7][0],vecmatrix[7][1],vecmatrix[7][2],vecmatrix[7][3];
        std::cout<<matrix<<std::endl;
        std::cout<<hmatrix<<std::endl;

        
    }

return matrix,hmatrix;
}

auto pointcloud_read(rs2::pipeline pipe)
{
    rs2::pointcloud pc;
    rs2::points points;

    //フィルタ
    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
    rs2::hole_filling_filter hf_filter;
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);


    rs2::frameset frames = pipe.wait_for_frames();

    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
        

    rs2::video_frame color_frame=aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth_frame = aligned_frames.get_depth_frame()/*.apply_filter(colormap)*/;
    //auto depth_frame = frames.get_depth_frame();
    //auto color_frame = frames.get_color_frame();


    depth_frame = dec_filter.process(depth_frame);
    depth_frame = depth_to_disparity.process(depth_frame);
    depth_frame = spat_filter.process(depth_frame);
    depth_frame = temp_filter.process(depth_frame);
    depth_frame = disparity_to_depth.process(depth_frame);
    depth_frame = hf_filter.process(depth_frame);

    pc.map_to(color_frame);
    points = pc.calculate(depth_frame);

    return std::make_pair(points, color_frame);


}

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(std::pair<rs2::points,rs2::frame> rs_points_frame)
{
    auto points =rs_points_frame.first;
    auto color=rs_points_frame.second;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	for (int i = 0; i < points.size(); ++i)
	{

        if(((vertices[i].z)<1.5)&&((vertices[i].x)<0.5)&&((vertices[i].x)>-0.5)){
            //std::cout<<vertices[i].z<<std::endl;
		    cloud->points[i].x = vertices[i].x;
		    cloud->points[i].y = vertices[i].y;
            cloud->points[i].z = vertices[i].z;

        }

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);
        if(((vertices[i].z)<1.5)&&((vertices[i].x)<0.5)&&((vertices[i].x)>-0.5)){
            //std::cout<<vertices[i].z<<std::endl;
            cloud->points[i].r = std::get<0>(current_color);
		    cloud->points[i].g = std::get<1>(current_color);
		    cloud->points[i].b = std::get<2>(current_color);

            

        }

        
	}

	return cloud;
}
auto zerozerozeroremove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud){
    
    pcl::PassThrough<pcl::PointXYZRGB> pass;
     
    pass.setInputCloud (pointcloud);
    pass.setFilterFieldName ("z");
    //passx.setFilterLimitsNegative (true);
    pass.setFilterLimits (0.0001, 1.5);
    pass.filter (*pointcloud);
    return pointcloud;


}
auto point_add_viewer(std::string name ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud,name);
    
    return viewer;
}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
    if (event.getKeySym () == "s" &&event.keyDown ())
    {   
        save_flag=1;
    }
    else if(event.getKeySym () == "c" &&event.keyDown ())
    {
        cluster_flag=1;
    }
}
auto cluster_extraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,int number)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.003f, 0.003f, 0.003f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PLYWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.006);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;

  }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (2000);
    ec.setMaxClusterSize (60000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" <<number<<"_"<< j << ".ply";
        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
        j++;
    } 



}

int main(int argc,char* argv[])
{
    int save_count=0;
    int cluster_count=0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_save;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //std::vector<Eigen::Matrix4d> Matrix;
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());   



    //using namespace std;
    ifstream file(argv[1]);

    Eigen::Matrix4d Matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d hMatrix = Eigen::Matrix4d::Identity();
    if(file.is_open())
    {
        double vecmatrix[10][10];
        for(int i = 0; i < 8; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
            file >> vecmatrix[i][j];
            file.get();
            }

        }
        //std::cout<<vecmatrix<<std::endl;

        Matrix <<
        vecmatrix[0][0],vecmatrix[0][1],vecmatrix[0][2],vecmatrix[0][3],
        vecmatrix[1][0],vecmatrix[1][1],vecmatrix[1][2],vecmatrix[1][3],
        vecmatrix[2][0],vecmatrix[2][1],vecmatrix[2][2],vecmatrix[2][3],
        vecmatrix[3][0],vecmatrix[3][1],vecmatrix[3][2],vecmatrix[3][3];

        hMatrix <<
        vecmatrix[4][0],vecmatrix[4][1],vecmatrix[4][2],vecmatrix[4][3],
        vecmatrix[5][0],vecmatrix[5][1],vecmatrix[5][2],vecmatrix[5][3],
        vecmatrix[6][0],vecmatrix[6][1],vecmatrix[6][2],vecmatrix[6][3],
        vecmatrix[7][0],vecmatrix[7][1],vecmatrix[7][2],vecmatrix[7][3];
        std::cout<<Matrix<<std::endl;
        std::cout<<hMatrix<<std::endl;
    }


    ifstream file2(argv[2]);

    Eigen::Matrix4d Matrix2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d hMatrix2 = Eigen::Matrix4d::Identity();
    if(file2.is_open())
    {
        double vecmatrix[10][10];
        for(int i = 0; i < 8; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
            file2 >> vecmatrix[i][j];
            file2.get();
            }

        }
        //std::cout<<vecmatrix<<std::endl;

        Matrix2 <<
        vecmatrix[0][0],vecmatrix[0][1],vecmatrix[0][2],vecmatrix[0][3],
        vecmatrix[1][0],vecmatrix[1][1],vecmatrix[1][2],vecmatrix[1][3],
        vecmatrix[2][0],vecmatrix[2][1],vecmatrix[2][2],vecmatrix[2][3],
        vecmatrix[3][0],vecmatrix[3][1],vecmatrix[3][2],vecmatrix[3][3];

        hMatrix2 <<
        vecmatrix[4][0],vecmatrix[4][1],vecmatrix[4][2],vecmatrix[4][3],
        vecmatrix[5][0],vecmatrix[5][1],vecmatrix[5][2],vecmatrix[5][3],
        vecmatrix[6][0],vecmatrix[6][1],vecmatrix[6][2],vecmatrix[6][3],
        vecmatrix[7][0],vecmatrix[7][1],vecmatrix[7][2],vecmatrix[7][3];
        std::cout<<Matrix2<<std::endl;
        std::cout<<hMatrix2<<std::endl;
    }

    ifstream file3(argv[3]);

    Eigen::Matrix4d Matrix3 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d hMatrix3 = Eigen::Matrix4d::Identity();
    if(file3.is_open())
    {
        double vecmatrix[10][10];
        for(int i = 0; i < 8; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
            file3 >> vecmatrix[i][j];
            file3.get();
            }

        }
        //std::cout<<vecmatrix<<std::endl;

        Matrix3 <<
        vecmatrix[0][0],vecmatrix[0][1],vecmatrix[0][2],vecmatrix[0][3],
        vecmatrix[1][0],vecmatrix[1][1],vecmatrix[1][2],vecmatrix[1][3],
        vecmatrix[2][0],vecmatrix[2][1],vecmatrix[2][2],vecmatrix[2][3],
        vecmatrix[3][0],vecmatrix[3][1],vecmatrix[3][2],vecmatrix[3][3];

        hMatrix3 <<
        vecmatrix[4][0],vecmatrix[4][1],vecmatrix[4][2],vecmatrix[4][3],
        vecmatrix[5][0],vecmatrix[5][1],vecmatrix[5][2],vecmatrix[5][3],
        vecmatrix[6][0],vecmatrix[6][1],vecmatrix[6][2],vecmatrix[6][3],
        vecmatrix[7][0],vecmatrix[7][1],vecmatrix[7][2],vecmatrix[7][3];
        std::cout<<Matrix3<<std::endl;
        std::cout<<hMatrix3<<std::endl;
    }
    /*
    std::cout<<"0"<<std::endl;
    Eigen::Matrix4d Matrix1;
    Eigen::Matrix4d Matrix2;
    std::cout<<"0.5"<<std::endl;
    Eigen::Matrix4d hMatrix1;
    Eigen::Matrix4d hMatrix2;
    std::cout<<"0.7"<<std::endl;
    */
    //Eigen::Matrix4d Matrix1;
    /* 
    std::cout<<"0.9"<<std::endl;
    Matrix[0] <<
    1,1,1,1,
    1,1,1,1,
    1,1,1,1,
    1,1,1,1;

    std::cout<<"0.99"<<std::endl;
    Matrix[1] <<
    1,1,1,1,
    1,1,1,1,
    1,1,1,1,
    1,1,1,1;
    */
    //Matrix1,hMatrix1=read_matrix(argv[1]);
    //Matrix2,hMatrix2=read_matrix(argv[2]);
    //std::cout<<Matrix1<<std::endl;
    //std::cout<<hMatrix1<<std::endl;

    //std::cout<<Matrix2<<std::endl;
    //std::cout<<hMatrix2<<std::endl;

    /*
    Matrix.emplace_back(read_matrix(argv[1]));
    Matrix.emplace_back(read_matrix(argv[2]));
    */
    //Matrix.emplace_back(read_matrix(argv[3]));
    std::cout<<"1"<<std::endl;    
    std::vector<rs2::pipeline> pipelines;
    rs2::context    ctx;
    rs2::colorizer  colorizer;
    std::cout<<"pipeline connect start"<<std::endl;
    for (auto&& dev : ctx.query_devices())
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        std::cout<<"connect"<<std::endl;
        sleep(4);
    }
    int loop_cnt=0;

    while(1)
    {
        int pipecount=0;
        for (auto &&pipe : pipelines)
        {
            std::stringstream ss;
            ss<<"viewer"<<pipecount;
            auto rs_points_color=pointcloud_read(pipe);
            auto cloud=points_to_pcl(rs_points_color);
            cloud=zerozerozeroremove(cloud);
            //pcl::transformPointCloud(*cloud,*cloud,hMatrix[pipecount]);
            if(loop_cnt != 0){
                std::cout<<"remove"<<std::endl;
                viewer->removePointCloud( ss.str() );    
            }
            if(pipecount==0){
                pcl::transformPointCloud(*cloud,*cloud,hMatrix);
                pcl::transformPointCloud(*cloud,*cloud,Matrix);
                cloud1=cloud;
            }else if(pipecount==1){
                pcl::transformPointCloud(*cloud,*cloud,hMatrix2);
                pcl::transformPointCloud(*cloud,*cloud,Matrix2);
                cloud2=cloud;
            }else if(pipecount==2){
                pcl::transformPointCloud(*cloud,*cloud,hMatrix3);
                pcl::transformPointCloud(*cloud,*cloud,Matrix3);
                cloud3=cloud;
            }


                        
            
            //viewer=point_add_viewer(ss.str(),cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud,ss.str());
            //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());

            

            

            pipecount++;

        }
    viewer->spinOnce();
    if(save_flag==1)
    {
        std::stringstream ss;
        ss<<"mergedcloud"<<save_count<<".ply";
        cloud_save=cloud1;
        *cloud_save+=*cloud2;
        *cloud_save+=*cloud3;
        pcl::io::savePLYFile(ss.str(),*cloud_save);
        save_count++;
        save_flag=0;

    }
    else if(cluster_flag==1)
    {
        cloud_save=cloud1;
        *cloud_save+=*cloud2;
        *cloud_save+=*cloud3;
        cluster_extraction(cloud_save,cluster_count);

        cluster_count++;
        cluster_flag=0;


    }








    loop_cnt++;
    }
    return 0;
}


