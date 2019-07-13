#include <librealsense2/rs.hpp>
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
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <tuple>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

#include <thread>
#include <mutex>

#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
auto read_matrix(std::string filename)
{
    using namespace std;
    ifstream file(filename);

    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    if(file.is_open())
    {
        double vecmatrix[4][4];
        for(int i = 0; i < 4; ++i)
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
        vecmatrix[2][0],vecmatrix[1][1],vecmatrix[1][2],vecmatrix[1][3],
        vecmatrix[2][0],vecmatrix[2][1],vecmatrix[2][2],vecmatrix[2][3],
        vecmatrix[3][0],vecmatrix[3][1],vecmatrix[3][2],vecmatrix[3][3];
        std::cout<<matrix<<std::endl;
        
    }

return matrix;
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
    auto depth_frame = frames.get_depth_frame();
    auto color_frame = frames.get_color_frame();


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
auto point_add_viewer(std::string name ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud,name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    
    return viewer;
}


int main(int argc,char* argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    std::vector<Eigen::Matrix4d> Matrix;

    Matrix.emplace_back(read_matrix(argv[1]));
    Matrix.emplace_back(read_matrix(argv[2]));
    Matrix.emplace_back(read_matrix(argv[3]));
    
    std::vector<rs2::pipeline> pipelines;
    rs2::context    ctx;
    rs2::colorizer  colorizer;
    std::cout<<"pipeline connect start"<<std::endl;
    for (auto&& dev : ctx.query_devices())
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        std::cout<<"connect"<<std::endl;
    }

    while(1)
    {
        int loop_cnt=0;
        std::stringstream ss;
        int pipecount=0;
        for (auto &&pipe : pipelines)
        {
            ss<<"viewer"<<pipecount;
            auto rs_points_color=pointcloud_read(pipe);
            auto cloud=points_to_pcl(rs_points_color);
            pcl::transformPointCloud(*cloud,*cloud,Matrix[pipecount]);
                        
            if(loop_cnt != 0){
                viewer->removePointCloud( ss.str() );    
            }
            viewer=point_add_viewer(ss.str(),cloud);


            

            

            pipecount++;

        }








    loop_cnt++;
    }
    return 0;
}


