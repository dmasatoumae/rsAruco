
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/highgui.h>
#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double x,double y,double z,double roll,double pitch,double yow){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>:: transformed_cloud;
    double theta_r = roll;
    double theta_p = pitch;
    double theta_y = yow;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x,y,z;
    //
    transform.rotate (Eigen::AngleAxisf (theta_y, Eigen::Vector3f::UnitZ()));
    transform.rotate (Eigen::AngleAxisf (theta_p, Eigen::Vector3f::UnitY()));
    transform.rotate (Eigen::AngleAxisf (theta_r, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud (*cloud, *cloud_trans, transform);

    return cloud_trans;

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


pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
{
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

        if((vertices[i].z)<2.5){
            //std::cout<<vertices[i].z<<std::endl;
		    cloud->points[i].x = vertices[i].x;
		    cloud->points[i].y = vertices[i].y;
            cloud->points[i].z = vertices[i].z;

        }

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);
        if((vertices[i].z)<2.5){
            //std::cout<<vertices[i].z<<std::endl;
            cloud->points[i].r = std::get<0>(current_color);
		    cloud->points[i].g = std::get<1>(current_color);
		    cloud->points[i].b = std::get<2>(current_color);

            

        }

        
	}

	return cloud;
}
int main(int argc,char* argv[])
{
    
    cv::FileStorage fs("../../calibration_params/calibration_paramsD4352.yml", cv::FileStorage::READ);
    float x;
    float y;
    float z;

    cv::Vec3d r;
    cv::Vec3d t;
    cv::Vec3d camR={14.738876,0.099123,0.136103};

    FILE *fp;
    char key;
    int wide = 1280;
    int height = 720;
    float degree = 180/3.141592;
    double PI =3.141592;
    //マーカーのサイズ(m)
    float actual_marker_length = 0.195;

    cv::Mat R(3, 3, cv::DataType<float>::type);
    cv::Mat Rt(3, 3, cv::DataType<float>::type);
    std::vector<cv::Point2f> pt;
	cv::Point2f center;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    cv::Ptr<cv::aruco::DetectorParameters> parameters;

    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<int> ids;


    //read カメラの内部パラメータ行列＆歪み
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;


    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;


    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    rs2::colorizer colormap;
    //rs2::pipeline pipe;
    rs2::pipeline pipelines;
    rs2::context ctx;
    rs2::decimation_filter dec_filter;
    //depth filter
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


    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    //cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    cfg.enable_stream(RS2_STREAM_COLOR, wide, height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, wide, height, RS2_FORMAT_Z16, 30);
    //std::cout<<"device connect2"<<std::endl;
    //for ()
    //ss<<bag_name<<".bag";
    pipe.start(cfg);

    cv::namedWindow("window",cv::WINDOW_AUTOSIZE);
    while(1){
        pt.clear();
        ids.clear();
        corners.clear();
        rs2::points points;
        rs2::pointcloud pc;
        //rs2::video_frame colored_frame;

        auto frames=pipe.wait_for_frames();
        
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned_frames = align.process(frames);
        

        rs2::video_frame colored_frame=aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame depth = aligned_frames.get_depth_frame()/*.apply_filter(colormap)*/;

        cv::Mat image(cv::Size(wide, height), CV_8UC3, (void*)colored_frame.get_data(), cv::Mat::AUTO_STEP);


        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        //cv::Mat outputImage 
        cv::cvtColor(image,image,CV_RGB2BGR); 
        

        if (ids.size()>0)
        {
            pc.map_to(colored_frame);

            points=(pc.calculate(depth));
            std::cout<<"1"<<std::endl;
            //camera number
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            //tとRのベクトル
            std::cout<<"2"<<std::endl;

            std::vector<cv::Vec3d> rvecs, tvecs;
            //マーカーのポーズ
            cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);

            std::cout<<"3"<<std::endl;

            //for(int i=0; i < ids.size(); i++){
            int i =0;
            for (auto&e : ids){
                if(e==0){
                    std::cout<<"3.5"<<std::endl;
                    cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.5);
                    std::cout <<"x: " << rvecs[i][0]*degree << " y: " << rvecs[i][1]*degree << " z: "<< rvecs[i][2]*degree <<std::endl;
                    //cv::Mat Rbefor(3, 3, cv::DataType<float>::type);
                    cv::Rodrigues(rvecs[i],R);
                    std::cout<<"4"<<std::endl;

                    //auto Ri = R.inv();
                    Rt=(R.t());
                    //std::cout<<R<<std::endl;
                    //std::cout<<Rt<<std::endl;


                
                    center.x = 0;
    		        center.y = 0;
                    /*
                    if(e==0)
                    {
                    */
                    std::cout<<"0atta"<<std::endl;
                    //r.emplace_back(rvecs);
                    //t.emplace_back(tvecs);
                    x=tvecs[i][0];
                    y=tvecs[i][1];

			        pt = corners.front();
    		        for(int n = 0; n < 4; n++ )
    		        {
        		        //std::cout << depth_frame.get_distance(pt[i].x,pt[i].y) << std::endl;
        		        center += pt[n];
   			        }
    		        int xx = (int) center.x / 4;
    		        int yy = (int) center.y / 4;
                    std::cout<<depth.get_distance(xx,yy)<<std::endl;
    		        std::cout << depth.get_distance(xx,yy) <<" m"<< std::endl;
                    z=depth.get_distance(xx,yy);
                    /*
                    z.emplace_back(depth.get_distance(x,y));
                    std::cout<<"aruco marker (x,y,z)="<<tvecs[idcount][0]<<","<<tvecs[idcount][1]<<",";
                    */
                }
            i++;

            }

            //std::cout<<"R="<<rvecs[i]<<std::endl;

            //std::cout<<"T="<<tvecs[i]<<std::endl;
            cv::imshow("window", image);
            //count++;
        }
        //cv::imshow("window", image);
        key=cv::waitKey(5);
        

        if (key == 's')
        {

            std::cout << "save ply_file" << std::endl;
            //cv::cvtColor(colored_frame[0], colored_frame[0], cv::RGB2BGR);
            //cv::cvtColor(colored_frame[1], colored_frame[1], cv::RGB2BGR);
            auto save_point0 = points_to_pcl(points,colored_frame);
        
            pcl::io::savePLYFileBinary("0.ply",*save_point0);
            //pcl::transformPointCloud()
        
            save_point0=transform_cloud(save_point0,-x,-y,-z,0,0,0);
            //auto new0=transform_cloud(save_point0,t[1][0]-t[0][0],t[1][1]-t[0][1],t[1][2]-t[0][2],r[1][0]-r[0][0],r[1][1]-r[0][1],r[1][2]-r[0][2]);
            pcl::io::savePLYFileBinary("h0.ply",*save_point0);
            //std::cout<<"x:"<<t[1][0]-t[0][0]<<"y:"<<t[1][1]-t[0][1]<<"z:"<<t[1][2]-t[0][2]<<std::endl;

            //std::cout<<"xk:"<<r[1][0]-r[0][0]<<"yk:"<<r[1][1]-r[0][1]<<"zk:"<<r[1][2]-r[0][2]<<std::endl;     //auto new1=transform_cloud(save_point1,-t[1][0],-t[1][1],-t[1][1],0,0,0);
            std::vector<double> ro;
            std::vector<double> pi;
            std::vector<double> yo;
        
            Eigen::Matrix4f transform_1=Eigen::Matrix4f::Identity();
            /* 
            transform_1 <<
            Rt[i](0,0),
            */
            //cv::Mat3b Rtpoi = Rt[0];
            std::cout<<Rt<<std::endl;
            //transform_1(0,0)=Rtpoi(cv::Point(0,0));
            Eigen::Matrix4f tmatrix;
            tmatrix <<
            Rt.at<double>(0,0), Rt.at<double>(0,1), Rt.at<double>(0,2), 0.0,
            Rt.at<double>(1,0), Rt.at<double>(1,1), Rt.at<double>(1,2), 0.0,
            Rt.at<double>(2.0), Rt.at<double>(2,1), Rt.at<double>(2,2), 0.0,
            0.0, 0.0, 0.0, 1;
            pcl::transformPointCloud(*save_point0, *save_point0, tmatrix);

            pcl::io::savePLYFileBinary(argv[1],*save_point0);
            /* 
            transform_1(0,0)=Rt.at<float>(0,0 );
        
            transform_1(0,1)=Rt.at<float>(0,1);
            transform_1(0,2)=Rt.at<float>(0,2);
            transform_1(1,0)=Rt.at<float>(1,0);
            transform_1(1,1)=Rt.at<float>(1,1);
            transform_1(1,2)=Rt.at<float>(1,2);
            transform_1(2,0)=Rt.at<float>(2,0);
            transform_1(2,1)=Rt.at<float>(2,1);
            transform_1(2,2)=Rt.at<float>(2,2);
            std::cout<<transform_1<<std::endl;
            */
            std::cout<<tmatrix<<std::endl;
            
            //transform_1
        

            break;
        }
    }
    
    return 0;

} 
