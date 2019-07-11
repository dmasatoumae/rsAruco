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
    std::vector<cv::Vec3d> r;
    std::vector<cv::Vec3d> t;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    //camera parameters read yml file
    cv::FileStorage fs("../../calibration_params/calibration_paramsD4151.yml", cv::FileStorage::READ);
    
    cv::FileStorage fs2("../../calibration_params/calibration_paramsD435i2.yml", cv::FileStorage::READ);
    int device_count=0;
    FILE *fp;
    char key;
    int wide = 1280;
    int height = 720;
    float degree = 180/3.141592;
    double PI =3.141592;
    //マーカーのサイズ(m)
    float actual_marker_length = 0.195;


	cv::Point2f center;
    std::vector<cv::Mat> camera_matrix;
    std::vector<cv::Mat> dist_coeffs;
    cv::Mat camera_matrix_b, dist_coeffs_b;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

	std::vector<cv::Point2f> pt;
    std::vector<std::string> windown;
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<int> ids;
    //read カメラの内部パラメータ行列＆歪み
    fs["camera_matrix"] >> camera_matrix_b;
    fs["distortion_coefficients"] >> dist_coeffs_b;
    camera_matrix.emplace_back(camera_matrix_b);
    dist_coeffs.emplace_back(dist_coeffs_b);
    fs2["camera_matrix"] >> camera_matrix_b;
    fs2["distortion_coefficients"] >> dist_coeffs_b;
    camera_matrix.emplace_back(camera_matrix_b);
    dist_coeffs.emplace_back(dist_coeffs_b);
    std::cout << "camera_matrix\n" << camera_matrix[0] << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs[0] << std::endl;

    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    rs2::colorizer colormap;
    //rs2::pipeline pipe;
    std::vector<rs2::pipeline> pipelines;
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


    // std::vector<int> window_name;
    for (auto&& dev : ctx.query_devices())
    {
        std::cout<<"device connect"<<std::endl;
        std::stringstream ss;
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        cfg.enable_stream(RS2_STREAM_COLOR, wide, height, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, wide, height, RS2_FORMAT_Z16, 30);
        //std::cout<<"device connect2"<<std::endl;
        //for ()
        //ss<<bag_name<<".bag";
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        //std::cout<<"device connect3"<<std::endl;

        ss<<"window"<<device_count;
        windown.push_back(ss.str());

        cv::namedWindow(ss.str(),cv::WINDOW_AUTOSIZE);


        device_count++;
    }

    std::cout<<"device connect end"<<std::endl;
    colormap.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 1.f);
    colormap.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);


/*
    for(int i = 0; i < 10; i++)
    {
        auto frames = pipe.wait_for_frames();
    }
*/
    while(1){
        int count = 0;

        std::vector<rs2::points> points;
        rs2::pointcloud pc;
        std::vector<rs2::video_frame> colored_frame;
        r.clear();
        t.clear();
        x.clear();
        y.clear();
        z.clear();

        for(auto &&pipe : pipelines)
        {
            auto frames = pipe.wait_for_frames();
            //auto depth = frames.get_depth_frame();
            //auto colored_frame = frames.get_color_frame();

            // align
            rs2::align align(RS2_STREAM_COLOR);
            auto aligned_frames = align.process(frames);
            colored_frame.emplace_back(aligned_frames.first(RS2_STREAM_COLOR));
            rs2::depth_frame depth = aligned_frames.get_depth_frame()/*.apply_filter(colormap)*/; 

            cv::Mat image(cv::Size(wide, height), CV_8UC3, (void*)colored_frame[count].get_data(), cv::Mat::AUTO_STEP);
            //cv::Mat depthmat(cv::Size(wide, height), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            cv::aruco::detectMarkers(image, dictionary, corners, ids);
            //cv::Mat outputImage 
            cv::cvtColor(image,image,CV_RGB2BGR); 

        
        

            if (ids.size()>0)
            {
                pc.map_to(colored_frame[count]);

                points.emplace_back(pc.calculate(depth));
                //camera number
                std::cout<<count<<"ban"<<std::endl;
                cv::aruco::drawDetectedMarkers(image, corners, ids);
                //tとRのベクトル
                std::vector<cv::Vec3d> rvecs, tvecs;
                //マーカーのポーズ
                cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length, camera_matrix[count], dist_coeffs[count], rvecs, tvecs);
                for(int i=0; i < ids.size(); i++){
                    cv::aruco::drawAxis(image, camera_matrix[count], dist_coeffs[count], rvecs[i], tvecs[i], 0.1);
                    std::cout <<"x: " << rvecs[i][0]*degree << " y: " << rvecs[i][1]*degree << " z: "<< rvecs[i][2]*degree <<std::endl;
                    cv::Mat R(3, 3, cv::DataType<float>::type);
                    cv::Rodrigues(rvecs[i],R);
                    //auto Ri = R.inv();
                    auto Rt =R.t();
                    std::cout<<R<<std::endl;
                    std::cout<<Rt<<std::endl;
                        
                    int idcount=0;
                    for (auto&e : ids){
                    center.x = 0;
    		        center.y = 0;

                        if(e==0){
                            std::cout<<"0atta"<<std::endl;
                            r.emplace_back(rvecs[idcount]);
                            t.emplace_back(tvecs[idcount]);
                            x.emplace_back(tvecs[idcount][0]);
                            y.emplace_back(tvecs[idcount][1]);

			                pt = corners.front();
    		                for(int n = 0; n < 4; n++ )
    		                {
        		                //std::cout << depth_frame.get_distance(pt[i].x,pt[i].y) << std::endl;
        		                center += pt[n];
   			                }
    		                int x = (int) center.x / 4;
    		                int y = (int) center.y / 4;
                            std::cout<<depth.get_distance(x,y)<<std::endl;
    		                std::cout << depth.get_distance(x,y) <<" m"<< std::endl;
                            z.emplace_back(depth.get_distance(x,y));
                            std::cout<<"aruco marker (x,y,z)="<<tvecs[idcount][0]<<","<<tvecs[idcount][1]<<",";
                            }
                        idcount++;

                    }
                    std::cout<<"R="<<rvecs[i]<<std::endl;
                    std::cout<<"T="<<tvecs[i]<<std::endl;
          
                    //std::vector<rs2::pipeline>  pipelines;

                    //rs2::context                ctx;            // Create librealsense context for managing devices

                    //rs2::colorizer              colorizer;
                    /*
			        pt = corners.front();
    		        for(int n = 0; n < 4; n++ )
    		        {
        		        //std::cout << depth_frame.get_distance(pt[i].x,pt[i].y) << std::endl;
        		        center += pt[n];
   			        }
    		        int x = (int) center.x / 4;
    		        int y = (int) center.y / 4;
                    std::cout<<depth.get_distance(x,y)<<std::endl;
    		        std::cout << depth.get_distance(x,y) <<" m"<< std::endl;
                    z.emplace_back(depth.get_distance(x,y));
                    
                    if((fp=fopen("save.csv","a"))!=NULL){
                    fprintf(fp,"step %d,\n",count);
                    fprintf(fp,"%f,%f,%f\n",rvecs[0],rvecs[1],rvecs[2]);
                    fclose(fp);
                
                    }*/
                }
                //depthmat.convertTo( depthmat, CV_8U, -255.0 / 10000.0, 255.0 );
                //cv::equalizeHist(depthmat, depthmat);
                //cv::applyColorMap(depthmat, depthmat, cv::COLORMAP_JET);
                cv::imshow(windown[count], image);
                //cv::imshow("Pose estimation2", depthmat);
                count++;

                
            }
        //cv::imshow("Pose estimation", image);
        key=cv::waitKey(1);
        }
    if (key == 's'&&count == 2)
    {

        //points[0].export_to_ply("1.ply",colored_frame[0]);

        //points[1].export_to_ply("2.ply",colored_frame[1]);
        std::cout << "save ply_file" << std::endl;
        //cv::cvtColor(colored_frame[0], colored_frame[0], cv::RGB2BGR);
        //cv::cvtColor(colored_frame[1], colored_frame[1], cv::RGB2BGR);
        auto save_point0 = points_to_pcl(points[0],colored_frame[0]);
        auto save_point1 = points_to_pcl(points[1],colored_frame[1]);
        
        pcl::io::savePLYFileBinary("0.ply",*save_point0);
        pcl::io::savePLYFileBinary("1.ply",*save_point1);
        //pcl::transformPointCloud()
        
        save_point0=transform_cloud(save_point0,-x[0],-y[0],-z[0],0,0,0);
        save_point1=transform_cloud(save_point1,-x[1],-y[1],-z[1],0,0,0);
        //auto new0=transform_cloud(save_point0,t[1][0]-t[0][0],t[1][1]-t[0][1],t[1][2]-t[0][2],r[1][0]-r[0][0],r[1][1]-r[0][1],r[1][2]-r[0][2]);
        pcl::io::savePLYFileBinary("h0.ply",*save_point0);
        pcl::io::savePLYFileBinary("h1.ply",*save_point1);
        //std::cout<<"x:"<<t[1][0]-t[0][0]<<"y:"<<t[1][1]-t[0][1]<<"z:"<<t[1][2]-t[0][2]<<std::endl;

        //std::cout<<"xk:"<<r[1][0]-r[0][0]<<"yk:"<<r[1][1]-r[0][1]<<"zk:"<<r[1][2]-r[0][2]<<std::endl;     //auto new1=transform_cloud(save_point1,-t[1][0],-t[1][1],-t[1][1],0,0,0);
        std::vector<double> ro;
        std::vector<double> pi;
        std::vector<double> yo;
        for(int i=0;i<2;i++)
        {
            //横軸//dekita
            if(r[i][0]>=0){ro.emplace_back((PI-r[i][0]));
            }else {ro.emplace_back(-(-PI+r[i][0]));}
            //ぐるぐる
            if(r[i][1]>=0){pi.emplace_back(r[i][1]);
            }else {pi.emplace_back(-r[i][1]);}
            //縦軸//delita
            if(r[i][2]>=0){yo.emplace_back(r[i][2]);
            }else {yo.emplace_back(r[i][2]);}


        }

        auto new0=transform_cloud(save_point0,0,0,0,ro[0],yo[0],pi[0]);
        auto new1=transform_cloud(save_point1,0,0,0,ro[1],yo[1],pi[1]);
        pcl::io::savePLYFileBinary("n0.ply",*new0);
        pcl::io::savePLYFileBinary("n1.ply",*new1);
        std::cout<<"横軸:"<<ro[0]<<","<<"tate:"<<yo[0]<<","<<"guru:"<<pi[0]<<","<<std::endl;
        std::cout<<"横軸:"<<ro[1]<<","<<"tate:"<<yo[1]<<","<<"guru:"<<pi[1]<<","<<std::endl;
        //std::cout<<ro[1]<<","<<yo[1]<<","<<pi[1]<<","<<std::endl;

        break;


    }
    if (key == 'q') break;
    }
    return 0;
}
