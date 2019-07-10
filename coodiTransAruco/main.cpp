#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>

int main(int argc,char* argv[])
{
    //camera parameters read yml file
    cv::FileStorage fs("../calibration_paramsD435i2.yml", cv::FileStorage::READ);
    
    int count = 0;
    int device_count=0;
    FILE *fp;

    int wide = 1280;
    int height = 720;
    float degree = 180/3.141592;
    //マーカーのサイズ(m)
    float actual_marker_length = 0.195;

	cv::Point2f center;
    cv::Mat camera_matrix, dist_coeffs;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

	std::vector<cv::Point2f> pt;
    std::vector<std::string> windown;
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
    std::vector<rs2::pipeline> pipelines;
    rs2::context ctx; 
    // std::vector<int> window_name;
    for (auto&& dev : ctx.query_devices())
    {
        std::stringstream ss;
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        cfg.enable_stream(RS2_STREAM_COLOR, wide, height, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, wide, height, RS2_FORMAT_Z16, 30);
        //ss<<bag_name<<".bag";
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        ss<<"window"<<device_count;
        windown.push_back(ss.str());

        cv::namedWindow(ss.str(),CV_WINDOW_AUTOSIZE);


        device_count++;
    }
    colormap.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 1.f);
    colormap.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);


/*
    for(int i = 0; i < 10; i++)
    {
        auto frames = pipe.wait_for_frames();
    }
*/
    while(1){

        for(auto &&pipe : pipelines)
        {
            auto frames = pipe.wait_for_frames();
            //auto depth = frames.get_depth_frame();
            //auto colored_frame = frames.get_color_frame();

            // align
            rs2::align align(RS2_STREAM_COLOR);
            auto aligned_frames = align.process(frames);
            rs2::video_frame colored_frame = aligned_frames.first(RS2_STREAM_COLOR);
            rs2::depth_frame depth = aligned_frames.get_depth_frame()/*.apply_filter(colormap)*/; 

            cv::Mat image(cv::Size(wide, height), CV_8UC3, (void*)colored_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthmat(cv::Size(wide, height), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            cv::aruco::detectMarkers(image, dictionary, corners, ids);
            //cv::Mat outputImage 
       

        
        

            if (ids.size()>0)
            {
                //camera number
                std::cout<<count<<"ban"<<std::endl;
                cv::aruco::drawDetectedMarkers(image, corners, ids);
                //tとRのベクトル
                std::vector<cv::Vec3d> rvecs, tvecs;
                //マーカーのポーズ
                cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);
                for(int i=0; i < ids.size(); i++){
                    cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
                    std::cout <<"x: " << rvecs[i][0]*degree << " y: " << rvecs[i][1]*degree << " z: "<< rvecs[i][2]*degree <<std::endl;

                    center.x = 0;
    		        center.y = 0;
          
                    std::vector<rs2::pipeline>  pipelines;

                    rs2::context                ctx;            // Create librealsense context for managing devices

                    rs2::colorizer              colorizer;
			        pt = corners.front();
    		        for(int n = 0; n < 4; n++ )
    		        {
        		        //std::cout << depth_frame.get_distance(pt[i].x,pt[i].y) << std::endl;
        		        center += pt[n];
   			        }
    		        int x = (int) center.x / 4;
    		        int y = (int) center.y / 4;
    		        std::cout << depth.get_distance(x,y) <<" m"<< std::endl;
                    /*
                    if((fp=fopen("save.csv","a"))!=NULL){
                    fprintf(fp,"step %d,\n",count);
                    fprintf(fp,"%f,%f,%f\n",rvecs[0],rvecs[1],rvecs[2]);
                    fclose(fp);
                
                    }*/
                }
                depthmat.convertTo( depthmat, CV_8U, -255.0 / 10000.0, 255.0 );
                cv::equalizeHist(depthmat, depthmat);
                cv::applyColorMap(depthmat, depthmat, cv::COLORMAP_JET);
                cv::imshow(windown[count], image);
                //cv::imshow("Pose estimation2", depthmat);
                
            }
        //cv::imshow("Pose estimation", image);
        cv::waitKey(1);
        count++;
        }
    }
    return 0;
}
