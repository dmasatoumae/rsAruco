#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>

int main(int argc,char* argv[])
{
    int wide = 1280;
    int height = 720;

	cv::Point2f center;
	std::vector<cv::Point2f> pt;
    std::vector<std::vector<cv::Point2f> > corners;
     std::vector<int> ids;

    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);


    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, wide, height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, wide, height, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    for(int i = 0; i < 10; i++)
    {
        auto frames = pipe.wait_for_frames();
    }
    while(1){
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto colored_frame = frames.get_color_frame();

         cv::Mat image(cv::Size(wide, height), CV_8UC3, (void*)colored_frame.get_data(), cv::Mat::AUTO_STEP);
         cv::Mat depthmat(cv::Size(wide, height), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
         cv::aruco::detectMarkers(image, dictionary, corners, ids);

         if (ids.size()>0)
         {

            center.x = 0;
    		center.y = 0;

				pt = corners.front();
    			for(int n = 0; n < 4; n++ )
    			{
        			//std::cout << depth_frame.get_distance(pt[i].x,pt[i].y) << std::endl;
        			center += pt[n];
   				 }
    			int x = (int) center.x / 4;
    			int y = (int) center.y / 4;
    			std::cout << depth.get_distance(x,y) <<" m"<< std::endl;
        depthmat.convertTo( depthmat, CV_8U, -255.0 / 10000.0, 255.0 );
        cv::equalizeHist(depthmat, depthmat);
        cv::applyColorMap(depthmat, depthmat, cv::COLORMAP_JET);
        cv::imshow("Pose estimation", image);
        cv::imshow("Pose estimation2", depthmat);
        cv::waitKey(5);
         }

    }
}
