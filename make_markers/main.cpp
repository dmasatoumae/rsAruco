#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

namespace {
const char* about = "Create a ChArUco marker image";
const char* keys  =
        "{@outfile |<none> | Output image }"
        "{sl       |       | Square side length (in pixels) }"
        "{ml       |       | Marker side length (in pixels) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{ids      |<none> | Four ids for the ChArUco marker: id1,id2,id3,id4 }"
        "{m        | 0     | Margins size (in pixels) }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }";
}

/**
 */
int main(int argc, char *argv[]) {
    //CommandLineParser parser(argc, argv, keys);
    //parser.about(about);

    //if(argc < 4) {
    //    parser.printMessage();
    //    return 0;
    //}

    //int squareLength = parser.get<int>("sl");
    //int markerLength = parser.get<int>("ml");
    //int dictionaryId = parser.get<int>("d");
    //string idsString = parser.get<string>("ids");
    //int margins = parser.get<int>("m");
    //int borderBits = parser.get<int>("bb");
    //bool showImage = parser.get<bool>("si");
    //String out = parser.get<String>(0);

    //if(!parser.check()) {
    //    parser.printErrors();
    //    return 0;
    //}

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250/*aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId)*/);

    //istringstream ss(idsString);
    //vector< string > splittedIds;
    //string token;
    //while(getline(ss, token, ','))
    //    splittedIds.push_back(token);
    //if(splittedIds.size() < 4) {
    //    cerr << "Incorrect ids format" << endl;
    //    parser.printMessage();
    //    return 0;
    //}
    //Vec4i ids;
    //for(int i = 0; i < 4; i++)
    //    ids[i] = atoi(splittedIds[i].c_str());

    Mat markerImg;
    aruco::drawMarker(dictionary, 23, 200, markerImg, 1);
    

    //if(showImage) {
        imshow("board", markerImg);
        waitKey(0);
    //}

    //imwrite(out, markerImg);

    return 0;
}
