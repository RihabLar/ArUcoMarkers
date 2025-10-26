
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;

namespace {
const char* about = "Create an ArUco marker image";
const char* keys  =
        "{@outfile |<none> | Output image }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{id       |       | Marker id in the dictionary }"
        "{ms       | 200   | Marker size in pixels }"
        "{si       | falsecd .. | show generated image }";
}


int main(int argc, char *argv[]) {
    // Create a CommandLineParser to parse the arguments based on 'keys'
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 4) {
        parser.printMessage();
        return 0;
    }
    
    // Read dictionary ID 
    int dictionaryId = parser.get<int>("d");
    // Read marker ID
    int markerId = parser.get<int>("id");
    // Read marker size
    int markerSize = parser.get<int>("ms");
    // Select if to show the generated image
    bool showImage = parser.get<bool>("si");

    String out = parser.get<String>(0);
    // Check if the parser found any errors
    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    // Draw the ArUco marker into 'markerImg'
    Mat markerImg;
    aruco::drawMarker(dictionary, markerId, markerSize, markerImg);

    // Add a white border around the marker./pose_estimation -d 16 -id 23 -l 0.05
    int borderWidth = markerSize / 10; // Border width is 10% of the marker size
    Mat markerWithBorder;
    copyMakeBorder(markerImg, markerWithBorder, borderWidth, borderWidth, borderWidth, borderWidth,
                    BORDER_CONSTANT, Scalar(255)); // Add a white border
    
    // If showImage == true, then pop up a window to display the marker
    if(showImage) {
        imshow("marker", markerWithBorder);
        waitKey(0);
    }
    // Save the result
    imwrite(out, markerWithBorder);

    return 0;
}