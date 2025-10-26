#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main(int argc, char** argv) {
    // Check if exactly 1 argument is provided (besides the program name)
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <dictionary_id>" << std::endl;
        return 1;
    }

    int dictionary_id = std::stoi(argv[1]); // Convert input argument to an integer
    // Validate that the dictionary ID is within the known range 0..16
    if (dictionary_id < 0 || dictionary_id > 16) { // Validate input
        std::cerr << "Invalid dictionary ID. Use a number between 0 and 16." << std::endl;
        return 1;
    }
   // Validate that the dictionary ID is within the known range 0..16
    cv::Ptr<cv::aruco::Dictionary> dictionary = 
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

    cv::VideoCapture inputVideo(0);
    if (!inputVideo.isOpened()) {
        std::cerr << "ERROR: Could not open video stream." << std::endl;
        return 1;
    }

    // Variables to hold the current frame from the camera and a copy for drawing
    cv::Mat frame, imageCopy;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    while (inputVideo.grab()) {
        // Retrieve (decode) the current frame
        inputVideo.retrieve(frame);
        // Make a copy of the original frame for drawing
        frame.copyTo(imageCopy);
        // Detect ArUco markers in the frame
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
        // If any markers have been found, draw them on the copy
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }
        // Show the processed frame 
        cv::imshow("Detected ArUco markers", imageCopy);
        if ((char)cv::waitKey(10) == 27) break; // ESC key to exit
    }

    return 0;
}
