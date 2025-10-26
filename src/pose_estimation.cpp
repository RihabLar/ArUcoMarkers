#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Text drawing utility
void drawText(Mat& image, const string& label, double value, Point position, 
             Scalar color = Scalar(255, 255, 255)) {
    string text = format("%s: %.2f", label.c_str(), value);
    putText(image, text, position, FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
}

int main(int argc, char **argv) {
    // Argument parsing
    CommandLineParser parser(argc, argv, 
        "{d|0|Dictionary ID}"
        "{l|0.05|Marker length (meters)}"
        "{id|0|Target marker ID}"
        "{calib||Calibration file}"
        "{help||Show help}");
    
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    float markerLength = parser.get<float>("l");
    int targetId = parser.get<int>("id");
    string calibFile = parser.get<string>("calib");

    if (calibFile.empty()) {
        cerr << "Error: Calibration file not specified! Use -calib to provide the file path." << endl;
        return 1;
    }

    cout << "Calibration file: " << calibFile << endl;  // Debug print


    // Load calibration
    Mat cameraMatrix, distCoeffs;
    FileStorage fs(calibFile, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open calibration file: " << calibFile << endl;
        return 1;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    // Video capture
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Failed to open video stream" << endl;
        return 1;
    }

    // ArUco setup
    Ptr<aruco::Dictionary> dictionary = 
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    while (cap.grab()) {
        Mat image, imageCopy;
        cap.retrieve(image);
        image.copyTo(imageCopy);

        // Marker detection
        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);

        // Pose estimation
        if (!ids.empty()) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids, Scalar(0, 255, 0));
            
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, markerLength, 
                                           cameraMatrix, distCoeffs, 
                                           rvecs, tvecs);

            // Find target marker
            for (size_t i = 0; i < ids.size(); i++) {
                if (ids[i] == targetId) {
                    // Draw coordinate axes
                    aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs,
                                  rvecs[i], tvecs[i], markerLength * 0.5);

                    // Display pose info
                    drawText(imageCopy, "X", tvecs[i][0], Point(10, 30), Scalar(0, 0, 255));
                    drawText(imageCopy, "Y", tvecs[i][1], Point(10, 60), Scalar(0, 255, 0));
                    drawText(imageCopy, "Z", tvecs[i][2], Point(10, 90), Scalar(255, 0, 0));
                    
                    // Display marker ID
                    putText(imageCopy, "ID: " + to_string(targetId),
                           Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.6, 
                           Scalar(255, 0, 255), 2);
                    break;
                }
            }
        }

        imshow("Pose Estimation", imageCopy);
        if (waitKey(10) == 27) break;
    }

    return 0;
}