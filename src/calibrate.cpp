#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <set>

using namespace std;
using namespace cv;

namespace {
    // 'keys' stores the argument definitions for CommandLineParser
    const char *keys =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side length (in meters) }"
        "{s        |       | Separation between markers (in meters) }"
        "{d        |       | Dictionary ID (DICT_ARUCO_ORIGINAL=16)}"
        "{@outfile |<none> | Output calibration file }"
        "{ci       | 0     | Camera ID }"
        "{dp       |       | Detector parameters file }"
        "{waitkey  | 10    | Delay for key press }"
        "{minframes| 20    | Minimum frames required }";
}
/**
* @brief Reads custom ArUco detector parameters from a file and applies them to 'params'
* @param filename Path to the YAML file with parameter definitions
* @param params   Ptr<DetectorParameters> to fill
* @return true if file was valid and parameters loaded, false otherwise
*/
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    // Load each parameter from the file into the DetectorParameters object 
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if (!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if (flags & CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;

    if (flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

int main(int argc, char *argv[]) {
    // Parse command-line arguments based on 'keys' definition
    CommandLineParser parser(argc, argv, keys);

    // We need at least 6 arguments for a valid calibration run (w, h, l, s, d, outfile)

    if (argc < 6) {
        parser.printMessage();
        return 0;
    }
    // Read values for board configuration, dictionary, and output file
    int markersX = parser.get<int>("w");
    int markersY = parser.get<int>("h");
    float markerLength = parser.get<float>("l");
    float markerSeparation = parser.get<float>("s");
    int dictionaryId = parser.get<int>("d");
    string outputFile = parser.get<String>(0);
    const int MIN_FRAMES = parser.get<int>("minframes");

    
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    // If a parameters file is provided, read it and overwrite default detectorParams
    if (parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if (!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    // Open the camera specified by "ci" (default=0)
    VideoCapture inputVideo(parser.get<int>("ci"));
    if (!inputVideo.isOpened()) {
        cerr << "Failed to open video input" << endl;
        return 1;
    }

    // Load the chosen ArUco dictionary
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionaryId);
    
    // Create a GridBoard (markersX x markersY) with the chosen dictionary
    Ptr<aruco::GridBoard> board = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);

    // These vectors will store all the corners/IDs detected across multiple frames
    vector<vector<vector<Point2f>>> allCorners;
    vector<vector<int>> allIds;
    Size imgSize;

    while (inputVideo.grab()) {
        Mat image, imageCopy;
        // Retrieve the latest frame from the camera
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);
        
        // Detect ArUco markers in the frame
        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // If we found any markers, draw them on the copy
        if (ids.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

        putText(imageCopy, format("Frames: %zu/%d | Press 'c' to capture", 
                                allIds.size(), MIN_FRAMES),
                Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
         
        // Show the annotated frame        
        imshow("Calibration", imageCopy);

        // Wait for a key press for 'waitkey' milliseconds
        char key = (char)waitKey(parser.get<int>("waitkey"));
        if (key == 27)   // ESC key to exit
            break;
        
        // If 'c' is pressed and we have detected markers, attempt to capture the frame
        if (key == 'c' && ids.size() > 0) {
            set<int> detectedIds(ids.begin(), ids.end());
            // Check if all markers in the board are present
            bool allMarkersPresent = true;
            for (int id : board->ids) {
                if (detectedIds.find(id) == detectedIds.end()) {
                    allMarkersPresent = false;
                    break;
                }
            }
            // Only capture if the board is fully visible
            if (allMarkersPresent) {
                cout << "Frame captured (" << allIds.size()+1 << "/" << MIN_FRAMES << ")" << endl;
                allCorners.push_back(corners);
                allIds.push_back(ids);
                imgSize = image.size();
            } else {
                cout << "Frame rejected - missing markers" << endl;
            }
        }
    }

    // If we didn't capture enough frames, calibration cannot be performed
    if (allIds.size() < MIN_FRAMES) {
        cerr << "Insufficient frames: " << allIds.size() << "/" << MIN_FRAMES << endl;
        return 1;
    }

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    vector<Mat> rvecs, tvecs;
    
    // We need to concatenate all corners and IDs into single arrays
    vector<vector<Point2f>> allCornersConcatenated;
    vector<int> allIdsConcatenated;
    vector<int> markerCounterPerFrame;
    for (size_t i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for (size_t j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // Perform camera calibration using the ArUco board detection
    double repError = aruco::calibrateCameraAruco(
        allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame,
        board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs
    );
    
    // Save the resulting camera parameters
    if (!saveCameraParams(outputFile, imgSize, 1.0, 0, cameraMatrix, distCoeffs, repError)) {
        cerr << "Failed to save calibration" << endl;
        return 1;
    }

    // Print results to console
    cout << "Calibration successful!" << endl;
    cout << "Reprojection error: " << repError << endl;
    cout << "Camera matrix:\n" << cameraMatrix << endl;
    cout << "Distortion coefficients: " << distCoeffs.t() << endl;

    return 0;
}