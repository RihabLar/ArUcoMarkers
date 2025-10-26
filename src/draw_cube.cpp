#include <iostream> // For standard input/output
#include <opencv2/aruco.hpp> // For ArUco marker functions
#include <opencv2/core.hpp> // For core OpenCV data structures
#include <opencv2/opencv.hpp> // For OpenCV operations (general)
#include <vector> // For std::vector
#include <cstdlib> // For C standard library functions

// Namespace for command-line options and default values
namespace
{
    const char *keys =
        "{d        |16    | dictionary: DICT_ARUCO_ORIGINAL = 16}" // Dictionary type for ArUco markers
        "{l        |      | Actual marker length in meter }" // Marker length (user input)
        "{v        |<none>| Custom video source, otherwise '0' }"; // Video source
}

// Function to draw a cube wireframe on the image
void drawCubeWireframe(
    cv::InputOutputArray image, cv::InputArray cameraMatrix,
    cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
    float l);

int main(int argc, char **argv)
{
    cv::CommandLineParser parser(argc, argv, keys); // Parse command-line arguments

    if (argc < 2) // Ensure the user provides sufficient arguments
    {
        parser.printMessage(); // Print usage instructions
        return 1; // Exit with error
    }

    int dictionaryId = parser.get<int>("d"); // Get dictionary ID
    float marker_length_m = parser.get<float>("l"); // Get marker length
    int wait_time = 10; // Time to wait between frames (in ms)

    if (marker_length_m <= 0) // Validate marker length
    {
        std::cerr << "marker length must be a positive value in meter"
                  << std::endl;
        return 1;
    }

    cv::String videoInput = "0"; // Default video source (webcam)
    cv::VideoCapture in_video;
    in_video.open(0); // Open the webcam

    if (!in_video.isOpened()) // Check if video source is available
    {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }

    cv::Mat image, image_copy; // Matrices for frames and their copies
    cv::Mat camera_matrix, dist_coeffs; // Camera calibration matrices
    std::ostringstream vector_to_marker; // To display marker info

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(dictionaryId); // Load ArUco dictionary

    cv::FileStorage fs("output_calibration4.yml", cv::FileStorage::READ); // Load camera calibration data

    fs["camera_matrix"] >> camera_matrix; // Read camera matrix
    fs["distortion_coefficients"] >> dist_coeffs; // Read distortion coefficients

    std::cout << "camera_matrix\n"
              << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n"
              << dist_coeffs << std::endl;

    int frame_width = in_video.get(cv::CAP_PROP_FRAME_WIDTH); // Frame width
    int frame_height = in_video.get(cv::CAP_PROP_FRAME_HEIGHT); // Frame height
    int fps = 30; // Frames per second for output video
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // Codec
    cv::VideoWriter video(
        "draw_cube.avi", fourcc, fps, cv::Size(frame_width, frame_height), true); // Video writer object

    while (in_video.grab()) // Grab frames from the video source
    {
        in_video.retrieve(image); // Retrieve the current frame
        image.copyTo(image_copy); // Create a copy for processing
        std::vector<int> ids; // IDs of detected markers
        std::vector<std::vector<cv::Point2f>> corners; // Corner points of detected markers
        cv::aruco::detectMarkers(image, dictionary, corners, ids); // Detect markers

        // If at least one marker is detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids); // Draw marker boundaries
            std::vector<cv::Vec3d> rvecs, tvecs; // Rotation and translation vectors
            cv::aruco::estimatePoseSingleMarkers(
                corners, marker_length_m, camera_matrix, dist_coeffs,
                rvecs, tvecs); // Estimate pose of each marker

            // Draw a 3D cube wireframe for each detected marker
            for (int i = 0; i < ids.size(); i++)
            {
                drawCubeWireframe(
                    image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i],
                    marker_length_m); // Draw cube
            }
        }

        video.write(image_copy); // Write processed frame to output video
        cv::imshow("Pose estimation", image_copy); // Display the frame
        char key = (char)cv::waitKey(wait_time); // Wait for user input
        if (key == 27) // Exit if 'Esc' key is pressed
            break;
    }

    in_video.release(); // Release video source

    return 0; // Exit successfully
}

// Function to draw a 3D cube wireframe on the detected marker
void drawCubeWireframe(
    cv::InputOutputArray image, cv::InputArray cameraMatrix,
    cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
    float l)
{
    CV_Assert(
        image.getMat().total() != 0 &&
        (image.getMat().channels() == 1 || image.getMat().channels() == 3)); // Ensure valid input image
    CV_Assert(l > 0); // Ensure cube size is positive
    float half_l = l / 2.0; // Half of cube edge length

    // Define cube corner points in 3D space
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(half_l, half_l, l));
    axisPoints.push_back(cv::Point3f(half_l, -half_l, l));
    axisPoints.push_back(cv::Point3f(-half_l, -half_l, l));
    axisPoints.push_back(cv::Point3f(-half_l, half_l, l));
    axisPoints.push_back(cv::Point3f(half_l, half_l, 0));
    axisPoints.push_back(cv::Point3f(half_l, -half_l, 0));
    axisPoints.push_back(cv::Point3f(-half_l, -half_l, 0));
    axisPoints.push_back(cv::Point3f(-half_l, half_l, 0));

    // Project 3D points to 2D image points
    std::vector<cv::Point2f> imagePoints;
    projectPoints(
        axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // Draw edges of the cube
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[4], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[2], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[3], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[5], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), 3);
}
