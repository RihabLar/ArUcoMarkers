#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace std;

namespace {
const char* about = "Create an ArUco grid board image";
const char* keys  =
        "{@outfile |<none> | Output image }"
        "{r        |       | Number of markers in X direction }"
        "{c        |       | Number of markers in Y direction }"
        "{l        |       | Marker side length (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | Dictionary ID (e.g., DICT_4X4_50=0, DICT_ARUCO_ORIGINAL=16)}"
        "{si       | false | Show generated image }";
}

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 7) {
        parser.printMessage();
        return -1;
    }

    int markersR = parser.get<int>("r"); // Rows
    int markersC = parser.get<int>("c"); // Columns
    int markerSize = parser.get<int>("l"); // Marker size
    int markerSeparation = parser.get<int>("s"); // Marker separation
    int dictionaryId = parser.get<int>("d"); // Dictionary ID
    bool showImage = parser.get<bool>("si"); // Show image flag
    String out = parser.get<String>(0); // Output file

    if (!parser.check()) {
        parser.printErrors();
        return -1;
    }

    // Calculate the total size of the board
    Size imageSize;
    imageSize.width = markersC * (markerSize + markerSeparation) - markerSeparation;
    imageSize.height = markersR * (markerSize + markerSeparation) - markerSeparation;

    // Get the predefined dictionary
    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // Create the ArUco board
    Ptr<aruco::GridBoard> board = aruco::GridBoard::create(
        markersC, markersR, float(markerSize), float(markerSeparation), dictionary);

    // Draw the board
    Mat boardImage;
    board->draw(imageSize, boardImage);

    // Add a white border around the entire board
    int borderSize = markerSize / 2; // Adjust the border size as needed
    Mat borderedImage(imageSize.height + 2 * borderSize, imageSize.width + 2 * borderSize, boardImage.type(), Scalar(255, 255, 255));
    boardImage.copyTo(borderedImage(Rect(borderSize, borderSize, imageSize.width, imageSize.height)));

    // Save the board image
    if (!imwrite(out, borderedImage)) {
        cerr << "Error: Failed to save the board image." << endl;
        return -1;
    }

    cout << "ArUco board generated and saved as " << out << endl;

    // Optionally display the board
    if (showImage) {
        imshow("board", borderedImage);
        waitKey(0);
    }

    return 0;
}