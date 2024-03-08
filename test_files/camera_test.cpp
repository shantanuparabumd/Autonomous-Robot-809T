#include <raspicam/raspicam.h>
#include <iostream>
#include <fstream>

int main() {
    // Initialize the camera
    raspicam::RaspiCam camera;
    if (!camera.open()) {
        std::cerr << "Error opening camera" << std::endl;
        return -1;
    }

    // Wait a moment to stabilize the camera
    raspicam::delay(1000);

    // Set the image width and height (adjust as needed)
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    // Allocate memory for the image
    unsigned int width = camera.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int height = camera.get(CV_CAP_PROP_FRAME_HEIGHT);
    cv::Mat image(height, width, CV_8UC3);

    // Capture an image
    camera.grab();
    camera.retrieve(image);

    // Save the image to a file (adjust the filename as needed)
    std::string filename = "captured_image.jpg";
    cv::imwrite(filename, image);

    // Print a message indicating success
    std::cout << "Image captured and saved to: " << filename << std::endl;

    // Release the camera
    camera.release();

    return 0;
}
