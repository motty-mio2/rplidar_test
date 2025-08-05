#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
  std::cout << "Hello, RPLIDAR!" << std::endl;
  // Additional code for RPLIDAR functionality can be added here.

  cv::imshow("Test Window", cv::Mat::zeros(300, 300, CV_8UC3));
  cv::waitKey(0);  // Wait for a key press to close the window
  return 0;
}