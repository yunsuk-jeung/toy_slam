#include <opencv2/opencv.hpp>

auto main() -> int {
  uint8_t* buffer = new uint8_t[4];

  {
    cv::Mat test(2, 2, CV_8UC1, buffer);
    cv::imshow("test", test);
    cv::waitKey();
    delete[] buffer;
  }
  delete[] buffer;

  return 0;
}
