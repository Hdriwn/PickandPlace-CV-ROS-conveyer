#include <opencv2/opencv.hpp>

int main()
{
  int board_width = 9; 
  int board_height = 6;
  int square_size = 30; 

  cv::Size pattern_size(board_width * square_size, board_height * square_size);
  cv::Mat pattern(pattern_size, CV_8UC1, cv::Scalar(255));

  for (int i = 0; i < board_height; ++i)
  {
    for (int j = 0; j < board_width; ++j)
    {
      if ((i + j) % 2 == 1)
      {
        cv::Rect rect(j * square_size, i * square_size, square_size, square_size);
        pattern(rect) = cv::Scalar(0);
      }
    }
  }

  std::cout << "Pattern size: " << pattern_size.width << " x " << pattern_size.height << std::endl;

  std::string filename = "calibration_pattern.png";
  cv::imwrite(filename, pattern);

  std::cout << "Calibration pattern saved to " << filename << std::endl;

  return 0;
}

