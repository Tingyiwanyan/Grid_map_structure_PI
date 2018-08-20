#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main() {
  std::vector<std::string> file_names;
  
  for (size_t i = 0; i <= 2000; i++) {
    file_names.push_back("results23679_threshold60/sogp_" + std::to_string(i) + "_mean.txt");
  }
  
  const double max = 33.775578;
  const double min = 31.969374;
  const double range = (max - min);
  
  for (size_t i = 0; i < file_names.size(); i++) {
    std::ifstream prediction_file(file_names[i]);
    cv::Mat image(391, 351, CV_8UC3, cv::Scalar(0, 0, 0));
    for (size_t j = 0; j < 391; j++) {
      for (size_t k = 0; k < 351; k++) {
        std::string line;
        std::getline(prediction_file, line);
        double scalar = std::stod(line);
        
        if (scalar == std::numeric_limits<double>::max()) {
          std::cout << "scalar " << scalar <<std::endl;
          continue;
        }

        double value = 255.0 - (scalar - min) / range * 255.0;
        if (value < 0) {
          value = 0;
        } else if (value > 255.0) {
          value = 255.0;
        }
        
        image.at<cv::Vec3b>(391 - j - 1, k) = cv::Vec3b(value, value, value);
      }
    }
    std::cout << "i " << i << std::endl;
    std::string output_name = "results23679_threshold60/sogp_";
    for (size_t j = 0; j < 3 - std::log10(i + 1); j++) {
      output_name += "0";
    }
    output_name += std::to_string(i + 1);
    cv::imwrite(output_name + ".png", image);
  }
}