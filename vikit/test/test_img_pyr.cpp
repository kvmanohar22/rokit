#include <vikit/utils.h>

using namespace rokit;
int main()
{
  const std::string root_dir = std::getenv("VIKIT_DIR");
  string path(root_dir+"/imgs/synthetic.png");  
  cv::Mat img;
  utils::loadImg(path, img);

  ImgPyr pyr;
  utils::constructImgPyramid(img, pyr);
  ROS_INFO_STREAM("pyramid size = " << pyr.size());
  int idx=1;
  for(const auto& img: pyr)
  {
    ROS_INFO_STREAM("lvl = " << idx << "\t size = " << img.size); 
    utils::displayImg("pyr", img);
    ++idx;
  }
}

