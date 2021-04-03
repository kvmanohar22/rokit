#include <rk_common/pinhole_camera.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <vikit/feature_detector.h>
#include <vikit/frame.h>
#include <vikit/utils.h>

namespace
{

class FeatureDetectionTest
{
public:
  FeatureDetectionTest(
     const size_t width, const size_t height);
  virtual ~FeatureDetectionTest();

  void test();

private:
  rokit::FeatureDetector*    feature_detector_; 
  size_t                     width_;
  size_t                     height_;
  std::string                dataset_root_;
  rokit::PinholeCamera*      camera_;
}; // class FeatureDetectionTest

FeatureDetectionTest::FeatureDetectionTest(
    const size_t width, const size_t height) :
  width_(width),
  height_(height),
  dataset_root_(std::getenv("RPG_SYNTHETIC_DATASET_DIR"))
{
  feature_detector_ = new rokit::FeatureDetector(width_, height_, 20, 20);
  camera_ = new rokit::PinholeCamera(
      329.115520046, 329.115520046, 320.0, 240.0,
      0.0, 0.0, 0.0, 0.0, 0.0,
      width_, height_);
}

FeatureDetectionTest::~FeatureDetectionTest()
{
  delete feature_detector_;
}

void FeatureDetectionTest::test()
{
  const std::string img_path(dataset_root_+"/img/img0001_0.png");
  const cv::Mat img = cv::imread(img_path.c_str(), cv::IMREAD_GRAYSCALE);
  if(img.empty())
  {
    ROS_FATAL_STREAM("Empty image.");
    return;
  }

  rokit::FramePtr frame;
  frame.reset(new rokit::Frame(img, camera_));
  rokit::Features fts;
  rokit::ImgPyr pyr;
  rokit::utils::constructImgPyramid(img, pyr);
  feature_detector_->detect(frame, pyr, fts);
  ROS_INFO_STREAM("Detected " << fts.size() << " features.");

  cv::Mat tmp = img.clone();
  cv::cvtColor(tmp, tmp, cv::COLOR_GRAY2BGR);
  rokit::utils::drawFeatures(tmp, fts);

  std::unordered_map<int, int> count;
  std::for_each(fts.begin(), fts.end(), [&](rokit::FeaturePtr& f) -> void {
    if(count.find(f->lvl_) == count.end())
      count[f->lvl_] = 1;
    else
      ++count[f->lvl_];
  });

  for(const auto c: count)
    ROS_INFO_STREAM("lvl = " << c.first << "\t features = " << c.second);

  // draw the grid
  for(int i=0; i<width_; i+=20)
    cv::line(tmp, cv::Point2f(i, 0), cv::Point2f(i, height_), cv::Scalar(255, 0, 0));
  for(int j=0; j<height_; j+=20)
    cv::line(tmp, cv::Point2f(0, j), cv::Point2f(width_, j), cv::Scalar(255, 0, 0));
  rokit::utils::displayImg("features", tmp);
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_detection_test_node");
  ros::NodeHandle nh;

  ::FeatureDetectionTest tester(640, 480);
  tester.test();
}

