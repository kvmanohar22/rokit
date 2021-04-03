#include <algorithm>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vikit/feature_detector.h>
#include <vikit/frame.h>
#include <vikit/point.h>
#include <vikit/utils.h>
#include <vikit/image_alignment.h>
#include <rk_common/pinhole_camera.h>
#include <rk_common/params_helper.h>

namespace {

using namespace rokit;
class TestImageAlignment
{
public:
  TestImageAlignment(
     const size_t width, const size_t height);
  virtual ~TestImageAlignment();

  void test();

  /// initializes the reference frame
  void initRefFrame();

  /// tracks current frame wrt to reference frame
  bool trackCurFrame(const string cur_img_idx);
  bool inline exit() const { return exit_; }

  void loadGtPose(const int idx, Vector3d& t, Eigen::Quaterniond& q);

private:
  FeatureDetector*     feature_detector_;
  size_t               width_;
  size_t               height_;
  string               dataset_root_;
  FramePtr             frame_ref_;          //!< ref frame
  FramePtr             frame_cur_;          //!< cur frame
  PinholeCamera*       camera_;
  string               ref_img_idx_;
  bool                 exit_;
  ImageAlignment*      img_align_;
  int                  n_curr_frames_;      //!< no. of current frames to align
  SE3                  T_w_last_;           //!< pose of camera pose in world to initialize
  std::string          results_file_;
  std::ofstream        ofs_;
}; // class TestImageAlignment

TestImageAlignment::TestImageAlignment(
    const size_t width, const size_t height) :
  width_(width),
  height_(height),
  dataset_root_(utils::getParam<string>("img_align/data_dir")),
  ref_img_idx_(utils::getParam<string>("img_align/ref_img_idx").substr(3)),
  n_curr_frames_(utils::getParam<int>("img_align/n_curr_frames")),
  results_file_(utils::getParam<string>("img_align/results_file")),
  exit_(false)
{
  feature_detector_ = new FeatureDetector(width_, height_, 20, 20);
  camera_ = new PinholeCamera(
      329.115520046, 329.115520046, 320.0, 240.0,
      0.0, 0.0, 0.0, 0.0, 0.0,
      width_, height_);

  ofs_.open(results_file_);
}

void TestImageAlignment::loadGtPose(const int idx, Vector3d& t, Eigen::Quaterniond& q)
{
  std::ifstream ifs(dataset_root_+"info/groundtruth.txt");
  if(!ifs.good())
    ROS_FATAL_STREAM("Couldn't load groundtruth data.");
  std::string line;
  int index=idx;
  while(!ifs.eof())
  {
    --index;
    std::getline(ifs, line);
    if(index == 0)
    {
      string val;
      stringstream ss(line);
      ss >> val;                    /* image index */
      ss >> val; t(0)  = stod(val); /* tx */
      ss >> val; t(1)  = stod(val); /* ty */
      ss >> val; t(2)  = stod(val); /* tz */
      ss >> val; q.x() = stod(val); /* qx */
      ss >> val; q.y() = stod(val); /* qy */
      ss >> val; q.z() = stod(val); /* qz */
      ss >> val; q.w() = stod(val); /* qw */
      q.normalize();
      break;
    }
  }
}

TestImageAlignment::~TestImageAlignment()
{
  delete feature_detector_;
  delete camera_;
  delete img_align_;
  ofs_.close();
  ROS_INFO_STREAM("Node killed.");
}

void TestImageAlignment::test()
{
  // we first initialize a reference frame
  initRefFrame();
  img_align_ = new ImageAlignment(frame_ref_, utils::getParam<bool>("img_align/verbose"));

  // align successive frames
  int n_succesfull_alignments=0;
  const int ref_img_idx = std::stoi(ref_img_idx_);
  for(int i=1; i<n_curr_frames_+1; ++i)
  {
    int cur_idx = ref_img_idx + i;
    string cur_img;
    if(cur_idx/10 == 0)
      cur_img = "000"+std::to_string(cur_idx);
    else if(cur_idx/100 == 0)
      cur_img = "00"+std::to_string(cur_idx);
    else if(cur_idx/1000 == 0)
      cur_img = "0"+std::to_string(cur_idx);
    else
      cur_img = std::to_string(cur_idx);

    ROS_DEBUG_STREAM("Aligning ref frame wrt cur frame: " << cur_img);
    frame_cur_.reset();
    if(trackCurFrame(cur_img))
      ++n_succesfull_alignments;
  }
  ROS_INFO_STREAM(
      n_succesfull_alignments << " frames converged out of " << n_curr_frames_ <<
      "\t Convergence percentage = " <<
      (n_succesfull_alignments/static_cast<double>(n_curr_frames_))*100 << "%");
  exit_ = true;
}

bool TestImageAlignment::trackCurFrame(const string cur_img_idx)
{
  const std::string img_path(dataset_root_+"img/img"+cur_img_idx+"_0.png");
  ROS_DEBUG_STREAM("Current frame image: " << img_path);
  cv::Mat img;
  utils::loadImg(img_path, img);

  // ground truth pose of this current frame
  Vector3d t; Eigen::Quaterniond q;
  const int idx = std::stoi(cur_img_idx);
  loadGtPose(idx, t, q);

  frame_cur_.reset(new Frame(img, camera_));

  // different ways to initialize current camera pose
  // frame_cur_->T_w_f_ = frame_ref_->T_w_f_;  /* reference frame */
  // frame_cur_->T_w_f_ = SE3(q, t);           /* ground truth    */
  frame_cur_->T_w_f_ = T_w_last_;              /* last pose       */

  // align frames
  img_align_->align(frame_cur_);

  // save the pose
  T_w_last_ = frame_cur_->T_w_f_;

  // compare against groundtruth
  SE3 T_c_gt = SE3(q, t) * frame_cur_->T_w_f_.inverse();
  const Vector3d translation_error = T_c_gt.translation();
  ROS_DEBUG_STREAM(
      "ref idx = " << ref_img_idx_ << "\t" <<
      "cur idx = " << cur_img_idx << "\t" <<
      "translational error = " << translation_error.norm());
  ofs_ << ref_img_idx_ << "," << cur_img_idx << ","
       << fabs(translation_error.x()) << ","
       << fabs(translation_error.y()) << ","
       << fabs(translation_error.z()) << endl;

  if(translation_error.norm() > 0.5)
  {
    ROS_ERROR_STREAM("Image alignment diverged: error = " <<
        std::setprecision(5) << translation_error.norm());
    return false;
  }
  ROS_INFO_STREAM("Image alignment converged.");
  return true;
}

void TestImageAlignment::initRefFrame()
{
  const std::string img_path(dataset_root_+"img/img"+ref_img_idx_+"_0.png");
  ROS_DEBUG_STREAM("Reference frame image: " << img_path);
  cv::Mat img;
  utils::loadImg(img_path, img);

  // init features
  frame_ref_.reset(new Frame(img, camera_));
  feature_detector_->detect(frame_ref_, frame_ref_->img_pyr_, frame_ref_->fts_);
  ROS_INFO_STREAM("Detected " << frame_ref_->fts_.size() << " features.");

  // set groundtruth pose
  Vector3d t; Eigen::Quaterniond q;
  const int idx = std::stoi(ref_img_idx_);
  ROS_DEBUG_STREAM("Reference frame ground truth idx: " << idx);
  loadGtPose(idx, t, q);
  frame_ref_->T_w_f_ = SE3(q, t);
  T_w_last_ = frame_ref_->T_w_f_;

  // read depth values
  const std::string depth_file(dataset_root_+"depth/img"+ref_img_idx_+"_0.depth");
  vector<double> depth_values; depth_values.reserve(width_*height_);
  utils::loadDepthImg(depth_file, depth_values);

  // initialize 3D points corresponding to features
  for(const auto& ftr: frame_ref_->fts_)
  {
    const int x = std::floor(ftr->px_[0]);
    const int y = std::floor(ftr->px_[1]);
    const int idx = y*width_ + x;
    Vector3d w_pt = frame_ref_->T_w_f_ * (ftr->b_*depth_values[idx]); /* point in world frame */
    PointPtr new_point(new Point(w_pt, frame_ref_));
    ftr->point_ = new_point; 
  }

  // display features with depth map as palette.
  ROS_INFO_STREAM("inverse depth max = " <<
      1.0/ *std::min_element(depth_values.begin(), depth_values.end()));
  ROS_INFO_STREAM("inverse depth min = " <<
      1.0/ *std::max_element(depth_values.begin(), depth_values.end()));

  cv::Mat tmp = frame_ref_->img_pyr_[0].clone();
  cv::cvtColor(tmp, tmp, cv::COLOR_GRAY2BGR);
  utils::drawPatches(tmp, frame_ref_->fts_, PATCH_SIZE);
  utils::displayImg("ref", tmp);
  cv::destroyAllWindows();
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sparse_image_alignment_test_node");
  ros::NodeHandle nh;

  ::TestImageAlignment img_align(640, 480);
  img_align.test();

  while(ros::ok() and !img_align.exit())
    ros::spinOnce();
}

