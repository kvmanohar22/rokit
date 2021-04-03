#include <fast/fast.h>
#include <memory>
#include <vikit/feature_detector.h>
#include <vikit/utils.h>

namespace rokit {

FeatureDetector::FeatureDetector(
    const size_t width,
    const size_t height,
    const size_t cell_size_x,
    const size_t cell_size_y)
  : width_(width),
    height_(height),
    cell_size_x_(cell_size_x),
    cell_size_y_(cell_size_y)
{
  ROS_ASSERT_MSG(
      cell_size_x > 0 and cell_size_x < width_ and
      cell_size_y > 0 and cell_size_y < height_,
      "Invalid cell dimensions.");

  n_cells_x_ = std::ceil(static_cast<double>(width)/cell_size_x);
  n_cells_y_ = std::ceil(static_cast<double>(height)/cell_size_y);
  ROS_DEBUG_STREAM("number of cells = " << n_cells_x_*n_cells_y_);

  grid_.resize(n_cells_x_*n_cells_y_);
  resetGrid();
}

bool FeatureDetector::cellOccupied(const double u, const double v)
{
  return grid_.at(
      static_cast<int>(v/cell_size_y_)*n_cells_x_+
      static_cast<int>(u/cell_size_x_));
}

bool FeatureDetector::cellOccupied(const Vector2d& px)
{
  return cellOccupied(px[0], px[1]);
}

void FeatureDetector::setOccupancy(const double u, const double v)
{
  grid_.at(
      static_cast<int>(v/cell_size_y_)*n_cells_x_+
      static_cast<int>(u/cell_size_x_)) = true;
}

void FeatureDetector::setOccupancy(const Vector2d& px)
{
  setOccupancy(px[0], px[1]);
}

void FeatureDetector::resetGrid()
{
  std::fill(grid_.begin(), grid_.end(), false);
}

void FeatureDetector::detect(FramePtr& frame, const ImgPyr& pyr, Features& fts)
{
  Corners corners; corners.resize(n_cells_x_*n_cells_y_, Corner(0, 0, 15.0, 0));
  for(int lvl=0; lvl<pyr.size(); ++lvl)
  {
    const double scale = 1<<lvl;
    const cv::Mat img = pyr[lvl];
    std::vector<fast::fast_xy> new_corners;
    fast::fast_corner_detect_10(
        (fast::fast_byte*)img.data, img.cols, img.rows, img.cols, 20, new_corners);

    vector<int> scores;
    fast::fast_corner_score_10(
        (fast::fast_byte*)img.data, img.cols, new_corners, 20, scores);
    vector<int> reduced_corners_idx;
    fast::fast_nonmax_3x3(new_corners, scores, reduced_corners_idx);

    for(vector<int>::iterator itr=reduced_corners_idx.begin(); itr!=reduced_corners_idx.end(); ++itr)
    {
      float u = new_corners[*itr].x;
      float v = new_corners[*itr].y;
      float score = utils::shiTomasiScore(img, u, v);
      u *= scale;
      v *= scale;
      if(!cellOccupied(u, v))
      {
        int idx = static_cast<int>(v/cell_size_y_)*n_cells_x_ + static_cast<int>(u/cell_size_x_);
        if(score > corners[idx].score)
          corners[idx] = Corner(u, v, score, lvl);
      }
    }
  }

  for(Corners::iterator itr=corners.begin(); itr!=corners.end(); ++itr)
  {
    if(itr->score > 15.0)
      fts.push_back(std::make_shared<Feature>(frame, Vector2d(itr->x, itr->y), itr->lvl));
  }
  resetGrid();
  ROS_DEBUG_STREAM("Detected " << fts.size() << " features");
}

} // namespace rokit
