#ifndef _VIKIT_FEATURE_DETECTOR_H_
#define _VIKIT_FEATURE_DETECTOR_H_

#include <vikit/feature.h>
#include <fast/fast.h>

namespace rokit {

/// Corner in an image
struct Corner
{
  float x, y;
  float score;
  int lvl;

  Corner(float x, float y, float score, int lvl)
    : x(x), y(y), score(score), lvl(lvl)
  {}
};
typedef std::vector<Corner> Corners;


/// Detect corners using fast feature detector
class FeatureDetector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureDetector(
      const size_t width,
      const size_t height,
      const size_t cell_size_x,
      const size_t cell_size_y);

  virtual ~FeatureDetector() {}

  /// reset the grid
  void resetGrid();

  /// check if a cell is filled
  bool cellOccupied(const double u, const double v);
  bool cellOccupied(const Vector2d& px);

  void setOccupancy(const double u, const double v);
  void setOccupancy(const Vector2d& px);

  /// detect features
  void detect(FramePtr& frame, const ImgPyr& img_pyr, Features& fts);

private:
  size_t              width_;
  size_t              height_;
  size_t              cell_size_x_;
  size_t              cell_size_y_;
  size_t              n_cells_x_;
  size_t              n_cells_y_;
  std::vector<bool>   grid_;
  
}; // class FeatureDetector

} // namespace rokit

#endif // _VIKIT_FEATURE_DETECTOR_H_

