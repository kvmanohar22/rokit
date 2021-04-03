#include <opencv2/imgproc.hpp>
#include <vikit/utils.h>
#include <rk_common/pinhole_camera.h>

using namespace rokit;

Vector3d point(const double d, const Vector3d& b, int type=0)
{
  Vector3d p;

  switch(type)
  {
    case 0: /* depth along optical ray */
      p = b * d;
      break;
    case 1: /* depth along optical axis (z) */
      p = b * (d / b.z());
      break;
  }
  return p;
}

int main()
{
  const std::string root_dir = std::getenv("RPG_SYNTHETIC_DATASET_DIR");
  string id="0759";
  string depth_file(root_dir+"/depth/img"+id+"_0.depth");
  vector<double> depth; depth.reserve(640*480);
  utils::loadDepthImg(depth_file, depth);

  string img_file(root_dir+"/img/img"+id+"_0.png");
  cv::Mat img;
  utils::loadImg(img_file, img);

  PinholeCamera* camera = new PinholeCamera(
      329.115520046, 329.115520046, 320.0, 240.0,
      0.0, 0.0, 0.0, 0.0, 0.0,
      640, 480);

  cv::Mat img_d;
  cv::cvtColor(img, img_d, cv::COLOR_GRAY2BGR);
  cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for(int i=0; i<480; i+=2)
  { 
    for(int j=0; j<640; j+=2)
    {
      double d = depth[i*640+j]; /* depth along optical ray */
      const Vector3d b = camera->img2cam(Vector2d(j, i)); /* unit bearing vector */
      const Vector3d p = point(d, b, 0);

      Vector3i c = utils::interpolateDepthColor(1.0/std::max(p.z(), 1e-7));
      cv::circle(img_d, cv::Point2f(j, i), 3, cv::Scalar(c(0), c(1), c(2)), -1);
    }
  }
  cv::Mat img_new;
  utils::concatImgs(img, img_d, img_new);
  utils::displayImg("depth", img_new);
  delete camera;
}

