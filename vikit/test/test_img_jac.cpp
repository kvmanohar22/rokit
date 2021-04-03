#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vikit/derivatives.h>
#include <vikit/utils.h>
#include <time.h>

using namespace rokit;

void testJacRegular(const std::string& img_path)
{
  cv::Mat img;
  utils::loadImg(img_path, img);
  utils::displayImg("img jac", img, 1e3);

  cv::Mat tmp;
  cv::cvtColor(img, tmp, cv::COLOR_GRAY2BGR);
  const int rows = img.rows;
  const int cols = img.cols;

  cout << "img dimensions: " << rows << " x " << cols << endl;
  for(int i=0; i<rows; i+=20)
  {
    for(int j=0; j<cols; j+=20)
    {
      Vector2d jac = imgJac_8uc1(img, j, i);
      utils::drawFlow(tmp, Vector2d(j, i), jac);
    }
  }
  utils::displayImg("img jac", tmp);
  cv::destroyWindow("img jac");
}

void testJacRandom(const std::string& img_path)
{
  cv::Mat img;
  utils::loadImg(img_path, img);
  utils::displayImg("img jac", img, 1e3);

  cv::Mat tmp;
  cv::cvtColor(img, tmp, cv::COLOR_GRAY2BGR);
  const int rows = img.rows;
  const int cols = img.cols;

  cout << "img dimensions: " << rows << " x " << cols << endl;
  for(int i=0; i<1e2; ++i)
  {
    const float r = rand() % rows;
    const float c = rand() % cols;
    Vector2d jac = imgJac_8uc1(img, c, r);
    utils::drawFlow(tmp, Vector2d(c, r), jac);
  }
  utils::displayImg("img jac", tmp);
  cv::destroyWindow("img jac");
}

void testJac()
{
  const std::string root_dir = std::getenv("VIKIT_DIR");
  testJacRandom(root_dir+"/imgs/checkerboard.png");
  testJacRandom(root_dir+"/imgs/random.jpeg");
  testJacRandom(root_dir+"/imgs/synthetic.png");
  testJacRandom(root_dir+"/imgs/circle.png");

  testJacRegular(root_dir+"/imgs/checkerboard.png");
  testJacRegular(root_dir+"/imgs/random.jpeg");
  testJacRegular(root_dir+"/imgs/synthetic.png");
  testJacRegular(root_dir+"/imgs/circle.png");
}

int main()
{
  srand(time(0));
  testJac();
}

