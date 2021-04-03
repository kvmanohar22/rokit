#ifndef _VIKIT_IMAGE_ALIGNMENT_H_
#define _VIKIT_IMAGE_ALIGNMENT_H_

#include <algorithm>
#include <ceres/iteration_callback.h>
#include <ceres/sized_cost_function.h>
#include <ceres/types.h>
#include <opencv2/imgproc.hpp>
#include <vikit/frame.h>
#include <vikit/point.h>
#include <vikit/utils.h>
#include <vikit/derivatives.h>
#include <sophus/se3.hpp>
#include <ceres/cost_function.h>
#include <rk_common/pinhole_camera.h>

namespace rokit {

/// container for reference patch
struct RefPatch
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImgRefType    patch_;  //!< ref patch (stored as row major values)
  ImgJacType    jac_;    //!< jacobian of patch 

  RefPatch()
  {
    patch_.setZero(); 
    jac_.setZero(); 
  }

  RefPatch(ImgRefType& patch, ImgJacType& jac)
    : patch_(patch),
      jac_(jac)
  {}
};


/// Aligns two images and estimates 6-DoF rigid body transformation
class ImageAlignment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImageAlignment(FramePtr frame_ref, bool verbose);
  virtual ~ImageAlignment();

  /// estimates the pose of current frame w.r.t ref frame
  void align(FramePtr frame_cur);

  /// align the images at this level
  void align(const int level, double* dT);

  /// pre-compute ref patch and jacobian since they will be constant
  void precomputeRefPatchAndJacobian(const int level);

  /// compute a patch around the given pixel (in row major)
  /// returns false if the patch is out of bounds of the image
  static bool getPatch(const cv::Mat& img, const Vector2d& px, ImgRefType& patch);

private:
  FramePtr                      frame_ref_;     //!< ref frame
  FramePtr                      frame_cur_;     //!< cur frame
  unordered_map<int, RefPatch>  ref_patches_;   //!< jacobian for each patch
  ceres::Solver::Options        ceres_opts_;    //!< options for ceres solver
  ceres::IterationCallback*     iter_cb_;       //!< callback after each iteration
  bool                          verbose_;
  bool                          cb_set_;
  int                           max_level_;     //!< max pyramid level
  int                           min_level_;     //!< min pyramid level
}; // class ImageAlignment


/// Image alignment cost functor using ceres
class ImageAlignmentCostFunctor : public ceres::SizedCostFunction<PATCH_AREA, 6>
{
public:
  ImageAlignmentCostFunctor(
      const RefPatch& ref_patch,
      const Vector3d& r_pt,
      const PinholeCamera* camera,
      const cv::Mat* img_c,
      const double scale,
      bool display_res=false) :
    ref_patch_(ref_patch),
    r_pt_(r_pt),
    camera_(camera),
    img_c_(img_c),
    scale_(scale),
    display_res_(display_res)
  {}
  virtual ~ImageAlignmentCostFunctor() {}

  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const
  {
    const Vector3d dR(parameters[0][0], parameters[0][1], parameters[0][2]);
    const Vector3d dp(parameters[0][3], parameters[0][4], parameters[0][5]);
    Vector6d dT; dT << dp, dR;  /* NOTE: sophus expects first translation then rotation */
    const SE3 T_c_r = *ImageAlignmentCostFunctor::T_c_r_ * SE3::exp(dT);

    // assign residual
    const Vector2d c_px = camera_->cam2img(T_c_r * r_pt_) * scale_;
    ImgRefType I_c;
    bool patch_in_bounds = ImageAlignment::getPatch(*img_c_, c_px, I_c);
    if(!patch_in_bounds)
    {
      for(int i=0; i<PATCH_AREA; ++i)
        residuals[i] = 0.0;
      ++ImageAlignmentCostFunctor::invalid_;
    } else {
      for(int i=0; i<PATCH_AREA; ++i)
      {
        residuals[i] = ref_patch_.patch_(i) - I_c(i);
        if(display_res_)
        {
          const int x = (int)c_px(0)-HALF_PATCH_SIZE+i%PATCH_SIZE;
          const int y = (int)c_px(1)-HALF_PATCH_SIZE+i/PATCH_SIZE;
          ImageAlignmentCostFunctor::res_img_.at<float>(y, x) = residuals[i]/255.0;
        }
      }
    }

    if(!jacobians) return true;
    double* jacobian = jacobians[0];
    if(!jacobian) return true;

    // assign jacobian. Jacobian is constant, nothing new to compute here. (row-major)
    if(!patch_in_bounds)
    {
      for(int i=0; i<PATCH_AREA; ++i)
        for(int j=0; j<6; ++j)
          jacobian[i*6+j] = 0.0;
    } else {
      for(int i=0; i<PATCH_AREA; ++i)
        for(int j=0; j<6; ++j)
          jacobian[i*6+j] = ref_patch_.jac_(i, j); /* jac of ith res wrt jth parameter */
    }
    return true;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static SE3*             T_c_r_;       //!< used for projecting points
  static cv::Mat          res_img_;     //!< residual image for visualization
  static int              invalid_;     //!< invalid projections count

private:
  const RefPatch          ref_patch_;   //!< holds reference frames patch and jac
  const Vector3d          r_pt_;        //!< 3D point expressed in reference frame
  const PinholeCamera*    camera_;      //!< camera
  const cv::Mat*          img_c_;       //!< image of current frame
  bool                    display_res_; //!< should we display res image
  double                  scale_;       //!< feature level (inverse)
}; // class ImageAlignmentCostFunctor


/// used for updating the parameters after each iteration
class IterationCallback : public ceres::IterationCallback
{
public:
  IterationCallback(
      FramePtr frame_ref,
      FramePtr frame_cur,
      double* dT,
      bool display_res=false,
      bool display_corrs=false)
    : frame_ref_(frame_ref),
      frame_cur_(frame_cur),
      dT_(dT),
      display_res_(display_res),
      display_corrs_(display_corrs)
  {}
  virtual ~IterationCallback() {}

  ceres::CallbackReturnType operator() (const ceres::IterationSummary& summary)
  {
    // display residual image if requested
    if(display_res_)
    {
      ROS_DEBUG_STREAM(
          "invalid reprojections = " << ImageAlignmentCostFunctor::invalid_ << "\t" <<
          "iteration no. = " << summary.iteration << "\t" <<
          "is succesfull? " << summary.step_is_successful << "\t" <<
          "step_norm = " << summary.step_norm
          );

      // reset the image
      utils::displayImg("residual image", ImageAlignmentCostFunctor::res_img_*10);
      const int h = ImageAlignmentCostFunctor::res_img_.rows;
      const int w = ImageAlignmentCostFunctor::res_img_.cols;
      ImageAlignmentCostFunctor::res_img_ = cv::Mat::zeros(h, w, CV_32FC1);
      ImageAlignmentCostFunctor::invalid_ = 0;
    }

    // display correspondances
    if(display_corrs_)
    {
      cv::Mat img_concat;
      utils::concatImgs(frame_ref_->img_pyr_[0], frame_cur_->img_pyr_[0], img_concat);
      cv::cvtColor(img_concat, img_concat, cv::COLOR_GRAY2BGR);

      // NOTE: Order is important here! First translation then rotation
      Vector6d dT; dT << dT_[3], dT_[4], dT_[5], dT_[0], dT_[1], dT_[2];
      SE3 T_c_r = (frame_cur_->T_w_f_.inverse()*frame_ref_->T_w_f_) * SE3::exp(dT);
      int count=0;
      vector<FeaturePtr> fts(frame_ref_->fts_.begin(), frame_ref_->fts_.end());
      std::random_shuffle(fts.begin(), fts.end());
      for(const auto& ftr: fts)
      {
        if(count > 20)
          break;
        const double depth = (ftr->point_->pt_-frame_ref_->pos()).norm();
        const Vector3d c_pt(T_c_r * (ftr->b_*depth));
        const Vector2d c_px(frame_cur_->cam_->cam2img(c_pt));

        // draw the line
        const Vector3i c = utils::randomColor();
        cv::line(img_concat,
            cv::Point2f(ftr->px_(0), ftr->px_(1)),
            cv::Point2f(c_px(0)+frame_ref_->img_pyr_[0].cols, c_px(1)),
            cv::Scalar(c(0), c(1), c(2)), 1, cv::LINE_AA);
        ++count;
      }
      utils::displayImg("correspondances", img_concat);
    }

    return ceres::SOLVER_CONTINUE;
  }

private:
  FramePtr  frame_ref_;
  FramePtr  frame_cur_;
  double*   dT_;                  //!< estimated params after each iteration
  bool      display_res_;         //!< display residual image?
  bool      display_corrs_;       //!< display correspondances?
}; // class IterationCallback

} // namespace rokit

#endif // _VIKIT_IMAGE_ALIGNMENT_H_

