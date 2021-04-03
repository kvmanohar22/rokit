#include <ceres/cost_function.h>
#include <ceres/types.h>
#include <vikit/image_alignment.h>
#include <vikit/feature.h>
#include <rk_common/utils.h>
#include <vikit/interpolate.h>

namespace rokit {

// static init
SE3* ImageAlignmentCostFunctor::T_c_r_;
cv::Mat ImageAlignmentCostFunctor::res_img_;
int ImageAlignmentCostFunctor::invalid_;

ImageAlignment::ImageAlignment(FramePtr frame_ref, bool verbose)
  : frame_ref_(frame_ref),
    verbose_(verbose),
    cb_set_(false),
    min_level_(0),
    max_level_(5)
{
  // setup ceres options
  ceres_opts_.linear_solver_type = ceres::DENSE_QR;
  ceres_opts_.check_gradients = false;
  if(verbose_) {
    ceres_opts_.minimizer_progress_to_stdout = true;
    ceres_opts_.update_state_every_iteration = true; /* to visualize correspondances after each iteration */
  }
}

ImageAlignment::~ImageAlignment()
{
  if(iter_cb_ != nullptr)
    delete iter_cb_;
}

void ImageAlignment::align(FramePtr frame_cur)
{
  frame_cur_ = frame_cur;
  double dT[] = {0.0, 0.0, 0.0, /* perturbation of attitude */
                 0.0, 0.0, 0.0  /* perturbation of position */ };

  if(verbose_ and !cb_set_)
  {
    iter_cb_ = new IterationCallback(frame_ref_, frame_cur_, dT, true, true);
    ceres_opts_.callbacks.push_back(iter_cb_);
    cb_set_ = true;
  }

  // pyramidal implementation
  for(int level = max_level_; level >= min_level_; --level)
  {
    ROS_DEBUG_STREAM("pyramid level = " << level);
    ref_patches_.clear();
    std::fill(dT, dT+6, 0.0);

    // align
    align(level, dT);
  }
}

void ImageAlignment::align(const int level, double* dT)
{
  SE3 T_c_r = frame_cur_->T_w_f_.inverse() * frame_ref_->T_w_f_;
  precomputeRefPatchAndJacobian(level);

  // initialize static variables
  ImageAlignmentCostFunctor::T_c_r_ = &T_c_r;
  ImageAlignmentCostFunctor::res_img_ = cv::Mat::zeros(
      frame_ref_->img_pyr_.at(level).rows, frame_ref_->img_pyr_.at(level).cols, CV_32FC1);
  ImageAlignmentCostFunctor::invalid_ = 0;

  // create residuals for each patch
  ceres::Problem problem;
  const double scale = 1.0/(1<<level);
  const cv::Mat img = frame_cur_->img_pyr_.at(level);
  ROS_DEBUG_STREAM("scale = " << scale);
  for(const auto& ftr: frame_ref_->fts_)
  {
    if(ref_patches_.find(ftr->point_->idx_) == ref_patches_.end())
      continue;

    const double depth = (ftr->point_->pt_-frame_ref_->pos()).norm();
    const Vector3d r_pt(ftr->b_*depth);
    ceres::CostFunction* cost_func = new ImageAlignmentCostFunctor(
      ref_patches_[ftr->point_->idx_],
      r_pt, frame_ref_->cam_, &img, scale, verbose_);

    // add the residual block
    problem.AddResidualBlock(cost_func, nullptr, dT);
  }

  // solve
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_opts_, &problem, &summary);

  // report the results
  if(verbose_) ROS_INFO_STREAM("Report:\n" << summary.FullReport());

  // update the parameters
  SE3 dT_opt = SE3::exp((Vector6d() << dT[3], dT[4], dT[5], dT[0], dT[1], dT[2]).finished());
  T_c_r = T_c_r * dT_opt;
  frame_cur_->T_w_f_ = frame_ref_->T_w_f_ * T_c_r.inverse();
}

void ImageAlignment::precomputeRefPatchAndJacobian(const int level)
{
  ROS_DEBUG_STREAM("Pre-computing reference patches and their jacobians.");
  const SE3 T_r_w = frame_ref_->T_w_f_.inverse();
  int invalid=0;
  const double scale = 1.0/(1<<level);
  const cv::Mat img = frame_ref_->img_pyr_.at(level);
  for(const auto& ftr: frame_ref_->fts_)
  {
    if(!ftr->point_)
    {
      ++invalid;
      continue;
    }

    // compute patch
    const Vector2d px(ftr->px_ * scale);
    ImgRefType patch;
    if(!ImageAlignment::getPatch(img, px, patch))
    {
      ++invalid;
      continue;
    }

    // compute jacobian
    ImgJacxyType img_jac = imgJac_8uc1(img, px(0), px(1), HALF_PATCH_SIZE);

    Matrix23d proj_jac;
    // NOTE: Cannot simply use the 3D point. There will be reprojection errors
    const double depth = (ftr->point_->pt_-frame_ref_->pos()).norm();
    const Vector3d r_pt(ftr->b_*depth); /* point expressed in reference frame */
    const double fx = frame_ref_->cam_->fx() * scale;
    const double fy = frame_ref_->cam_->fy() * scale;
    const double pt_z2 = r_pt(2) * r_pt(2);
    proj_jac << fx/r_pt(2), 0.0, -fx*r_pt(0)/pt_z2,
                0.0, fy/r_pt(2), -fy*r_pt(1)/pt_z2;
    Matrix3d p_skew = utils::sqew(r_pt);
    ImgJacType jac; jac.setZero();

    jac.block<PATCH_AREA, 3>(0, 0).noalias() =  img_jac * proj_jac * p_skew; /* wrt rotation    */
    jac.block<PATCH_AREA, 3>(0, 3).noalias() = -img_jac * proj_jac;          /* wrt translation */

    // insert
    RefPatch ref_patch(patch, jac);
    ref_patches_[ftr->point_->idx_] = ref_patch; 
  }
  ROS_DEBUG_STREAM("Pre-computed " << ref_patches_.size() << " patches. Invalid = " << invalid);
}

bool ImageAlignment::getPatch(const cv::Mat& img, const Vector2d& px, ImgRefType& patch)
{
  const float u = px(0);
  const float v = px(1);
  const float xmin = u-HALF_PATCH_SIZE;
  const float ymin = v-HALF_PATCH_SIZE;
  const float xmax = u+HALF_PATCH_SIZE;
  const float ymax = v+HALF_PATCH_SIZE;
  if(xmin < 1.0 or ymin < 1.0 or xmax > img.cols-2 or ymax > img.rows-2)
    return false;

  float yy = ymin, xx = xmin;
  int idx=0;
  for(int y=0; y<PATCH_SIZE; ++y)
  {
    xx = xmin;
    for(int x=0; x<PATCH_SIZE; ++x, ++idx)
    {
      patch(idx, 0) = utils::interpolate_8uc1<float>(img, xx, yy);
      xx += 1.0;
    }
    yy += 1.0;
  }
  return true;
}

} // namespace rokit
