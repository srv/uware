#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>

#include "utils.h"
#include "constants.h"

using namespace std;

typedef pcl::PointXYZRGB          Point;
typedef pcl::PointCloud<Point>    PointCloud;
typedef pcl::IterativeClosestPoint<Point, Point> IterativeClosestPoint;

namespace uware
{

class Registration
{

public:

  struct Params
  {
    string outdir;                  //!> Output directory
    string indir;                   //!> Input directory (the output of pre-process)
    double max_icp_fitness_score;   //!> Maximum icp fitness score to consider a valid icp registration
    double desc_matching_th;        //!> Descriptor matching threshold
    int min_desc_matches;           //!> Minimum number of descriptor matches to proceed with the Sim3
    double reproj_err;              //!> Sim3 reprojection error
    int min_inliers;                //!> Minimum number of inliers to consider a Sim3 valid
    double max_reg_err;             //!> Maximum registration error between odometry and ICP or Sim3 (euclidean distance in meters)
    double z_diff_th;               //!> The z difference threshold to consider the existence of 3D structure (in percentage respect to the mean z)
    double min_salient_ids;         //!> Minimum percentage of salient indices to consider a pointcloud valid for registration

    // Debug
    bool show_generic_logs;         //!> Show generic logs
    bool show_salient_ids;          //!> Show the salient indices percentage
    bool show_icp_score;            //!> Show the icp fitness score
    bool show_icp_tf;               //!> Show the icp output transformation
    bool show_num_of_kp;            //!> Show the number of keypoints
    bool save_icp_clouds;           //!> Save paired clouds

    // Default settings
    Params () {
      outdir                  = "";
      indir                   = "";
      max_icp_fitness_score   = 0.1;
      desc_matching_th        = 0.7;
      min_desc_matches        = 50;
      reproj_err              = 4.0;
      min_inliers             = 20;
      max_reg_err             = 0.3;
      z_diff_th               = 15.0;
      min_salient_ids         = 15.0;

      show_generic_logs       = false;
      show_salient_ids        = false;
      show_icp_score          = false;
      show_icp_tf             = false;
      show_num_of_kp          = false;
      save_icp_clouds         = false;
    }
  };

  /** \brief Set class params
   * \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Class constructor
   */
  Registration();

  void pipeline(const uint& id_src,
                const uint& id_tgt,
                const tf::Transform& initial_guest,
                bool& valid_sim3,
                bool& valid_icp,
                int& sim3_inliers,
                double& icp_score,
                const cv::Mat& camera_matrix,
                const bool & is_loop,
                tf::Transform& out);

  /** \brief Performs a 3D registration
   * @return true if registration is valid
   * \param source pointcloud
   * \param target pointcloud
   * \param output fitness score (if registration is valid)
   * \param Output estimated movement (used as well as initial guest)
   */
  bool regClouds(PointCloud::Ptr src,
                 PointCloud::Ptr tgt,
                 double& score,
                 tf::Transform &output);

  /** \brief Performs a visual registration
   * @return true if registration is valid
   * \param source descriptors
   * \param target descriptors
   * \param source keypoints
   * \param target world points
   * \param camera matrix
   * \param target output the number of inliers
   * \param output estimated transform (used as well as initial guest)
   */
  bool regImages(cv::Mat src_desc,
                 cv::Mat tgt_desc,
                 vector<cv::KeyPoint> src_kp,
                 vector<cv::Point3d> tgt_wp,
                 cv::Mat camera_matrix,
                 int& inliers_num,
                 tf::Transform& est_movement);


protected:


  /** \brief Pointcloud to pointcloud ICP registration
   * @return true if icp is valid
   * \param input source cloud
   * \param input target cloud
   * \param output fitness score
   * \param output transformation to align the pointclouds
   */
  bool pairAlign(PointCloud::Ptr src,
                 PointCloud::Ptr tgt,
                 double& score,
                 tf::Transform &output);

  /** \brief Performs a visual registration
   * @return true if registration is valid
   * \param id
   * \param previous left descriptor matrix
   * \param current left descriptor matrix
   * \param previous left keypoints
   * \param current world points
   */
  void readImgData(const string& id_src,
                   const string& id_tgt,
                   cv::Mat& src_desc,
                   cv::Mat& tgt_desc,
                   vector<cv::KeyPoint>& src_kp,
                   vector<cv::Point3d>& tgt_wp);

private:

  Params params_; //!> Stores parameters.

};

} // namespace

#endif // REGISTRATION_H