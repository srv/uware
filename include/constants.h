#ifndef CONSTANTS_H
#define CONSTANTS_H

using namespace std;

namespace uware
{
  static const string PREPROCESS_DIR = "preprocess";

  static const string RECONSTRUCTION_DIR = "reconstruction";

  static const string PC_DIR = "pointclouds";

  static const string IMG_DIR = "images";

  static const string PAIRED_CLOUDS_DIR = "paired_pointclouds";

  static const string CAMERA_MATRIX_FILE = "camera_matrix.yaml";

  static const string ODOM_FILE = "odom.txt";

  static const string OMAP_FILE = "map.txt";

  static const string NAVSTS_FILE = "navsts.csv";

  static const string LATLONCORNERS_FILE = "latloncorners.csv" ; 

  static const string F2F_POSES = "frame2frame_poses.txt";
  static const string F2F_EDGES = "frame2frame_edges.txt";

  static const string OPTIMIZED_POSES = "optimized_poses.txt";
  static const string OPTIMIZED_EDGES = "optimized_edges.txt";


} // namespace

#endif // CONSTANTS_H
