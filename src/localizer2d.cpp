#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"
#include "ros_bridge.h"
#include <typeinfo>

Localizer2D::Localizer2D() 
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  // TODO

  // Create KD-Tree
  //check if the map is valid
  if (!_map){
    std::cerr << "The map is not initialized" << std::endl;
    return;
  } 

  Map world_map = (*_map);

  auto map_rows = std::ceil( world_map.rows() * world_map.resolution() );
  auto map_cols = std::ceil( world_map.cols() * world_map.resolution() );
  
  //obtains the dimensions of the map
  cv::Size2i matrix_size = _map->size();
  auto mat_rows = matrix_size.height;
  auto mat_cols = matrix_size.width;

  if( setMap_debug ){ std::cerr << "-- setMap -> rows: " << mat_rows << ", map_rows: " << map_rows << std::endl;
                      std::cerr << "-- setMap -> cols: " << mat_cols << ", map_cols: " << map_cols << std::endl;
                      std::cerr << "-- frame_grid size: " << _map->map().size() << std::endl;
                       }
  // Scan all the cells in the map
  for( int r=0; r < mat_rows; r++){
    for( int c=0; c < mat_cols; c++){

      auto point_in_grid = (*_map)(r,c);   

      if(point_in_grid == CellType::Occupied){
        
        float x_map = r;
        float y_map = c;

        auto my_point = cv::Point2i(r,c);
        auto converted_point = _map->grid2world(my_point);
        //Eigen::Vector2f converted_point = Eigen::Vector2f(x_map, y_map);
        //std::cerr << "converted point ---> " << converted_point[0] << ", " << converted_point[1] << std::endl;
        _obst_vect.push_back(converted_point);
        
        //std::cerr << converted_point << std::endl;
      }
    }

  }

  // Create KD-Tree with the obstacles found
  TreeType my_kd_tree(_obst_vect.begin(), _obst_vect.end(), 10);
 // _obst_tree_ptr.reset( &my_kd_tree);
  _obst_tree_ptr = std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end(), 10);

  if(setMap_debug){ std::cerr << "-- setMap and kd-tree-> size: " << _obst_vect.size() << std::endl; }

}


/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO
  Eigen::Isometry2f new_init_pose;
  _laser_in_world = initial_pose_;
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Get a prediction of what the laser scan should 'see' in the map
  // TODO

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO

  ContainerType prediction;

  getPrediction(prediction);
  std::cerr << "Pred data length: " << prediction.size() << std::endl;  
  
  ICP icp = ICP(prediction, scan_, _range_max);
  //Use the actual estimate like a initialization for ICp
  icp.X() = X();
  
  icp.run(30);
  Eigen::Isometry2f new_iso;
  new_iso.translation() = icp.X().translation();
  new_iso.linear() = icp.X().linear();

  setInitialPose(new_iso);
  std::cerr << "New: translation: [ " << X().translation()[0] << ", " << X().translation()[1] << " ]" << std::endl;
  //std::cerr << X().translation() << std::endl;
  //std::cerr << X().linear() << std::endl;
  std::cerr << "--------------------- " << std::endl;
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  // TODO

  TreeType::AnswerType neighbors;
  _obst_tree_ptr->fullSearch(neighbors, X().translation(), _range_max);
  //std::cerr << "-- neighbors: " << neighbors.size() << std::endl;

  for( auto n: neighbors){
    prediction_.push_back(*n);
  }
  return;

}