#include <joint_mapping/LocalAgent.h>
#include <iostream>

//#include <mutex>

using namespace std;
using namespace gtsam;

namespace comap {

/*******
// Initialize the robot with default parameters
******/
void LocalAgent::init(AgentSetting& setting) {

  // TODO initialize the names of the foreign agents

  setting_ = setting;
  matcher_ = boost::shared_ptr<Matcher3D>(new Matcher3D(setting_.matcher_setting, name_,
                                                        setting_.icp_config_consec,
                                                        setting_.loop_detect_setting,
                                                        setting_.loop_match_setting));

  // Using the typedef to create default 2D or 3D pose
  curr_pose_ = PoseD();

  isam_ = ISAM2();

  pose_count_ = 0;
  trained_scan_count_ = 0;
  last_trained_count_ = 0;
  // TODO make this agnostic to 2d/3d
  last_trained_point_ = Point3();

  local_smallEM_ = SmallEM(ID_, ID_, setting_.smallEM_setting);
}

void LocalAgent::addForeignRobot(std::string fname, unsigned char fID) {
  ROS_INFO("Adding foreign agent %s", fname.c_str());
  ForeignAgent::Ptr foreigner = ForeignAgent::Ptr(new ForeignAgent(fname, fID, num_robots_));
  foreigner->initialize(setting_, ID_, matcher_, network_ );
  foreign_agent_names_.push_back(fname);
  foreign_agents_[fname] = foreigner;
  foreigner->setLocalAgent(boost::shared_ptr<LocalAgent>(this));
}

bool LocalAgent::update(const PoseD& slam_pose, Scan& scan) {

  // factor graph / init values used to update isam
  NonlinearFactorGraph update_graph;
  Values init_values;

  PoseD delta_odom;
  // Get the current height
  double height = slam_pose.z();
  // On the first iteration, add prior
  if (pose_count_ == 0) {
    // add prior factor to the first pose
    noiseModel::Isotropic::shared_ptr prior_model = noiseModel::Isotropic::Sigma(6, 0.01);
    update_graph.push_back(PriorFactor<PoseD>(Symbol(ID_, 0), curr_pose_, prior_model));
    // init value
    curr_pose_ = PoseD(Rot3(), Point3(0,0,height));

    init_values.insert(Symbol(ID_, 0), curr_pose_);
    curr_values_.insert(Symbol(ID_, 0), curr_pose_);

  } else {

    delta_odom = last_odompose_.between(slam_pose);
    curr_pose_ = curr_pose_.compose(delta_odom);

    // TODO Hack to get height correct. problem due to startup
    if (pose_count_ < setting_.min_pose_count) {
      double dz = height - curr_pose_.z();
      PoseD height_fix = PoseD(Rot3(), Point3(0,0,dz));
      delta_odom = delta_odom.compose(height_fix);
      curr_pose_ = curr_pose_.compose(height_fix);
    }

    update_graph.push_back(BetweenFactor<PoseD>(Symbol(ID_, pose_count_-1),
        Symbol(ID_, pose_count_), delta_odom, setting_.icp_default_model));
    // init value
    init_values.insert(Symbol(ID_, pose_count_), curr_pose_);
    // curent value also insert (relative pose inference may need it)
    curr_values_.insert(Symbol(ID_, pose_count_), curr_pose_);

    // local loop closure
    // TODO update this
    NonlinearFactorGraph loop_graph = matcher_->findLocalLoopClosure(curr_pose_, scan);
    update_graph.push_back(loop_graph);
  }

  network_->preparePackage(name_, pose_count_, curr_pose_, delta_odom);

  //double scan_saliency = scan.getSaliency();

  // Check if we gone through a min number of iterations and if we have moved enough away from
  // Previous position.

  if ( pose_count_ > setting_.min_pose_count &&
      (pose_count_ - last_trained_count_) >= setting_.train_min_iter &&
      (curr_pose_.translation() - last_trained_point_).norm() > setting_.train_min_distance) {

    ROS_INFO(" %s is training kd-tree at pose count %zu ", name_.c_str(), pose_count_);

    // Add scan into some matcher thing.
    // Matcher maintains a local cache of the scans
    // TODO Make sure add training scan filters
    Scan downsampled_scan = matcher_->addLocalScan(*scan, curr_pose_, pose_count_);

    trained_scan_count_++;

    last_trained_count_ = pose_count_;
    last_trained_point_ = curr_pose_.translation();

    // Queus the output packet in the network and it sends it out when ready
    network_->addScan(name_, pose_count_, curr_pose_, downsampled_scan);

    /*
    sensor_msgs::PointCloud2 projected_scan = matcher_->project(*downsampled_scan,
                                            curr_pose_,
                                            PoseD());
                                            */
    sensor_msgs::PointCloud2 cropped_scan = matcher_->crop_z(*downsampled_scan, curr_pose_);
    network_->sendScantoMap(curr_pose_, cropped_scan);
  }

  /* Chech the foreign agents for multi robot loop closure */
  // TODO Put each foreign agent on its own thread and have it determine the closure and return it in graph and value format

  for (size_t i = 0; i < foreign_agent_names_.size(); i++) {
    std::string robot_name = foreign_agent_names_[i];

    ForeignAgent::Ptr foreign_agent = foreign_agents_[robot_name];

    // Returns true when their is a matched scan
    if (foreign_agent->process_received_packages(update_graph, init_values) &&
        !foreign_agent->is_rel_pose_built()) {

      ROS_INFO("Running Relative Pose Inference");

      HypothesisSelectResult result = this->relativePoseInference(robot_name);

      // one selected
      if (result.hypothesis) {
        // update status
        foreign_agent->set_rel_pose(result.hypothesis->relative_pose);
        foreign_agent->sendScanstoMap();

        // update full graph before clean status, only insert inliers indicated by hypothesis
        GraphValues foreign_graphvalues = foreign_agent->generateAgentGraph(*(result.hypothesis));
        update_graph.push_back(foreign_graphvalues.first);
        init_values.insert(foreign_graphvalues.second);

        // display
        cout << "*** [NOTE] ***: Relative Pose is Built between '" << name_ << "' and '"
            << foreign_agent->name_  << "'" << endl;
        result.hypothesis->print();

        // TODO Shouldn't this come after removing outliers
        // init small EM
        foreign_agent->init_smallEM();

        // update agent status, only keep inliers
        // so the status could be used for futher loop closure detection
        foreign_agent->remove_outliers(*(result.hypothesis));

      } else {
        cout << " Relative Pose is not found" << endl;
      }
    }
  }

  Logger::instance()->startTimer("Optimization");
  ISAM2Result result;
  try {
    result = isam_.update(update_graph, init_values);
  } catch (IndeterminantLinearSystemException& exception) {
    Symbol key(exception.nearbyVariable());
    ROS_WARN("[ERROR:RobotAgent] %s:  Indeterminant Linear System Exception got, near : ", name_.c_str());
    key.print();
  }

  curr_values_ = isam_.calculateEstimate();

  Logger::instance()->endTimer();

  // update curr_pose_ depends on optimized value
  //curr_pose_ = curr_values_.at<PoseD>(Symbol(ID_, pose_count_));

  //std::cout << name_ << " has pose " << std::endl;
  //curr_pose_.translation().print();

  last_odompose_ = slam_pose;
  pose_count_++;

  return true;
}

void LocalAgent::updateGraph( NonlinearFactorGraph& update_graph, Values& init_values, size_t pose_count) {

  // Deprecated
}

HypothesisSelectResult LocalAgent::relativePoseInference(std::string robot_name) {

  ForeignAgent::Ptr f_agent = foreign_agents_[robot_name];

  cout << endl << "Inference pose between '" << name_ << "' and '" << f_agent->name_ << "' ..." << endl;

  cout << "Clustering ..." << endl;
  // get raw relative pose vector
  vector<PoseD> raw_relpose = originRelativePose(ID_, f_agent->ID_,
      f_agent->get_inlier_poses(), f_agent->get_inlier_indices(), curr_values_,
      f_agent->curr_values());

  // =========================================================================
  // clustering
  vector<PoseD> cluster_relpose = clusterRelativePoses(raw_relpose, setting_.cluster_setting);
  cout << cluster_relpose.size() << " poses clusters remains" << endl;

  if (cluster_relpose.size() == 0)
    return HypothesisSelectResult();

  // =========================================================================
  // relatove pose EM inference, get hypothesis
  cout << "Performing RelativePoseEM ..." << endl;

  vector<Hypothesis> hypothesis = performRelativePoseEM(ID_, f_agent->ID_,
      f_agent->get_inlier_poses(), f_agent->get_inlier_indices(),
      cluster_relpose, curr_values_, f_agent->curr_values(),
      setting_.relativeEM_setting);

  if (hypothesis.size() == 0)
    return HypothesisSelectResult();

  cout << " after hypothesis function" << endl;

  // =========================================================================
  // merge hypothesis
  vector<Hypothesis> merged_hypothesis = mergeHypothesis(hypothesis, setting_.hypo_merge_setting);
  cout << merged_hypothesis.size() << " Hypothesis left" << endl;

  for (size_t i = 0; i < merged_hypothesis.size(); i++ ) {
    hypothesis.at(i).print();
  }

  cout << "Performing Hypothesis Selection ..." << endl;

  // make selection
  HypothesisSelectResult select_result = selectHypothesis(ID_, f_agent->ID_,
      f_agent->get_inlier_poses(), f_agent->get_inlier_indices(),
      curr_values_, f_agent->curr_values(),
      merged_hypothesis, setting_.hypo_select_setting);

  return select_result;
}
}
