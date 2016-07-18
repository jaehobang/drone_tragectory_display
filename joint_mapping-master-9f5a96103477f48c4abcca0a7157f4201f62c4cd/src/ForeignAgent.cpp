
#include <joint_mapping/Matcher3D.h>
#include <joint_mapping/LocalAgent.h>
#include <joint_mapping/ForeignAgent.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using namespace std;

namespace comap {

  void ForeignAgent::initialize(AgentSetting setting, unsigned char localID, boost::shared_ptr<Matcher> matcher, boost::shared_ptr<Network> network) {
    setting_ = setting;
    small_EM_ = SmallEM(localID, ID_, setting.smallEM_setting);
    matcher_ = matcher;
    network_ = network;
  }

  /* Returns true if there are packages with multi robot closure possibilities */
  bool ForeignAgent::process_received_packages(NonlinearFactorGraph& update_graph, Values& init_values) {

    if (received_packages_.size() == 0)
      return false;

    bool received_matchscan = false;

    std::cout << "There are " << received_packages_.size() << " packages " << std::endl;

    /* Loop through the received packages and check for loop closure */

    boost::unique_lock<boost::mutex> scoped_lock(package_mutex);
    for (std::vector<AgentPackage>::iterator it = received_packages_.begin(); it != received_packages_.end(); it++) {
      AgentPackage package = *it;

      size_t robot_pose_count = package.pose_count;
      PoseD robot_curr_pose = package.curr_pose;

      // ROS_INFO("Processing package from %s at pose count %zu", name_.c_str(), robot_pose_count);

      // typedef in agent_definitions
      DerivedScan scan;
      if (package.informative_scan) {
        //scan.initialize(package.scan);
        scan = boost::make_shared<sensor_msgs::PointCloud2>(package.scan);
      }

      // process received scan (match with local cached scans)
      if (rel_pose_built_) {

        cout << name_ << " is updating graph after rel pose built" << endl;
        // ====================================================================
        // if relative pose built: update current full graph
        // update consecutive frame
        GraphValues agentgraphvalue = this->updateAgentGraph( robot_pose_count, robot_curr_pose);

        update_graph.push_back(agentgraphvalue.first);
        init_values.insert(agentgraphvalue.second);
        // also insert in curr_value in case this cycle use
        local_agent_->insert_value(agentgraphvalue.second);
        // cur values no longer used after we make rel pose since local agent gets everything
        // curr_values_.insert(agentgraphvalue.second);

        // find agent loop closure if informative scan received to compare
        if (package.informative_scan) {
          foreign_scans_.push_back(package.scan);
          foreign_scans_pose_.push_back(robot_curr_pose);
          sendScanstoMap();

          cout << "finding between agent loop closure (after built) ..." << endl;
          NonlinearFactorGraph loopgraph = this->findAgentLoopClosure(robot_pose_count, package.robot_name, robot_curr_pose, scan, update_graph);
          update_graph.push_back(loopgraph);
        }

        // insert received loop closure if received
        if (package.flag_matches) {
          //NonlinearFactorGraph loopgraph_received = this->receiveAgentLoopClosure(package);
          //update_graph.push_back(loopgraph_received);
        }

      } else {
        // ====================================================================
        // if not built relative pose
        // consective information : insert value
        curr_values_.insert(Symbol(ID_, robot_pose_count), robot_curr_pose);

        // process informative scan information, only if relative pose not built
        if (package.informative_scan) {
          foreign_scans_.push_back(package.scan);
          foreign_scans_pose_.push_back(robot_curr_pose);

          std::vector<LoopResult> loop_result;
          loop_result = matcher_->findLoopClosure(*scan, package.robot_name, robot_pose_count);

//          cout << "found " << loop_result.size() << " loop closures between " << local_agent_->name_ << " and " << name_  << endl << endl;

          for (size_t j = 0; j < loop_result.size(); j++) {
            // insert local measurement
            inlier_pose_indices.push_back(make_pair(loop_result[j].scan_idx, robot_pose_count));
            inlier_relposes.push_back(loop_result[j].delta_pose);
            received_matchscan = true;

            cout << "Between local " << local_agent_->name_ << " robots scan: " << loop_result[j].scan_idx << " and foreign pose: " << robot_pose_count << endl;

            PoseD local_cur_pose = local_agent_->curr_values().at<PoseD>(Symbol(local_agent_->ID_, loop_result[j].scan_idx));

            /*
               local_cur_pose.translation().print("Local agent: ");
               cout << endl << " and " << robot_pose_count << endl;
               robot_curr_pose.translation().print("Foreign agent: ");


               PoseD true_rel_pose = local_cur_pose.inverse().compose(robot_curr_pose);
               true_rel_pose.print( " but it really should be ");
               */

            cout << "The estimate rel pose is : " << endl;
            PoseD estim = (local_cur_pose * loop_result[j].delta_pose) * robot_curr_pose.inverse();
            estim.translation().print(" translation: ");
            cout << endl;
//          loop_result[j].delta_pose.print(" the relpose is ");

            Logger::instance()->sendRelPoseEstimate(estim.x(),
                                                    estim.y(),
                                                    estim.rotation().yaw());

            network_->addMatches(name_, local_agent_->name_,
                                 robot_pose_count, loop_result[j].scan_idx,
                                 loop_result[j].delta_pose, robot_curr_pose, local_cur_pose);
          }
        }

        // save received matching information, trig received_matchscan on
        if (package.flag_matches) {
          for (size_t j = 0; j < package.match_robot_id.size(); j++) {

            // swap local/agent
            std::string local_robot = package.match_robot_names.at(j).second;
            std::string foreign_robot = package.match_robot_names.at(j).first;

            // just insert measurement belonging to us
            if (local_robot == local_agent_->name_ && foreign_robot == name_) {
              size_t local_meas_idx = package.match_meas_index.at(j).second;
              size_t agent_meas_idx = package.match_meas_index.at(j).first;

              std::cout << " Inserting " << local_robot << " posecount: " << local_meas_idx << " and " << foreign_robot << " posecount: " << agent_meas_idx << std::endl;

              inlier_pose_indices.push_back(make_pair(local_meas_idx, agent_meas_idx));
              // here inverse to swap local/agent
              inlier_relposes.push_back(package.match_measurement.at(j).inverse());
              received_matchscan = true;
            }
          }
        } // if (package.flag_matches)
      } // if rel pose built not built
    }

    received_packages_.clear();
    std::cout << "Done processing the received packages" << std::endl;

    return received_matchscan;
  }

  void ForeignAgent::sendScanstoMap() {

    ROS_WARN("Sending scans to map");
    for (int i = 0; i < foreign_scans_.size(); i++) {
      /*
      sensor_msgs::PointCloud2 scan = matcher_->project(foreign_scans_[i],
                                                       foreign_scans_pose_[i],
                                                       relative_pose_);
                                                       */
      sensor_msgs::PointCloud2 scan = matcher_->crop_z(foreign_scans_[i], foreign_scans_pose_[i]);

      network_->sendScantoMap(relative_pose_ * foreign_scans_pose_[i], scan);
    }
    foreign_scans_.clear();
    foreign_scans_pose_.clear();
  }

  // generate factor graph and init value for iSAM full graph optimization
  GraphValues ForeignAgent::generateAgentGraph(const Hypothesis& hypothesis) {

    const std::vector<Key> keylist = curr_values_.keys();
    std::vector<Key>::const_iterator iter = keylist.begin();

    // graph of agent:
    // 1. consecutive frame factors, noise model use default icp model
    // 2. between agents factors, noise model from relative pose EM inlier model.
    NonlinearFactorGraph insert_graph;
    while (iter != keylist.end()) {
      Symbol symb1 = Symbol(*iter);
      PoseD pose1 = curr_values_.at<PoseD>(symb1);
      iter++;
      if (iter == keylist.end())
        break;
      Symbol symb2 = Symbol(*iter);
      PoseD delta_odometry = pose1.between(curr_values_.at<PoseD>(symb2));

      insert_graph.push_back(BetweenFactor<PoseD>(symb1, symb2, delta_odometry, setting_.icp_default_model));
    }

    for (size_t i = 0; i < inlier_relposes.size(); i++) {
      // check whether it's inlier according to the hypothesis
      if (hypothesis.inlier_vec.at(i)) {
        size_t local_pose_idx = inlier_pose_indices.at(i).first;
        size_t foreign_pose_idx = inlier_pose_indices.at(i).second;
        PoseD measpose = inlier_relposes.at(i);
        insert_graph.push_back(BetweenFactor<PoseD>(Symbol(local_agent_->ID_, local_pose_idx),
                                                    Symbol(ID_, foreign_pose_idx), measpose, setting_.loop_default_model));
      }
    }

    //std::cout << " Generated the first graph and here it is : " << std::endl;

    //insert_graph.print();

    // init values:
    // compose relative pose and trajectory values
    Values init_values;
    for (iter = keylist.begin(); iter != keylist.end(); iter++) {
      PoseD newvalue = relative_pose_.compose(
                                              curr_values_.at<PoseD>(Symbol(*iter)));
      init_values.insert(*iter, newvalue);
    }

    //init_values.print(" The initial values ");

    // initial last_recorded_index_ & last_recorded_pose_
    // for updateAgentGraph();
    // get and use last pose index
    iter = keylist.end(); iter--;
    Symbol symb_last = Symbol(*iter);
    last_recorded_index = symb_last.index();
    last_recorded_pose = curr_values_.at<PoseD>(symb_last);

    std::cout << " The last recorded index is : " << last_recorded_index << " at loc " << last_recorded_pose << std::endl;

    GraphValues result = make_pair(insert_graph, init_values);
    return result;
  }

  /* ************************************************************************* */
  // received and insert loop closure from received package
  gtsam::NonlinearFactorGraph ForeignAgent::receiveAgentLoopClosure(const AgentPackage& package) {

    NonlinearFactorGraph graph;

    cout << local_agent_->name_ << " is inside its foreign receive agent loop closure " << endl;

    // use received matching information to generate loop closure graph
    if (package.flag_matches) {
      for (size_t j = 0; j < package.match_robot_names.size(); j++) {

        // swap local/agent
        std::string local_robot_name = package.match_robot_names.at(j).second;
        std::string foreign_robot_name = package.match_robot_names.at(j).first;
        size_t local_meas_idx = package.match_meas_index.at(j).second;
        size_t foreign_meas_idx = package.match_meas_index.at(j).first;

        if (foreign_robot_name != local_agent_->name_) {

          unsigned char foreign_robot_id = local_agent_->get_foreign_ID(foreign_robot_name);

          if (local_agent_->curr_values().exists(Symbol(local_agent_->ID_, local_meas_idx)) &&
              local_agent_->curr_values().exists(Symbol(foreign_robot_id, foreign_meas_idx))) {


            std::cout << " Received closure: between " << local_robot_name << " at local_meas_idx " << local_meas_idx << "  and " << foreign_robot_name << " at " << foreign_meas_idx << std::endl;
            package.match_measurement.at(j).print(" The relpose is ");
            // here inverse to swap local/agent
            PoseD pose_measment = package.match_measurement.at(j).inverse();

            // matched: insert between robot factor
            // TODO: should I use EM factor??!!??
            graph.push_back(BetweenFactor<PoseD>(Symbol(local_agent_->ID_, local_meas_idx),
                                                 Symbol(foreign_robot_id, foreign_meas_idx),
                                                 pose_measment, setting_.loop_default_model));
          }
        }
      }
    }
    return graph;
  }

  /* ************************************************************************* */
  // incremental update agent's graph and value for iSAM
  GraphValues ForeignAgent::updateAgentGraph(size_t pose_idx,
                                             const PoseD& slam_pose) {

    // 1. get odom: compare with last recorded pose (still in local frame)
    // 2. generate between factor: use last recorded index
    // 3. insert value in init_values, after composed with last optmized pose

    // get odometry
    // not use received odom: since may have package lost
    PoseD odom = last_recorded_pose.between(slam_pose);

    // insert between factor
    NonlinearFactorGraph graph;
    graph.push_back(BetweenFactor<PoseD>(Symbol(ID_, last_recorded_index),
                                         Symbol(ID_, pose_idx), odom, setting_.icp_default_model));

    // insert value
    PoseD last_optm_value = local_agent_->curr_values().at<PoseD>(Symbol(ID_,
                                                                         last_recorded_index));

    Values initvalue;
    initvalue.insert(Symbol(ID_, pose_idx), last_optm_value.compose(odom));

    // update last recorded variables
    last_recorded_index = pose_idx;
    last_recorded_pose = slam_pose;

    GraphValues result = make_pair(graph, initvalue);
    return result;
  }

  gtsam::NonlinearFactorGraph ForeignAgent::findAgentLoopClosure(size_t pose_idx, std::string robot_name,
                                                                 const PoseD& slam_pose, Scan& agent_scan,
                                                                 const gtsam::NonlinearFactorGraph& update_graph) {

    NonlinearFactorGraph graph;

    // get looped index
    std::vector<LoopResult> loop_result;
    loop_result = matcher_->findLoopClosure(*agent_scan, robot_name, pose_idx);

    for (size_t i = 0; i < loop_result.size(); i++) {
      PoseD relpose = loop_result[i].delta_pose;
      pair<size_t, size_t> relidx = make_pair(loop_result[i].scan_idx, pose_idx);

      // graph for small EM update noise model
      // comment out if not update
      NonlinearFactorGraph full_graph = local_agent_->getisam().getFactorsUnsafe();
      cout << "got the full graph " << endl;
      full_graph.push_back(update_graph);

      // returns true if this scan is an inlier correspondence
      if (small_EM_.perform(relpose, relidx, local_agent_->curr_values(), full_graph)) {
        // matched: insert between robot factor
        graph.push_back(BetweenFactor<PoseD>(Symbol(local_agent_->ID_, loop_result[i].scan_idx), Symbol(ID_, pose_idx),
                                             relpose, setting_.loop_default_model));

        // insert remote measurement for output package
        PoseD local_cur_pose = local_agent_->curr_values().at<PoseD>(Symbol(local_agent_->ID_, loop_result[i].scan_idx));

        network_->addMatches(name_, local_agent_->name_, pose_idx, loop_result[i].scan_idx, relpose, slam_pose, local_cur_pose);
      }
    }
    return graph;
  }

  // When hypothesis is accepted, remove outliers
  void ForeignAgent::remove_outliers(const Hypothesis& hypothesis) {

    curr_values_.clear();

    for (int i = inlier_pose_indices.size()-1; i >= 0; i--)  {
      // TODO rename to is_inlier
      if (!hypothesis.inlier_vec.at(i)) {
        inlier_pose_indices.erase(inlier_pose_indices.begin() + i);
        inlier_relposes.erase(inlier_relposes.begin() + i);
      }
    }
  }

}
