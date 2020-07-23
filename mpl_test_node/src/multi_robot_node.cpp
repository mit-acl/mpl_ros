/**
 * @brief Multi-robot test node in a tunnel case
 */
#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

#include "bag_writter.hpp"
#include "robot_team.hpp"

#include <std_msgs/Bool.h>

#include <fstream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  std::string file_name;
  std::string states_name, starts_name, polys_name, paths_name, prs_name;
  nh.param("file", file_name, std::string("sim.bag"));
  nh.param("states_name", states_name, std::string("/states"));
  nh.param("starts_name", starts_name, std::string("/starts"));
  nh.param("polys_name", polys_name, std::string("/polyhedrons"));
  nh.param("paths_name", paths_name, std::string("/paths"));
  nh.param("prs_name", prs_name, std::string("/prs"));

  ros::Publisher poly_pub =
      nh.advertise<decomp_ros_msgs::PolyhedronArray>(polys_name, 1, true);
  ros::Publisher bound_pub =
      nh.advertise<decomp_ros_msgs::PolyhedronArray>("bound", 1, true);
  ros::Publisher state_pub =
      nh.advertise<sensor_msgs::PointCloud>(states_name, 1, true);
  ros::Publisher start_pub =
      nh.advertise<sensor_msgs::PointCloud>(starts_name, 1, true);
  ros::Publisher prs_pub =
      nh.advertise<planning_ros_msgs::PrimitiveArray>(prs_name, 1, true);
  ros::Publisher path_pub =
      nh.advertise<planning_ros_msgs::PathArray>(paths_name, 1, true);

  ros::Publisher end_of_sim_pub =
      nh.advertise<std_msgs::Bool>("/end_of_sim", 1, true);

  Vec3f origin, dim;
  nh.param("origin_x", origin(0), -5.0);
  nh.param("origin_y", origin(1), -5.0);
  nh.param("origin_z", origin(2), -5.0);
  nh.param("range_x", dim(0), 10.0);
  nh.param("range_y", dim(1), 10.0);
  nh.param("range_z", dim(2), 10.0);
  bool decentralized=false; //if false, it will use sequential
  nh.param("decentralized", decentralized, false);

  // Initialize planner
  double dt, v_max, a_max;
  double u;
  int num;
  nh.param("dt", dt, 1.0);
  nh.param("v_max", v_max, -1.0);
  nh.param("a_max", a_max, -1.0);
  nh.param("u", u, 1.0);
  nh.param("num", num, 1); //number of samples for the input

  // Set input control
  vec_E<VecDf> U;
  const decimal_t du = u / num;
  std::cout<<"du= "<<du<<std::endl;
  for (decimal_t dx = -u; dx <= u; dx += du)
    for (decimal_t dy = -u; dy <= u; dy += du)
      for (decimal_t dz = -u; dz <= u; dz += du) U.push_back(Vec3f(dx, dy, dz));
  std::cout<<"U.size()= "<<U.size()<<std::endl;

  bool use_config1, use_config2;
  nh.param("use_config1", use_config1, true);
  nh.param("use_config2", use_config2, false);

  // Set robot geomtric shape
  Polyhedron3D rec;
  double radius_drone=0.15;
  double bbox_half_side=2*radius_drone; //This is FROM the perspective of other agent (that's why 2* is needed, to inflate them

  // double side_box=2*radius_drone; 
  rec.add(Hyperplane3D(Vec3f(-bbox_half_side, 0, 0), -Vec3f::UnitX())); //The value here is half the side of the bbox
  rec.add(Hyperplane3D(Vec3f(bbox_half_side, 0, 0), Vec3f::UnitX()));
  rec.add(Hyperplane3D(Vec3f(0, -bbox_half_side, 0), -Vec3f::UnitY()));
  rec.add(Hyperplane3D(Vec3f(0, bbox_half_side, 0), Vec3f::UnitY()));
  rec.add(Hyperplane3D(Vec3f(0, 0, -bbox_half_side), -Vec3f::UnitZ()));
  rec.add(Hyperplane3D(Vec3f(0, 0, bbox_half_side), Vec3f::UnitZ()));

  std::vector<sensor_msgs::PointCloud> state_msgs;
  std::vector<sensor_msgs::PointCloud> start_msgs;
  std::vector<decomp_ros_msgs::PolyhedronArray> poly_msgs;
  std::vector<planning_ros_msgs::PathArray> path_msgs;
  std::vector<planning_ros_msgs::PrimitiveArray> prs_msgs;

  std::unique_ptr<HomogeneousRobotTeam<3>> robot_team;
  // if (use_config1)
  //   robot_team.reset(new Team1(0.01));
  // else if (use_config2)
  //   robot_team.reset(new Team2(0.01));
  // else
  //   robot_team.reset(new HomogeneousRobotTeam<3>());
  robot_team.reset(new Team2(0.01));

  robot_team->set_v_max(v_max);
  robot_team->set_a_max(a_max);
  robot_team->set_u(U);
  robot_team->set_dt(dt);
  robot_team->set_map(origin, dim);
  robot_team->set_geometry(rec);

 
  ros::Time t0 = ros::Time::now();
  double comp_time_begin=ros::Time::now().toSec();
  std::cout<<std::setprecision(20)<<"t0= "<<t0.toSec()<<" seconds" <<std::endl;
  robot_team->init(); //The first plan happens here (see last for loop in init() function)

  int num_of_agents=robot_team->get_robots().size();
  std::vector<std::vector<Vec3f>> positions_of_agents_synchronized;
  for (int i=0; i<num_of_agents; i++){
    std::vector<Vec3f> tmp; 
    positions_of_agents_synchronized.push_back(tmp);
  }


  vec_E<Polyhedron3D> bbox;
  bbox.push_back(robot_team->get_robots().front()->get_bbox());
  decomp_ros_msgs::PolyhedronArray bbox_msg =
      DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bbox_msg.header.stamp = ros::Time::now();
  bound_pub.publish(bbox_msg);

  // Start the replan loop
  ros::Rate loop_rate(100);
  decimal_t update_t = 0.01;
  decimal_t time = 0;
  decimal_t prev_time = -1000;

  double comp_time;
  double exec_time;
  double exec_time_init;
  bool started_execution=false;

  while (ros::ok()) {
    time += update_t;

    // plan
    bool success;

    if(decentralized==false){
      success=robot_team->update_centralized(time);//the planning happens here. There is only one replanning (see static variable inside this function)
      
      
      if(started_execution==false){
        comp_time=ros::Time::now().toSec()-comp_time_begin; //End of computation time
        exec_time_init=ros::Time::now().toSec(); //Start of execution time
      }
      started_execution=true;
    }
    else
    {
      success=robot_team->update_decentralized(time);//the planning happens here
    }

     if(!success) {
    //if (!robot_team->update_decentralized(time)) { 
      ROS_INFO("Robot fails to plan, ABORT!");
      break;
    }
    std::cout<<std::endl;

    // set obstacle simultaneously
    auto poly_obs = robot_team->get_obs();  // set obstacles at time

    int index_agent=0;
    for (auto& it : robot_team->get_robots()) {
      Vec3f current_pos=it->get_state(time).pos;
      positions_of_agents_synchronized[index_agent].push_back(current_pos);
      index_agent=index_agent+1;
    }

    // Visualizing current status at 10 Hz
    if (time - prev_time >= 0.1) {
      // Reduce the size of obstacles manually. 
      // This is to visualize the real size of the drone (and not the inflated ones)
      for (auto& it : poly_obs) {
        for (auto& itt : it.vs_) {
            itt.p_ -= itt.n_ * (radius_drone);  
                                                // was 0.25 (hand-coded for the original rec dimensions I think)
          }
      }
      //////////////////

      decomp_ros_msgs::PolyhedronArray poly_msg =
          DecompROS::polyhedron_array_to_ros(poly_obs);
      poly_msg.header.frame_id = "map";
      poly_msg.header.stamp = t0 + ros::Duration(time);
      poly_pub.publish(poly_msg);
      poly_msgs.push_back(poly_msg);

      vec_E<vec_Vec3f> path_array;
      vec_Vec3f states;
      vec_Vec3f starts;
      for (auto& it : robot_team->get_robots()) {
        starts.push_back(it->get_start().pos);
        states.push_back(it->get_state(time).pos);
        path_array.push_back(it->get_history());
      }

      auto path_msg = path_array_to_ros(path_array);
      path_msg.header.frame_id = "map";
      path_msg.header.stamp = t0 + ros::Duration(time);
      path_pub.publish(path_msg);
      path_msgs.push_back(path_msg);

      auto state_msg = vec_to_cloud(states);
      state_msg.header.frame_id = "map";
      state_msg.header.stamp = t0 + ros::Duration(time);
      state_pub.publish(state_msg);
      state_msgs.push_back(state_msg);

      auto start_msg = vec_to_cloud(starts);
      start_msg.header.frame_id = "map";
      start_msg.header.stamp = t0 + ros::Duration(time);
      start_pub.publish(start_msg);
      start_msgs.push_back(start_msg);

      vec_E<Primitive3D> prs_array;
      for (auto& it : robot_team->get_robots()) {
        auto prs = it->get_primitives();
        prs_array.insert(prs_array.end(), prs.begin(), prs.end());
      }

      auto prs_msg = toPrimitiveArrayROSMsg(prs_array);
      prs_msg.header.frame_id = "map";
      prs_msg.header.stamp = t0 + ros::Duration(time);
      prs_pub.publish(prs_msg);
      prs_msgs.push_back(prs_msg);

      prev_time = time;
    }

    if (robot_team->finished(time)) {
      ROS_INFO("All robots reached!");
      exec_time=ros::Time::now().toSec()-exec_time_init;
      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  
  double total_time=(ros::Time::now() - t0).toSec();
  //ROS_WARN("Total time (seconds): %f", (ros::Time::now() - t0).toSec());

  int num_pos_per_agent=positions_of_agents_synchronized.front().size(); //all of them have the same number of position


  ////////////////////// COMPUTE SAFETY RATIO
  double min_dist=std::numeric_limits<double>::max();
  for (int i=0; i<num_of_agents; i++){
    for (int j=i+1; j<num_of_agents; j++){
      std::cout<<"Agent "<<i<<" --> Agent "<<j<<std::endl;
      for (int k=0; k<num_pos_per_agent; k++){
        min_dist=std::min(min_dist,(positions_of_agents_synchronized[i][k]-positions_of_agents_synchronized[j][k]).norm());
      }
    }
  }
  double safety_margin_ratio=min_dist/(2*radius_drone);

  //////////////////// COMPUTE TOTAL DISTANCE
  double total_dist=0;
  for (int i=0; i<num_of_agents; i++){
    for (int k=0; k<=(num_pos_per_agent-2); k++){
        total_dist=total_dist+(positions_of_agents_synchronized[i][k+1]-positions_of_agents_synchronized[i][k]).norm();
    }

     //The last element of positions_of_agents_synchronized is the position of the drone when it did the last replan. 
     //Therefore, I need to add the length of that trajectory. Assumming straight line here. 
     total_dist=total_dist+(robot_team->global_goals_[i]-positions_of_agents_synchronized[i].back()).norm();
  }

  //////////////////// COMPUTE t_1start, t_laststart, t_laststart
  double t_1start=std::numeric_limits<double>::max(); //moment when the first agent starts moving
  double t_laststart=0.0; //moment when the last agent starts moving
  double t_1end=std::numeric_limits<double>::max(); //moment when the first agent reaches the goal

  for (auto& it : robot_team->get_robots()) {

    std::cout<<"it->t_start_= "<<it->t_start_<<std::endl;
    t_1start=std::min(t_1start, it->t_start_);
    t_laststart=std::max(t_laststart, it->t_start_);
    t_1end=std::min(t_1end, it->t_end_);

  }

  t_1start=t_1start-t0.toSec();
  t_laststart=t_laststart-t0.toSec();
  t_1end=t_1end-t0.toSec();

  ////////////////////// PRINT RESULTS
  std::cout<<"Sum dist all the agents (m): "<<total_dist<<std::endl;  //Should be > 1 to ensure safety
  std::cout<<"Safety Margin Ratio: "<<safety_margin_ratio<<std::endl; //Should be > 1 to ensure safety
  std::cout<<"Total time (s): "<<total_time<<std::endl;               //Should be > 1 to ensure safety
  if(decentralized){
    std::cout<<"t_1start (s): "<<t_1start<<std::endl;
    std::cout<<"t_laststart (s): "<<t_laststart<<std::endl;
    std::cout<<"t_1end (s): "<<t_1end<<std::endl;
  }
  else{
    std::cout<<"Comp time (s): "<<comp_time<<std::endl;
    std::cout<<"Exec time (s): "<<exec_time<<std::endl;
  }


  ////////////////////// WRITE RESULTS
  std::ofstream outfile;
  outfile.open("/home/jtorde/Desktop/ws_mpl/src/mpl_ros/results/results.txt", std::ios::out | std::ios::app); // append instead of overwrite

  outfile<< "=============================="<<"\n";
  outfile <<"num=(Num of samples-1)/2: "<<num<<"\n";
  outfile <<"u (m/s3) :"<<u<<"\n";

  if(decentralized==true){
    outfile <<"Approach: DECENTRALIZED \n";
  }
  else{
    outfile <<"Approach: SEQUENTIAL \n";
  }
  outfile<< "___"<<"\n";

  outfile <<"Sum dist all the agents (m): "<<total_dist<<"\n";
  outfile <<"Safety Margin Ratio: "<<safety_margin_ratio<<"\n";
  outfile <<"Total time (s): "<<total_time<<"\n";

  if(decentralized){
    outfile <<"t_1start (s): "<<t_1start<<std::endl;
    outfile <<"t_laststart (s): "<<t_laststart<<std::endl;
    outfile <<"t_1end (s): "<<t_1end<<std::endl;
  }
  else{
    outfile <<"Comp time (s): "<<comp_time<<std::endl;
    outfile <<"Exec time (s): "<<exec_time<<std::endl;
  }


  outfile.close();
  /////////////////////////////////////

  ROS_INFO("End of simulation");

  std_msgs::Bool msg;
  msg.data=true;
  end_of_sim_pub.publish(msg);


  // Write to bag (optional)
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Write);

  for (const auto& it : start_msgs) bag.write(starts_name, it.header.stamp, it);
  for (const auto& it : state_msgs) bag.write(states_name, it.header.stamp, it);
  for (const auto& it : poly_msgs) bag.write(polys_name, it.header.stamp, it);
  for (const auto& it : path_msgs) bag.write(paths_name, it.header.stamp, it);
  for (const auto& it : prs_msgs) bag.write(prs_name, it.header.stamp, it);

  bag.close();

  return 0;
}
