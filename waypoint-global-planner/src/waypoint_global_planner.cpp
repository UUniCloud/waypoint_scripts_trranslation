#include "waypoint_global_planner/waypoint_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)
//加载插件类，即将WaypointGlobalPlanner加到BaseGlobalPlanner中。

namespace waypoint_global_planner//
{
//WaypointGlobalPlanner类的构造函数WaypointGlobalPlanner()继承了costmap_ros_（）， initialized_(）， clear_waypoints_(false)
{
//继承基类的这三个成员变量，并且都赋以初值，costmap_ros_=NULL,initialized_=false,clear_waypoints_=false
WaypointGlobalPlanner::WaypointGlobalPlanner() : costmap_ros_(NULL), initialized_(false), clear_waypoints_(false)
{
}

//拷贝构造函数，即带参数的类的对象的初始化函数
//其中costmap_2d这个类是在costmap.h这个里面声明定义的，而costmap.h又在waypoint_global_planner.h里面include进来离，所以这里可以直接用它来定义变量
WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  //costmap_ros是一个类指针对象，调用initialize函数，所以下面的initialize函数不仅仅是定义，载入参数即开始实际执行。
  initialize(name, costmap_ros);//以规划器的名字和代价地图初始化规划器，这里调用初始化函数initialize，具体实现在下面定义
}

WaypointGlobalPlanner::~WaypointGlobalPlanner()//析构函数
{
}

void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)//初始化函数的具体实现
{
  if(!initialized_)//为真的的条件：initialized_=false
  {
    // get the costmap
    //costmap用于将laser扫面数据或者点云数据转化成一个2d的网格地图
    costmap_ros_ = costmap_ros;//get the global_cost_map，costmap_ros是一个类指针对象，类型为costmap_2d类中的Costmap2DROS
    costmap_ = costmap_ros_->getCostmap();//根据costmap_ros_调用其getCostmap()，返回值给costmap_。
    world_model_ = new base_local_planner::CostmapModel(*costmap_);//再以得到的costmap_驱动CostmapModel函数

    ros::NodeHandle nh;//定义一个节点nh
    ros::NodeHandle pnh("~" + name);//
    // load parameters加载参数
    pnh.param("epsilon", epsilon_, 1e-1);//0.1
    pnh.param("waypoints_per_meter", waypoints_per_meter_, 20);//每米的点数

    // initialize publishers and subscribers初始化发布器和订阅器，100指的是队列长度
    //这里执行上2个订阅器的时候，就同时开始订阅相应的回调函数，回调函数在下面有另外的定义。
    waypoint_sub_ = pnh.subscribe("/clicked_point", 100, &WaypointGlobalPlanner::waypointCallback, this);
    external_path_sub_ = pnh.subscribe("external_path", 1, &WaypointGlobalPlanner::externalPathCallback, this);
    waypoint_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);//
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);//
    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);//
    //订阅器在被调用的时候就已经开始调用了回调函数，而发布器只有执行goal.pub.publish()时才真正调用。

    initialized_ = true;//初始化完成后，initialized_置为true。
    ROS_INFO("Planner has been initialized");
  }
  else
  {
    ROS_WARN("This planner has already been initialized");
  }
}

//核心全局路径规划器函数
//路径规划主函数，其中调用的函数都在下面定义。
//该函数返回值为bool型的，即返回值为true ，即规划路径成功。为false,则不成功。
bool WaypointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  //开始点和目标点都是里程消息的位姿类型，plan是一个容器类型，里面存放的都是posestamped类型的元素。
{
  //path_是在waypoint_global_planner.h里面声明的nav_msgs::Path型的变量（对象）
  path_.poses.insert(path_.poses.begin(), start_pose);//begin()返回一个指向path_.poses向量开头的迭代器。
  //iterator insert( iterator loc, const TYPE &val );
  //在指定位置loc前插入值为val的元素,返回指向这个元素的迭代器。
  //总之，这句是将起始点start_pose插入向量path_.poses之中。
  interpolatePath(path_);//在下面单独定义的函数，将临时路径向量temp_path中的所有路径点都加到了要发布的path_中
  plan_pub_.publish(path_);//发布规划的全局路径到话题“global_plan”上
  plan = path_.poses;//将path_中的所有路径点取出来暂时放到plan中。至于哪里会用到plan呢？？？？？？？？
  ROS_INFO("Published global plan");//打印消息日志：全局路径已发布
  return true;//返回函数值为“true”,表示成功规划了全局路径。
}

//这些回调函数在初始化的时候就已经调用了
//订阅/click point话题的回调函数，clicked_point话题上的消息类型也是geometry_msgs：：PointStampedConstPtr指针。
//clicked_point是在rviz中加入publish_point之后出现的话题。可以查看其类型
void WaypointGlobalPlanner::waypointCallback(const geometry_msgs::PointStampedConstPtr& waypoint)
{
  if (clear_waypoints_)//判断有咩有清空点，通过标志clear_waypoints_的值来判断
  {
    waypoints_.clear();//waypoints_没有清空，就执行清空操作。
    //vector将会清空vector中的所有元素
    clear_waypoints_ = false;//路径点清空后，标志clear_waypoints_置为false
  }

  // add waypoint to the waypoint vector将从话题上订阅到的路径点消息加入到waypoint路径点向量中，往向量的末尾加的。
  waypoints_.push_back(geometry_msgs::PoseStamped());//waypoints_的末尾添加一个geometry_msgs::PoseStamped()对象
  waypoints_.back().header = waypoint->header;//.back是访问向量的最后一个元素（即上一句添加的对象），waypoint_.back即最后一个元素。
  waypoints_.back().pose.position = waypoint->point;//将waypoint的point给向量中新添加的位置位赋值
  waypoints_.back().pose.orientation.w = 1.0;//表示方向的四元数的w为直接赋值为1,即180度

  // create and publish markers创建并发布可视化标记
  createAndPublishMarkersFromPath(waypoints_);//调用函数，将waypoints向量发布成可视化的标记

  if (waypoints_.size() < 2)//如果向量waypoints_里就一个元素（即一个路径点）时，此回调函数就此返回，结束。
    return;
  //当waypoints_里有两个元素时，即上面的if条件不满足是以waypoints_size（）=2退出的。
  geometry_msgs::Pose *p1 = &(waypoints_.end()-2)->pose;//waypoints_.end()返回的是指向waypoints_最后元素的下一个位置
  //waypoints_.end()-2即向量第一个元素的位置，加上”&“，即第一个元素，将其pose属性拿出来给wpose型指针p1
  geometry_msgs::Pose *p2 = &(waypoints_.end()-1)->pose;
  //waypoints_.end()-1即向量第二个元素的位置，加上”&“，即第二个元素，将其pose属性拿出来给wpose型指针p2
  //waypoints_.end()相当于尾指针，总是指向向量数组的最后一个元素的下一个位置。

  // calculate orientation of waypoints计算路径点的方向
  //这里用yaw偏航角的名称来定义，求两点之间连线的反正切值
  //atan2函数是求y/x（弧度表示）的反正切值，值域在(-π/2，+π/2)。
  double yaw = atan2(p2->position.y - p1->position.y, p2->position.x - p1->position.x);
  p1->orientation = tf::createQuaternionMsgFromYaw(yaw);//将角度（弧度数）转换为四元数
  // calculate distance between latest two waypoints and check if it surpasses the threshold epsilon
  //计算最新两个点之间的距离，并检查是否超过阈值epsilon
  double dist = hypot(p1->position.x - p2->position.x, p1->position.y - p2->position.y);

  //hypot函数，对于给定的直角三角形的两个直角边，求其斜边的长度
  if (dist < epsilon_)//如果距离在阈值范围内，做处理；大于阈值，不做处理。
  {
     p2->orientation = p1->orientation;//第二个点的方位角等于第一个点的方位角
    path_.header = waypoint->header;//路径的消息头等于位置点的消息头
    path_.poses.clear();//清除路径点
    path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
    //在path_.poses的最后一个元素的下一个位置，插入waypoints_中从begin()到end()的所有路径点。其实就是两个点。
    //insert也是vector的方法：void insert( iterator loc, input_iterator start, input_iterator end );
    //在指定位置loc前插入区间[start, end)的所有元素。注意不包括.end,因为是圆括弧
    goal_pub_.publish(waypoints_.back());//发布waypoints_中最后一个元素到话题/move_base_simple/goal上
    clear_waypoints_ = true;//置标志clear_waypoints_等于true
    ROS_INFO("Published goal pose");//打印日志消息，发布了目标位置姿态。
  }
}

//参数是Path类型的，即poses,一个装满了pose点的list。
void WaypointGlobalPlanner::interpolatePath(nav_msgs::Path& path)//插入路径path函数
{
  //传入的实参path_中已经有了第一个路径点
  std::vector<geometry_msgs::PoseStamped> temp_path//定义一个临时路径向量容器
  //static_cast<int>(expression)是把expression的类型转换为int类型
  for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
  {
    // calculate distance between two consecutive waypoints两个连续的点之间计算距离
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i+1].pose.position.x;
    double y2 = path.poses[i+1].pose.position.y;
    //利用hypot求两点之间的连线直线距离。
    double dist =  hypot(x1-x2, y1-y2);//hypot函数：对于给定的直角三角形的两个直角边，求其斜边的长度
    //waypoints_per_meter_为指定的每米上的路径点数
    int num_wpts = dist * waypoints_per_meter_;//两点之间的距离乘以每米距离的路径点数，即dist距离上所有的点。
    //将path中第i个路径点加到temp_path的尾部;num_wpts为两个目标点之间的所有路径点数。
    temp_path.push_back(path.poses[i]);//push_back是c++中的方法，在vector类中作用为在vector尾部加入一个数据。
    geometry_msgs::PoseStamped p = path.poses[i];//将path中第i个路径点暂存到变量p中
    //
    //将两个点连线上的所有点都加到临时路径向量之中。
    for (int j = 0; j < num_wpts - 2; j++)//-2是除却开头和结尾两个点
    {
	    //将两个点连线上的所有点都加到临时路径向量之中。
      //由已知的两个点，求这两个点连线上的等距离的（20-2）（与1米为例）个点的坐标。
      p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
      //j/ num_wpts即所求点所在位置与整个连线的比例k，k×(x2 - x1)即为所求点的x坐标
      p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
      //j/ num_wpts即所求点所在位置与整个连线的比例k，k×(y2 - y1)即为所求点的y坐标
      temp_path.push_back(p);//将p放到临时路径向量。
    }
  }
  //所有点放到temp_path中，然后更新位姿list的序列。update sequence of poses
  //size_t是size type。一种用来记录大小的数据类型。通常我们用sizeof(XXX)操作，这个操作所得到的结果就是size_t类型。
  //update sequence of poses。
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);//每个路径点的消息头序列等于循环变量

  temp_path.push_back(path.poses.back());//最后将路径的最后一个点(目标点)放到临时路径变量里
  path.poses = temp_path;//将临时路径向量中的所有路径点都加到了要发布的path中
}

//订阅全局路径话题external_path的回调函数，该话题上的消息类型为nav_msgs::PathConstPtr& plan
//nav_msgs::PathConstPtr& plan消息
void WaypointGlobalPlanner::externalPathCallback(const nav_msgs::PathConstPtr& plan)
{
  path_.poses.clear();//清空path_路径向量,应该属于初始化一下
  clear_waypoints_ = true;//将标志位置“true”
  path_.header = plan->header;//消息头赋值
  path_.poses = plan->poses;//将从话题上接收到的路径指针plan中的路径posesh存到path_的poses中
  createAndPublishMarkersFromPath(path_.poses);//以路径“path_poses”调用可视化标记函数，达到在rviz中显示路径的目的。
  goal_pub_.publish(path_.poses.back());//最后发布目标点，取路径的最后一个点进行发布
}

//从路径中创建并发布可视化标记，函数的参数是geometry_msgs::path类型的
void WaypointGlobalPlanner::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  // clear previous markers每次显示之前，先清楚之前的标记
  visualization_msgs::MarkerArray markers;//定义一个标记列表，里面的元素都是Marker类型的元素
  visualization_msgs::Marker marker;//定义一个标记变量（标记对象），marker
  marker.header = path[0].header;//marker的消息头取自path中第一个位置点的消息头
  marker.ns = "/move_base/waypoint_global_planner";//命名空间
  marker.type = visualization_msgs::Marker::SPHERE;//标记的类型为球形
  marker.action = visualization_msgs::Marker::DELETEALL;//先删除所有标记
  marker.scale.x = 0.2;//设置标记的规模，x边为0.2。
  marker.scale.y = 0.2;//y边为0.2
  marker.scale.z = 0.2;//z边为0.2
  marker.color.a = 1.0;//透明度为1，0表示完全透明
  marker.color.r = 1.0;//rgb为100,即红色标记
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.id = 0;//命名空间（ns）和id是用来给这个marker创建一个唯一的名字的。此处为初始化
  //如果接收到一个相同命名空间和id的marker，那新的就会把老的替换掉。
  markers.markers.push_back(marker);//将设置好的marker放到标记marker列表中,依次往后排列
  waypoint_marker_pub_.publish(markers);//将markers发布到/waypoints话题上
  marker.action = visualization_msgs::Marker::ADD;//创建可视化标记
  markers.markers.clear();//清空markers列表

  for (size_t i = 0; i < path.size(); i++)
  {
    marker.id = i;//为了所有的点不会覆盖，都能显示，所以每个点的显示标记要设的不一样。
    marker.pose.position = path[i].pose.position;//标记所在位置的三维坐标为每一个要显示点的position
    markers.markers.push_back(marker);//将标记放到markers列表中
  }

  waypoint_marker_pub_.publish(markers);//发布到/waypoint话题上。rviz通过订阅该消息，执行显示功能。
}

}  // namespace waypoint_global_planner
