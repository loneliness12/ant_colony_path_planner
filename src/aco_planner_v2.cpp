//
// Created by fabio on 27/12/21.
//

#include "aco_planner_v2.h"
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
using namespace std;
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(aco_ros::aco_planner_v2, nav_core::BaseGlobalPlanner)

namespace aco_ros{
    /***** CONSTRUCTORS/DESTRUCTORS *****/
    aco_planner_v2::aco_planner_v2() : costmap_(nullptr), initialized_(false){}
    aco_planner_v2::aco_planner_v2(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_(nullptr), initialized_(false){
        initialize(name,costmap_ros);
    }
    aco_planner_v2::~aco_planner_v2() = default;

    /***** GLOBAL PLANNER EXTENSION *****/
    void aco_planner_v2::initialize(string name, costmap_2d::Costmap2DROS *costmap_ros) {
        setlocale(LC_ALL,"");
        if( !initialized_ ) {
            ROS_INFO("Initializing ACO planner.");
            //ofstream traceFile_ros;
            //traceFile_ros.open("traceFile_ros.txt");
            // Initialize map
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();//getCostmap得到代价地图里主要的那层代价地图的指针,接下来路径规划的栅格地图就是根据它创建的
            unsigned char *mapChar = costmap_->getCharMap();//getCharMap返回一个数组即代价地图上每一点的代价值
            // Get origin
            costmap_->worldToMap(costmap_->getOriginX(), costmap_->getOriginY(), originX, originY);//将代价地图的起始点坐标从world坐标系转换到map坐标系中originX,originY=0
            frame_id_ = costmap_ros_->getGlobalFrameID();//frame_id_=map
            

            // Generate an OccupancyGridMap from costmap_
            occupancyGridMap =new OccupancyGridMap();//创建蚂蚁算法的栅格地图类，包括：
                //1.FREE,值为0，如果栅格的值为0，表示没有障碍可以通行
                //2.OBSTACLE，值为100，如果栅格的值为100，表示有障碍不可以通行
                //3.UNKNOWN，值为-1，如果栅格的值为-1，表示未知区域
                //4.VALID
                //5.MOVE_COST
                //6.DIAGONAL_MOVE_COST
                //7.INFINIT_COST
                //8.width,栅格地图的宽
                //9.height,栅格地图的高
                //10.resolution，栅格地图的分辨率
                //11.mapLayout，栅格地图
                //12.obstacleSize，
                //float obstacleRatio;
            // Size
            unsigned int width = costmap_->getSizeInCellsX(); //获得地图x方向总共用几个网格
            unsigned int height = costmap_->getSizeInCellsY();//获得地图y方向总共用几个网格
            double resolution = costmap_->getResolution();//获得代价地图的分辨率
            // Content
            auto **mapData = new unsigned int *[height];//将代价地图中栅格的代价值转化成0-100的值，mapData的作用就是可以将其赋值给occupancyGridMap类的mapLayout
            // Initialize the matrix
            for (int i = 0; i < height; i++) {
                mapData[i] = new unsigned int[width];
            }
            //int cnt = 0;//用于记录障碍物地图中有多少可行点
            //traceFile_ros << "Costmap converted into Occupancy Grid " << endl;
            //std:: ofstream mapfile("/home/xcl/rosdemo/demo05_ws/src/ant_colony_path_planner/src/map.txt");  
            // mapfile.open("/home/xcl/rosdemo/demo05_ws/src/ant_colony_path_planner/src/map.txt");
            for (unsigned int iy = 0; iy < height; iy++) {
                //mapfile<<"\n";
                for (unsigned int ix = 0; ix < width; ix++) {
                    unsigned char cost = costmap_->getCost(iy, ix);//得到代价地图每一个网格中的代价值
                    // ROS_INFO("CURR CELL COST %d", cost);
                    if (cost == 0  ) {
                        //ROS_INFO("CELL (%d, %d) IS FREE AND COST %d", iy, ix, cost);
                        mapData[iy][ix] = 0;
                        //mapfile<<0<<" ";
                        //traceFile_ros << mapData[iy][ix] << " ";
                        //cnt++;
                    } /* else if (cost == 255) {
                        //ROS_INFO("IS UNKNOWN AND COST %d", cost);
                        mapData[iy][ix] = -1;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        //cnt++;
                    } else if (cost > 190) {
                        //ROS_INFO("IS LETHAL AND COST %d", cost);
                        mapData[iy][ix] = 100;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        cnt++;
                    } else if (cost > 150){
                        ROS_INFO("IS OCCUPIED AND COST %d", cost);
                        mapData[iy][ix] = 99;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        //cnt++;
                    }
                    else {
                        ROS_INFO("HAS THIS COST %d", cost);
                        mapData[iy][ix] = cost;//char(1 + (97 * (cost - 1)) / 251);
                        traceFile_ros << mapData[iy][ix] << " ";
                        //cnt++;
                    } */
                    // else if( cost >= 128 ){
                    //     mapData[iy][ix] = 100;
                    // }
                    // else{
                    //     mapData[iy][ix] = cost;
                    // }
                    else if (cost >= 254|| cost == -1 )
                    {
                        mapData[iy][ix] = 100;
                        //mapfile<<1<<" ";
                    }
                    else if (cost == 253)
                    {
                        mapData[iy][ix] = 99;
                        //mapfile<<1<<" ";
                    }
                    else
                    {
                        mapData[iy][ix] = char(1 + (97 * (cost - 1)) / 251);
                        //mapfile<<1<<" ";
                    }
                }
                //traceFile_ros << endl;
            }
            //mapfile.close();
            ros::NodeHandle private_nh("~/" + name);//创建发布节点
            // Update the map
            occupancyGridMap->setWidth(width);//栅格地图的宽x方向
            occupancyGridMap->setHeight(height);//栅格地图的高y方向
            occupancyGridMap->setResolution(resolution);//栅格地图的分辨率
            occupancyGridMap->setMapLayout(mapData);//栅格地图每个方格的值
            //ROS_INFO("Free cells: %d", cnt);//输出可行走的栅格个数
            ROS_INFO("HEIGHT %u , WIDTH %u, RESOLUTION %.1f", height, width, resolution);//输出栅格地图的大小
            ROS_INFO("Costmap origin (%d , %d) and cost %d", originX, originY,
                     occupancyGridMap->getCellCost(originX, originY));//输出栅格地图的起始点和起始点的值。

            // ACO planner's parameters
            double alpha = 1; // Pheromone information weight.信息素重要程度的参数
            double beta = 7; // Heuristic information weight.启发式因子重要程度的参数
            double evaporationRate = 0.3; // Pheromone evaporation rate.信息素蒸发因子
            double initialPheromoneValue = 8; // Initialization value.信息素初始值
            int antNumber = 20;//蚂蚁数量
            int iterationsLimit = 10;//迭代总次数
            long int seed = 12345;//随机数
            // Prepare pheromone and total matrix
            initializePheromoneMatrix(occupancyGridMap, initialPheromoneValue);//初始化信息素矩阵pheromoneMatrix_
            initializeTotalMatrix(occupancyGridMap, alpha, beta);//初始化决策矩阵total_ 
            // Generate visited vector for that will be used by all colony's component that uses row major indexing
            vector<bool> visited(height * width, false);
            // Initialize the colony
            //vector<Ant* > initCol(antNumber, new Ant());
            //colony_ = initCol;
            colony_.reserve(antNumber);//设置蚂蚁数量，用vector<Ant*>类型的colony_存储每只蚂蚁
            for (int i = 0; i < antNumber; ++i){
                colony_.emplace_back(new Ant());//向colony_容器中填入每只蚂蚁Ant类，Ant类中包含：
                //1.存储每只蚂蚁关于路径的Path类：Path类中包括：1.path_：用来记录每一只蚂蚁走的路径的数据，元素为栅格的下标 2.cost_：路径长度
                // 2.用于标记蚂蚁已经访问的节点visited_ 3.蚂蚁所在当前的位置currPos_ 4.蚂蚁的名字 5.蚂蚁是否已经搜索过
                colony_[i]->setName(i+1);//设置蚂蚁的名字从1到antNumber
                colony_[i]->setVisited(visited);//初始化用于标记蚂蚁访问节点的数组
            }
            ROS_INFO("Colony initialized. Size %zu", colony_.size());
            // Compute total information
            // Generate the planner
            planner = new ACO_v2(alpha, beta, evaporationRate, antNumber, iterationsLimit, seed, colony_,
                                 pheromoneMatrix_, total_);//将以上初始化蚁群算法需要的元素都传递给封装好的类中，该类再实现蚁群算法
            // Plan publisher
            planPublisher = private_nh.advertise<nav_msgs::Path>("plan", 100);//创建发布者对象，发布路径

            ROS_INFO("ACO planner successfully initialized.");
            initialized_ = true;
            //traceFile_ros.close();
        }
        else{
            ROS_WARN("ACO planner already initialized, doing nothing.");
        }

    }
    bool aco_planner_v2::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        setlocale(LC_ALL,"");
        if (!initialized_) {
            ROS_ERROR("The planner has not been initialized.");
            return false;
        }
        // Clear the plan
        plan.clear();//先清理原路径plan的值

        string global_frame = frame_id_;//global_frame的值为map
        
        // string global_frame=start.header.frame_id;
        ROS_INFO("开始点的参考系%s,frame_id=%s",global_frame.c_str(),frame_id_.c_str());

        // Until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame 检查起始点和目标点是否与代价地图在一个坐标系
        if (goal.header.frame_id != global_frame) {
            ROS_ERROR(
                    "The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
            return false;
        }
        if (start.header.frame_id != global_frame) {
            ROS_ERROR(
                    "The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
            return false;
        }

        double wx = start.pose.position.x;
        double wy = start.pose.position.y;
        ROS_INFO("start:(%.2f,%.2f)",wx,wy);

        unsigned int startX_i, startY_i, goalX_i, goalY_i;
        unsigned int startX, startY, goalX, goalY;
        // Check if the start position is inside the costmap 检查起始点是否在代价图内
        if (!costmap_->worldToMap(wx, wy, startX_i, startY_i)) {
            ROS_WARN(
                    "The robot's start position is off the global costmap.");
            return false;
        }

        // Check if goal position is inside the costmap 检查目标点是否在代价图内
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;
        ROS_INFO("goL:(%.2f,%.2f)",wx,wy);
        if (!costmap_->worldToMap(wx, wy, goalX_i, goalY_i)) {
            ROS_WARN_THROTTLE(1.0,
                              "The goal sent to the ACO planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }

        // Clear the starting cell within the occupancy grid because we know it can't be an obstacle 将栅格地图上的起始点的值设置为0
        clearRobotCell(start, startX_i, startY_i);


        // Transform start and goal from pose stamped to cellIndex if they are valid 将起始点和终止点的坐标点转化为位置索引
        unsigned int startCellIndex = costmap_->getIndex(startX_i, startY_i);
        unsigned int goalCellIndex = costmap_->getIndex(goalX_i, goalY_i);
        //ROS_INFO("Start cell = (%d, %d), Start index = %d Goal cell = (%d, %d), Goal index = %d", startX_i, startY_i, startCellIndex, goalX_i, goalY_i, goalCellIndex);
        //planner->computeTotalInformation(occupancyGridMap, goalCellIndex);
        vector<unsigned int> path_; // Path found by ACO 用于存储生成的路径
        ros::Time sTime = ros::Time::now();
        ROS_INFO("路径规划开始时间：%.2f",sTime.toSec());
        path_ = planner->computePaths(occupancyGridMap, startCellIndex, goalCellIndex);//生成路径的函数
        //ROS_INFO("路径长度为%d",path_.size());
        ros::Duration eTime = ros::Time::now() - sTime;
        //ROS_INFO("结束时间：%.2f",eTime.toSec());
        // path = planner->computeParallelPaths(occupancyGridMap, startCellIndex, goalCellIndex);

        if (!path_.empty()) {
            ROS_INFO("ACO found a path in %f seconds!", eTime.toSec());
            // Prepare the plan
            //plan.push_back(start);
            ros::Time plan_time = ros::Time::now();
            // convert the points to poses
            for (unsigned int i : path_) {
                //std::cout << path[i].first << " " << path[i].second << std::endl;
                unsigned int x_i = 0;
                unsigned int y_i = 0;
                wx = 0.0;
                wy = 0.0;
                // convertToCoordinates(path->getPath()[i], x, y);
                costmap_->indexToCells(i, x_i, y_i);
                costmap_->mapToWorld(x_i, y_i, wx, wy);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame;
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                plan.push_back(pose);
                // ROS_INFO("%.1f %.1f Pushed back in plan", wx, wy);
            }
            publishGlobalPlan(plan);
            return true;
        }
        else
        {
            ROS_ERROR("ACO planner couldn't find a path.");
            return false;
        }
    }

    /***** UTILS *****/
    void aco_planner_v2::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free 将起始点对应的栅格设置为0
        occupancyGridMap->setFree(mx, my);
        // setCost(mx, my, costmap_2d::FREE_SPACE);
    }
    void aco_planner_v2::publishGlobalPlan(vector<geometry_msgs::PoseStamped> &path) {
        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }
        //ros::Rate rate(1);
        planPublisher.publish(gui_path);
    }

    /***** ACO MATRIX MANAGEMENT *****/
    void aco_planner_v2::initializePheromoneMatrix(OccupancyGridMap* map, double initialPheromoneValue) {
        ROS_INFO("Initializing pheromone matrix.");
        //traceFile << "Initial pheromone matrix" << endl ;
        vector<vector<double>> init(map->getHeight(), vector<double>(map->getWidth()));//使信息素矩阵的长，宽与栅格地图一样
        pheromoneMatrix_ = init;//信息素矩阵pheromoneMatrix_

        for (int i = 0; i < map->getHeight(); i++) {

            for (int j = 0; j < map->getWidth(); j++) {

                /* if (map->isFree(i, j) ) {
                    pheromoneMatrix_[i][j] = initialPheromoneValue;
                    //traceFile << getInitialPheromoneValue() << " ";
                }
                else if ( map->isValid(i, j) ){
                    pheromoneMatrix_[i][j] = 0.0;
                } */
                pheromoneMatrix_[i][j] = initialPheromoneValue;
            }
            //traceFile << endl;
        }
        ROS_INFO("Initialized pheromone matrix.");
    }
    void aco_planner_v2::initializeTotalMatrix(OccupancyGridMap* map, double alpha, double beta) {
        ROS_INFO("Initializing pheromone + heuristic matrix.");
        vector<vector<double>> init( map->getHeight(), vector<double>(map->getWidth()) );
        total_ = init;//初始化每个栅格上的（信息素与栅格到目标点距离之和）的决策矩阵
        double heuristic = 1;
        for (unsigned int i = 0; i < map->getHeight(); i++) {
            for (unsigned int j = 0; j < map->getWidth(); j++) {
                // The heuristic information is the inverse of the cell cost
                //if ( map->isFree(i,j) )//判断栅格的值是否为0，为0表示可行栅格
                //    heuristic = 1.0;
                //else
                //    heuristic = 1.0/map->getCellCost(i, j);//有问题

                total_[i][j] = pow(pheromoneMatrix_[i][j], alpha) * pow(heuristic, beta);
            }
        }
    }
}

double euclideanDistance(unsigned int x1, unsigned int y1,unsigned int x2, unsigned int y2){
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}
