//
// Created by fabio on 27/12/21.
//

#include "ACO_v2.h"
#include "Ant.h"
#include "OccupancyGrid.h"
#include "Path.h"

#include <ros/ros.h>

#include <cmath>
#include <random>
#include <vector>

/* constants for a random number generator, for details see numerical recipes in C */
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836

using namespace std;
/***** CONSTRUCTORS/DESTRUCTORS *****/
ACO_v2::ACO_v2() = default;
ACO_v2::ACO_v2(double alpha, double beta, double evaporationRate, int antNumber,int iterationsLimit, long int seed, vector<Ant*> &colony,
               vector<vector<double>>& pheromoneMatrix, vector<vector<double>>& total) {
    setAlpha(alpha);
    setBeta(beta);
    setEvaporationRate(evaporationRate);
    setAntNumber(antNumber);
    setIterationsLimit(iterationsLimit);
    bestIteration = 0;
    colony_ = colony;
    pheromoneMatrix_ = pheromoneMatrix;
    total_ = total;
    seed_ = seed;
}
ACO_v2::~ACO_v2(void) = default;

/***** MUTATORS *****/
void ACO_v2::setPheromoneMatrix(vector<vector<double>>& pheromoneMatrix) {
    pheromoneMatrix_ = pheromoneMatrix;
}
void ACO_v2::setTotalMatrix(vector<vector<double>>& total) {
    total_ = total;
}
void ACO_v2::setAlpha(double alpha) {
    alpha_ = alpha;
}
void ACO_v2::setBeta(double beta) {
    beta_ = beta;
}
void ACO_v2::setEvaporationRate(double evaporationRate) {
    evaporationRate_ = evaporationRate;
}
void ACO_v2::setAntNumber(int antNumber) {
    antNumber_ = antNumber;
}
void ACO_v2::setIterationsLimit(int it) {
    iterationsLimit_ = it;
}

vector<vector<double>> ACO_v2::getPheromoneMatrix() {
    return pheromoneMatrix_;
}
vector<vector<double>> ACO_v2::getTotal(){
    return total_;
}
double ACO_v2::getAlpha() {
    return alpha_;
}
double ACO_v2::getBeta() {
    return beta_;
}
double ACO_v2::getEvaporationRate() {
    return evaporationRate_;
}
int ACO_v2::getAntNumber() {
    return antNumber_;
}
int ACO_v2::getIterationsLimit() {
    return iterationsLimit_;
}

/***** UTILITIES *****/
vector<unsigned int> ACO_v2::getNeighbours(OccupancyGridMap* map, Ant* ant) {
    unsigned int mx = 0;//存储栅格的索引在栅格中的宽
    unsigned int my = 0;//存储栅格的索引在栅格中的高
    //unsigned int debugIndex = 0;
    map->indexToCells(ant->getConstructedPath()->getPath().back(), mx, my);//将蚂蚁前一步走的位置用mx,my表示出来
    vector<unsigned int> neighbourCells;//用于记录可以访问的栅格

    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++) {
            // With i = 0 and j = 0 current cell is obtained. In that case do nothing
            if( i == 0 && j == 0 )
                continue;
            // Check if the coordinates are legal
            if ( (mx + i > 0) &&  (mx + i < map->getHeight()) && (my + j > 0) && (my + j < map->getWidth()))//判断点是否在栅格内
            {
                if ( map->isValid(map->getCellIndex(mx + i, my + j))  && !ant->isVisited(map->getCellIndex(mx + i, my + j)) )//判断蚂蚁前一位置的上下左右位置的在栅格地图中是否为障碍物以及是否已经走过
                    neighbourCells.emplace_back(map->getCellIndex(mx + i, my + j));//如果满足条件就将其存入到数组中
            }
        }
    return neighbourCells;
}
unsigned int ACO_v2::chooseNeighbour(OccupancyGridMap* map, vector<unsigned int> neighbours) {
    // Define a probability vector. Its size depends on the amount of neighbours
    vector<double> probabilities;//用于存储可行点位置上的启发值
    //probabilities.reserve(neighbours.size());
    vector<double> cumulativeProbabilities;//存储每个位置的比例区间
    //cumulativeProbabilities.reserve(probabilities.size());
    // Store the cumulative probability
    double probSum = 0.0;//记录总共的启发值
    // Used to check the selection condition
    double partialSum = 0;
    // Output
    unsigned int chosen = 0;//用于记录最后选择的位置
    // Temporary values to navigate the map
    unsigned int mx = 0;//存储栅格的索引在栅格中的宽
    unsigned int my = 0;//存储栅格的索引在栅格中的高

    for (auto i: neighbours) { // For each neighbour cell check if free
        map->indexToCells(i, mx, my);//转换成在栅格中的宽和高
        probabilities.emplace_back(total_[mx][my]); // Total computed cost of the cell  存储可行点位置上的启发值
        probSum += total_[mx][my];//总共的启发值
        //ROS_INFO("Ant %d, TIMES %d, VALID %d, VISITED %d", ant->getName(), l, map->isValid(i), isVisited(ant, i));
    }
    // Normalize the probabilities
    if ( probSum != 0 ){
        for(double & p : probabilities){
            p = p / probSum;//计算每个位置在总启发值中的比例
        }
    }

    // Prepare cumulative probabilities vector
    cumulativeProbabilities.reserve(probabilities.size());
    cumulativeProbabilities.emplace_back(probabilities[0]);
    for( int i = 1; i < probabilities.size(); i++ ){
            cumulativeProbabilities.emplace_back( probabilities[i-1] + probabilities[i] );//设置不同比例区间
        }

    // Generate a random number
    // double rand=getRandom(0,1);//生成随机数
    double rand = getRandom01(&seed_);//生成随机数 有问题
    // ROS_INFO("随机数是%.2f",rand);

    if( rand >= 0 && rand <= cumulativeProbabilities[0] ){
        chosen = neighbours[0];
    }
    else if ( rand > cumulativeProbabilities.back() && rand <= 1){
        chosen = neighbours.back();
    }
    else{
        for (int i = 0; i < cumulativeProbabilities.size()-1; i++){
            if (rand > cumulativeProbabilities[i] && rand <= cumulativeProbabilities[i + 1]){
                //long double difference1 = rand - cumulativeProbabilities[i];
                //long double difference2 = cumulativeProbabilities[i + 1] - rand;
                //if (difference1 < difference2)
                //    chosen = neighbours[i];
                //else
                chosen = neighbours[i + 1];
            }
        }
    }

    /* // Choose the neighbour
    if( probSum <= 0 ){ // No feasible neighbour cells.
        //ROS_INFO("STUCK");
        // Delete the last element of the path since it leads to a deadlock
        ant->getConstructedPath()->popBack();
        // Update the cost
        ant->getConstructedPath()->computeCost(map);
        // Update the ant position
        ant->setPosition(ant->getConstructedPath()->getPath().back());
        // Recursive call with updated ant position, path cells and path cost.
        chosen = chooseBestNext(map, ant);
        chosen = chooseBestNext(map, ant);
    }
    else{ // At least one of the neighbours is eligible. Select one according to the probabilities rule.
        // Select a random01 value
        double random = getRandom(0, probSum);//getRandom01(&seed_);
        //random *= probSum;
        // Index for updating partialSum
        int i = 0;
        // Set the initial value for partial sum
        partialSum = probabilities[0].second;
        while ( partialSum <= random ){
            //ROS_INFO("I: %d Partial sum %.1f, ProbSum %.1f, RANDOM %.1f", i, partialSum, probSum, random);
            i++;
            partialSum += probabilities[i].second;
            //ROS_INFO("I: %d Partial sum %.1f, ProbSum %.1f, RANDOM %.1f", i, partialSum, probSum, random);
        }

        if ( i == neighbours.size()) {
            chosen = chooseBestNext(map, ant);
        }
        else {

            chosen = probabilities[i].first;
            map->indexToCells(chosen, mx, my);
            //ROS_INFO("CHOSEN NORMALMENTE %d, %d", mx, my);
        }
    } */
    //ROS_INFO("选择的栅格是：%d",chosen);
    return chosen;
}
void ACO_v2::updateStatistics(OccupancyGridMap* map, int iteration) {//有问题
    // Find best ant of the current iteration based on its path length.
    unsigned int iterationBestAnt = findBestAnt();//寻找此次迭代中路径最小的蚂蚁

    // Update bestAnt if a best ant of iteration has been found
    if ( iterationBestAnt != -1 ){//如果有最短路径
        //ROS_INFO("Best ant of iteration: %d, path length: %zu, cost: %f", colony_[iterationBestAnt]->getName(),
                 //colony_[iterationBestAnt]->getConstructedPath()->getPath().size(),
                 //colony_[iterationBestAnt]->getConstructedPath()->getCost());
        // ROS_INFO("最短路径:%.2f",bestAnt.getConstructedPath()->getCost());
        if ( bestAnt.getConstructedPath()->getCost() == 0 || colony_[iterationBestAnt]->getConstructedPath()->getCost() <
        bestAnt.getConstructedPath()->getCost() ) {//每只蚂蚁的cost_设置了吗
            copyFromTo(colony_[iterationBestAnt], &bestAnt);//将最优蚂蚁复制给bestAnt
            copyFromTo(colony_[iterationBestAnt], &restartBestAnt);//将最优蚂蚁复制给restartBestAnt
            bestIteration = iteration;//将最优迭代次数复制给bestIteration
            restartBestIteration = iteration;//将最优迭代次数复制给restartBestIteration
        }
        if (colony_[iterationBestAnt]->getConstructedPath()->getCost() <
            restartBestAnt.getConstructedPath()->getCost()) {
            ACO_v2::copyFromTo(colony_[iterationBestAnt], &restartBestAnt);
            restartBestIteration = iteration;
            ROS_INFO("Restart best: %f, restartBestIteration %d\n", restartBestAnt.getConstructedPath()->getCost(),
                     restartBestIteration);
        }


        //ROS_INFO("Best ant: %d, path length: %zu, cost: %.1f", bestAnt.getName(),
                 //bestAnt.getConstructedPath()->getPath().size(),
                 //bestAnt.getConstructedPath()->getCost());
    }
    else{
        //ROS_INFO("All ants have been stuck during iteration %d.", iteration);
        // ROS_INFO("最短路径:%.2f",bestAnt.getConstructedPath()->getCost());
    }
}
int ACO_v2::findBestAnt() {
    unsigned long min=-1;
    int best=-1;

    for(int k = 0; k < colony_.size(); k++){
        if ( !colony_[k]->isSearching() && (min == -1 || colony_[k]->getConstructedPath()->getCost() < min) ) {//有问题没有计算长度
            min = colony_[k]->getConstructedPath()->getCost();
            best = k;
        }
    }
    return best;
}
void ACO_v2::copyFromTo ( Ant* a1, Ant* a2 ){
    a2->getConstructedPath()->clearPath();//清理最优蚂蚁的路径
    a2->clearVisited();//清理最优蚂蚁的访问过的栅格
    a2->getConstructedPath()->setCost(a1->getConstructedPath()->getCost());//设置最优蚂蚁的路径长度
    //ROS_INFO("cost best ant %f", a1->getConstructedPath()->getCost());
    a2->getConstructedPath()->setPath(a1->getConstructedPath()->getPath());//设置最优蚂蚁的路径
    //ROS_INFO("dim path best ant of iteration %zu", a1->getConstructedPath()->getPath().size());

    vector<bool> temp;//复制找到的最优蚂蚁访问过的栅格
    for ( auto i: a1->getVisited() ) {
        temp.emplace_back(i);
    }


    a2->setVisited(temp);//设置最优蚂蚁的访问过的栅格
    a2->setName(a1->getName());//设置最优蚂蚁的名字
    //ROS_INFO("dim path best ant %zu", a2->getConstructedPath()->getPath().size());
    //ROS_INFO("test3");
}
double ACO_v2::getRandom(double minBound, double maxBound){
    random_device rd;
    default_random_engine rdGenerator( rd() );
    uniform_real_distribution<> rdValue(minBound, maxBound);

    return rdValue(rdGenerator);
}
double ACO_v2::getRandom01(long* idum){
    long k;
    double ans;

    k =(*idum)/IQ;
    *idum = IA * (*idum - k * IQ) - IR * k;
    if (*idum < 0 ) *idum += IM;
    ans = AM * (*idum);
    return ans;
}
bool ACO_v2::endSearchCondition(unsigned int goalCellID, int searchCondition){
    while( searchCondition <= 10000 ) {
        for (auto k: colony_) {
            if (k->getPosition() != goalCellID)
                return false;
        }
    }
    return true;
}
unsigned int ACO_v2::chooseBestNext(OccupancyGridMap* map, Ant* ant){
    double value_best = -1.;
    unsigned int nxtCell = -1;
    unsigned int mx = 0, my = 0;
    int cnt = 0;
   /* vector<unsigned int> neighbours = getNeighbours(map, neighbours);

    for( auto i: neighbours ){
        if( map->isFree(i) && !ant->isVisited(i)){
            // ROS_INFO("NEIGHBOURS %d in choseBestNext is Free", i);
            map->indexToCells(i, mx, my);
            if( value_best < total_[mx][my] ){ // Total values are always >= 0
                value_best = total_[mx][my];
                nxtCell = i;
            }
        }
        //else
        //cnt++;
    }
    if( nxtCell == -1 ){

    }
    if( cnt == neighbours.size() ){
        return -1;
    } */
    return nxtCell;
}
double ACO_v2::euclideanDistance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2){
    return 1/sqrt(pow(x2-x1, 2) + pow(y2-y1, 2)+2);//计算两点之间距离
}
void ACO_v2::placeAnts(OccupancyGridMap* map, unsigned int startCellID){
    //ROS_INFO("Placing ants");
    for( auto k: colony_ ){
        // Clear the path
        //ROS_INFO("Size colony %zu", colony_.size());
        //ROS_INFO("VISITED PRIMA DI CLEAR %zu", k.getVisited().size());
        k->getConstructedPath()->clearPath();//清理每只蚂蚁的路径，为下一次迭代作准备
        k->clearVisited();//清理记录蚂蚁已经访问的栅格标记
        k->getConstructedPath()->setCost(0);//设置蚂蚁走过的路径长度为0
        k->setSearchFalse();//设置记录蚂蚁是否正在寻找路径的参数为0

        //ROS_INFO("CLEARED PATH LENGTH %zu, VISITED CLEARED %zu", k->getConstructedPath()->getPath().size(),  k->getVisited().size());
        // Place the ant at start cell
        k->getConstructedPath()->insertCell(map, startCellID);//将开始点放入记录蚂蚁访问栅格的数组path_中
        k->addToVisited(startCellID);//将开始点放入到记录蚂蚁访问过的栅格数组visited_中
        k->setPosition(startCellID);//将蚂蚁的现在的位置设置在开始位置
        k->setSearchTrue();//将记录蚂蚁是否寻找过路径的参数设置为1，表示蚂蚁正在寻找路径
        //ROS_INFO("Formica %d piazzata in pos %d", k.getName(), k.getPosition());
        // ROS_INFO("START CELL AGGIUNTA %d", startCellID);
        //ROS_INFO("PATH AFTER FIRST CELL %zu cell inserted %d", k.getConstructedPath()->getPath().size(),  k.getConstructedPath()->getPath()[0]);
    }
}

/***** PHEROMONE + HEURISTIC *****/
void ACO_v2::pheromoneTrailUpdate(OccupancyGridMap* map, unsigned int goalCellID) {
    // Coordinates of the cell
    unsigned int mx = 0;
    unsigned int my = 0;
    // Pheromone evaporation
    for( int i = 0; i < map->getHeight(); i++ ){
        for( int j = 0; j < map->getWidth(); j++ ){
            pheromoneMatrix_[i][j] *= (1 - evaporationRate_);//计算每个栅格上迭代后剩余的信息素
        }
    }

    // Ant System pheromone trail deposit
    asUpdate(map);//更新栅格的信息素 = 剩余信息素+蚂蚁走过分泌的信息素

    // Compute combined information of pheromone trail and heuristic
    //computeTotalInformation(map, goalCellID);
}
void ACO_v2::asUpdate(OccupancyGridMap* map) {
    for( auto k: colony_ ){
        if(k->getConstructedPath()->getPath().size()!=0)
            globalPheromoneUpdate(map, k);//计算每只蚂蚁在走过的路径上释放的信息素
    }
}
void ACO_v2::globalPheromoneUpdate(OccupancyGridMap* map, Ant* ant) {
    // Cells coordinates
    unsigned int mx = 0;
    unsigned int my = 0;
    // Reinforce only the edges used in ant's solution
    double d_tau = 1.0 / ant->getConstructedPath()->getCost();//得到蚂蚁走的路径长度的倒数

    //ROS_INFO("test");
    for( unsigned int i: ant->getConstructedPath()->getPath() ) {
        // Update mx and my
        map->indexToCells(i, mx, my);//转换成栅格的宽，高
        pheromoneMatrix_[mx][my] += d_tau;//信息素加上蚂蚁释放的信息素
    }
}
void ACO_v2::computeTotalInformation(OccupancyGridMap* map, unsigned int goalCellID) {
    unsigned int mx = 0, my = 0;
    map->indexToCells(goalCellID, mx, my);//计算目标点在栅格地图上的位置

    double heuristic = 0;//用于计算栅格上每一个点到目标点的距离
    for (unsigned int i = 0; i < map->getHeight(); i++) {
        for (unsigned int j = 0; j < map->getWidth(); j++) {
            // The heuristic information is the inverse of the cell distance to the goal
            if((i!=mx)&&(j!=my))
            {
                heuristic =euclideanDistance(i, j, mx, my);
                total_[i][j] = pow(pheromoneMatrix_[i][j], alpha_) * pow(heuristic, beta_);
            }
            else
            total_[i][j]=pow(pheromoneMatrix_[i][j], alpha_) * pow(100, beta_);
        }
    }
}

/***** PATH PLANNING *****/
void ACO_v2::constructSolutions(OccupancyGridMap* map, Ant* ant, unsigned int startCellID, unsigned int goalCellID) {

    unsigned int newCell = 0;//用于存储蚂蚁下一个位置
    vector<unsigned int> neighbours;//记录此时蚂蚁可以访问的栅格位置
    //ROS_INFO("ANT STARTING POINT %d", startCellID);
    ant->setSearchTrue();//将蚂蚁设置为正在寻找路径中
    //ROS_INFO("Ant %d is searching %d before construct",ant.getName(), ant.isSearching() );

    // Let the ant create the path
    while (ant->getPosition() != goalCellID ){//判断蚂蚁是否已经找到终点
        // Find free unvisited neighbours
        neighbours = getNeighbours(map, ant);//查找下一个可以走的点存入到一个数组中
        //ROS_INFO("Neighbours cells %zu", neighbours.size());
        // Choose neighbour
        if( !neighbours.empty() ) {
            newCell = chooseNeighbour(map, neighbours);//轮盘赌法选择蚂蚁的下一个位置
            //ROS_INFO("蚂蚁%d,选择的下一个栅格是%d",ant->getName(),newCell);
            // Insert new cell
            unsigned int previous=ant->getConstructedPath()->getPath().back();
            float lens= ant->getConstructedPath()->getCost()+map->getMoveCost(previous,newCell);
            ant->getConstructedPath()->setCost(lens);
            ant->getConstructedPath()->insertCell(map, newCell);//将下一个位置插入到路径中
            ant->setPosition(newCell);//将下一个位置设置为蚂蚁的现在位置
            ant->addToVisited(newCell);//将此位置设置为已经访问过
            //ROS_INFO("ADDED CELL %d", newCell);
        }
        else{ // If no neighbours have been found move the ant back of 5 positions. If this way it comes back to starting point delete it
            /* for( int i = 0; i < 5; i++ ){
                ant->removeVisited(ant->getConstructedPath()->getPath().rbegin()[1]);
                ant->getConstructedPath()->getPath().pop_back();
            }
            if ( ant->getConstructedPath()->getPath().empty() || ant->getConstructedPath()->getPath().back() == startCellID ) {
                ROS_INFO("ANT %d GOT STUCK", ant->getName());
                return;
            } */
            ant->getConstructedPath()->clearPath();//清除所有路径
            ant->setPosition(startCellID);//将蚂蚁设置到起点
            //ROS_INFO("蚂蚁%d被重置",ant->getName());
            return;//结束这只蚂蚁的路径搜索，等待下一轮规划
        }
    }
    ant->setSearchFalse();//设置记录蚂蚁搜索状态的变量为false表示已经找到路径
    ROS_INFO("已经找到路径，路径长度为%.2f",ant->getConstructedPath()->getCost());
    //ROS_INFO("Ant %d is searching %d", ant.getName(), ant.isSearching() );
    //ROS_INFO("Ant %d found solution ", ant->getName());
}

vector<unsigned int> ACO_v2::computePaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID) {
    setlocale(LC_ALL,"");
    //traceFile.open ("/home/fabio/catkin_ws/src/thesis_matherials/path_planning/ACO/ant_colony_path_planner/traceFile.txt");
    /* int notFree = 0;
    for(unsigned int i = 0; i < map->getHeight(); i++){
        for(unsigned int j = 0; j < map->getWidth(); j++){
            if(map->getCellCost(i, j) != 0) notFree++;
                //ROS_INFO("CELL %d %d free", i, j );
        }
    } */
    //ROS_INFO("ANT STARTING POINT %d", startCellID);

    // Number of iterations made. The termination condition for the external loop is reaching a certain number of iterations
    int iteration = 0;//迭代次数
    bestAntIndex = 0;//最佳蚂蚁
    //int test = 0;//记录蚂蚁走了多少栅格
    //ros::Time global = ros::Time::now();
    while( iteration < getIterationsLimit() ){
        // traceFile << "Iteration: " << iteration << endl;
        //ROS_INFO("Iteration %d", iteration);
        placeAnts(map, startCellID);
        //ROS_INFO("Placing ants");
        /*for( auto k: colony_ ){
            // Clear the path
            //ROS_INFO("Size colony %zu", colony_.size());
            ROS_INFO("VISITED PRIMA DI CLEAR %zu", k.getVisited().size());
            k->getConstructedPath()->clearPath();
            k->clearVisited();
            k->setSearchFalse();

            //ROS_INFO("CLEARED PATH LENGTH %zu, VISITED CLEARED %zu", k->getConstructedPath()->getPath().size(),  k->getVisited().size());
            // Place the ant at start cell
            k->getConstructedPath()->insertCell(map, startCellID);
            k->addToVisited(startCellID);
            k->setPosition(startCellID);
            k->setSearchTrue();

            //ROS_INFO("Formica %d piazzata in pos %d", k.getName(), k.getPosition());
            // ROS_INFO("START CELL AGGIUNTA %d", startCellID);
            //ROS_INFO("PATH AFTER FIRST CELL %zu cell inserted %d", k.getConstructedPath()->getPath().size(),  k.getConstructedPath()->getPath()[0]);
        }*/
        computeTotalInformation(map, goalCellID);
        for(auto & i : colony_) {
            //ros::Time start = ros::Time::now();
            // First construct solution for k ant
            //ROS_INFO("Ant %d is building a path", i->getName());
            // for( auto j:  i->getVisited() ){
            //    if (j){
            //        test++;
            //    }
            // }
            //ROS_INFO("Ant %d already visited %d", i->getName(), test);
            // test = 0;
            //ROS_INFO("ANT %d STARTING POINT %d", colony_[i].getName(), colony_[i].getConstructedPath()->getPath()[0]);
            constructSolutions(map, i, startCellID, goalCellID);//每只蚂蚁寻找到达终点的路径
            //ROS_INFO("Ant %d is searching in compute paths %d", colony_[i].getName(), colony_[i].isSearching());
        }
        //ROS_INFO("test");
        // Update statistics
        updateStatistics(map, iteration);//更新最优路径变量

        // Update pheromone trail
        //traceFile << "Map updated at iteration " << iteration << endl;
        pheromoneTrailUpdate(map, goalCellID);//更新信息素矩阵pheromoneMatrix_

        iteration++;
    }

    ROS_INFO("Ant %d found shortest path with length %zu", bestAnt.getName(), bestAnt.getConstructedPath()->getPath().size());
    return bestAnt.getConstructedPath()->getPath();
}
