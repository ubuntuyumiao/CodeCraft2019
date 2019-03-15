#ifndef __INCLUDE__H__
#define __INCLUDE__H__

#include <iostream>
#include <stdio.h>
#include <queue>
#include <fstream>
#include <cstring>
#include <stack>
#include <vector>
#include<algorithm>
#define MAX_ROAD    7000
#define MAX_CAR     15000
#define MAX_CROSS   100
#define INF 0x3f3f3f3f

#define MAX_LANE 10
#define MAX_LANE_LENGHT 30

#define RED "\033[31m" /*red*/
#define YELLOW "\033[33m" /*Green*/
#define RESET "\033[0m"
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

typedef enum drive_state
{
  still_stored, 
  wait_schedule,
  completed
};
typedef enum sche_direct
{
  straight, 
  left,
  right
};
typedef struct Magic_garage
{
    int road_id;
    std::vector<int> garage[2];
}Magic_garage;
typedef struct Car
{
    int id;
    int set;
    int goal;
    int max_speed;
    int set_time;
    int cross_path[MAX_CROSS];
    int now_road;
    int next_road;
    drive_state state;  //调度状态 ： 等待出发态 等待调度态 终止态
    sche_direct move_ori;
}Car;
typedef struct Road
{
    int id;
    int road_length;
    int limit_speed;
    int lane_num;
    int start;
    int end;
    int flag_twoway;
    int car_on_road;
    
    //定义： load[0]存放 start-->end方向道路 load[1]存放 end->start方向道路
    int load[2][MAX_LANE][MAX_LANE_LENGHT];
}Road;

typedef struct Cross
{
    int id;
    int road_id[4];
    
    int prior_uproad;
    int prior_rightroad;
    int prior_downroad;
    int prior_leftroad;   //该段数据供所有通过这路口的车辆查阅 对比优先级
    
    
    int up_cross_id;
    int right_cross_id;
    int down_cross_id;
    int left_cross_id;
    
    bool up_cro_to_me_isempty;
    bool right_cro_to_me_isempty;
    bool down_cro_to_me_isempty;
    bool left_cro_to_me_isempty;
    
    
}Cross;
typedef struct road_empty
{
    bool is_empty;// 是否空标志
    int  lane;    // 非空的话优先级最高的有容量的车道
    int  offset;    // 非空的话优先级最高的有容量的车道 的倒数第几个车位
}road_empty;

typedef struct Global
{
    bool all_car_iscompleted;// 是否空标志
    int car_inroad_iscompleted;    // 非空的话优先级最高的有容量的车道
}Global;

typedef struct MGraph
{
    int edges[MAX_ROAD][MAX_ROAD];//邻接矩阵，记录的是两点之间的距离，也就是权值 
    int n,e;//顶点数和边数
}MG;


//检查该道路的车是否都已是终止态，且返回非终止态车数量
int has_car_wait_inroad(Cross* cross_,Road* road_,Car *car_array_);
// 如果该道路在位置上最靠前 且为终止态 ，且下一个时刻即将过路口，将其信息发送到其下一个路口公告字段
void update_to_cross(Car* car_,Road* road_,Cross* cross_);
//检查某道路的神奇车库是否有车 输入参数 当前调度的路口 和道路
bool check_garage(Cross* cross_,Road* road_,Magic_garage* garage_);
//检查此车是否在等待列表中出现
bool check_in_list(int car_id_,std::vector<int>wait_list_);
//全局车辆的两种判断 1、所有车辆是否全是终止态 2、所有已上路车辆全是终止态 
Global All_car_iscompleted(Car *car_array,int min_car_id_,int max_car_id_);
//检查某道路是否为空 不为空 那最高优先级的有余量的车道是哪条 最优先车位的下标？
road_empty check_road_empty(Cross *cur_cross_,Road *cur_road_);
void print_time(const char * const head);
//Astart寻路
bool Astar_search(Car *car_,Road* road_array_,int min_road_id,int max_road_id,Cross* cross_array_,int min_cross_id,int max_cross_id,int (*weight_)[MAX_CROSS]);
int campare_settime(const void * a, const void * b);
//发车排序
void quickSortOfCpp(Car* car_list,int car_begin,int car_end);
bool cmp(int a,int b);
#endif

