#ifndef __INCLUDE__H__
#define __INCLUDE__H__

#include <stdio.h>
#include <queue>
#include <fstream>
#include <cstring>
#include <stack>
#include <vector>
#include<algorithm>

// #define  CAR_NUM   40960
// #define  ROAD_NUM   162
// #define  CROSS_NUM  100
#define  CAR_NUM   10240
#define  ROAD_NUM   105
#define  CROSS_NUM  64

#define INF 0x3f3f3f3f
#define MAX_LANE 10
#define MAX_LANE_LENGHT 30


struct System_Para
{

	double T1_roadlenghtspace_w ;              //第一次规划时道路空间影响权值系数 
	double car_willonroad;                      //规划时路径信息浓度影响系数
	
        int max_car_road  ;            //地图某时刻允许最大车辆
	int T_SOFT  ;                  //允许车辆递增开始时间
	double T_SOFT_RATE  ;          //增长系数

	double normalize_length_w ;    //道路长度归一化后的放大系数（还是要作为是否选择道路的大头）
	double speed_near_w ;          //限速与自身速度差值对权值的影响系数
	double road_percent ;          //道路最大占有率
	double DECAY ;                 //占有率衰减系数
	double min_road_per ;          //道路最小占有率
	   
	double length_di_speed;
	double best_space_w;  
	double car_onroad_w;   
	double garage_size_w; 
};

typedef enum drive_state
{
  still_stored, //还在车库
  wait_schedule, //某时刻在路上等待调度
  completed,     //某时刻已经调度完成
  reached        //到达终点
}drive_state;
typedef enum sche_direct
{
  go_straight, 
  turn_left,
  turn_right
}sche_direct;
 struct MGraph
{
    int edges[CROSS_NUM][CROSS_NUM];//邻接矩阵，记录的是两点之间的距离，也就是权值 
    int cross_num;
    int road_num;  //顶点数和边数
    
    int set[CROSS_NUM];
    int dist[CROSS_NUM];
    int path[CROSS_NUM];    
    int goal_path[CROSS_NUM];
};
 struct DIJK_map
{
    int map[CROSS_NUM][CROSS_NUM][CROSS_NUM];
};
typedef struct how_tonext
{
  int next_road;
  sche_direct direct;
}how_tonext;
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
    int cross_path[CROSS_NUM];
    int now_road;
    int next_road;
    bool wait_anthor;
    int route;
    bool not_allow_research;
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
    bool completed;
    int car_onroad;
    int car_willonroad;
    int will_gocross;
    float best_space_per;
    int front_car;
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

}Cross;
typedef struct sch_pos
{
    int  road_completed; 
    int  lane;    
    int  offset;       //车辆所在车位下标  
    int  next_road;
    bool  block;
}sch_pos; 

typedef struct road_space
{
    bool is_empty;  // 是否空标志
    int  lane;      // 非空的话优先级最高的有容量的车道
    int  offset;    // 非空的话优先级最高的有容量的车道 的倒数第几个车位
}road_space;


typedef struct drive_toroad
{
    int  no_drive;    //  0:被等待态车阻挡  1：被终止钛车阻挡
    int  lane;      //被终止态车阻挡的话  是否能有余量进入该道路   -1：没有   非-1：有 对应车道
    int  offset;    //对应下一条路的
}drive_toroad;



typedef struct pior_cross
{
  int road_id;
  int dir;
}pior_cross;


int cross_tosub(int cross_id_,std::vector<int>&cross_dict_);
int car_tosub(int car_id_,std::vector<int>&car_dict_);
int road_tosub(int road_id_,std::vector<int>&road_dict_);
void init_MGraph(struct MGraph &g) ;
void init_Para(struct System_Para &para_) ;
void dijk_insert(struct MGraph &g,int u, int v, int w);
int dijk_search(struct MGraph &g, int from,int to,Car* car_,
		std::vector<int>&road_dict_ ,std::vector<int>&cross_dict_,Road* road_array_,
		Road map_[][CROSS_NUM],struct System_Para &para_);

int dijk_research(struct DIJK_map &dijk_map,int pre_cross_sub_,struct MGraph &g, int from,int to,int now_road_sub_, 
		  Car* car_,Cross* cross_array_,Road* road_array_,std::vector<int>&road_dict_,
		  std::vector<int>&cross_dict_,Road map_[][CROSS_NUM],int ori_sub, struct System_Para &para_);
int update_map(struct DIJK_map &dijk_map,int from,int to,struct MGraph &g,struct System_Para &para_);
int not_equal(int a,int b);
int min(int a, int b);
void out(std::string s);
void print_time(const char * const head);
void quickSort(Car* car_list,int car_begin,int car_end);

//路口优先级对比
int compare_prior_sch(Car* car_array_,std::vector<int>&car_dict_, Cross* cross_,Road* road_array,Road* road_,Cross *cross_array);

//通过当前所在道路输出下一条道路 以及转向关系
how_tonext next_road(Car* car_,Road* cur_road,Cross* cross_array_,
		     std::vector<int>&cross_dict_,Cross* cross_,Cross* next_cross_,Road map_[][CROSS_NUM]);

how_tonext next_road_drive(Car* car_,Road* cur_road,Cross *cross_array_,std::vector<int>&cross_dict_,
		     Cross* cross_,Road map_[][CROSS_NUM]);
// 将所有终止态的车改为等待态
void chang_completed_towait(Car *car_array_,std::vector<int>&wait_list_,std::vector<int>&car_dist_);

// 如果该道路在位置上最靠前 且为终止态 ，且下一个时刻即将过路口，将其信息发送到其下一个路口公告字段
void update_to_cross(Car* car_,Road* road_,Cross* cross_,Cross *cross_array_);

bool clear_cross_proir(Car* car_,Road* road_,Cross* cross_);

//检查此车是否在等待列表中出现
bool check_in_list(int car_id_,std::vector<int> &wait_list_); 

//查找并删除
bool check_and_delete(int car_id,std::vector<int> &wait_list_);

// 为进入该道路的车辆提供数据   注意： 此次检查发生在调度道路的车库 所以 车道为驶离路口
road_space check_road_space(Cross *cur_cross_,Road *cur_road_);

//输入参数 当前路口  目标道路
drive_toroad check_road_drive_space(Cross *cur_cross_,Road *road_,Car* car_array_,
				    std::vector<int>&car_dict_, std::vector<int>&road_dict_,int max_offset);
void dis_cusroad(int road_sub,Road* road,std::vector<int>car_dict,Car* car,int T);
// 注意：发车与调度也不一样
void update_to_cross_drive(Car* car_,Road* road_,Cross* cross_);

void init_waitanthor(Car* car_array,std::vector<int>&car_dict_,Road* road_array_,
		     std::vector<int>&road_dict_);

bool write_output(std::string path,Car *car_array_,int car_num_,Road map_[][CROSS_NUM],std::vector<int>&cross_dict_);

int campare_settime(const void * a, const void * b);


//发车排序
void quickSortOfCpp(Car* car_list,int car_num_);

void map_matrix(Cross* cross_array_,std::vector<int>&cross_dict_,std::vector<int>&road_dict_
	        ,Road* road_array_,Road map_[][CROSS_NUM],struct MGraph &dijk_graph,int max_length,int min_length
	       ,struct System_Para &para_);

bool read_file(std::string cross_path, Cross *cross_array_,Cross *cross_sortedarray_,int* min_cross_id_,int* max_cross_id_,
	       std::string road_path, Road *road_array_,Road *road_sortedarray_,int* min_road_id_,int* max_road_id,
	       std::string car_path,Car *car_array_,Car *car_sortedarray_,int* min_car_id_,int* max_car_id_,Road map_[][CROSS_NUM]
	      ,std::vector<int>&car_dict_,std::vector<int>&road_dict_,std::vector<int>&cross_dict_,
	      int* min_roadlength_,int* max_roadlength_
	      );
void deal_with_car(std::vector<int>&car_dict_,Car* car_array_,Car* car_sortedarray_);
void debug_dir_tocross(std::vector<int>&road_dict_,Road *road_array_,std::vector<int>&cross_dict_,Cross *cross_array_,struct MGraph &dijk_graph);

//路口调度 获得对应道路等待态优先级最高的车（不含已调度过 wait_another的车）
sch_pos sch_most_prior(Car *car_array_,std::vector<int>&car_dict_,Road* road_,
		       std::vector<int>&road_dict_,Cross* cross_,std::vector<int>&cross_dict_,int offset,
		       int cur_road[],Cross *cross_array_,Road* road_array_,Road map_[][CROSS_NUM],
		       std::vector<int> &wait_list_,std::vector<int> &block_list_,int T_,
		       int *reached_car_,int *wait_num_);

void ready_garage(std::vector<int>&car_dict_,std::vector<int>&road_dict_,std::vector<int>&cross_dict_,
		  Cross* cross_array,
		   Magic_garage* garage_,Road* road_array_,Car *car_array_,Car *car_sortedarray_,Road map_[][CROSS_NUM]);

//检查是否有车未到终点
bool All_car_isreached(Car* car_array,int car_num_);

void sch_allcross_garage(Car* car_array,
			 Cross* cross_array_,std::vector<int>&cross_dict_,
			 Road* road_array,std::vector<int>&road_dict_,std::vector<int>&car_dict_,
			 Magic_garage* garage,
			 Road map_[][CROSS_NUM],
			 int T,int *wait_num_,int *reached_car_,int car_num_,
			 struct MGraph &dijk_graph,
			 int* min_roadlength,int* max_roadlength,
			 struct System_Para &para_
			);

bool sch_allcross_drive(Car* car_array,std::vector<int>&car_dict_,
			 Cross* cross_array,std::vector<int>&cross_dict_,
			 Road* road_array,std::vector<int>&road_dict_,
			 Magic_garage* garage,
			 Road map_[][CROSS_NUM],
			 int T,
			 std::vector<int> &wait_list_,
			 std::vector<int> &bloack_list_,int *reached_car_,int *wait_num_
			);
bool print_map(int coord_max,Car* car_);
#endif

