#include "io.h"

#define _DEBUG

#define INLINE  static __inline
#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif
//检查该道路的车是否都已是终止态，且返回非终止态 最高优先车id
// 0: 该车道全终止态   非0：该车道 非终止态 优先级最高的车
//返回两个元素的数组  分别为道路上等待调度的车的数量 以及 最高优先级的车辆id
int* has_car_wait_inroad(Cross* cross_,Road* road_,Car *car_array_)
{
       bool has_wait_car=false;
       bool this_line_findcar=false;
       int car_id=0;
       int ret[2]={0,0};
       for(int j=0;j<road_->lane_num;j++)
	for(int l=0;l<road_->road_length;l++)
         {
	    if(road_->load[(cross_->id==road_->end)?0:1][j][l]!=0)   //查询该车状态
	    {
	      if(car_array_[road_->load[(cross_->id==road_->end)?0:1][j][l]].state==wait_schedule)
	      {
		if(!has_wait_car)
	       {
		 has_wait_car=true;
		 car_id=road_->load[(cross_->id==road_->end)?0:1][j][l];
	       }
	       ret[0]++;
	      }
	    }
         }      
               ret[1]=car_id;  
  return ret;
}
// 如果该道路在位置上最靠前 且为终止态 ，且下一个时刻即将过路口，将其信息发送到其下一个路口公告字段
void update_to_cross(Car* car_array_,Road* road_,Cross* cross_)
{
  for(int i=0;i<road_->lane_num;i++)
  {
    for(int j=0;j<road_->limit_speed;j++)
      //可能冲出路口的范围内有车
      if(road_->load[(road_->end==cross_->id)?0:1][i][j]!=0)   
      {
	if(car_array_[road_->load[(road_->end==cross_->id)?0:1][i][j]].state==completed)
	{
	 if (road_->id==cross_->road_id[0])        cross_->prior_uproad= car_array_[road_->load[(road_->end==cross_->id?0:1)][i][j]].id  ;                //up
	 else if (road_->id==cross_->road_id[1])   cross_->prior_uproad= car_array_[road_->load[(road_->end==cross_->id?0:1)][i][j]].id  ;                   //right
	 else if (road_->id==cross_->road_id[2])   cross_->prior_uproad= car_array_[road_->load[(road_->end==cross_->id?0:1)][i][j]].id  ;                   //down
	 else if (road_->id==cross_->road_id[3])   cross_->prior_uproad= car_array_[road_->load[(road_->end==cross_->id?0:1)][i][j]].id  ;                   //left
	}
	//终止搜索
// 	  else 
	  { i=road_->lane_num;j=road_->limit_speed;}
      }
  }
}
//检查某道路的神奇车库是否有车 输入参数 当前调度的路口 和道路
bool check_garage(Cross* cross_,Road* road_,Magic_garage* garage_)
{
  return garage_[road_->id].garage[(cross_->id==road_->start?0:1)].empty();
}
//检查此车是否在等待列表中出现
bool check_in_list(int car_id_,std::vector<int>wait_list_)
{
  if(!wait_list_.empty()) { std::cout << YELLOW << "Logitic Warning !" <<std::endl; return false; }
  std::vector<int>::iterator it;
  it =find(wait_list_.begin(),wait_list_.end(),car_id_);
  if (it!=wait_list_.end()) return true;
    else  return false;
}
//全局车辆的两种判断 1、所有车辆是否全是终止态 2、所有已上路车辆全是终止态  
Global All_car_iscompleted(Car* car_array,int min_car_id_,int max_car_id_)
{
  Global global;
  global.all_car_iscompleted=true;
  global.car_inroad_iscompleted=true;
   for(int i=min_car_id_;i<=max_car_id_;i++)
    {
      //路上还有车等待调度
      if(car_array[i].state==wait_schedule)  
      { 
	global.car_inroad_iscompleted=false;
      }
      //所有车辆未调度完
     if((car_array[i].state==still_stored) ||(car_array[i].state==wait_schedule))
      { 
	global.all_car_iscompleted=false;
      }
      if((!global.car_inroad_iscompleted)&&(!global.car_inroad_iscompleted)) break;
    }
return global;
}
//检查是否有车未到终点 
bool All_car_isreached(Car* car_array,int min_car_id_,int max_car_id_)
{
  bool all_car_isreached=true;
   for(int i=min_car_id_;i<=max_car_id_;i++)
    {
      //路上还有车等待调度
      if(car_array[i].state!=reached)  
      { 
	all_car_isreached=false;
	break;
      }

    }
return all_car_isreached;
}
/******************************检查某道路是否为空 不为空 那最高优先级的有余量的车道是哪条 最优先车位的下标？******************************/
// 为进入该道路的车辆提供数据   注意： 此次检查发生在调度路上车辆 所以 车道为指向路口
road_empty check_road_empty(Cross *cur_cross_,Road *cur_road_)
{
       road_empty road_situation;
       road_situation.is_empty=true;
       road_situation.lane=-1;
       road_situation.offset=-1;
       bool this_line_findcar=false;
       int sub[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};  //存放每个车道最末尾的车
       for(int j=0;j<cur_road_->lane_num;j++)
	for(int l=cur_road_->road_length-1;l>=0;l--)
         {
	    if(cur_road_->load[(cur_cross_->id==cur_road_->end)?0:1][j][l]!=0)   //查询该车位是否有车
	    {
	       if(road_situation.is_empty) road_situation.is_empty=false;
	       break;
	    }
	    sub[j]++;
         }         
       if(road_situation.is_empty)  return road_situation;
       //路上一定有车
       for(int k=0;k<cur_road_->lane_num;k++)
       {
	 //该行有空位
	 if(sub[k]!=-1)   {road_situation.lane=k;road_situation.offset=cur_road_->road_length-sub[k]-1;break; }           
       }
       
return road_situation; 
}
// 为进入该道路的车辆提供数据   注意： 此次检查发生在调度道路的车库 所以 方向为驶离路口
// 0316 修正 ： 如果偏移量大于该段道路限速则等于限速
road_space check_road_space(Cross *cur_cross_,Road *cur_road_)
{
       road_space space_situation;
       space_situation.is_empty=true;
       space_situation.lane=-1;     //-1 代表该道路无车位
       space_situation.offset=-1;
       bool this_line_findcar=false;
       int sub[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};  //存放每个车道最末尾的车
       for(int j=0;j<cur_road_->lane_num;j++)
	for(int l=cur_road_->road_length-1;l>=0;l--)
         {
	    if(cur_road_->load[(cur_cross_->id==cur_road_->start)?0:1][j][l]!=0)   //查询该车位是否有车
	    {
	       if(space_situation.is_empty) space_situation.is_empty=false;
	       break;
	    }
	    sub[j]++;
         }         
       if(space_situation.is_empty)  return space_situation;
       //路上一定有车
       for(int k=0;k<cur_road_->lane_num;k++)
       {
	 //该行有空位
	 if(sub[k]!=-1)   
	 {
	   space_situation.lane=k;
	   if((sub[k]+1)>cur_road_->limit_speed) space_situation.offset= cur_road_->road_length-sub[k]-1;
	     else space_situation.offset=cur_road_->road_length-cur_road_->limit_speed;
	   break; 
	}           
       }
       
return space_situation; 
}

/******************************调度k号车辆后 更新道路******************************/
void car_change_raod(Car *car_,Road *road_)
{
  
}
/******************************车辆调度规则执行******************************/


bool Astar_search(Car *car_,Road* road_array_,int min_road_id,int max_road_id,Cross* cross_array_,int min_cross_id,int max_cross_id,int (*weight_)[MAX_CROSS])
{
    
    int  from=  car_->set  ;
    int	 to=  car_->goal     ;
    int i=0;
    A_star *a=new A_star(road_array_, min_road_id, max_road_id,cross_array_,min_cross_id, max_cross_id,weight_);
    
    
    node *start=new node(from);
    node *end=new node(to);
    
    a->search(start,end);
    if(a->find_path==true){
    while(!a->route_stack.empty()){
        car_->cross_path[i]= a->route_stack.top() ;
//	if(a->route_stack.top()!=to) std::cout<< a->route_stack.top()"--->";
	//打印栈顶元素，实现了顶点的逆序打印
	a->route_stack.pop();      
	i++;
	//出栈
    }
    }
//     std::cout << std::endl<<std::endl;
    return a->find_path;

}

void print_time(const char *head)
{
#ifdef _DEBUG
    struct timeb rawtime;
    struct tm * timeinfo;
    ftime(&rawtime);
    timeinfo = localtime(&rawtime.time);

    static int ms = rawtime.millitm;
    static unsigned long s = rawtime.time;
    int out_ms = rawtime.millitm - ms;
    unsigned long out_s = rawtime.time - s;
    ms = rawtime.millitm;
    s = rawtime.time;

    if (out_ms < 0)
    {
        out_ms += 1000;
        out_s -= 1;
    }
    printf("\n%s date/time is: %s \tused time is %lu s %d ms.\n", head, asctime(timeinfo), out_s, out_ms);
    
#endif
}
//将车辆的出发时间进行排序
int  campare_settime(const void * a, const void * b)
{
 return (*(Car *)a).set_time > (*(Car *)b).set_time ? 1 : -1; 
}

void quickSortOfCpp(Car* car_list,int car_begin,int car_end)
{
    qsort(car_list+car_begin, car_end-car_begin+1, sizeof(car_list[0]), campare_settime);
}
bool cmp(int a,int b){
    return a > b;
}
int min_ready_road_id(Road *road_,Cross *cross_)
{
   int up,right,down,left;
   if(cross_->up_cross_id ==-1)    up=-1;
   if(cross_->right_cross_id ==-1) right=-1;
   if(cross_->down_cross_id ==-1)  down=-1;
   if(cross_->left_cross_id ==-1)  left=-1;
}
//输入
int not_equal(int a,int b)
{
  if(a==b) return 0;
    else return 1;
}
int min(int a, int b)
{
  if(a>b) return b;
     return a;
}