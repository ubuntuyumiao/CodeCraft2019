#include "io.h"

#define _DEBUG

#define INLINE  static __inline
#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif
//在调度车库时的检查  注意： 要与调度路口车辆区分
bool check_most_prior(int car_id,Road* road_,Cross* cross_)
{
  for(int i=0;i<road_->lane_num;i++)
    for(int j=0;j<road_->road_length;j++)
    {
      if(road_->load[(cross_->id==road_->start)?0:1][i][j]!=0)   //查询该车位是否有车
      {
	if(road_->load[(cross_->id==road_->start)?0:1][i][j]==car_id) return true;
	  else { return false;} 
      }
    }
    return true;
}
//通过当前所在道路输出下一条道路 以及转向关系转向  注意：调度车库与调度路口方向不同
how_tonext next_road(Car* car_,Road* cur_road,Cross *cross_array_,Road map_[][MAX_CROSS])
{
  how_tonext tonext;
  tonext.next_road=-1;
  tonext.direct=go_straight;
  int cur_corss,next_cross,last_cross;
  for(int i=0;i<MAX_CROSS;i++)
  {
    if(car_->cross_path[i+2]==0) {return tonext;break;}
    //通过下个路口与下下个路口算出转换关系
    if(map_[car_->cross_path[i]][car_->cross_path[i+1]].id==cur_road->id)
    {
      //car_->cross_path[i+2]   car_->cross_path[i+1]
      tonext.next_road=map_[car_->cross_path[i+1]][car_->cross_path[i+2]].id;
      cur_corss=car_->cross_path[i] ;  next_cross=car_->cross_path[i+1] ; next_cross=car_->cross_path[i+2] ;
      if(cur_corss==cross_array_[next_cross].up_cross_id) 
      {
	 if(last_cross==cross_array_[next_cross].right_cross_id) {tonext.direct=turn_left;return tonext;}
	 else if(last_cross==cross_array_[next_cross].down_cross_id) {tonext.direct=go_straight;return tonext;}
         else if(last_cross==cross_array_[next_cross].left_cross_id) {tonext.direct=turn_right;return tonext;}
      }
      else if(cur_corss==cross_array_[next_cross].right_cross_id) 
      { 
	if(last_cross==cross_array_[next_cross].up_cross_id) {tonext.direct=turn_right;return tonext;}
	 else if(last_cross==cross_array_[next_cross].down_cross_id) {tonext.direct=turn_left;return tonext;}
         else if(last_cross==cross_array_[next_cross].left_cross_id) {tonext.direct=go_straight;return tonext;}
      }
      else if(cur_corss==cross_array_[next_cross].down_cross_id) 
      {
        if(last_cross=cross_array_[next_cross].right_cross_id) {tonext.direct=turn_right;return tonext;}
	 else if(last_cross==cross_array_[next_cross].up_cross_id) {tonext.direct=go_straight;return tonext;}
         else if(last_cross==cross_array_[next_cross].left_cross_id) {tonext.direct=turn_left;return tonext;}
      }
      else if(cur_corss==cross_array_[next_cross].left_cross_id) 
      {
         if(last_cross==cross_array_[next_cross].right_cross_id) {tonext.direct=go_straight;return tonext;}
	 else if(last_cross==cross_array_[next_cross].down_cross_id) {tonext.direct=turn_right;return tonext;}
         else if(last_cross==cross_array_[next_cross].left_cross_id) {tonext.direct=turn_left;return tonext;}
      }
    }
  }
  return tonext;
}
// 将所有终止态的车改为等待态
void chang_completed_towait(int min_car_id_,int max_car_id_,Car *car_array_,std::vector<int> *wait_list_)
{
  for(int sub=min_car_id_;sub<=max_car_id_;sub++)
  {
    if(car_array_[sub].state==completed)
    {
      car_array_[sub].state=wait_schedule;
      wait_list_->push_back(sub);
    }
  }
}
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
void update_to_cross(Car* car_,Road* road_,Cross* cross_,Cross *cross_array_)
{
     int next_cross;
     if(cross_->id==road_->start) next_cross=road_->end;
      else next_cross=road_->start;
     
      if(cross_->id=cross_array_[next_cross].up_cross_id) cross_array_[next_cross].prior_uproad=car_->id;
	else if(cross_->id=cross_array_[next_cross].down_cross_id) cross_array_[next_cross].prior_downroad=car_->id;
	  else if(cross_->id=cross_array_[next_cross].left_cross_id) cross_array_[next_cross].left_cross_id=car_->id;
	    else if(cross_->id=cross_array_[next_cross].right_cross_id) cross_array_[next_cross].prior_rightroad=car_->id;
}

//检查此车是否在等待列表中出现
bool check_in_list(int car_id_,std::vector<int>wait_list_)
{
  if(wait_list_.empty()) { std::cout << YELLOW << "Logitic Warning !" <<std::endl; return false; }
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
// 0316增加： 若非空 返回优先级最高的车的id
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
	       if(space_situation.is_empty) space_situation.is_empty=false;        break;
	    }
           sub[j]++;
         }         
       if(space_situation.is_empty) 
       {       
	 space_situation.lane=0;    
         space_situation.offset=0;  
	 return space_situation; 
      }
       //路上一定有车
       for(int k=0;k<cur_road_->lane_num;k++)
       {
	 //该行有空位
	 if(sub[k]!=-1)   
	 {
	   space_situation.lane=k;
	   if((sub[k]+1)>cur_road_->limit_speed){ space_situation.offset= cur_road_->road_length-cur_road_->limit_speed-1;}
	     else {  space_situation.offset=cur_road_->road_length-sub[k]-1;}
	   break; 
	}           
       }
      
return space_situation; 
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
bool sch_allcross_drive(Car* car_array,
			 Cross* cross_array_,int min_cross_id,int max_cross_id,
			 Road* road_array,
			 Magic_garage* garage,
			 Road map_[][MAX_CROSS],
			 int T,
			 std::vector<int> *wait_list_
			)
{
         //检锁表
       std::vector<int> block_list;  
       for(int sch_cross_drive=min_cross_id;sch_cross_drive<=max_cross_id;sch_cross_drive++)
       {
	 std::cout<<std::endl;
	    //因为道路上的所有车都已是终止态 所以先调度那个方向的都可以 如果发车的可行驶距离超过道路长度 直接放到路口
	    /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
	    int cur_cross_road[4];
	    std::memcpy(cur_cross_road,cross_array_[sch_cross_drive].road_id,sizeof(cross_array_[sch_cross_drive].road_id));
	    int array_offset=3;
	  for(int i=0;i<4;i++)
	    if(cur_cross_road[i]!=-1)
	      if(road_array[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口不是起点
		if(sch_cross_drive!=road_array[cur_cross_road[i]].start)  cur_cross_road[i]=-1;
	    std::sort(cur_cross_road,cur_cross_road+4);    
	  //去掉不存在的道路 或者不调度 不进入该交叉口的道路
	    for(;array_offset>=0;array_offset--)
	    {
	      if((array_offset==0)&&(cur_cross_road[array_offset]!=-1)) break;
	      if(cur_cross_road[array_offset]==-1) { array_offset+=1;  break;}
	    } 
	  /******需要调度的道路id升序存于cur_cross_road中    起始下标为 array_offset******/
	    for(int sch_road_drive_offset=array_offset;sch_road_drive_offset<4;sch_road_drive_offset++)
		  {    
		    int sch_road_garage = cur_cross_road[sch_road_drive_offset];
		    
		    
		    
		  }
	 
	 
	 return true;
       }
  
}
void sch_allcross_garage(Car* car_array,
			 Cross* cross_array_,int min_cross_id,int max_cross_id,
			 Road* road_array,
			 Magic_garage* garage,
			 Road map_[][MAX_CROSS],
			 int T
			)
{   
  	      for(int sch_cross_garage=min_cross_id;sch_cross_garage<=max_cross_id;sch_cross_garage++)
		 {
		   std::cout<<std::endl;
	            //因为道路上的所有车都已是终止态 所以先调度那个方向的都可以 如果发车的可行驶距离超过道路长度 直接放到路口
		    /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
		    int cur_cross_road[4];
		    std::memcpy(cur_cross_road,cross_array_[sch_cross_garage].road_id,sizeof(cross_array_[sch_cross_garage].road_id));
		    int array_offset=3;
		  for(int i=0;i<4;i++)
		    if(cur_cross_road[i]!=-1)
		      if(road_array[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口不是起点
			if(sch_cross_garage!=road_array[cur_cross_road[i]].start)  cur_cross_road[i]=-1;
		    std::sort(cur_cross_road,cur_cross_road+4);    
		  //去掉不存在的道路 或者不调度 不进入该交叉口的道路
		    for(;array_offset>=0;array_offset--)
		    {
		      if((array_offset==0)&&(cur_cross_road[array_offset]!=-1)) break;
		      if(cur_cross_road[array_offset]==-1) { array_offset+=1;  break;}
		    } 
		  /******需要调度的道路id升序存于cur_cross_road中    起始下标为 array_offset******/
// 		  std::cout <<sch_cross_garage<<" connect: ";
		  for(int sch_road_garage_offset=array_offset;sch_road_garage_offset<4;sch_road_garage_offset++)
		  {   
		    
		    int sch_road_garage = cur_cross_road[sch_road_garage_offset]; 	
		    //判断该路的车库是否有车调度
		    int cur_dup=not_equal(sch_cross_garage,road_array[sch_road_garage].start);
		     if(garage[sch_road_garage].garage[cur_dup].empty())
		       continue; 
		      else
			 {   
			      //检查道路最高优先级车道 以及偏移量
			      road_space space_Condition = check_road_space(&cross_array_[sch_cross_garage],&road_array[sch_road_garage]);
// 			       std::cout << "test: "<<space_Condition.lane <<std::endl;
			      if(space_Condition.lane==-1) continue;
// 			      std::cout<<"sch_road_garage "<< sch_road_garage<< " "<<cur_dup;
			      //道路是否有余量
			      int cur_dup=not_equal(sch_cross_garage,road_array[sch_road_garage].start);
			      //循环的条件 道路非满且车库非空
			      while((space_Condition.lane!=-1)&&(!garage[road_array[sch_road_garage].id].garage[cur_dup].empty())
				&&car_array[garage[road_array[sch_road_garage].id].garage[cur_dup][0]].set_time<=T
			      )
			      {
				    int cur_dup=not_equal(sch_cross_garage,road_array[sch_road_garage].start);
				    std::vector<int> car_garage(garage[road_array[sch_road_garage].id].garage[cur_dup]); 
				    //向量拷贝 只是为了让后面看起来短一点 没什么太大软用
				    //判断发车时刻.<=T
// 				    if(car_array[car_garage[0]].set_time>T) 
// 				    {
// // 				      printf("%d 号车发车时刻为 %d 现在时刻为 %d\n",car_garage[0],car_array[car_garage[0]].set_time,T);
// 				      break;
// 				    } else std::cout << " "<<car_array[car_garage[0]].id;
// 			            std::cout<<car_garage[0]<< "  ";  //将要出发车辆的id
				    //判断该车速度与可行驶距离大小 
				    //D=min（car_speed,offset）
				    ;
				    int how_far = min(road_array[sch_road_garage].road_length-space_Condition.offset,
						      min(car_array[car_garage[0]].max_speed,road_array[sch_road_garage].limit_speed));
				    int space_offset = road_array[sch_road_garage].road_length-how_far;
				      if(space_offset<0) space_offset=0;
				    /**** 终于将车安排上了 更新车道数组****/
				    road_array[sch_road_garage].load[cur_dup][space_Condition.lane][space_offset]=car_garage[0];
				    /**** 终于将车安排上了 更新车道数组****/
				    /**** 上路后还有一系列操作 比如 ***/
				      /***写car结构体中的state now_road next_road move_ori settime ****/
				    car_array[car_garage[0]].set_time=T;
				    car_array[car_garage[0]].now_road=sch_road_garage;
				    car_array[car_garage[0]].state=completed;
				      /***如果下一时刻即将过路口 则更新路口公共字段****/
				      //现在所在位置-最大可行驶距离 
				    if(((space_offset-how_far)<0)&&(check_most_prior(car_garage[0],&road_array[sch_road_garage],&cross_array_[sch_cross_garage])))
				     {
				      
				       //检查是否是该道路优先级最高 是则将其信息发送到公共字段 否则不发
// 				       std::cout << sch_cross_garage<<" "<<car_garage[0]<<" " <<"will run out "<< sch_road_garage<<" "<<T+1 ; 
				       how_tonext to_next=next_road(&car_array[car_garage[0]],&road_array[sch_road_garage],cross_array_,map_);
// 				       std::cout<<sch_cross_garage<<" "<< sch_road_garage<<" "<< car_garage[0]<<" ";
// 				       std::cout << "run to "<<to_next.next_road<<" ";
				       if(to_next.next_road==-1){ 
					 car_array[car_garage[0]].move_ori =go_straight;
					 car_array[car_garage[0]].next_road=sch_road_garage;
					 std::cout << "Attention 1" <<std::endl;
				       }
					else{
					  if(to_next.next_road==-1){std::cout<<"Attetion 2"<<std::endl;}
					   else{
						car_array[car_garage[0]].move_ori =to_next.direct ;
					        car_array[car_garage[0]].next_road=to_next.next_road ;
						update_to_cross(&car_array[car_garage[0]],&road_array[sch_road_garage],&cross_array_[sch_cross_garage],cross_array_);
					   }
					}
				     }
				     else{
				       car_array[car_garage[0]].next_road= sch_road_garage;
				       car_array[car_garage[0]].move_ori = go_straight;
				     }

				    //发车成功  将其从车库中删掉
			  	   garage[road_array[sch_road_garage].id].
				    garage[cur_dup].erase(garage[road_array[sch_road_garage].id].
					garage[cur_dup].begin());
				      
				space_Condition = check_road_space(&cross_array_[sch_cross_garage],&road_array[sch_road_garage]);
			      }
			      //如果道路已经无车位驶入 调度下一道路车库
			       continue;
		  }
		  std::cout<<std::endl;
		  }
		}
}
void debug_dir_leavecross(Road *road_array_,int min_cross_id,int max_cross_id,Cross *cross_array_)
{
  	      for(int test_cross=min_cross_id;test_cross<=max_cross_id;test_cross++)
	      {
		/******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
		int cur_cross_road[4];
		std::memcpy(cur_cross_road,cross_array_[test_cross].road_id,sizeof(cross_array_[test_cross].road_id));
		int array_offset=3;
	      for(int i=0;i<4;i++)
		if(cur_cross_road[i]!=-1)
		  if(road_array_[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口不是起点
		    if(test_cross!=road_array_[cur_cross_road[i]].start)  cur_cross_road[i]=-1;
		std::sort(cur_cross_road,cur_cross_road+4);    
	      //去掉不存在的道路 或者不调度 不进入该交叉口的道路
		for(;array_offset>=0;array_offset--)
		{
		  if((array_offset==0)&&(cur_cross_road[array_offset]!=-1)) break;
		  if(cur_cross_road[array_offset]==-1) { array_offset+=1;  break;}
		} 
		for(int test_offset=array_offset;test_offset<4;test_offset++)
	      { 
		int test_road = cur_cross_road[test_offset];
		if(road_array_[test_road].start==test_cross)
		 std::cout <<cross_array_[test_cross].id<<"-------"<<cur_cross_road[test_offset]<<"------->"<<road_array_[test_road].end<<std::endl;
		else std::cout <<cross_array_[test_cross].id<<"-------"<<cur_cross_road[test_offset]<<"------->"<<road_array_[test_road].start<<std::endl;
		for(int lane=0;lane<road_array_[test_road].lane_num;lane++)
		{
		  for(int j=road_array_[test_road].road_length-1;j>=0;j--)
		  { 
		    std::cout<< road_array_[test_road].load[(test_cross==road_array_[test_road].start)?0:1][lane][j]<<"  "; 
		  }
		  std::cout<<std::endl;
		}
		if(road_array_[test_road].start==test_cross)
		 std::cout <<cross_array_[test_cross].id<<"-------"<<cur_cross_road[test_offset]<<"------->"<<road_array_[test_road].end<<std::endl;
		else std::cout <<cross_array_[test_cross].id<<"-------"<<cur_cross_road[test_offset]<<"------->"<<road_array_[test_road].start<<std::endl;
		 std::cout<<std::endl<<std::endl;
	      }
	      std::cout<<std::endl;
	      }
	      
	      	for(int i=min_cross_id;i<=max_cross_id;i++)
	{std::cout<< "Cross: "<<i<< " "<<  cross_array_[i].prior_uproad <<" "
	     <<  cross_array_[i].prior_downroad<<" "
	     <<  cross_array_[i].prior_leftroad<<" "
	     <<  cross_array_[i].prior_rightroad ;
         std::cout<<std::endl;}
}
void debug_dir_tocross(Road *road_array_,int min_cross_id,int max_cross_id,Cross *cross_array_)
{
    	      for(int test_cross=min_cross_id;test_cross<=max_cross_id;test_cross++)
	      {
		/******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
		int cur_cross_road[4];
		std::memcpy(cur_cross_road,cross_array_[test_cross].road_id,sizeof(cross_array_[test_cross].road_id));
		int array_offset=3;
	      for(int i=0;i<4;i++)
		if(cur_cross_road[i]!=-1)
		  if(road_array_[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口不是起点
		    if(test_cross!=road_array_[cur_cross_road[i]].start)  cur_cross_road[i]=-1;
		std::sort(cur_cross_road,cur_cross_road+4);    
	      //去掉不存在的道路 或者不调度 不进入该交叉口的道路
		for(;array_offset>=0;array_offset--)
		{
		  if((array_offset==0)&&(cur_cross_road[array_offset]!=-1)) break;
		  if(cur_cross_road[array_offset]==-1) { array_offset+=1;  break;}
		} 
		for(int test_offset=array_offset;test_offset<4;test_offset++)
	      { 
		int test_road = cur_cross_road[test_offset];
		if(road_array_[test_road].end==test_cross)
		 std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"-------"<<road_array_[test_road].start<<std::endl;
		else std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"-------"<<road_array_[test_road].end<<std::endl;
		for(int lane=road_array_[test_road].lane_num-1;lane>=0;lane--)
		{
		  for(int j=0;j<road_array_[test_road].road_length;j++)
		  { 
		    std::cout<< road_array_[test_road].load[(test_cross==road_array_[test_road].end)?0:1][lane][j]<<"  "; 
		  }
		  std::cout<<std::endl;
		}
		if(road_array_[test_road].end==test_cross)
		 std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"-------"<<road_array_[test_road].start<<std::endl;
		else std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"-------"<<road_array_[test_road].end<<std::endl;
	
		 std::cout<<std::endl<<std::endl;
	      }
	      std::cout<<std::endl;
	      }
	      	      	for(int i=min_cross_id;i<=max_cross_id;i++)
	{std::cout<< "Cross: "<<i<< " "<<  cross_array_[i].prior_uproad <<" "
	     <<  cross_array_[i].prior_downroad<<" "
	     <<  cross_array_[i].prior_leftroad<<" "
	     <<  cross_array_[i].prior_rightroad ;
         std::cout<<std::endl;}
}