#include "io.h"

// #define _DEBUG

#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif

int  campare_dir(const void * a, const void * b)
{
 return (*(pior_cross *)a).dir > (*(pior_cross *)b).dir ? 1 : -1; 
}
int  campare_id(const void * a, const void * b)
{
 return (*(pior_cross *)a).road_id > (*(pior_cross *)b).road_id ? 1 : -1; 
}
void quickSort(Car* car_list,int car_begin,int car_end)
{
    qsort(car_list+car_begin, car_end-car_begin+1, sizeof(car_list[0]), campare_settime);
}
//路口优先级对比
int compare_prior_sch(Car* car_array_,Cross* cross_,Road* road_array,Road* road_,Cross *cross_array)
{
  //优先级对比 选出优先级最高的车道 1、方向 直行>左转>右转  2、道路id升序
   if((cross_->prior_uproad==0)&&(cross_->prior_rightroad==0)
     &&(cross_->prior_downroad==0)&&(cross_->prior_leftroad==0))   return road_->id ;

   pior_cross prior[4];

 if((cross_->prior_uproad==0)||(cross_->prior_uproad==-1)) { prior[0].road_id=65535;prior[0].dir=65535;}
      else{
	if((car_array_[cross_->prior_uproad].state==completed)||(car_array_[cross_->prior_uproad].wait_anthor==true))
	{ prior[0].road_id=65535;prior[0].dir=65535;}
	else{
	prior[0].road_id=cross_->road_id[0]; 
	 if(car_array_[cross_->prior_uproad].move_ori==go_straight)       prior[0].dir=1;
	  else if(car_array_[cross_->prior_uproad].move_ori==turn_left)   prior[0].dir=2;
	   else if(car_array_[cross_->prior_uproad].move_ori==turn_right) prior[0].dir=3;
	    else std::cout<<"Cross: "<<cross_->id << "The Car"<<" "<<car_array_[cross_->prior_uproad].id<<" " <<"has no DIR"<<std::endl;
	}
	  
	}
    if((cross_->prior_rightroad==0)||(cross_->prior_rightroad==-1)) { prior[1].road_id=65535;prior[1].dir=65535;}
      else{
	if((car_array_[cross_->prior_rightroad].state==completed)||(car_array_[cross_->prior_rightroad].wait_anthor==true))
	{ prior[1].road_id=65535;prior[1].dir=65535;}
	else{
	prior[1].road_id=cross_->road_id[1]; 
	 if(car_array_[cross_->prior_rightroad].move_ori==go_straight) prior[1].dir=1;
	  else if(car_array_[cross_->prior_rightroad].move_ori==turn_left) prior[1].dir=2;
	   else if(car_array_[cross_->prior_rightroad].move_ori==turn_right) prior[1].dir=3;
	    else  std::cout<<"Cross: "<<cross_->id << "The Car"<<" "<<car_array_[cross_->prior_rightroad].id<<" " <<"has no DIR"<<std::endl;
      }
      }
    if((cross_->prior_downroad==0)||(cross_->prior_downroad==-1)) { prior[2].road_id=65535;prior[2].dir=65535;}
      else{
	if((car_array_[cross_->prior_downroad].state==completed)||(car_array_[cross_->prior_downroad].wait_anthor==true))
	{ prior[2].road_id=65535;prior[2].dir=65535;}
	else{
	prior[2].road_id=cross_->road_id[2]; 

	 if(car_array_[cross_->prior_downroad].move_ori==go_straight) prior[2].dir=1;
	  else if(car_array_[cross_->prior_downroad].move_ori==turn_left) prior[2].dir=2;
	   else if(car_array_[cross_->prior_downroad].move_ori==turn_right) prior[2].dir=3;
	    else std::cout<<"Cross: "<<cross_->id <<"The Car"<<" "<<car_array_[cross_->prior_downroad].id<<" " <<"has no DIR"<<std::endl;
      }
      }
    if((cross_->prior_leftroad==0)||(cross_->prior_leftroad==-1)) { prior[3].road_id=65535;prior[3].dir=65535;}
      else{
	if((car_array_[cross_->prior_leftroad].state==completed)||(car_array_[cross_->prior_leftroad].wait_anthor==true))
	{ prior[3].road_id=65535;prior[3].dir=65535;}
	else{
	prior[3].road_id=cross_->road_id[3]; 
	 if(car_array_[cross_->prior_leftroad].move_ori==go_straight) prior[3].dir=1;
	  else if(car_array_[cross_->prior_leftroad].move_ori==turn_left) prior[3].dir=2;
	   else if(car_array_[cross_->prior_leftroad].move_ori==turn_right) prior[3].dir=3;
	    else  std::cout<<"Cross: "<<cross_->id << "The Car"<<" "<<car_array_[cross_->prior_leftroad].id<<" " <<"has no DIR"<<std::endl;
      }
      }
       if((prior[0].road_id==65535)&&(prior[1].road_id==65535)&&(prior[2].road_id==65535)&&(prior[3].road_id==65535))  
	   return   road_->id ;  
	 
   qsort(prior, 4, sizeof(prior[0]), campare_dir);
   qsort(prior, 4, sizeof(prior[0]), campare_id);
   return prior[0].road_id;
}
//路口调度 获得对应道路等待态优先级最高的车（不含已调度过 wait_another的车）   起始偏移量     路口道路数组
sch_pos sch_most_prior(Car *car_array_,Road* road_,Cross* cross_,int offset,
		       int cur_road[],Cross *cross_array_,Road* road_array_,Road map_[][MAX_CROSS],
		       std::vector<int> &wait_list_,std::vector<int> &block_list_,int T_
		      )
{	      
	    sch_pos sch_most;
	    sch_most.block=false;
	    sch_most.road_completed=false;
	    sch_most.next_road=road_->id;
	    int front_car=0;
	    int huan[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
	    int j,i;

   for(j=0;j<road_->road_length;j++)
   {
    for(i=0;i<road_->lane_num;i++)
    {  
      if(road_->load[(cross_->id==road_->end)?0:1][i][j]!=0)   //查询该车位是否有车
      {
	int car_id=road_->load[(cross_->id==road_->end)?0:1][i][j];
	
	if((car_array_[car_id].state==wait_schedule)&&(huan[i]!=0)&&(front_car==0)) 
	  front_car=car_id;
	else if((car_array_[road_->load[(cross_->id==road_->end)?0:1][i][j]].state==completed)&&(huan[i]==-1))  
	{ huan[i]=0;front_car=car_id; }

	//有车情况下 该车是等待态 
	if((car_array_[car_id].state==wait_schedule)&&(car_array_[car_id].wait_anthor==false))
          {

// 	    car_array_[car_id].wait_anthor=true;
	     //是front_car

		if(car_id==front_car)
		{       	
		    if(j-min(car_array_[car_id].max_speed,road_->limit_speed)<0)
			{
			 //下一时刻可以过路口    
			      how_tonext to_next=next_road(&car_array_[car_id],
						           &road_array_[road_->id],
				                           &cross_array_[cross_->id],
							   cross_array_,map_);
			  if(to_next.next_road==-1)
			    { 
			      car_array_[car_id].move_ori =go_straight;
			      car_array_[car_id].next_road=road_->id;
			      std::cout << "Attention 1" <<std::endl;
			    }
			  else
			    {
			      car_array_[car_id].move_ori =to_next.direct ;
			      car_array_[car_id].next_road=to_next.next_road ;
			      //是最高优先级才将其信息发到公共字段
			      update_to_cross_drive(&car_array_[car_id],
						 &road_array_[road_->id],
						 &cross_array_[cross_->id]);
			   }
			}
					    	    	    
		  //next_road 是当前道路
		    if(car_array_[car_id].next_road==road_->id)
		    {
	              //min（carspeed，roadspeed）移动后小于零 
		       int max_can= min(car_array_[car_id].max_speed,road_->limit_speed);
		         if(j-max_can<0)
		          {  
			    //路口是其终点 更新车道数组 车辆改为reached状态 break;
			    int next_cross=cross_->id;
			      if(next_cross==car_array_[car_id].goal) 
			         {
			           road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
			           car_array_[car_id].state=reached;
			           check_and_delete(car_id,block_list_);
			           check_and_delete(car_id,wait_list_);
				   std::cout<<"Car: "<< car_id<<"Reached!!!!!!!!!!!!!!"<<std::endl;
			           memset(huan,-1,sizeof(huan));front_car=0;
			            i=0;j=0;break;
			         }
			      else 
			         {  
			           std::cout<<"not car goal"<<std::endl;
				   sch_most.block=true;return sch_most;
			         }
			    //不是终点 报错/return sch_most.block=true
		          }
		         else        //移动后还在车道 更新道路数组 车辆状态 更新路口公共字段
			 {
			   road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
			   road_->load[(cross_->id==road_->end)?0:1][i][j-max_can]=car_id;
			   check_and_delete(car_id,block_list_);
			   check_and_delete(car_id,wait_list_);
			   car_array_[car_id].state=completed;
			   car_array_[car_id].wait_anthor=false;
			   car_array_[car_id].now_road=road_->id;
			   //下一时刻可以过路口
			    if((j-max_can-max_can)<0)
			      {      
				 how_tonext to_next=next_road(&car_array_[car_id],
							      &road_array_[road_->id],
				                              &cross_array_[cross_->id],
							      cross_array_,map_);
			         if(to_next.next_road==-1)
				 { 
				   car_array_[car_id].move_ori =go_straight;
				   car_array_[car_id].next_road=road_->id;
				   std::cout << "Attention 1" <<std::endl;
			         }
			         else
				 {
			           car_array_[car_id].move_ori =to_next.direct ;
			           car_array_[car_id].next_road=to_next.next_road ;
			           //是最高优先级才将其信息发到公共字段
				    update_to_cross_drive(&car_array_[car_id],
							  &road_array_[road_->id],
						          &cross_array_[cross_->id]);
			         }
			      }
			    else
			     {
			       car_array_[car_id].next_road= road_->id;
			       car_array_[car_id].move_ori = go_straight;
			     }
			 memset(huan,-1,sizeof(huan));front_car=0;i=0;j=0;break;
		        }
		   }
		  else
		   {
		    
		     int proir_id= compare_prior_sch(car_array_,
						     &cross_array_[cross_->id],
		                                     road_array_,
				                     &road_array_[road_->id],
				                     cross_array_);
// 	           if((cross_->id==13)&&(road_->id==5027))  
// 		          {
// // 		            debug_dir_tocross(road_array_,15,15,cross_array_);
// 		             out("here");std::cout<<" "<<cross_->prior_leftroad<<std::endl; sleep(1);} 
		      // next road！=当前道路 return sch_most.next road=next road
		      if(proir_id==road_->id)
		        {  	     
			  //该道路优先级最高 该车准备过路口 
			  //最大可行驶距离：   j-min(car_speed,next_road.limitspeed)  (>0 =0 <0)
			  //检查去往道路的余量 ：  当前道路
			  int max_can=min(car_array_[car_id].max_speed,
					  road_array_[car_array_[car_id].next_road].limit_speed)-j;
			    if(max_can<=0)
			     {
			       road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
			       //注意 ：不满足规则 留在原车道
			       road_->load[(cross_->id==road_->end)?0:1][i][0]=car_id;
			       check_and_delete(car_id,block_list_);
			       check_and_delete(car_id,wait_list_);
			       car_array_[car_id].state=completed;
			       car_array_[car_id].wait_anthor=false;
			       car_array_[car_id].now_road=road_->id;
			       //下一时刻可以过路口    
			       how_tonext to_next=next_road(&car_array_[car_id],
						            &road_array_[road_->id],
				                            &cross_array_[cross_->id],
							    cross_array_,map_);
			        if(to_next.next_road==-1)
			         { 
			           car_array_[car_id].move_ori =go_straight;
			           car_array_[car_id].next_road=road_->id;
			           std::cout << "Attention 1" <<std::endl;
			         }
			        else
				{
			           car_array_[car_id].move_ori =to_next.direct ;
			           car_array_[car_id].next_road=to_next.next_road ;
			           //是最高优先级才将其信息发到公共字段
			           update_to_cross_drive(&car_array_[car_id],
							 &road_array_[road_->id],
					                 &cross_array_[cross_->id]);
			        }
			      }
			     else 
			      { 
				       
		                int goal_road = car_array_[car_id].next_road;
			        drive_toroad to_road=check_road_drive_space(&cross_array_[cross_->id],
									    &road_array_[goal_road],
									     car_array_,road_array_[goal_road].road_length-max_can) ;
                 							
                              //被等待态车阻挡 
                                if(to_road.no_drive==0)
				  {
				    road_->load[(cross_->id==road_->end)?0:1][i][j]=car_id;
				    car_array_[car_id].now_road=road_->id;
				    car_array_[car_id].next_road=goal_road;
			            car_array_[car_id].state=wait_schedule;
				    car_array_[car_id].wait_anthor=true;
				    if(check_in_list(car_id,block_list_)) 
				      {
					sch_most.block=true;
					std::cout<<"System Is Blocked at Cross: "<< cross_->id<<" of car: "<<car_id<<std::endl;
					std::cout<<"System Is Blocked at Cross: "<< cross_->id<<" of car: "<<car_id<<std::endl;
					return sch_most;
				      }
				    block_list_.push_back(car_id);
				    memset(huan,-1,sizeof(huan));front_car=0;i=0;j=0;break;
				  }
				 else 
				  {
				    if(to_road.lane==-1){
				      //不能进入车道 因为都被终止态车堵住  该车也置为终止态
				      road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
				      road_->load[(cross_->id==road_->end)?0:1][i][0]=car_id;
				      check_and_delete(car_id,block_list_);
				      check_and_delete(car_id,wait_list_);
				      car_array_[car_id].state=completed;
				      car_array_[car_id].wait_anthor=false;
				      car_array_[car_id].now_road=road_->id;
				      car_array_[car_id].next_road=goal_road;
				      memset(huan,-1,sizeof(huan));front_car=0;i=0;j=0;break;
				    }
				    else
				      {
									
					road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
					road_array_[goal_road].load[(cross_->id==road_array_[goal_road].start)?0:1][i][to_road.offset]=car_id;
					clear_cross_proir(&car_array_[car_id],&road_array_[road_->id],
					  &cross_array_[cross_->id]
					);
					check_and_delete(car_id,block_list_);
					check_and_delete(car_id,wait_list_);
					car_array_[car_id].state=completed;
					car_array_[car_id].wait_anthor=false;
					car_array_[car_id].now_road=goal_road;
					memset(huan,-1,sizeof(huan));front_car=0;i=0;j=0;break;
				      }
				  }
                                //不能驶入
			      }
		        }
		      //优先级被抢
		       else 
		       {
		         sch_most.road_completed=false;
			 sch_most.next_road=proir_id;  return  sch_most;
		       }
		    }
	        }
	     //不是front car
	     else
	       {  
		 //min（carspeed，roadspeed）移动     
	          //该车位到(min（carspeed，roadspeed）<0)?0:min（carspeed，roadspeed）的范围内有
		  bool front_block=false;
		  int pos_offset=min(car_array_[car_id].max_speed,road_->limit_speed);
		  if(j-pos_offset<0) pos_offset=0;
	             else pos_offset=j-pos_offset;
		  int arrage;
	               for(arrage=j-1;arrage>=pos_offset;arrage--) 
			 if(road_->load[(cross_->id==road_->end)?0:1][i][arrage]!=0) 
			   {
			     front_block=true;break; 
			   }
			 if(front_block)
			   {
			     if(car_array_[road_->load[(cross_->id==road_->end)?0:1][i][arrage]].state==wait_schedule)
			       {  //检查该车辆是否出现在检索列表  在 ： return sch_most.block=true
				  //不在检锁列表  ：  加入检锁列表 wait_anthor==true continue;
			         if(check_in_list(car_id,block_list_)) 
			          {
				    std::cout<<"System Is Blocked at Cross: "<< cross_->id<<" of car: "<<car_id<<std::endl;
				    std::cout<<"System Is Blocked at Cross: "<< cross_->id<<" of car: "<<car_id<<std::endl;
				    sch_most.block=true; return sch_most;
			          }
			         else
				 {
				    block_list_.push_back(car_id);
				    car_array_[car_id].wait_anthor=true;i=0;j=0; break;
			         }
			       }
			     else if(car_array_[road_->load[(cross_->id==road_->end)?0:1][i][arrage]].state==completed)
			       {
	
	                         //阻碍车辆的状态为终止态 尾随 更新车道数组 车辆状态 now_road nextroad
			         road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
			         road_->load[(cross_->id==road_->end)?0:1][i][arrage+1]=car_id;
			         check_and_delete(car_id,block_list_);
			         check_and_delete(car_id,wait_list_);
			         car_array_[car_id].state=completed;
			         car_array_[car_id].now_road=road_->id;
			         car_array_[car_id].wait_anthor=false;
			         memset(huan,-1,sizeof(huan));front_car=0;i=0;j=0;break;
			       }
			    }
			  else
			    {
			      //该车位到0的范围内无车 直接移到该车道0处 更新车道数组 车辆状态 now_road nextroad
			      road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
			      road_->load[(cross_->id==road_->end)?0:1][i][pos_offset]=car_id;
			      car_array_[car_id].state=completed;
			      car_array_[car_id].now_road=road_->id;
			      car_array_[car_id].wait_anthor=false;
			      check_and_delete(car_id,block_list_);
			      check_and_delete(car_id,wait_list_);
			    }
			      memset(huan,-1,sizeof(huan));front_car=0;i=0;j=0;break;
	     }
	  }
      }
    }
         if(i==0)  j=-1;
   }
    sch_most.next_road=road_->id;
    sch_most.road_completed=true;
    return sch_most;
}
//将其从等代表中删除
bool check_and_delete(int car_id,std::vector<int> &wait_list_)
{
    if(wait_list_.empty()) return false;
  std::vector<int>::iterator it;
  it =find(wait_list_.begin(),wait_list_.end(),car_id);
  if (it!=wait_list_.end())  wait_list_.erase(it); 
     else return false;
  return true;
}
//检查此车是否在检锁列表中出现
bool check_in_list(int car_id_,std::vector<int> &wait_list_)
{
  if(wait_list_.empty()) { return false; }
  std::vector<int>::iterator it;
  it =find(wait_list_.begin(),wait_list_.end(),car_id_);
  if (it!=wait_list_.end()) return true;
    else  return false;
}
//在调度车库时的检查  注意： 要与调度路口车辆区分
bool check_most_prior(int car_id,Road* road_,Cross* cross_)
{
  for(int i=0;i<road_->road_length;i++)
    for(int j=0;j<road_->lane_num;j++)
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
how_tonext next_road(Car* car_,Road* cur_road,Cross *cross_array_,Cross* cross_,Road map_[][MAX_CROSS])
{
  how_tonext tonext;
  tonext.next_road=-1;
  tonext.direct=go_straight;
  int pre_corss,last_cross;
  //当前路口是终点
  if(cross_->id==car_->goal) 
  {
    tonext.next_road=cur_road->id;
    tonext.direct=go_straight;
    return tonext;
  }
  for(int i=0;i<MAX_CROSS;i++)
  {
    //找到当前路口
    if(car_->cross_path[i]==cross_->id) 
    {  
     {
      tonext.next_road=map_[car_->cross_path[i]][car_->cross_path[i+1]].id;
      if(tonext.next_road==-1) { std::cout<<"THE WAY IS NOT EXSIT??"<<std::endl; return tonext;}	
      
      pre_corss=car_->cross_path[i-1] ;          last_cross=car_->cross_path[i+1] ;
      if(pre_corss==cross_->up_cross_id) 
      {
	 if(last_cross==cross_->right_cross_id) {tonext.direct=turn_left;return tonext;}
	 else if(last_cross==cross_->down_cross_id) {tonext.direct=go_straight;return tonext;}
         else if(last_cross==cross_->left_cross_id) {tonext.direct=turn_right;return tonext;}
      }
      else if(pre_corss==cross_->right_cross_id) 
      { 
	if(last_cross==cross_->up_cross_id) {tonext.direct=turn_right;return tonext;}
	 else if(last_cross==cross_->down_cross_id) {tonext.direct=turn_left;return tonext;}
         else if(last_cross==cross_->left_cross_id) {tonext.direct=go_straight;return tonext;}
      }
      else if(pre_corss==cross_->down_cross_id) 
      {
        if(last_cross==cross_->right_cross_id) {tonext.direct=turn_right;return tonext;}
	 else if(last_cross==cross_->up_cross_id) {tonext.direct=go_straight;return tonext;}
         else if(last_cross==cross_->left_cross_id) {tonext.direct=turn_left;return tonext;}
      }
      else if(pre_corss==cross_->left_cross_id) 
      {
         if(last_cross==cross_->right_cross_id) {tonext.direct=go_straight;return tonext;}
	 else if(last_cross==cross_->down_cross_id) {tonext.direct=turn_right;return tonext;}
         else if(last_cross==cross_->left_cross_id) {tonext.direct=turn_left;return tonext;}
      }
    }
    }
  }
  
  return tonext;
}

// 将所有终止态的车改为等待态

void chang_completed_towait(int min_car_id_,int max_car_id_,Car *car_array_,std::vector<int> &wait_list_)
{

  for(int sub=min_car_id_;sub<=max_car_id_;sub++)
  {
    if(car_array_[sub].state==completed)
    {
      car_array_[sub].state=wait_schedule;
      wait_list_.push_back(sub);
    }
  }

}

// 如果该道路在位置上最靠前 且为终止态 ，且下一个时刻即将过路口，将其信息发送到其下一个路口公告字段
void update_to_cross(Car* car_,Road* road_,Cross* cross_,Cross *cross_array_)
{

     int next_cross;
     if(cross_->id==road_->start) next_cross=road_->end;
      else next_cross=road_->start;
     
      if(cross_->id==cross_array_[next_cross].up_cross_id) cross_array_[next_cross].prior_uproad=car_->id;
	else if(cross_->id==cross_array_[next_cross].down_cross_id) cross_array_[next_cross].prior_downroad=car_->id;
	  else if(cross_->id==cross_array_[next_cross].left_cross_id) cross_array_[next_cross].left_cross_id=car_->id;
	    else if(cross_->id==cross_array_[next_cross].right_cross_id) cross_array_[next_cross].prior_rightroad=car_->id;
}
// 注意：发车与调度也不一样
void update_to_cross_drive(Car* car_,Road* road_,Cross* cross_)
{
   
      if(road_->id==cross_->road_id[0]) cross_->prior_uproad=car_->id;
	else if(road_->id==cross_->road_id[2]) cross_->prior_downroad=car_->id;
	  else if(road_->id==cross_->road_id[3]) cross_->prior_leftroad=car_->id;
	    else if(road_->id==cross_->road_id[1]) cross_->prior_rightroad=car_->id; 
}
//清楚该路口此道路优先级
void clear_cross_proir(Car* car_,Road* road_,Cross* cross_)
{
     
      if(road_->id==cross_->road_id[0]) cross_->prior_uproad=0;
	else if(road_->id==cross_->road_id[2]) cross_->prior_downroad=0;
	  else if(road_->id==cross_->road_id[3]) cross_->left_cross_id=0;
	    else if(road_->id==cross_->road_id[1]) cross_->prior_rightroad=0;
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
// road_empty check_road_empty(Cross *cur_cross_,Road *cur_road_)
// {
//        road_empty road_situation;
//        road_situation.is_empty=true;
//        road_situation.lane=-1;
//        road_situation.offset=-1;
//        bool this_line_findcar=false;
//        int sub[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};  //存放每个车道最末尾的车
//        for(int j=0;j<cur_road_->lane_num;j++)
// 	for(int l=cur_road_->road_length-1;l>=0;l--)
//          {
// 	    if(cur_road_->load[(cur_cross_->id==cur_road_->end)?0:1][j][l]!=0)   //查询该车位是否有车
// 	    {
// 	       if(road_situation.is_empty) road_situation.is_empty=false;
// 	       break;
// 	    }
// 	    sub[j]++;
//          }         
//        if(road_situation.is_empty)  return road_situation;
//        //路上一定有车
//        for(int k=0;k<cur_road_->lane_num;k++)
//        {
// 	 //该行有空位
// 	 if(sub[k]!=-1)   {road_situation.lane=k;road_situation.offset=cur_road_->road_length-sub[k]-1;break; }           
//        }
//        
// return road_situation; 
// }
// 为进入该道路的车辆提供数据   注意： 此次检查发生在调度道路的车库 所以 方向为驶离路口
// 0316 修正 ： 如果偏移量大于该段道路限速则等于限速
road_space check_road_space(Cross *cur_cross_,Road *cur_road_)
{
       road_space space_situation;
       space_situation.is_empty=true;
       space_situation.lane=-1;     //-1 代表该道路无车位
       space_situation.offset=-1;
       int sub[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};  //存放每个车道最末尾的车
       for(int j=0;j<cur_road_->lane_num;j++)
       {
	for(int l=cur_road_->road_length-1;l>=0;l--)
         {
	    if(cur_road_->load[(cur_cross_->id==cur_road_->start)?0:1][j][l]!=0)   //查询该车位是否有车
	    {
	       if(space_situation.is_empty) space_situation.is_empty=false;    break;
	    }
           sub[j]++;
         }         
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
//要进入某个路口 检查是否有车位驶入
//     int  no_drive;    //  0:被等待态车阻挡  1：被终止钛车阻挡 
//     int  lane;      //被终止态车阻挡的话  是否能有余量进入该道路   -1：没有   非-1：有 对应车道
//     int  offset;    //对应下一条路的车位
drive_toroad check_road_drive_space(Cross *cur_cross_,Road *road_,Car *car_array_ ,int max_offset)
{
  
       drive_toroad road_situ;
       int i=0,j=0,com_last_num=0;

       for( i=0;i<road_->lane_num;i++)
       {
	for( j=road_->road_length-1;j>=max_offset;j--)
         {
	   int car_id=road_->load[(cur_cross_->id==road_->start)?0:1][i][j];
	    if(car_id!=0)   //查询该车位是否有车
	    {
	     if((car_array_[car_id].state==completed)&&(j==road_->road_length-1)&&(road_->lane_num==1)) 
	      {
		road_situ.no_drive=1;
		road_situ.lane=-1;
		road_situ.offset=-1;
		return road_situ;
	      }
             else if((car_array_[car_id].state==completed)&&(j!=road_->road_length-1)) 
	      {
		road_situ.no_drive=1;
		road_situ.lane=i;
		road_situ.offset=j+1;
		return road_situ;
	      }
	      else if((car_array_[car_id].state==completed)&&(j==road_->road_length-1)) com_last_num++;
	      else if(car_array_[car_id].state==wait_schedule)
	      {
		road_situ.no_drive=0;
		road_situ.lane=-1;
		road_situ.offset=-1;
		return road_situ;
	      }
	    }
         }    
         if(j==max_offset-1)
	  {  
	    road_situ.no_drive=1;
	    road_situ.lane=i;
	    road_situ.offset=max_offset;
	    return road_situ;
	  }
       }
      if(com_last_num==road_->lane_num) 
      {
		road_situ.no_drive=1;
		road_situ.lane=-1;
		road_situ.offset=max_offset;
      }
      
return road_situ; 

}
bool read_file(std::string cross_path, Cross *cross_array_,Cross *cross_sortedarray_,int* min_cross_id_,int* max_cross_id_,
	       std::string road_path, Road *road_array_,Road *road_sortedarray_,int* min_road_id_,int* max_road_id,
	       std::string car_path,Car *car_array_,Car *car_sortedarray_,int* min_car_id_,int* max_car_id_,Road map_[][MAX_CROSS])
{
  
  
  
   std::ifstream fin_car(car_path);
    std::ifstream fin_road(road_path);
    std::ifstream fin_cross(cross_path);
    
    if(!fin_road.is_open())  {std::cout<<"/**ERROR:No Road input file**/"<<std::endl;return false;} 
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Car input file**/"<<std::endl;return false;}
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Cross input file**/"<<std::endl;return false;}
 
    int count=1;

    //设置标志位 将文件中的ID号作为数组下标 例如car[5000] 即为编号为5000 的车子信息 道路等同
    bool first_line=true;
    int Subscript=0;
    
    std::cout<<"read start"<<std::endl;
    while(!fin_road.eof())
      {
       char str_tr[50];
       fin_road.getline(str_tr,'\r\n');
       if(str_tr[0]=='#')  continue;
        else if(str_tr[0]=='(')
         {
	   
	    std::sscanf(str_tr,"(%d,%d,%d,%d,%d,%d,%d)",   &(road_array_[Subscript].id),
						    &(road_array_[Subscript].road_length),
						    &(road_array_[Subscript].limit_speed),
						    &(road_array_[Subscript].lane_num),
						    &(road_array_[Subscript].start),
						    &(road_array_[Subscript].end),
						    &(road_array_[Subscript].flag_twoway));
	  if(first_line) 
	  {
	    first_line=false; Subscript=road_array_[Subscript].id;
	    *min_road_id_= Subscript;
	    road_sortedarray_[Subscript] = road_array_[Subscript]=road_array_[0];
	  }
	road_array_[Subscript].completed=false;
	road_sortedarray_[Subscript] = road_array_[Subscript];
	Subscript++;
	count++;
         }
       }
       fin_road.close();
       *max_road_id = Subscript-1;
       count=1;
       Subscript=0;
       first_line=true;
    while(!fin_car.eof())
      {
       char str_tc[50];
       fin_car.getline(str_tc,'\r\n');
       if(str_tc[0]=='#')  continue;
        else if(str_tc[0]=='(')
         {
	   
	   std::sscanf(str_tc,"(%d,%d,%d,%d,%d)",   &(car_array_[Subscript].id),
						    &(car_array_[Subscript].set),
						    &(car_array_[Subscript].goal),
						    &(car_array_[Subscript].max_speed),
						    &(car_array_[Subscript].set_time));
	  if(first_line) 
	  {
	    first_line=false; Subscript=car_array_[Subscript].id;
	    *min_car_id_ = Subscript;
	    car_sortedarray_[Subscript] = car_array_[Subscript] = car_array_[0];
	  } 
	car_sortedarray_[Subscript] = car_array_[Subscript];
	car_array_[Subscript].state = still_stored;
	car_array_[Subscript].wait_anthor =false;
	car_array_[Subscript].now_road= car_array_[Subscript].next_road =-1;
	Subscript++;
	count++;
         }
       }
       fin_car.close();
       *max_car_id_= Subscript-1;
       count=1;
       Subscript=0;
       first_line=true;
    while(!fin_cross.eof())
      {
       char str_ts[50];
       fin_cross.getline(str_ts,'\r\n');
       if(str_ts[0]=='#')  continue;
        else if(str_ts[0]=='(')
         {
	   
	   std::sscanf(str_ts,"(%d,%d,%d,%d,%d)",   &(cross_array_[Subscript].id),
						    &(cross_array_[Subscript].road_id[0]),
						    &(cross_array_[Subscript].road_id[1]),
						    &(cross_array_[Subscript].road_id[2]),
						    &(cross_array_[Subscript].road_id[3]));
	  if(first_line) 
	  {
	    first_line=false; Subscript=cross_array_[Subscript].id;
	    *min_cross_id_ = Subscript;
	    cross_sortedarray_[Subscript] =cross_array_[Subscript] = cross_array_[0];
	  } 
	cross_sortedarray_[Subscript] =cross_array_[Subscript];
	Subscript++;
	count++;
         }
       }
       fin_cross.close();
       *max_cross_id_ = Subscript-1;
       std::cout<<"read finished"<<std::endl;
       
       printf("road_num: %d  car_num: %d  cross_num: %d \n",
	      *max_road_id-*min_road_id_+1,*max_car_id_-*min_car_id_+1,*max_cross_id_-*min_cross_id_+1);
      
       //初始化地图 将所有路口连接置为-1 表示无连接 共有1～(cross_num-1) 个路口
       //数组以0下表开始 直观起见 map[i][j]直接指第i个路口到第j个路口信息
       
       for(int i=*min_cross_id_;i<=*max_cross_id_;i++)
       {
	 //初始化路口的十字连通关系
	 cross_array_[*min_cross_id_+i-1].up_cross_id = 
	    cross_array_[*min_cross_id_+i-1].right_cross_id =
	      cross_array_[*min_cross_id_+i-1].down_cross_id =
		cross_array_[*min_cross_id_+i-1].left_cross_id=-1;
	 for(int j=*min_cross_id_;j<=*max_cross_id_;j++)         
	    map_[i][j].id= -1;
       }
       return true;
}

void map_matrix(Cross* cross_array_,int min_cross_id,int max_cross_id
  ,int (*weight_)[MAX_CROSS],Road* road_array_,Road map_[][MAX_CROSS])
{
  
       for(int i=min_cross_id;i<max_cross_id+1;i++)
       {
	  weight_[i][i]=INF;
	 for(int j=i+1;j<max_cross_id+1;j++)
	 {
	   if((cross_array_[i].road_id[0]!=-1)&&(cross_array_[i].road_id[0]==cross_array_[j].road_id[2])) 
	      {
		weight_[i][j]=10;                              
		//先初始化所有权重为10
		map_[i][j]=road_array_[cross_array_[i].road_id[0]];
		cross_array_[i].up_cross_id=j;
		if(road_array_[cross_array_[i].road_id[0]].flag_twoway==1) {   
		  weight_[j][i]=10;
		  //如果是双向道路 （j,i）元素值与(i,j)处相等 下同
		  map_[j][i]=road_array_[cross_array_[i].road_id[0]];
		  cross_array_[j].down_cross_id=i;
		}
	      }
	   if((cross_array_[i].road_id[1]!=-1)&&(cross_array_[i].road_id[1]==cross_array_[j].road_id[3]))
	      { 
		weight_[i][j]=10;                              
		//先初始化所有权重为10
		map_[i][j]=road_array_[cross_array_[i].road_id[1]];
		cross_array_[i].right_cross_id=j;
		if(road_array_[cross_array_[i].road_id[1]].flag_twoway==1){
		  weight_[j][i]=10;
		  map_[j][i]=road_array_[cross_array_[i].road_id[1]];
		  cross_array_[j].left_cross_id=i;
		}
	      }
	   if((cross_array_[i].road_id[2]!=-1)&&(cross_array_[i].road_id[2]==cross_array_[j].road_id[0])) 
	      {
		weight_[i][j]=10;                              
		//先初始化所有权重为10
		map_[i][j]=road_array_[cross_array_[i].road_id[2]];
		cross_array_[i].down_cross_id=j;
		if(road_array_[cross_array_[i].road_id[2]].flag_twoway==1){
		  weight_[j][i]=10;
		  map_[j][i]=road_array_[cross_array_[i].road_id[2]];
		  cross_array_[j].up_cross_id=i;
		}
	      }
	   if((cross_array_[i].road_id[3]!=-1)&&(cross_array_[i].road_id[3]==cross_array_[j].road_id[1])) 
	      { 
		weight_[i][j]=10;                              
		//先初始化所有权重为10
		map_[i][j]=road_array_[cross_array_[i].road_id[3]];
		if(road_array_[cross_array_[i].road_id[3]].flag_twoway==1){
		  weight_[j][i]=10;
		  cross_array_[i].left_cross_id=j;
		  map_[j][i]=road_array_[cross_array_[i].road_id[3]];
		  cross_array_[j].right_cross_id=i;
		}
	      }
	 }
       }
}
bool write_output(std::string path,Car *car_array_,int min_car_id_,int max_car_id_,Road map_[][MAX_CROSS])
{
       std::ofstream fout(path, std::ios::app);
      if(!fout.is_open()) { std::cout<< "No output file" <<std::endl; return false;}
      std::fstream file(path, std::ios::out);
      fout <<"#(carId,StartTime,road...)"<<std::endl;
       for(int i=min_car_id_;i<=max_car_id_;i++)
     {
       int road_path;
       fout <<"("<<car_array_[i].id<<","<<car_array_[i].set_time<<",";          
       for(int j=0;j<MAX_CROSS;j++)
       {   
	 //为考虑特殊情况 就一条路径？？
	 if(car_array_[i].cross_path[2]==0)
	 {
	   road_path=map_[car_array_[i].cross_path[0]][car_array_[i].cross_path[1]].id ;
	   fout << road_path<<")"<<std::endl;
	   break;
	 }
	  if(car_array_[i].cross_path[j+1]==0) break;
	  road_path=map_[car_array_[i].cross_path[j]][car_array_[i].cross_path[j+1]].id ;
	  fout <<road_path;
	  if(car_array_[i].cross_path[j+2]!=0) fout <<",";
	    else fout <<")"<<std::endl; 
        }   
      }
      fout.close();
      std::cout <<"Write ok!!!"<<std::endl;
      return true;
}
void ready_garage(int min_car_id_,int max_car_id_,Magic_garage* garage_,
		  Road* road_array_,Car *car_array_,Car *car_sortedarray_,Road map_[][MAX_CROSS])
{
      for(int i=min_car_id_;i<=max_car_id_;i++)
     {
       
       if(car_sortedarray_[i].id==10240){std::cout<<"xx:"<<car_sortedarray_[i].cross_path[0]; }
       int ga_roadpath,cur_dup;    
	 //为考虑特殊情况 就一条路径？？
	 if(car_array_[i].cross_path[1]==0)
	 {  std::cout<<"The: "<<i<<"Has no path"<<std::endl;    break;}
	 
	    ga_roadpath = map_[car_array_[car_sortedarray_[i].id].cross_path[0]][car_array_[car_sortedarray_[i].id].cross_path[1]].id;
	    cur_dup = not_equal(car_array_[car_sortedarray_[i].id].set,road_array_[ga_roadpath].start);
	    garage_[ga_roadpath].garage[cur_dup].push_back(car_sortedarray_[i].id); 
          
      }
  
  
  
  
  
  
//        std::ofstream fout(argv[4], std::ios::app);
//       if(!fout.is_open()) { std::cout<< "No output file" <<std::endl; return 0;}
//       std::fstream file(argv[4], std::ios::out);
//       fout <<"#(carId,StartTime,road...)"<<std::endl;
//      for(int i=min_car_id;i<=max_car_id;i++)
//      {
//        {
//        int road_path,ga_roadpath;
//        fout <<"("<<car[i].id<<","<<car[i].set_time<<",";          
//        for(int j=0;j<MAX_CROSS;j++)
//        {   
// 	 //为考虑特殊情况 就一条路径？？
// 	 if(car[i].cross_path[2]==0)
// 	 {
// 	   road_path= map[car[i].cross_path[j]][car[i].cross_path[j+1]].id ;
// 	  fout << road_path<<")"<<std::endl;
// 	   break;
// 	 }
// 	  if(car[i].cross_path[j+1]==0) break;
// 	  road_path= map[car[i].cross_path[j]][car[i].cross_path[j+1]].id ;
// 	  fout << road_path; 
// 	    if(j==0)  
// 	    {
// 	      ga_roadpath = map[car[car_sorted[i].id].cross_path[j]][car[car_sorted[i].id].cross_path[j+1]].id;
// 	      int cur_dup = not_equal(car[car_sorted[i].id].set,road[ga_roadpath].start);
// 	      garage[ga_roadpath].garage[cur_dup].push_back(car_sorted[i].id); 
// 	    }
// 	  if(car[i].cross_path[j+2]!=0) fout <<",";
// 	    else fout <<")"<<std::endl; 
//         }   
//      for(int j=0;j<MAX_CROSS;j++)
//        {
// 	 if(car[i].cross_path[j+1]==0) 
// 	 { 
// 	   fout <<car[i].cross_path[j];break;
// 	 }
// 	   else fout <<car[i].cross_path[j]<<"--->";
//        }
//         fout <<std::endl;
// 	fout <<std::endl;}
//       }	fout.close();
}
bool Astar_search(Car *car_,Road* road_array_,int min_road_id,int max_road_id,Cross* cross_array_,
		  int min_cross_id,int max_cross_id,int (*weight_)[MAX_CROSS],Road map_[][MAX_CROSS])
{
    
    int  from=  car_->set  ;
    int	 to=  car_->goal     ;
    int i=0;
    A_star *a=new A_star(car_,road_array_, min_road_id, max_road_id,cross_array_,min_cross_id, max_cross_id,weight_,map_);
    
    
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
void init_waitanthor(Car* car_array,int min_car_id_,int max_car_id_,Road *road_array_,int min_road_id_,int max_road_id_)
{
  for(int i=min_car_id_;i<=max_car_id_;i++)
    car_array[i].wait_anthor=false;
  
  for(int i=min_road_id_;i<=max_road_id_;i++)
    road_array_[i].completed=false;
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



bool sch_allcross_drive(Car* car_array,int min_car_id,int max_car_id,
			 Cross* cross_array,int min_cross_id,int max_cross_id,
			 int min_road_id_,int max_road_id_,Road* road_array,
			 Magic_garage* garage,
			 Road map_[][MAX_CROSS],
			 int T,
			 std::vector<int> &wait_list_,std::vector<int> &bloack_list_
			)
{    
          bloack_list_.erase(bloack_list_.begin(),bloack_list_.end());
           if(T==3)std::cout<<"!!! Research !!!"<<std::endl; 
	   init_waitanthor(car_array,min_car_id,max_car_id,road_array,min_road_id_,max_road_id_);
       for(int sch_cross_drive=min_cross_id;sch_cross_drive<=max_cross_id;sch_cross_drive++)
       {   
	 if(wait_list_.size()==0) {return false;  break;}
	    //将所有车 的waitanthor初始化
	   
// 	    std::cout<<std::endl;
	    //因为道路上的所有车都已是终止态 所以先调度那个方向的都可以 如果发车的可行驶距离超过道路长度 直接放到路口
	    /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
	    int cur_cross_road[4];
	    std::memcpy(cur_cross_road,cross_array[sch_cross_drive].road_id,sizeof(cur_cross_road));
	    int array_offset=3;
	  for(int i=0;i<4;i++)
	    if(cur_cross_road[i]!=-1)
	      if(road_array[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口不是起点
		if(sch_cross_drive!=road_array[cur_cross_road[i]].end)  cur_cross_road[i]=-1;
	    std::sort(cur_cross_road,cur_cross_road+4);    
	  //去掉不存在的道路 或者不调度 不进入该交叉口的道路
	    for(;array_offset>=0;array_offset--)
	    {
	      if((array_offset==0)&&(cur_cross_road[array_offset]!=-1)) break;
	      if(cur_cross_road[array_offset]==-1) { array_offset+=1;  break;}
	    } 
	  int road_complete[4]={0,0,0,0}; 
	  for(int i=0;i<array_offset;i++) road_complete[i]=1;
	  /******需要调度的道路id升序存于cur_cross_road中    起始下标为 array_offset******/
	  //先从最小id道路开始
          bool all_complete=false;		  
          while(!all_complete)
	    {    
	     int sch_road_drive_offset=array_offset;
	     sch_pos sch_car_road;
	     sch_car_road.next_road = cur_cross_road[array_offset];
 	   for(;sch_road_drive_offset<4;)
	    {  
	      sch_car_road= sch_most_prior(car_array, 
			    &road_array[sch_car_road.next_road],
			    &cross_array[sch_cross_drive],
			    sch_road_drive_offset,cur_cross_road,
			    cross_array,road_array,map_,
			    wait_list_,bloack_list_,T) ;
		  if(sch_car_road.block==true)
		     {
		       while(!bloack_list_.empty())
			{
			  std::cout<<bloack_list_[0]<<"  ";
			  bloack_list_.erase(bloack_list_.begin());
			}
		       return true;
		     }
	          if(sch_car_road.road_completed==true)
		   { 
		     //此处造成路口不能轮询周边道路
		     road_complete[sch_road_drive_offset]=1; 
		     sch_road_drive_offset++;
		     if(sch_road_drive_offset>=4) break;
		     sch_car_road.next_road=cur_cross_road[sch_road_drive_offset];
		   }
	           else
		    {
		      if(sch_car_road.next_road==-1) { out(" Logis error!!!"); return true; }
		      //该道路还没调度 优先级被抢
		      else
		       {
		         road_complete[sch_road_drive_offset]=0;
		         sch_road_drive_offset++;
		         if(sch_road_drive_offset>=4) break;
		       }
		     }	
//                        if(T>=1)
	                   {
	                   std::cout<<"Cross: "<<sch_cross_drive<<" "
	                   <<"Road: "  <<sch_car_road.next_road<<" " 
			   <<wait_list_.size()<<" " <<T<<" "<<wait_list_.size()<<" " <<car_array[wait_list_[0]].id
			   <<" " <<car_array[wait_list_[1]].id
			   <<" " <<car_array[wait_list_[2]].id
			   <<std::endl;	 
			  if((sch_cross_drive==13)&&(sch_car_road.next_road==5016))
			    {	 
			     debug_dir_tocross(road_array,19,19,cross_array);	  
			    }
			      sleep(1);
			    }
	         }
	         for(int k=array_offset;k<4;k++) 
		   if(road_complete[k]==1)  road_array[cur_cross_road[k]].completed=true;
	         if((road_complete[0]==1)&&(road_complete[1]==1)&&(road_complete[2]==1)&&(road_complete[3]==1))  
		   all_complete=true;
	         else
		  {
		    sch_road_drive_offset=array_offset; sch_car_road.next_road = cur_cross_road[array_offset];
		  }
	    }
      }
  return false;
}
//更新当前路口的公共字段 输入：当前路口 当前道路
bool update_cross_prior_garage(int T_,Cross* cross_,Road  *road_array_,Road* road_,Car *car_array_,Cross *cross_array_,Road map_[][MAX_CROSS])
{

    if(map_[(cross_->id==road_->end)?road_->start:road_->end][cross_->id].id==-1) return false;             
  for(int j=0;j<road_->road_length;j++)
   {
    for(int i=0;i<road_->lane_num;i++)
    {  
      if(road_->load[(cross_->id==road_->end)?0:1][i][j]!=0)   //查询该车位是否有车
      {     
        int car_id=road_->load[(cross_->id==road_->end)?0:1][i][j]; 
	if(j-min(car_array_[car_id].max_speed,road_->limit_speed)<0)
	{
	  //下一时刻可以过路口    
	  how_tonext to_next=next_road(&car_array_[car_id],
				        &road_array_[road_->id],
				         &cross_array_[cross_->id],
				    cross_array_,map_);
	  if(to_next.next_road==-1){ 
	    car_array_[car_id].move_ori =go_straight;
	    car_array_[car_id].next_road=road_->id;
	    std::cout << "Attention in garage: "<<car_id <<" "<<cross_->id<<" "<<road_->id<<" "
	    <<std::endl;
	  }
	  else{
	  car_array_[car_id].move_ori =to_next.direct ;
	  car_array_[car_id].next_road=to_next.next_road ;
	  
    if(road_->id==cross_->road_id[0]) cross_->prior_uproad=car_id;
	else if(road_->id==cross_->road_id[2]) cross_->prior_downroad=car_id;
	  else if(road_->id==cross_->road_id[3]) cross_->prior_leftroad=car_id;
	    else if(road_->id==cross_->road_id[1]) cross_->prior_rightroad=car_id;

	  }
	}
	return true;
      }
    }
   }
   return true;
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
                    cross_array_[sch_cross_garage].prior_downroad=cross_array_[sch_cross_garage].prior_uproad
                       =cross_array_[sch_cross_garage].prior_leftroad=cross_array_[sch_cross_garage].prior_rightroad=0;
// 		   std::cout<<std::endl;
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
		     if(!garage[sch_road_garage].garage[cur_dup].empty()) 
			 {   
			      //检查道路最高优先级车道 以及偏移量
			      road_space space_Condition = check_road_space(&cross_array_[sch_cross_garage],&road_array[sch_road_garage]);
// 			       std::cout << "test: "<<space_Condition.lane <<std::endl;
			      if(space_Condition.lane==-1) continue;
// 			      std::cout<<"sch_road_garage "<< sch_road_garage<< " "<<cur_dup;
			      //道路是否有余量
			      int cur_dup=not_equal(sch_cross_garage,road_array[sch_road_garage].start);
			      //循环的条件 道路非满且车库非空 优先级最高的车的发车时间不大于该时刻
			      
			      while((space_Condition.lane!=-1)&&(!garage[road_array[sch_road_garage].id].garage[cur_dup].empty())
				&&car_array[garage[road_array[sch_road_garage].id].garage[cur_dup][0]].set_time<=T
			      )
			      {
				
				    int cur_dup=not_equal(sch_cross_garage,road_array[sch_road_garage].start);
				    std::vector<int> car_garage(garage[road_array[sch_road_garage].id].garage[cur_dup]); 
				    //向量拷贝 只是为了让后面看起来短一点 没什么太大软用
	                            
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
				    //发车成功  将其从车库中删掉
			  	    garage[road_array[sch_road_garage].id].
				    garage[cur_dup].erase(garage[road_array[sch_road_garage].id].
					garage[cur_dup].begin());
				      
				space_Condition = check_road_space(&cross_array_[sch_cross_garage],&road_array[sch_road_garage]);
			      }
		  }
                    //更新路口的公共字段
                    update_cross_prior_garage(T,&cross_array_[sch_cross_garage],road_array,
					      &road_array[sch_road_garage] ,car_array,cross_array_,map_);
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
	      
// 	      for(int i=min_cross_id;i<=max_cross_id;i++)
// 		{std::cout<< "Cross: "<<i<< " "<<  cross_array_[i].prior_uproad <<" "
// 	     <<  cross_array_[i].prior_downroad<<" "
// 	     <<  cross_array_[i].prior_leftroad<<" "
// 	     <<  cross_array_[i].prior_rightroad ;
//                std::cout<<std::endl;}
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
void out(std::string s)
{
  std::cout<< "Debug: "<< s <<std::endl;
}

