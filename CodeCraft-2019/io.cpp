#include "io.h"
#include "small_func.h"
#include <unistd.h>

#ifdef DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif

void init_MGraph(struct MGraph &g) {
  //默认为INF
    memset(g.edges, INF, sizeof(g.edges));
}
void dijk_insert(struct MGraph &g,int u, int v, int w) {
    g.edges[u][v] = w;//
}
int dijk_search(struct MGraph &g, int from,int to,Car* car_,std::vector<int>&road_dict_,
		std::vector<int>&cross_dict_,Road* road_array_, Road map_[][CROSS_NUM],struct System_Para &para_)
{ 
    int min;
    memset(g.path, 0, sizeof(g.path));
    memset(g.set, 0, sizeof(g.set));
    memset(g.dist, 0, sizeof(g.dist));
    //对各个数组进行初始化
    for(int i = 0; i < g.cross_num; i++){
        g.dist[i] = g.edges[from][i];
        g.set[i] = 0;
        if(g.edges[from][i] < INF){
            g.path[i] = from;
        }else{
            g.path[i] = -1;
        }
    } 
    
    g.set[from] = 1; 
    g.path[from] = -1;
    int u=0;
    //初始化结束，关键操作开始
    for(int i = 0; i < g.cross_num - 1; i++)
    {
        min = INF;//找到的点   目前最小 
        //这个循环每次从剩余顶点中选出一个顶点，通往这个顶点的路径在通往所有剩余顶点的路径中是长度最短的
        for(int j = 0; j < g.cross_num; j++)
	{
            if(g.set[j] == 0 && g.dist[j] < min)
	    {
                u = j;
                min = g.dist[j];
            }
        } 
        g.set[u] = 1;   //某点 DONE标志
	//将选出的顶点并入最短路径中
        //这个循环以刚并入的顶点作为中间点，对所有通往剩余顶点的路径进行检测
 	
        for(int j = 0; j < g.cross_num; j++) 
	{
            //这个if判断顶点u的加入是否会出现通往顶点j的更短的路径，如果出现，则改变原来路径及其长度，否则什么都不做
            if(g.set[j] == 0 && (g.dist[u] + g.edges[u][j]+ 
	        para_.speed_near_w* abs(car_->max_speed-map_[u][j].limit_speed))< g.dist[j])
	    {
                g.dist[j] = g.dist[u] + g.edges[u][j];
		//更新路径长度 
                g.path[j] = u;
		//更新路径顶点 
            } 
        } 
    }        
    memset(g.goal_path, -1, sizeof(g.goal_path));
      stack<int> s;
    //这个循环以由叶子结点到根结点的顺序将其入栈
    while(g.path[to] != -1){
        s.push(to);     
        to = g.path[to];
    } 
    s.push(to);
    int offset=0;
    int best_next=from;
    while(!s.empty()){
      g.goal_path[offset] = s.top();
      car_->cross_path[offset]=cross_dict_[g.goal_path[offset]];
    if(offset==1) { best_next=s.top();}
    if(offset>0)
    {
      map_[(g.goal_path[offset-1])][(g.goal_path[offset])].car_willonroad++;
    }
      s.pop();      //出栈
      offset++;
      car_->route++;
    }
    
    return  best_next;
};
int dijk_research(struct DIJK_map &dijk_map, int pre_cross_sub_, struct MGraph &g, int from,int to,int now_road_sub_,
		  Car* car_,Cross* cross_array_,Road* road_array_,std::vector<int>&road_dict_,
		  std::vector<int>&cross_dict_,Road map_[][CROSS_NUM],int ori_sub,struct System_Para &para_)
{ 
  int saved_to=to;
  if(dijk_map.map[from][to][1]==pre_cross_sub_)  //not allowed back
  {
    int min;
    memset(g.path, 0, sizeof(g.path));
    memset(g.set, 0, sizeof(g.set));
    memset(g.dist, 0, sizeof(g.dist));
    //对各个数组进行初始化
    for(int i = 0; i < g.cross_num; i++){
        g.dist[i] = g.edges[from][i];
        g.set[i] = 0;
        if(g.edges[from][i] < INF){
            g.path[i] = from;
        }else{
            g.path[i] = -1;
        }
    } 
    
    g.set[from] = 1; 
    g.path[from] = -1;
    int u=0;
    //初始化结束，关键操作开始
    for(int i = 0; i < g.cross_num - 1; i++)
    {
        min = INF;//找到的点   目前最小 
        //这个循环每次从剩余顶点中选出一个顶点，通往这个顶点的路径在通往所有剩余顶点的路径中是长度最短的
        for(int j = 0; j < g.cross_num; j++)
	{
            if(g.set[j] == 0 && g.dist[j] < min)
	    {
                u = j;
                min = g.dist[j];
            }
        } 
        g.set[u] = 1;   //某点 DONE标志
	//将选出的顶点并入最短路径中
        //这个循环以刚并入的顶点作为中间点，对所有通往剩余顶点的路径进行检测
 	
        for(int j = 0; j < g.cross_num; j++) 
	{
            //这个if判断顶点u的加入是否会出现通往顶点j的更短的路径，如果出现，则改变原来路径及其长度，否则什么都不做
            if(g.set[j] == 0 && (g.dist[u] + g.edges[u][j] +para_.speed_near_w* abs(car_->max_speed-map_[u][j].limit_speed))< g.dist[j])
	    {
                g.dist[j] = g.dist[u] + g.edges[u][j];
		//更新路径长度 
                g.path[j] = u;
		//更新路径顶点 
            } 
        } 
    }        
    stack<int> s;
    //这个循环以由叶子结点到根结点的顺序将其入栈
    while(g.path[to] != -1){
        s.push(to);     
        to = g.path[to];
    } 
    s.push(to);
    bool clear=false;
    int k;
    int offset=0;

    
    for(k=car_->route-1;k>=0;)
      {
	if((car_->cross_path[k]!=cross_dict_[from])&&(!clear))
	 {car_->cross_path[k]=0; k--;}
	else { clear=true; k++;}
       if(clear)
	{
	  while(!s.empty()){
	  car_->cross_path[k-1]=cross_dict_[s.top()];
	  g.goal_path[offset] = s.top();
	if(offset>0)
          {
            map_[(g.goal_path[offset-1])][(g.goal_path[offset])].car_willonroad++;
          }
          offset++;
          s.pop();      
          k++;
          }
           break;
	}
      }
    car_->route=k-1;
    }
    else 
    {
      for(int map_off=0;map_off<CROSS_NUM;map_off++)
      {
	if(cross_dict_[dijk_map.map[from][saved_to][map_off]]==car_->goal)
	{
	  car_->cross_path[(ori_sub+map_off)]=cross_dict_[dijk_map.map[from][saved_to][map_off]];
	  if( car_->route>ori_sub+map_off+1)
	  {
	    for(int r=ori_sub+map_off+1;r<=car_->route;r++) car_->cross_path[r]=0;
	  }
	  car_->route=ori_sub+map_off+1;
	  break;
	}
	else
	{
	  car_->cross_path[(ori_sub+map_off)]=cross_dict_[dijk_map.map[from][saved_to][map_off]];
	}
      }
    }
    int next_cr_sub=cross_tosub(car_->cross_path[(ori_sub+1)],cross_dict_);
   // cross_dict_[from]   
   //update the next road and ori  
  //bug find here 
    how_tonext re_next= next_road(car_,&road_array_[now_road_sub_],cross_array_,
			             cross_dict_,&cross_array_[from],&cross_array_[next_cr_sub],map_);
 
    if(re_next.next_road==-1)
      { 
	car_->move_ori =go_straight;
	car_->next_road=road_array_[now_road_sub_].id;
	std::cout << "Attention 1" <<std::endl;
      }
    else
      {
	car_->move_ori =re_next.direct ;
	car_->next_road=re_next.next_road ;
      } 
      if(clear_cross_proir(car_,&road_array_[now_road_sub_],&cross_array_[from]))
      {
	update_to_cross_drive(car_, &road_array_[now_road_sub_], &cross_array_[from]);
      }
      
      
      
    return  true;
};

int update_map(struct DIJK_map &dijk_map,int from,int to,struct MGraph &g,struct System_Para &para_)
{ 
    int min;
    int save_to=to;
    memset(g.path, 0, sizeof(g.path));
    memset(g.set, 0, sizeof(g.set));
    memset(g.dist, 0, sizeof(g.dist));
    //对各个数组进行初始化
    for(int i = 0; i < g.cross_num; i++){
        g.dist[i] = g.edges[from][i];
        g.set[i] = 0;
        if(g.edges[from][i] < INF){
            g.path[i] = from;
        }else{
            g.path[i] = -1;
        }
    } 
    
    g.set[from] = 1; 
    g.path[from] = -1;
    int u=0;
    //初始化结束，关键操作开始
    for(int i = 0; i < g.cross_num - 1; i++)
    {
        min = INF;//找到的点   目前最小 
        //这个循环每次从剩余顶点中选出一个顶点，通往这个顶点的路径在通往所有剩余顶点的路径中是长度最短的
        for(int j = 0; j < g.cross_num; j++)
	{
            if(g.set[j] == 0 && g.dist[j] < min)
	    {
                u = j;
                min = g.dist[j];
            }
        } 
        g.set[u] = 1;   //某点 DONE标志
	//将选出的顶点并入最短路径中
        //这个循环以刚并入的顶点作为中间点，对所有通往剩余顶点的路径进行检测
 	
        for(int j = 0; j < g.cross_num; j++) 
	{
            //这个if判断顶点u的加入是否会出现通往顶点j的更短的路径，如果出现，则改变原来路径及其长度，否则什么都不做
            if(g.set[j] == 0 && (g.dist[u] + g.edges[u][j] )< g.dist[j])
	    {
                g.dist[j] = g.dist[u] + g.edges[u][j];
		//更新路径长度 
                g.path[j] = u;
		//更新路径顶点 
            } 
        } 
    }        
    stack<int> s;
    //这个循环以由叶子结点到根结点的顺序将其入栈
    while(g.path[to] != -1){
        s.push(to);     
        to = g.path[to];
    } 
    s.push(to);
    
    bool clear=false;
    int k;
    int offset=0;
    int best_next=from;
    while(!s.empty())
    {
      g.goal_path[offset] = s.top();
      dijk_map.map[from][save_to][offset] = s.top();
      s.pop();     
      offset++;
    }
      
    return  true;
};
//路口优先级对比
void quickSortOfCpp(Car* car_list,int car_num_)
{
//     qsort(car_list, car_num_, sizeof(car_list[0]), campare_settime);
//        qsort(car_list, car_num_, sizeof(car_list[0]), campare_speed);
//        qsort(car_list, car_num_, sizeof(car_list[0]), campare_carid);
  qsort(car_list, car_num_, sizeof(car_list[0]), campare_route);
  qsort(car_list, car_num_, sizeof(car_list[0]), campare_speed);
  
}
int cross_tosub(int cross_id_,std::vector<int>&cross_dict_)
{
    vector<int>::iterator it = find(cross_dict_.begin(), cross_dict_.end(),cross_id_);
 
    if(it != cross_dict_.end())
        return &*it-&cross_dict_[0];
    else
    { cout<<"can not find cross"<<" "<<cross_id_<<endl ;return 0;};
 
    return &*it-&cross_dict_[0];
}
int car_tosub(int car_id_,std::vector<int>&car_dict_)
{
    vector<int>::iterator it = find(car_dict_.begin(), car_dict_.end(),car_id_);
 
    if(it != car_dict_.end())
        return &*it-&car_dict_[0];
    else
        cout<<"can not find car"<<car_id_<<endl;
 
    return &*it-&car_dict_[0];
}
int road_tosub(int road_id_,std::vector<int>&road_dict_)
{
    vector<int>::iterator it = find(road_dict_.begin(), road_dict_.end(),road_id_);
 
    if(it != road_dict_.end())
        return &*it-&road_dict_[0];
    else
        cout<<"can not find road "<<road_id_ <<endl;
 
    return &*it-&road_dict_[0]; 
}
int compare_prior_sch(Car* car_array_,std::vector<int>&car_dict_,Cross* cross_,Road* road_array,Road* road_,Cross *cross_array)
{
  //优先级对比 选出优先级最高的车道 1、方向 直行>左转>右转  2、道路id升序
   if((cross_->prior_uproad==0)&&(cross_->prior_rightroad==0)
     &&(cross_->prior_downroad==0)&&(cross_->prior_leftroad==0))   return road_->id ;

    pior_cross prior[4];

 if((cross_->prior_uproad==0)||(cross_->prior_uproad==-1)) { prior[0].road_id=65535;prior[0].dir=65535;}
      else{
	int up_car_sub=car_tosub(cross_->prior_uproad,car_dict_);
	if((car_array_[up_car_sub].state==completed)||(car_array_[up_car_sub].wait_anthor==true))
	{ prior[0].road_id=65535;prior[0].dir=65535;}
	else{
	prior[0].road_id=cross_->road_id[0]; 
	 if(car_array_[up_car_sub].move_ori==go_straight)       prior[0].dir=1;
	  else if(car_array_[up_car_sub].move_ori==turn_left)   prior[0].dir=2;
	   else if(car_array_[up_car_sub].move_ori==turn_right) prior[0].dir=3;
	    else std::cout<<"Cross: "<<cross_->id << "The Car"<<" "<<car_array_[(cross_->prior_uproad)].id<<" " <<"has no DIR"<<std::endl;
	}
	  
	}
    if((cross_->prior_rightroad==0)||(cross_->prior_rightroad==-1)) { prior[1].road_id=65535;prior[1].dir=65535;}
      else{
	int right_car_sub=car_tosub(cross_->prior_rightroad,car_dict_);
	if((car_array_[right_car_sub].state==completed)||(car_array_[right_car_sub].wait_anthor==true))
	{ prior[1].road_id=65535;prior[1].dir=65535;}
	else{
	prior[1].road_id=cross_->road_id[1]; 
	 if(car_array_[right_car_sub].move_ori==go_straight) prior[1].dir=1;
	  else if(car_array_[right_car_sub].move_ori==turn_left) prior[1].dir=2;
	   else if(car_array_[right_car_sub].move_ori==turn_right) prior[1].dir=3;
	    else  std::cout<<"Cross: "<<cross_->id << "The Car"<<" "<<car_array_[right_car_sub].id<<" " <<"has no DIR"<<std::endl;
      }
      }
    if((cross_->prior_downroad==0)||(cross_->prior_downroad==-1)) { prior[2].road_id=65535;prior[2].dir=65535;}
      else{
	int down_car_sub=car_tosub(cross_->prior_downroad,car_dict_);
	if((car_array_[down_car_sub].state==completed)||(car_array_[down_car_sub].wait_anthor==true))
	{ prior[2].road_id=65535;prior[2].dir=65535;}
	else{
	prior[2].road_id=cross_->road_id[2]; 

	 if(car_array_[down_car_sub].move_ori==go_straight) prior[2].dir=1;
	  else if(car_array_[down_car_sub].move_ori==turn_left) prior[2].dir=2;
	   else if(car_array_[down_car_sub].move_ori==turn_right) prior[2].dir=3;
	    else std::cout<<"Cross: "<<cross_->id <<"The Car"<<" "<<car_array_[down_car_sub].id<<" " <<"has no DIR"<<std::endl;
      }
      }
    if((cross_->prior_leftroad==0)||(cross_->prior_leftroad==-1)) { prior[3].road_id=65535;prior[3].dir=65535;}
      else{
	int left_car_sub=car_tosub(cross_->prior_leftroad,car_dict_);
	if((car_array_[left_car_sub].state==completed)||(car_array_[left_car_sub].wait_anthor==true))
	{ prior[3].road_id=65535;prior[3].dir=65535;}
	else{
	prior[3].road_id=cross_->road_id[3]; 
	 if(car_array_[left_car_sub].move_ori==go_straight) prior[3].dir=1;
	  else if(car_array_[left_car_sub].move_ori==turn_left) prior[3].dir=2;
	   else if(car_array_[left_car_sub].move_ori==turn_right) prior[3].dir=3;
	    else  std::cout<<"Cross: "<<cross_->id << "The Car"<<" "<<car_array_[left_car_sub].id<<" " <<"has no DIR"<<std::endl;
      }
      }
       if((prior[0].road_id==65535)&&(prior[1].road_id==65535)&&(prior[2].road_id==65535)&&(prior[3].road_id==65535))  
	   return   road_->id ;  
	 
   qsort(prior, 4, sizeof(prior[0]), campare_dir);
   qsort(prior, 4, sizeof(prior[0]), campare_id);
   return prior[0].road_id;
}
int search_frontcar(Cross* cross_,Road* road_,Car *car_array_,std::vector<int>&car_dict_)
{
  int front_car=-1;
  int has_frontcar[MAX_LANE];
   memset(has_frontcar,0,sizeof(has_frontcar));
   for(int j=0;j<road_->limit_speed;j++)
   {
    for(int i=0;i<road_->lane_num;i++)
    { 
      if(has_frontcar[i]!=-1)
      {
     if(road_->load[(cross_->id==road_->end)?0:1][i][j]!=0)   
       {
	 int car_id=road_->load[(cross_->id==road_->end)?0:1][i][j];
	 int car_sub=car_tosub(car_id,car_dict_);
	 if((car_array_[car_sub].state==completed)||car_array_[car_sub].wait_anthor==true)
	  {
	   has_frontcar[i]=-1; // no front car this lane 
	   continue;
	  }
	 else if(car_array_[car_sub].state==wait_schedule)
	  {
	    road_->front_car=car_id;
	    front_car=car_id; return front_car;
	  }
	}
      }
    }	
   }
   return front_car;
}
//路口调度 获得对应道路等待态优先级最高的车（不含已调度过 wait_another的车）   起始偏移量     路口道路数组
sch_pos sch_most_prior(Car *car_array_,std::vector<int>&car_dict_,Road* road_,
		       std::vector<int>&road_dict_,Cross* cross_,std::vector<int>&cross_dict_,int offset,
		       int cur_road[],Cross *cross_array_,Road* road_array_,Road map_[][CROSS_NUM],
		       std::vector<int> &wait_list_,std::vector<int> &block_list_,int T_,int *reached_car_,int *wait_num_)

{
            sch_pos sch_most;
            sch_most.block=false;
            sch_most.road_completed=false;
            sch_most.next_road=road_->id;
            int front[MAX_LANE];
            memset(front,-1,sizeof(front));
            int most_prior_car=-1;
            int cross_sub=cross_tosub(cross_->id,cross_dict_);
            int road_sub=road_tosub(road_->id,road_dict_);
           int i;
           for(int j=0;j<road_->road_length;j++)
           {
               for(i=0;i<road_->lane_num;i++)
               {
                 if(road_->load[(cross_->id==road_->end)?0:1][i][j]!=0)
                  {
                    int car_id=road_->load[(cross_->id==road_->end)?0:1][i][j];
                    int car_sub=car_tosub(car_id,car_dict_);

 
                    if(front[i]==-1)   front[i]=car_id;

                    if((most_prior_car==-1)&&(car_array_[car_sub].state==wait_schedule))  most_prior_car=car_id;
 
                    if((car_array_[car_sub].state==wait_schedule)&&(car_array_[car_sub].wait_anthor==false))
                    {
                        if(car_id==front[i])
                        {
                          int max_can= min(car_array_[car_sub].max_speed,road_->limit_speed);
                            if(car_id==most_prior_car)

                            {
                                if((j-max_can)<0)
                                {
                                      how_tonext to_next=next_road_drive(&car_array_[car_sub],
						                                                 &road_array_[road_sub],
				                                                        cross_array_,cross_dict_,
				                                                     &cross_array_[cross_sub],map_);
 
                                      if(to_next.next_road==-1)
                                        {
                                          car_array_[car_sub].move_ori =go_straight;
                                          car_array_[car_sub].next_road=road_->id;
                                          std::cout << "Attention 1" <<std::endl;
                                        }
                                      else
                                        {
                                          car_array_[car_sub].move_ori =to_next.direct ;
                                          car_array_[car_sub].next_road=to_next.next_road ;
                                          //no block
                                          update_to_cross_drive(&car_array_[car_sub],
                                                 &road_array_[road_sub],
                                                 &cross_array_[cross_sub]);
                                       }
                                     int proir_id= compare_prior_sch(car_array_,car_dict_,
                                                                     &cross_array_[cross_sub],
                                                                     road_array_, &road_array_[road_sub],
                                                                      cross_array_);
                                     if(proir_id==road_->id)
                                     {
                                         if(cross_->id==car_array_[car_sub].goal)
                                         {
                                             road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
                                             car_array_[car_sub].state=reached;
                                             car_array_[car_sub].wait_anthor=false;
                                             check_and_delete(car_id,wait_list_);
                                             (*reached_car_)++;
                                             int map_e=  cross_tosub(cross_->id,cross_dict_);
                                             int map_s = cross_tosub(((road_->end==cross_->id)?(road_->start):(road_->end)),cross_dict_);
                                             map_[map_s][map_e].car_onroad--;
                                             clear_cross_proir(&car_array_[car_sub],&road_array_[road_sub],
					                                                          &cross_array_[cross_sub]);
					     front[i]=-1;most_prior_car=-1;i=0;j=0;break;
                                         }
                                         else if(car_array_[car_sub].next_road!=road_->id)
                                         {
                                            int road_next_sub=road_tosub(car_array_[car_sub].next_road,road_dict_);
			                    int max_can=min(car_array_[car_sub].max_speed,road_array_[road_next_sub].limit_speed)-j;
			                    if(max_can<=0)
                                            {
                                                road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
                                                if(road_->load[(cross_->id==road_->end)?0:1][i][0]!=0)
                                                    std::cout<<"B has car!!!!!!!!"<<std::endl;
                                                road_->load[(cross_->id==road_->end)?0:1][i][0]=car_id;
                                                car_array_[car_sub].not_allow_research=true;
                                                car_array_[car_sub].state=completed;
                                                car_array_[car_sub].wait_anthor=false;
                                                car_array_[car_sub].now_road=road_->id;
                                                check_and_delete(car_id,wait_list_);
                                                most_prior_car=-1;i=0;j=0;break;
                                            }
                                            else
                                            {
                                                int goal_road = car_array_[car_sub].next_road;
                                                int goal_road_sub=road_tosub(goal_road,road_dict_);
                                                drive_toroad to_road=check_road_drive_space(&cross_array_[cross_sub],
									                       &road_array_[goal_road_sub],
									                       car_array_,car_dict_,road_dict_,
						                                               road_array_[goal_road_sub].road_length-max_can) ;
                                                 if(to_road.no_drive==0) // blocked by a waited car
                                                  {
                                                    road_->load[(cross_->id==road_->end)?0:1][i][j]=car_id;
                                                    car_array_[car_sub].now_road=road_->id;
                                                    car_array_[car_sub].next_road=goal_road;
                                                    car_array_[car_sub].state=wait_schedule;
                                                    car_array_[car_sub].not_allow_research=true;
                                                    car_array_[car_sub].wait_anthor=true;
                                                    continue;
                                                  }
                                                  else
                                                  {
                                                    if(to_road.lane==-1)
                                                     {
                                                      //blocked by a completed car
                                                      road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
                                                    if(road_->load[(cross_->id==road_->end)?0:1][i][0]!=0)
                                                       std::cout<<"C has car!!!!!!!!"<<std::endl;
                                                      road_->load[(cross_->id==road_->end)?0:1][i][0]=car_id;
                                                      car_array_[car_sub].not_allow_research=true;
                                                      car_array_[car_sub].state=completed;
                                                      car_array_[car_sub].wait_anthor=false;
                                                      car_array_[car_sub].now_road=road_->id;
                                                      car_array_[car_sub].next_road=goal_road;
                                                     check_and_delete(car_id,wait_list_);
                                                     most_prior_car=-1;i=0;j=0;break;
                                                    }
                                                    else
                                                      {
                                                        road_->load[(cross_->id==road_->end)?0:1][i][j]=0;

                                                        if(road_array_[goal_road_sub].load[(cross_->id==road_array_[goal_road_sub].start)?0:1]
                                                                        [to_road.lane][to_road.offset]!=0)
                                                          std::cout<<"D has car!!!!!!!!"<<std::endl;
                                                        road_array_[goal_road_sub].load[(cross_->id==road_array_[goal_road_sub].start)?0:1]
                                                                                                        [to_road.lane][to_road.offset]=car_id;
                                                        int map_e=cross_tosub(cross_->id,cross_dict_);
                                                            int map_s = cross_tosub(((road_->end==cross_->id)?road_->start:road_->end),cross_dict_);
                                                            map_[map_s][map_e].car_onroad--;
                                                        int next_e=cross_tosub(((road_array_[goal_road_sub].end==cross_->id)?
                                                                                     road_array_[goal_road_sub].start:
                                                                                       road_array_[goal_road_sub].end),cross_dict_);
                                                        map_[map_e][next_e].car_onroad++;
                                    // 					check_and_delete(car_id,block_list_);
                                                        check_and_delete(car_id,wait_list_);
                                                        car_array_[car_sub].state=completed;
                                                        car_array_[car_sub].wait_anthor=false;
                                                        car_array_[car_sub].now_road=goal_road;
                                                        clear_cross_proir(&car_array_[car_sub],
                                                                          &road_array_[road_sub],
                                                                          &cross_array_[cross_sub]);
                                                        front[i]=-1;most_prior_car=-1;i=0;j=0;break;
                                                      }
                                                  }

                                            }
                                         }
                                         else
                                         {
                                             std::cout<<"not car goal"<<std::endl;
				                             sch_most.block=true;return sch_most;
                                         }
                                     }
                                    else
                                     {
                                         sch_most.road_completed=false;
                                         sch_most.next_road=proir_id;
                                         return  sch_most;
                                     }
                                }
                                else
                                {
                                    road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
                                    if(road_->load[(cross_->id==road_->end)?0:1][i][j-max_can]!=0)
                                        std::cout<<"A has car!!!!!!!!"<<std::endl;
			                        road_->load[(cross_->id==road_->end)?0:1][i][j-max_can]=car_id;
                                   car_array_[car_sub].state=completed;
                                   car_array_[car_sub].wait_anthor=false;
                                   car_array_[car_sub].now_road=road_->id;
                                   check_and_delete(car_id,wait_list_);
				   most_prior_car=-1;
                                   continue;
                                }
                            }
                            else
                            {
                               if(j-max_can<0)
                               {
                                   car_array_[car_sub].wait_anthor=true;
                                   continue;
                               }
                               else
                               {
                                    road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
                                    if(road_->load[(cross_->id==road_->end)?0:1][i][j-max_can]!=0)
                                        std::cout<<"A' has car!!!!!!!!"<<std::endl;
			           road_->load[(cross_->id==road_->end)?0:1][i][j-max_can]=car_id;
                                   car_array_[car_sub].state=completed;
                                   car_array_[car_sub].wait_anthor=false;
                                   car_array_[car_sub].now_road=road_->id;
                                   check_and_delete(car_id,wait_list_);
				   continue;
                               }
                            }
                        }
                        else
                        {
                               bool front_block=false;
                               int pos_offset=min(car_array_[car_sub].max_speed,road_->limit_speed);
			       
			       
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
	                                 int car_arrange=car_tosub(road_->load[(cross_->id==road_->end)?0:1][i][arrage],car_dict_);
	                                 if(car_array_[car_arrange].state==wait_schedule)
                                       {
                                        car_array_[car_sub].wait_anthor=true;continue;
                                       }
                                     else if(car_array_[car_arrange].state==completed)
                                     {
                                      //blocked by completed car
                                         road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
                                     if( road_->load[(cross_->id==road_->end)?0:1][i][arrage+1]!=0)
                                        std::cout<<"E has car!!!!!!!!"<<std::endl;

                                         road_->load[(cross_->id==road_->end)?0:1][i][arrage+1]=car_id;
                                         check_and_delete(car_id,wait_list_);
                                         car_array_[car_sub].state=completed;
                                         car_array_[car_sub].now_road=road_->id;
                                         car_array_[car_sub].wait_anthor=false;
                                         
                                         if(car_id==most_prior_car)
					 {
					   most_prior_car=-1;continue;
					 }
                                         else
                                           continue;
                                     }
                                    else std::cout<<"warning logic!!!"<<std::endl;

	                             }
	                             else
                                 {
				   if(pos_offset==0)   {std::cout<<"WHY THIS PHEN"<<std::endl;out("DEBUG"); sleep(5);}
                                      //该车位到0的范围内无车 直接移到该车道0处 更新车道数组 车辆状态 now_road nextroad
                                      road_->load[(cross_->id==road_->end)?0:1][i][j]=0;
                                      if( road_->load[(cross_->id==road_->end)?0:1][i][pos_offset]!=0)  
                                         std::cout<<"F has car!!!!!!!!"<<std::endl; 
                                      road_->load[(cross_->id==road_->end)?0:1][i][pos_offset]=car_id;
                                      car_array_[car_sub].state=completed;
                                      car_array_[car_sub].now_road=road_->id;
                                      car_array_[car_sub].wait_anthor=false;
                                      check_and_delete(car_id,wait_list_);
                                      continue;
                                 }
                        }
                    }
                  }

               }
               if(i==0) j=-1;
           }
    sch_most.road_completed=true;
    return sch_most;

}
how_tonext next_road(Car* car_,Road* cur_road,Cross *cross_array_,std::vector<int>&cross_dict_,
		     Cross* cross_,Cross* next_cross_,Road map_[][CROSS_NUM])
{
  how_tonext tonext;
  tonext.next_road=-1;
  tonext.direct=go_straight;
  //当前路口是终点
  if(cross_->id==car_->goal) 
  {
    tonext.next_road=cur_road->id;
    tonext.direct=go_straight;
    return tonext;
  }  
    //找到当前路口 
      int map_s=cross_tosub(cross_->id,cross_dict_);
      int map_e=cross_tosub(next_cross_->id,cross_dict_);
      tonext.next_road=map_[map_s][map_e].id;

      if(tonext.next_road==-1) { std::cout<<"THE WAY IS NOT EXSIT??"<<std::endl; return tonext;}	
      
      if(cur_road->id==cross_->road_id[0]) 
      {
	 if(tonext.next_road==cross_->road_id[1]) {tonext.direct=turn_left;return tonext;}
	 else if(tonext.next_road==cross_->road_id[2]) {tonext.direct=go_straight;return tonext;}
         else if(tonext.next_road==cross_->road_id[3]) {tonext.direct=turn_right;return tonext;}
      }
      else if(cur_road->id==cross_->road_id[1]) 
      { 
	if(tonext.next_road==cross_->road_id[0]) {tonext.direct=turn_right;return tonext;}
	 else if(tonext.next_road==cross_->road_id[2]) {tonext.direct=turn_left;return tonext;}
         else if(tonext.next_road==cross_->road_id[3]) {tonext.direct=go_straight;return tonext;}
      }
      else if(cur_road->id==cross_->road_id[2]) 
      {
        if(tonext.next_road==cross_->road_id[1]) {tonext.direct=turn_right;return tonext;}
	 else if(tonext.next_road==cross_->road_id[0]) {tonext.direct=go_straight;return tonext;}
         else if(tonext.next_road==cross_->road_id[3]) {tonext.direct=turn_left;return tonext;}
      }
      else if(cur_road->id==cross_->road_id[3]) 
      {
         if(tonext.next_road==cross_->road_id[1]) {tonext.direct=go_straight;return tonext;}
	 else if(tonext.next_road==cross_->road_id[2]) {tonext.direct=turn_right;return tonext;}
         else if(tonext.next_road==cross_->road_id[0]) {tonext.direct=turn_left;return tonext;}
      }
    
  
     std::cout<<"can not find current in path!!"<< car_->id <<std::endl; out(" ERROR");
     sleep(5); 
  
  
  return tonext;
  
}
//通过当前所在道路输出下一条道路 以及转向关系转向  注意：调度车库与调度路口方向不同
how_tonext next_road_drive(Car* car_,Road* cur_road,Cross *cross_array_,std::vector<int>&cross_dict_,
		     Cross* cross_,Road map_[][CROSS_NUM])
{
  how_tonext tonext;
  tonext.next_road=-1;
  tonext.direct=go_straight;
  //当前路口是终点
  if(cross_->id==car_->goal) 
  {
    tonext.next_road=cur_road->id;
    tonext.direct=go_straight;
    return tonext;
  }  
  for(int i=car_->route-1;i>=0;i--)
  {
    //找到当前路口
    if(car_->cross_path[i]==cross_->id) 
    {  
      int map_s=cross_tosub(car_->cross_path[i],cross_dict_);
      int map_e=cross_tosub(car_->cross_path[i+1],cross_dict_);
      tonext.next_road=map_[map_s][map_e].id;
      
     
      if(tonext.next_road==-1) { std::cout<<"THE WAY IS NOT EXSIT??"<<std::endl; return tonext;}	
      
      if((tonext.next_road==cross_->road_id[0])&&(cur_road->id==cross_->road_id[1]))
      {
	  tonext.direct=turn_right;
	  return tonext;
      }
      else if((tonext.next_road==cross_->road_id[0])&&(cur_road->id==cross_->road_id[2]))
      {
	  tonext.direct=go_straight;
	  return tonext;	
      }
      else if((tonext.next_road==cross_->road_id[0])&&(cur_road->id==cross_->road_id[3]))
      {
	  tonext.direct=turn_left;
	  return tonext;	
      }
	else if((tonext.next_road==cross_->road_id[1])&&(cur_road->id==cross_->road_id[0]))
	{
	  tonext.direct=turn_left;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[1])&&(cur_road->id==cross_->road_id[2]))
	{
	  tonext.direct=turn_right;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[1])&&(cur_road->id==cross_->road_id[3]))
	{
	  tonext.direct=go_straight;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[2])&&(cur_road->id==cross_->road_id[0]))
	{
	  tonext.direct=go_straight;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[2])&&(cur_road->id==cross_->road_id[1]))
	{
	  tonext.direct=turn_left;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[2])&&(cur_road->id==cross_->road_id[3]))
	{
	  tonext.direct=turn_right;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[3])&&(cur_road->id==cross_->road_id[0]))
	{
	  tonext.direct=turn_right;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[3])&&(cur_road->id==cross_->road_id[2]))
	{
	  tonext.direct=turn_left;
	  return tonext;	  
	}
	else if((tonext.next_road==cross_->road_id[3])&&(cur_road->id==cross_->road_id[1]))
	{
	  tonext.direct=go_straight;
	  return tonext;	  
	}

	else continue;
    }
  }
     std::cout<<"can not find current in path!!"<< car_->id <<std::endl;out(" ERROR");
     sleep(5); 
  
  
  return tonext;
}

// 将所有终止态的车改为等待态

void chang_completed_towait(Car *car_array_,std::vector<int> &wait_list_,std::vector<int>&car_dist_)
{

  for(unsigned int sub=0;sub<car_dist_.size();sub++)
  {
    car_array_[sub].wait_anthor=false;
    car_array_[sub].not_allow_research=false;
    if(car_array_[sub].state==completed)
    {
      car_array_[sub].state=wait_schedule;
      wait_list_.push_back(car_dist_[sub]);
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
bool clear_cross_proir(Car* car_,Road* road_,Cross* cross_)
{
      if(road_->id==cross_->road_id[0]) {cross_->prior_uproad=0;return true ;}
	else if(road_->id==cross_->road_id[2]) {cross_->prior_downroad=0;return true ;}
	  else if(road_->id==cross_->road_id[3]) {cross_->prior_leftroad=0;return true ;}
	    else if(road_->id==cross_->road_id[1]) {cross_->prior_rightroad=0;return true ;}
	    
   return false ;
}

//检查是否有车未到终点 
bool All_car_isreached(Car* car_array,int car_num_)
{
  bool all_car_isreached=true;
   for(int i=0;i<car_num_;i++)
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
drive_toroad check_road_drive_space(Cross *cur_cross_,Road *road_,Car *car_array_ ,
				    std::vector<int>&car_dict_,std::vector<int>&road_dict_,int max_offset)
{
  
       drive_toroad road_situ;
       int j=0;
       for(int i=0;i<road_->lane_num;i++)
       {
	for(j=road_->road_length-1;j>max_offset-1;j--)
         {
	   int car_id=road_->load[(cur_cross_->id==road_->start)?0:1][i][j];
	    if(car_id!=0)   //查询该车位是否有车
	    {
	      int car_sub=car_tosub(car_id,car_dict_);
	      if(car_array_[car_sub].state==wait_schedule)
	      {
		road_situ.no_drive=0;
		road_situ.lane=-1;
		road_situ.offset=-1;
		return road_situ;
	      }
	      else if(car_array_[car_sub].state==completed)
	      {
		if(j==road_->road_length-1)
		{
		  if((i+1)>=road_->lane_num)
		  {
		    road_situ.no_drive=1;
		    road_situ.lane=-1;
		    road_situ.offset=-1;
		    return road_situ;
		  }
		  else{
		    break;
		  }
		}
		else
		{
		  road_situ.no_drive=1;
		  road_situ.lane=i;
		  road_situ.offset=j+1;
		  return road_situ;
		}
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
return road_situ; 

}
bool read_file(std::string cross_path, Cross *cross_array_,Cross *cross_sortedarray_,int* min_cross_id_,int* max_cross_id_,
	       std::string road_path, Road *road_array_,Road *road_sortedarray_,int* min_road_id_,int* max_road_id,
	       std::string car_path,Car *car_array_,Car *car_sortedarray_,int* min_car_id_,int* max_car_id_,Road map_[][CROSS_NUM]
	      ,std::vector<int>&car_dict_,std::vector<int>&road_dict_,std::vector<int>&cross_dict_,
	      int* min_roadlength_,int* max_roadlength_
	      )
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
    *min_roadlength_ =INF;
    *max_roadlength_ =-1000 ;
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
	    first_line=false; 
	    *min_road_id_= road_array_[0].id;
	    road_sortedarray_[0] = road_array_[0]=road_array_[0];
	  }
	road_dict_.push_back(road_array_[Subscript].id);
	road_array_[Subscript].completed=false;
	road_array_[Subscript].car_onroad=road_array_[Subscript].car_willonroad=0;
	road_array_[Subscript].best_space_per=1.0;
	if(road_array_[Subscript].road_length>(*max_roadlength_)) *max_roadlength_ =road_array_[Subscript].road_length;
	else  if(road_array_[Subscript].road_length<(*min_roadlength_)) *min_roadlength_ =road_array_[Subscript].road_length; 

	road_sortedarray_[Subscript] = road_array_[Subscript];
	Subscript++;
	count++;
         }
       }
       fin_road.close();
       *max_road_id =(Subscript-1);
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
	    first_line=false; 
	    *min_car_id_ = car_array_[0].id;;
	    car_sortedarray_[0] = car_array_[0] = car_array_[0];
	  }
	car_dict_.push_back(car_array_[Subscript].id);
	car_sortedarray_[Subscript] = car_array_[Subscript];
	car_array_[Subscript].state = still_stored;
	car_array_[Subscript].wait_anthor =false;
	car_array_[Subscript].now_road= car_array_[Subscript].next_road =-1;
	Subscript++;
	count++;
         }
       }
       fin_car.close();
       *max_car_id_=(Subscript-1);
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
	    first_line=false; 
	    *min_cross_id_ = cross_array_[0].id;
	    cross_sortedarray_[0] =cross_array_[0] = cross_array_[0];
	  } 
	cross_dict_.push_back(cross_array_[Subscript].id);
	cross_sortedarray_[Subscript] =cross_array_[Subscript];
	cross_sortedarray_[Subscript].up_cross_id=cross_sortedarray_[Subscript].down_cross_id
	=cross_sortedarray_[Subscript].left_cross_id=cross_sortedarray_[Subscript].right_cross_id=-1;
	Subscript++;
	count++;
         }
       }
       fin_cross.close();
       *max_cross_id_ = Subscript-1;
       std::cout<<"read finished"<<std::endl;
       
       //初始化地图 将所有路口连接置为-1 表示无连接 共有1～(cross_num-1) 个路口
       //数组以0下表开始 直观起见 map[i][j]直接指第i个路口到第j个路口信息
       sort(car_dict_.begin(),car_dict_.end());
       sort(road_dict_.begin(),road_dict_.end());
       sort(cross_dict_.begin(),cross_dict_.end());
       
       for(unsigned int i=0;i<cross_dict_.size();i++)
       {
	 //初始化路口的十字连通关系
	 cross_array_[i].up_cross_id = 
	    cross_array_[i].right_cross_id =
	      cross_array_[i].down_cross_id =
		cross_array_[i].left_cross_id=-1;
	 for(unsigned int j=0;j<=cross_dict_.size();j++)         
	    map_[i][j].id= -1;
       }
       

       return true;
}
void deal_with_car(std::vector<int>&car_dict_,Car* car_array_,Car* car_sortedarray_)
{
  int prior_speed_num=0;
  int prior_route_num=0;
  int prior_settime_num=0;
  for(unsigned int i=0;i<car_dict_.size();i++)
  {
    if(car_sortedarray_[i].max_speed>=4)
    {
      prior_speed_num++;
    }
    else if(car_sortedarray_[i].route<=8)
    {
       prior_route_num++;
    }
    else if(car_sortedarray_[i].set_time<=5)
    {
       prior_settime_num++;
    }
  }
  qsort(car_sortedarray_, car_dict_.size(), sizeof(car_sortedarray_[0]), campare_speed);
  qsort(car_sortedarray_,prior_speed_num, sizeof(car_sortedarray_[0]), campare_settime);
//    qsort(car_sortedarray_,prior_speed_num, sizeof(car_sortedarray_[0]), campare_carid);
    
  qsort(car_sortedarray_+prior_speed_num, car_dict_.size()-prior_speed_num, sizeof(car_sortedarray_[0]), campare_route);
  qsort(car_sortedarray_+prior_speed_num,prior_route_num, sizeof(car_sortedarray_[0]), campare_settime);
//      qsort(car_sortedarray_+prior_speed_num,prior_route_num, sizeof(car_sortedarray_[0]), campare_carid);
     
  qsort(car_sortedarray_+prior_speed_num+prior_route_num, car_dict_.size()-prior_speed_num-prior_route_num, 
	                                                 sizeof(car_sortedarray_[0]), campare_settime);
  qsort(car_sortedarray_+prior_speed_num+prior_route_num,car_dict_.size()-prior_speed_num-prior_route_num, 
	                                                 sizeof(car_sortedarray_[0]), campare_settime);
//   qsort(car_sortedarray_+prior_speed_num+prior_route_num,car_dict_.size()-prior_speed_num-prior_route_num, 
// 	                                                 sizeof(car_sortedarray_[0]), campare_carid);

}
void map_matrix(Cross* cross_array_,std::vector<int>&cross_dict_,std::vector<int>&road_dict_
  ,Road* road_array_,Road map_[][CROSS_NUM],struct MGraph &dijk_graph,int max_length,int min_length
,struct System_Para &para_)
{
 
  for(unsigned int i=0;i<cross_dict_.size();i++)
       {
	 for(unsigned int j=i+1;j<cross_dict_.size();j++)
	 {
	   if((cross_array_[i].road_id[0]!=-1)&&(cross_array_[i].road_id[0]==cross_array_[j].road_id[2])) 
	      {

		int road_sub=road_tosub(cross_array_[i].road_id[0],road_dict_);
		int init_w =road_array_[road_sub].road_length/(float)(max_length-min_length)  *para_.normalize_length_w;
		cross_array_[i].up_cross_id=cross_dict_[j];
		cross_array_[j].down_cross_id=cross_dict_[i];
		if(road_array_[road_sub].flag_twoway==1) {   

		  map_[j][i]=road_array_[road_sub];
		  map_[i][j]=road_array_[road_sub];
		  dijk_insert(dijk_graph,i,j,init_w);
		  dijk_insert(dijk_graph,j,i,init_w);
		}
		else if(road_array_[road_sub].end==cross_array_[i].id)
		{
		  map_[j][i]=road_array_[road_sub];
		  dijk_insert(dijk_graph,j,i,init_w);
		  map_[i][j].id=-1 ;
		}
		else{

		  map_[j][i].id=-1;
		  map_[i][j]=road_array_[road_sub];	
		  dijk_insert(dijk_graph,i,j,init_w);
		}
	      }
	   if((cross_array_[i].road_id[1]!=-1)&&(cross_array_[i].road_id[1]==cross_array_[j].road_id[3]))
	      { 
		int road_sub=road_tosub(cross_array_[i].road_id[1],road_dict_); 
		int init_w =road_array_[road_sub].road_length/(float)(max_length-min_length)  *para_.normalize_length_w;
		cross_array_[i].right_cross_id=cross_dict_[j];
		 cross_array_[j].left_cross_id=cross_dict_[i];
		if(road_array_[road_sub].flag_twoway==1){
		  map_[j][i]=road_array_[road_sub];                          
		  map_[i][j]=road_array_[road_sub];
		  dijk_insert(dijk_graph,i,j,init_w);
		  dijk_insert(dijk_graph,j,i,init_w);
		}
		else if(road_array_[road_sub].end==cross_array_[i].id)
		{
		  map_[j][i]=road_array_[road_sub];                             
		  map_[i][j].id=-1 ;
		  dijk_insert(dijk_graph,j,i,init_w);
		}
		else{
		  map_[j][i].id=-1;                            
		  map_[i][j]=road_array_[road_sub];
		  dijk_insert(dijk_graph,i,j,init_w);
		}
	      }
	   if((cross_array_[i].road_id[2]!=-1)&&(cross_array_[i].road_id[2]==cross_array_[j].road_id[0])) 
	      {
		int road_sub=road_tosub(cross_array_[i].road_id[2],road_dict_);
		int init_w =road_array_[road_sub].road_length/(float)(max_length-min_length)  *para_.normalize_length_w;
		cross_array_[i].down_cross_id=cross_dict_[j];
		cross_array_[j].up_cross_id=cross_dict_[i];
		if(road_array_[road_sub].flag_twoway==1){
		  map_[j][i]=road_array_[road_sub];                            
		  map_[i][j]=road_array_[road_sub];
		  dijk_insert(dijk_graph,i,j,init_w);
		  dijk_insert(dijk_graph,j,i,init_w);
		}
		else if(road_array_[road_sub].end==cross_array_[i].id)
		{
		  map_[j][i]=road_array_[road_sub];                             
		  map_[i][j].id=-1 ;
		  dijk_insert(dijk_graph,j,i,init_w);
		}
		else{
		  map_[j][i].id=-1;                            
		  map_[i][j]=road_array_[road_sub];
		  dijk_insert(dijk_graph,i,j,init_w);
		}
	      }
	   if((cross_array_[i].road_id[3]!=-1)&&(cross_array_[i].road_id[3]==cross_array_[j].road_id[1])) 
	      { 
		int road_sub=road_tosub(cross_array_[i].road_id[3],road_dict_);
                int init_w =road_array_[road_sub].road_length/(float)(max_length-min_length)  *para_.normalize_length_w;
		cross_array_[i].left_cross_id=cross_dict_[j];
		cross_array_[j].right_cross_id=cross_dict_[i];
		if(road_array_[road_sub].flag_twoway==1){
		  map_[j][i]=road_array_[road_sub];                            
		  map_[i][j]=road_array_[road_sub];
		  dijk_insert(dijk_graph,i,j,init_w);
		  dijk_insert(dijk_graph,j,i,init_w);
		}
		else if(road_array_[road_sub].end==cross_array_[i].id)
		{
		  map_[j][i]=road_array_[road_sub];                            
		  map_[i][j].id=-1 ;
		  dijk_insert(dijk_graph,j,i,init_w);
		}
		else{
		  map_[j][i].id=-1;                          
		  map_[i][j]=road_array_[road_sub];
		  dijk_insert(dijk_graph,i,j,init_w);
		}
	      }
	 }
       }
}

bool write_output(std::string path,Car *car_array_,int car_num_,Road map_[][CROSS_NUM],std::vector<int>&cross_dict_)
{
       std::ofstream fout(path, std::ios::app);
      if(!fout.is_open()) { std::cout<< "No output file" <<std::endl; return false;}
      std::fstream file(path, std::ios::out);
      fout <<"#(carId,StartTime,road...)"<<std::endl;
       for(int i=0;i<car_num_;i++)
     {
       int road_path;
       fout <<"("<<car_array_[i].id<<","<<car_array_[i].set_time<<",";          
       for(unsigned int j=0;j<cross_dict_.size();j++)
       {   
	 if(car_array_[i].cross_path[j+1]==0) break;
	 
	 int map_s=cross_tosub(car_array_[i].cross_path[j],cross_dict_);
	 int map_e=cross_tosub(car_array_[i].cross_path[j+1],cross_dict_);
	 //为考虑特殊情况 就一条路径？？
	 if(car_array_[i].cross_path[2]==0)
	 {
	   road_path=map_[map_s][map_e].id ;
	   fout << road_path<<")"<<std::endl;
	   break;
	 }

	  road_path=map_[map_s][map_e].id ;
	  fout <<road_path;
	  if(car_array_[i].cross_path[j+2]!=0) fout <<",";
	    else fout <<")"<<std::endl; 
        }   
      }
      fout.close();
      std::cout <<"Write ok!!!"<<std::endl;
      return true;
      
  
  /*
       std::ofstream fout(path, std::ios::app);
      if(!fout.is_open()) { std::cout<< "No output file" <<std::endl; return 0;}
      std::fstream file(path, std::ios::out);
      fout <<"#(carId,StartTime,road...)"<<std::endl;
     for(int i=min_car_id_;i<=max_car_id_;i++)
     {
       {
       int road_path,ga_roadpath;
       fout <<"("<<car_array_[i].id<<","<<car_array_[i].set_time<<",";          
       for(int j=0;j<MAX_CROSS;j++)
       {   
	 //为考虑特殊情况 就一条路径？？
	 if(car_array_[i].cross_path[2]==0)
	 {
	   road_path= map_[car_array_[i].cross_path[j]][car_array_[i].cross_path[j+1]].id ;
	  fout << road_path<<")"<<std::endl;
	   break;
	 }
	  if(car_array_[i].cross_path[j+1]==0) break;
	  road_path= map_[car_array_[i].cross_path[j]][car_array_[i].cross_path[j+1]].id ;
	  fout << road_path; 

	  if(car_array_[i].cross_path[j+2]!=0) fout <<",";
	    else fout <<")"<<std::endl; 
        }   
     for(int j=0;j<MAX_CROSS;j++)
       {
	 if(car_array_[i].cross_path[j+1]==0) 
	 { 
	   fout <<car_array_[i].cross_path[j];break;
	 }
	   else fout <<car_array_[i].cross_path[j]<<"--->";
       }
        fout <<std::endl;
	fout <<std::endl;}
      }	fout.close();*/
}


void ready_garage(std::vector<int>&car_dict_,std::vector<int>&road_dict_,std::vector<int>&cross_dict_,
		 Cross* cross_array,
		   Magic_garage* garage_,Road* road_array_,Car *car_array_,Car *car_sortedarray_,Road map_[][CROSS_NUM])
{
      for(unsigned int i=0;i<car_dict_.size();i++)
     {
       int ga_roadpath,cur_dup;   
	 //为考虑特殊情况 就一条路径？？
	 if(car_array_[i].cross_path[1]==0)
	 {  std::cout<<"The: "<<i<<"Has no path"<<std::endl;   break;}
	    
	    int car_sub=car_tosub(car_sortedarray_[i].id,car_dict_);
	    int cros_s_sub=cross_tosub(car_array_[car_sub].cross_path[0],cross_dict_);
	    int cros_e_sub=cross_tosub(car_array_[car_sub].cross_path[1],cross_dict_);
	    
	    ga_roadpath = map_[cros_s_sub][cros_e_sub].id;
	    int road_sub=road_tosub(ga_roadpath,road_dict_);
	    cur_dup = not_equal(car_array_[car_sub].set,
				road_array_[road_sub].start);
	    garage_[road_sub].garage[cur_dup].push_back(car_sortedarray_[i].id); 
      }
      
      /*********************************神奇车库  测试输出*********************************/
      
       for(unsigned int sch_cross_garage=0;sch_cross_garage<cross_dict_.size();sch_cross_garage++)
      {
        int cur_cross_road[4];
	  std::memcpy(cur_cross_road,cross_array[sch_cross_garage].road_id,sizeof(cross_array[sch_cross_garage].road_id));
	  int array_offset=3;
	for(int i=0;i<4;i++)
	  if(cur_cross_road[i]!=-1) 
	  {
	    int road_sub=road_tosub(cur_cross_road[i],road_dict_);
	    if(road_array_[road_sub].flag_twoway!=1)                     //该道路为单向道，且该路口不是起点
	      if(cross_dict_[sch_cross_garage]!=road_array_[road_sub].start)  cur_cross_road[i]=-1;
	  }
	  std::sort(cur_cross_road,cur_cross_road+4);    
	//去掉不存在的道路 或者不调度 不进入该交叉口的道路
	  for(;array_offset>=0;array_offset--)
	  {
	    if((array_offset==0)&&(cur_cross_road[array_offset]!=-1)) break;
	    if(cur_cross_road[array_offset]==-1) { array_offset+=1;  break;}
	  } 
	/******需要调度的道路id升序存于cur_cross_road中    起始下标为 array_offset******/
  // 		  std::cout <<sch_cross_garage<<" connect: ";
       std::cout<< "CROSS: "<< cross_dict_[sch_cross_garage]<<" "<<std::endl;
	for(int sch_road_garage_offset=array_offset;sch_road_garage_offset<4;sch_road_garage_offset++)
	{
	  int ro_sub =road_tosub(cur_cross_road[sch_road_garage_offset],road_dict_);
     for(int j=0;j<=1;j++)
       {
	std::vector<int>  copy_garage(garage_[ro_sub].garage[j]);
	 if(j==0) std::cout <<"   road: "<< road_dict_[ro_sub]<<"num:    "<<copy_garage.size() ;
  	  else  std::cout <<" road: "<< road_dict_[ro_sub]<<"num:   "<<copy_garage.size()<<std::endl;
//       while(copy_garage.size()>0)
// 	  {
// 	   std::cout<<copy_garage[0]<< "  ";
// 	    copy_garage.erase(copy_garage.begin());
// 	  }
       }
	  
	}
    std::cout<<std::endl;
      }
}
void init_waitanthor(Car* car_array,std::vector<int>&car_dict_,Road* road_array_,
		     std::vector<int>&road_dict_)
{
  for(unsigned int i=0;i<car_dict_.size();i++)
    car_array[i].wait_anthor=false;
  
  for(unsigned int i=0;i<road_dict_.size();i++)
    road_array_[i].completed=false;
}
void print_time(const char *head)
{
#ifdef DEBUG
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
void dis_cusroad(int road_sub,Road* road,std::vector<int>car_dict,Car* car,int T)
{
  		for(int lane=road[27].lane_num-1;lane>=0;lane--)
		{
		  for(int j=0;j<road[27].road_length;j++)
		  { 
		    if(road[27].load[0][lane][j]!=0)
		    {
		      int c_sub=car_tosub(road[27].load[0][lane][j],car_dict);
		      if(car[c_sub].state==completed) 
			std::cout<< road[27].load[0][lane][j]<<"("<<1<<")"<<  "  "; 
		      else std::cout<< road[27].load[0][lane][j]<<"("<<0<<")"<<  "  "; 
		    }
		      else
		    std::cout<< road[27].load[0][lane][j]<<"  "; 
		  }
		  std::cout<<std::endl;
		}	
		std::cout<<std::endl<<" "<< T<<" "<<road[27].front_car<<" ";
		int fr_sub =car_tosub(road[27].front_car,car_dict);
		if(car[fr_sub].state==completed) std::cout<<111<<std::endl;
		else std::cout<<"000"<<std::endl; 
		out(" ERROR");
		sleep(1);
}
bool sch_allcross_drive(Car* car_array,std::vector<int>&car_dict_,
			 Cross* cross_array,std::vector<int>&cross_dict_,
			 Road* road_array,std::vector<int>&road_dict_,
			 Magic_garage* garage,
			 Road map_[][CROSS_NUM],
			 int T,
			 std::vector<int> &wait_list_,
			 std::vector<int> &bloack_list_,int *reached_car_,int *wait_num_
			)
{    
	   init_waitanthor(car_array,car_dict_,road_array,road_dict_);
       for(unsigned int sch_cross_drive=0;sch_cross_drive<cross_dict_.size();sch_cross_drive++)
       {   
	   
	    //将所有车 的waitanthor初始化
// 	    std::cout<<std::endl;
	    //因为道路上的所有车都已是终止态 所以先调度那个方向的都可以 如果发车的可行驶距离超过道路长度 直接放到路口
	    /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
	    int cur_cross_road[4];
	    std::memcpy(cur_cross_road,cross_array[sch_cross_drive].road_id,sizeof(cur_cross_road));
	    int array_offset=3;
	  for(int i=0;i<4;i++)
	    if(cur_cross_road[i]!=-1)
	    {
	      int road_sub=road_tosub(cur_cross_road[i],road_dict_);
	      if(road_array[road_sub].flag_twoway!=1)                    
		if(cross_dict_[sch_cross_drive]!=road_array[road_sub].end)  cur_cross_road[i]=-1;
	    }
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
	       if(wait_list_.size()==0) return false; 
	       int road_sub=road_tosub(sch_car_road.next_road,road_dict_);
	       road_array[road_sub].front_car=-1;
	       sch_car_road= sch_most_prior(car_array, car_dict_,
			    &road_array[road_sub],road_dict_,
			    &cross_array[sch_cross_drive],cross_dict_,
			    sch_road_drive_offset,cur_cross_road,
			    cross_array,road_array,map_,
			    wait_list_,bloack_list_,T,reached_car_,wait_num_) ;
 #ifdef DEEP_DEBUG    
                       if(T>=9)
	                   {
	                   std::cout<<"      Cross: "<<cross_dict_[sch_cross_drive]<<" "
	                   <<"Road: "  <<sch_car_road.next_road<<"  " 
			   <<wait_list_.size()<<" || " <<T<<" || "<<(*wait_num_)<<"      |"<<(*reached_car_) 
			   <<std::endl;
                           sleep(1);
			   }
 #endif
		  if(sch_car_road.block==true)
		     {
		       return true;
		     }
	          if(sch_car_road.road_completed==true)
		   {   
		     //此处造成路口不能轮询周边道路
		     road_complete[sch_road_drive_offset]=1; 
		     sch_road_drive_offset++;
		     if(sch_road_drive_offset>=4){      break;}
		     sch_car_road.next_road=cur_cross_road[sch_road_drive_offset];
		   }
	           else
		    {
		      if(sch_car_road.next_road==-1) { out(" Logis error!!!"); out("here");sleep(5); return true; }
		      //该道路还没调度 优先级被抢
		      else
		       { 
		         road_complete[sch_road_drive_offset]=0;
		         sch_road_drive_offset++;
		         if(sch_road_drive_offset>=4) break;
		       }
		     }
	         }
	         if((road_complete[0]==1)&&(road_complete[1]==1)&&(road_complete[2]==1)&&(road_complete[3]==1))  break;
	         else
		  {
		     for(int i=array_offset;i<4;i++)
		      if(road_complete[i]!=1)
		      { sch_car_road.next_road = cur_cross_road[i]; break;}
		  }
	    }
	    
      }
  return false;
}
//更新当前路口的公共字段 输入：当前路口 当前道路
bool update_cross_prior_garage(int T_,Cross* cross_,std::vector<int>&cross_dict_,Road  *road_array_,Road* road_,std::vector<int>&road_dict_,std::vector<int>&car_dict_,
			       Car *car_array_,Cross *cross_array_,Road map_[][CROSS_NUM],
			       int cur_road_sub_,int map_e,int map_s)
{

    if(map_[map_s][map_e].id==-1) return false;   

    int best_space_car=road_->limit_speed*road_->lane_num;
    bool check=false;
    //clear something 
    map_[map_s][map_e].car_onroad=0;
    map_[map_s][map_e].car_willonroad=0;
    map_[map_s][map_e].will_gocross=0;
  for(int j=0;j<road_->road_length;j++)
   {
    for(int i=0;i<road_->lane_num;i++)
    {  
      if(road_->load[(cross_->id==road_->start)?0:1][i][j]!=0)   //查询该车位是否有车
      {         
	 map_[map_s][map_e].car_onroad++; 
       if((j>=0)&&(j<road_->limit_speed))
       { 
        int car_id=road_->load[(cross_->id==road_->start)?0:1][i][j]; 
	int car_sub=car_tosub(car_id,car_dict_);
	if(j-min(car_array_[car_sub].max_speed,road_->limit_speed)<0)
	{
	  map_[map_s][map_e].will_gocross++;
	  //下一时刻可以过路口    
	  how_tonext to_next=next_road_drive(&car_array_[car_sub],
				        &road_array_[cur_road_sub_],
				          cross_array_,cross_dict_,
				          &cross_array_[map_e],map_);
	  if(car_array_[car_sub].goal==cross_array_[map_e].id)
	  {
	  check=true;
	  if(road_->id==cross_array_[map_e].road_id[0]) 
	   cross_array_[map_e].prior_uproad=car_id;
	      else if(road_->id==cross_array_[map_e].road_id[2]) 
		cross_array_[map_e].prior_downroad=car_id;
		else if(road_->id==cross_array_[map_e].road_id[3]) 
		  cross_array_[map_e].prior_leftroad=car_id;
		  else if(road_->id==cross_array_[map_e].road_id[1]) 
		    cross_array_[map_e].prior_rightroad=car_id;
	  }
	  else if(to_next.next_road==-1){ 
	    car_array_[car_sub].move_ori =go_straight;
	    car_array_[car_sub].next_road=road_->id;
	    std::cout << "Attention in garage: "<<car_id <<" "<<cross_->id<<" "<<road_->id<<" "
	    <<std::endl;
	  }
	  else{
	  int road_next=road_tosub(to_next.next_road,road_dict_);
	  int next_map_e=cross_tosub(((cross_dict_[map_e]==road_array_[road_next].end)?
	                              road_array_[road_next].start:road_array_[road_next].end),
				        cross_dict_);
          
	  map_[map_e][next_map_e].car_willonroad++;  
	  car_array_[car_sub].move_ori =to_next.direct ;
	  car_array_[car_sub].next_road=to_next.next_road ;
	  if(!check)
	  {
	  check=true;
	  if(road_->id==cross_array_[map_e].road_id[0]) 
	   cross_array_[map_e].prior_uproad=car_id;
	      else if(road_->id==cross_array_[map_e].road_id[2]) 
		cross_array_[map_e].prior_downroad=car_id;
		else if(road_->id==cross_array_[map_e].road_id[3]) 
		  cross_array_[map_e].prior_leftroad=car_id;
		  else if(road_->id==cross_array_[map_e].road_id[1]) 
		    cross_array_[map_e].prior_rightroad=car_id;
	  }
	  }
	}
// 	return true;
      
    }
    else if(j>=road_->road_length-road_->limit_speed) best_space_car--;
    }
    }

   }
   map_[map_s][map_e].best_space_per= best_space_car/(float)(road_->limit_speed*road_->lane_num);
   
   return true;
}


void sch_allcross_garage(Car* car_array,
			 Cross* cross_array_,std::vector<int>&cross_dict_,
			 Road* road_array,std::vector<int>&road_dict_,std::vector<int>&car_dict_,
			 Magic_garage* garage,
			 Road map_[][CROSS_NUM],
			 int T,int *wait_num_ ,int *reached_car_,int car_num_,struct MGraph &dijk_graph
			 ,int* min_roadlength,int* max_roadlength,struct System_Para &para_
			)
{      
  for(unsigned int sch_cross_garage=0;sch_cross_garage<cross_dict_.size();sch_cross_garage++)
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
	  {
	    int road_sub=road_tosub(cur_cross_road[i],road_dict_);
	    if(road_array[road_sub].flag_twoway!=1)                     //该道路为单向道，且该路口不是起点
	      if(cross_dict_[sch_cross_garage]!=road_array[road_sub].start)  cur_cross_road[i]=-1;
	  }
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
	  int road_sub=road_tosub(cur_cross_road[sch_road_garage_offset],road_dict_);
	  int map_e=cross_tosub(((cross_dict_[sch_cross_garage]==road_array[road_sub].end)
	                                   ?road_array[road_sub].start:road_array[road_sub].end),cross_dict_);
	  //判断该路的车库是否有车调度
	  int cur_dup=not_equal(cross_dict_[sch_cross_garage],road_array[road_sub].start);

	  if(!garage[road_sub].garage[cur_dup].empty())
		{   	      
		      int set_now=0;  
		      //检查道路最高优先级车道 以及偏移量
		    road_space space_Condition = check_road_space(&cross_array_[sch_cross_garage],
								  &road_array[road_sub]);
  // 			       std::cout << "test: "<<space_Condition.lane <<std::endl;
		    if(space_Condition.lane==-1) continue;
  // 			      std::cout<<"sch_road_garage "<< sch_sch_allcross_garageroad_garage<< " "<<cur_dup;
		    //道路是否有余量
		    int cur_dup=not_equal(cross_dict_[sch_cross_garage],road_array[road_sub].start);
		    //循环的条件 道路非满且车库非空 优先级最高的车的发车时间不大于该时刻
		    int MAX_CAR=0;		      float limit_per;
		    if(T>para_.T_SOFT) 
		      MAX_CAR=para_.max_car_road*(1.0+(T-para_.T_SOFT)*(T-para_.T_SOFT)*para_.T_SOFT_RATE);
		      else MAX_CAR=para_.max_car_road;
		   
		    while((*wait_num_-*reached_car_)<MAX_CAR && (space_Condition.lane!=-1)
			    &&(!garage[road_sub].garage[cur_dup].empty())
			&&((car_array[car_tosub(garage[road_sub].garage[cur_dup][0],car_dict_)].set_time)<=T)
		      )
		    {
		      int cr_e_sub =cross_tosub(((cross_dict_[sch_cross_garage]==road_array[road_sub].end)?
			road_array[road_sub].start:road_array[road_sub].end),  cross_dict_);

                        if(para_.road_percent-T*para_.DECAY>para_.min_road_per)
			    limit_per=para_.road_percent-T*para_.DECAY ;
		        else limit_per=para_.min_road_per; 
		      if((map_[sch_cross_garage][cr_e_sub].car_onroad/((float)(road_array[road_sub].lane_num
			*road_array[road_sub].road_length))>limit_per)) break;
			  (*wait_num_)++;  
			    set_now++;
			  int cur_dup=not_equal(cross_dict_[sch_cross_garage],road_array[road_sub].start);
			  std::vector<int> car_garage(garage[road_sub].garage[cur_dup]);
			  int car_sub = car_tosub(car_garage[0],car_dict_);

			  //向量拷贝 只是为了让后面看起来短一点 没什么太大软用
			  
			  int how_far = min(road_array[road_sub].road_length-space_Condition.offset,
					    min(car_array[car_sub].max_speed,road_array[road_sub].limit_speed));
			  int space_offset = road_array[road_sub].road_length-how_far;
			    if(space_offset<0)  space_offset=0;
			  map_[sch_cross_garage][cr_e_sub].car_onroad++;
			    /**** 终于将车安排上了 更新车道数组****/
			  road_array[road_sub].load[cur_dup][space_Condition.lane][space_offset]=car_garage[0];
			  /**** 终于将车安排上了 更新车道数组****/
			  /**** 上路后还有一系列操作 比如 ***/
			    /***写car结构体中的state now_road next_road move_ori settime ****/
			  car_array[car_sub].set_time=T;
			  car_array[car_sub].now_road=sch_road_garage;
			  car_array[car_sub].state=completed;
			  //发车成功  将其从车库中删掉
			  garage[road_sub].
			  garage[cur_dup].erase(garage[road_sub].
			      garage[cur_dup].begin());
		      space_Condition = check_road_space(&cross_array_[sch_cross_garage],&road_array[road_sub]);
		    }
	}
	  //更新路口的公共字段
	  update_cross_prior_garage(T,&cross_array_[sch_cross_garage],cross_dict_, road_array,
				    &road_array[road_sub],road_dict_,car_dict_
				      ,car_array,cross_array_,map_,road_sub,map_e ,sch_cross_garage);
	   //change current road weight
	  int to_cross=((cross_array_[sch_cross_garage].id==road_array[road_sub].start)?
	      road_array[road_sub].end:road_array[road_sub].start);
	  int to_cross_sub = cross_tosub( to_cross ,cross_dict_);
	  if(map_[sch_cross_garage][to_cross_sub].id==-1){dijk_insert(dijk_graph,sch_cross_garage,to_cross_sub,INF); continue;}
	  
	  float normal_roadlength =  road_array[road_sub].road_length/((float)
	                     (* max_roadlength)-(* min_roadlength))                    *para_.normalize_length_w;
			     
	  float length_di_speed =  road_array[road_sub].road_length/((float)
	                     (* max_roadlength)-(* min_roadlength)) /
	                      ((float)road_array[road_sub].limit_speed)                  * para_.length_di_speed ;
			      
	  int best_space =   (1-map_[sch_cross_garage][to_cross_sub].best_space_per)         *para_.best_space_w;
	                       
	  
	  int car_onroad = map_[sch_cross_garage][to_cross_sub].car_onroad  
	                      /(float)road_array[road_sub].road_length
	                        * road_array[road_sub].lane_num                           * para_.car_onroad_w;
	                        
	  int garage_size = garage[road_sub].garage[cur_dup].size()                      * para_.garage_size_w ;
	  
	  int car_willonroad =  map_[sch_cross_garage][to_cross_sub].car_willonroad       * para_.car_willonroad;
	  
	  int new_weight = 	normal_roadlength+  length_di_speed +  
	                        best_space + car_onroad+ garage_size + car_willonroad;
#ifdef DEBUG				
            std::cout << "Road: "<<map_[sch_cross_garage][to_cross_sub].id<<"   "
			         <<"   "<<YELLOW<<normal_roadlength <<"   "
			         <<CYAN<<length_di_speed<<"   "
			         <<LIGHT_BLUE<<best_space<<"   "
			         <<GREEN<< map_[sch_cross_garage][to_cross_sub].car_onroad<<"   "
				 <<PURPLE<<garage_size<<"   "
				 <<DARY_GRAY<<car_willonroad<<"   -------->   " <<RED<< new_weight;
				 std::cout<<WHITE<<std::endl;
#endif
	 if(new_weight<=0) new_weight=map_[sch_cross_garage][to_cross_sub].road_length*0.3;
	  dijk_insert(dijk_graph,sch_cross_garage,to_cross_sub,new_weight);
	}
     } 

	
      
}

void debug_dir_tocross(std::vector<int>&road_dict_,Road *road_array_,std::vector<int>&cross_dict_,Cross *cross_array_,struct MGraph &dijk_graph)
{

    	      for(unsigned int test_cross=0;test_cross<cross_dict_.size();test_cross++)
	      {
		/******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
		int cur_cross_road[4];
		std::memcpy(cur_cross_road,cross_array_[test_cross].road_id,sizeof(cross_array_[test_cross].road_id));
		int array_offset=3;
	      for(int i=0;i<4;i++)
		if(cur_cross_road[i]!=-1)
		{
		  int road_sub=road_tosub(cur_cross_road[i],road_dict_);
		  if(road_array_[road_sub].flag_twoway!=1)                      //该道路为单向道，且该路口不是起点
		    if(cross_dict_[test_cross]!=road_array_[road_sub].start)  cur_cross_road[i]=-1;
		}
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
		int road_sub=road_tosub(test_road,road_dict_);
		if(road_array_[road_sub].end==cross_dict_[test_cross])
		 std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"-------"<<road_array_[road_sub].start<<std::endl;
		else std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"-------"<<road_array_[road_sub].end<<std::endl;
		for(int lane=road_array_[road_sub].lane_num-1;lane>=0;lane--)
		{
		  for(int j=0;j<road_array_[road_sub].road_length;j++)
		  { 
		    std::cout<< road_array_[road_sub].load[(cross_dict_[test_cross]==road_array_[road_sub].end)?0:1][lane][j]<<"  "; 
		  }
		  std::cout<<std::endl;
		}
		int s=cross_tosub((cross_array_[test_cross].id==road_array_[road_sub].end)?road_array_[road_sub].start:road_array_[road_sub].end ,cross_dict_);
		if(road_array_[road_sub].end==cross_dict_[test_cross])
		 std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"----"<<dijk_graph.edges[s][test_cross]<<"---"<<road_array_[road_sub].start<<std::endl;
		else std::cout <<cross_array_[test_cross].id<<"<-------"<<cur_cross_road[test_offset]<<"----"<<dijk_graph.edges[s][test_cross]<<"---"<<road_array_[road_sub].end<<std::endl;
	
		 std::cout<<std::endl<<std::endl;
	      }
	      std::cout<<std::endl;
	      }
	      for(unsigned int i=0;i<cross_dict_.size();i++)
		{std::cout<< "Cross: "<<cross_dict_[i]<< " "<<  cross_array_[i].prior_uproad <<" "
	     <<  cross_array_[i].prior_downroad<<" "
	     <<  cross_array_[i].prior_leftroad<<" "
	     <<  cross_array_[i].prior_rightroad ;
         std::cout<<std::endl;}

}

bool print_map(int coord_max,Car* car_)
{
  
  int cross_id=0;
  int flag=0;
     cross_id=0;
  for(int i=1;i<=coord_max;i++)
  {
    for(int j=1;j<=coord_max;j++)
    {
      cross_id++;
      for(int k=0;k<CROSS_NUM;k++)
      {
	if(car_->cross_path[k]==cross_id) { flag=1; break;}
	 if(car_->cross_path[k]==0) break;
      }
      if(flag==1) 
      {
	flag=0;
	if(i==1)
	    {
	if(cross_id==car_->set) std::cout<<YELLOW<< cross_id << "   ";
	else if (cross_id==car_->goal) std::cout<<RED<< cross_id << "   ";
	else std::cout<<GREEN<< cross_id << "   ";
	  }
	  else
	  {
	if(cross_id==car_->set) std::cout<<YELLOW<< cross_id << "  ";
	else if (cross_id==car_->goal) std::cout<<RED<< cross_id << "  ";
	else std::cout<<GREEN<< cross_id << "  ";
	  }
      }
      else
      {
	if(i==1) std::cout<<WHITE<< cross_id << "   ";
	  else 
            std::cout<<WHITE<< cross_id << "  ";
      }
    }
    std::cout<<std::endl<<std::endl;
  }
  std::cout<<std::endl<<std::endl;
  return true;
}

  
   