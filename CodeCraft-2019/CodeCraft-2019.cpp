#include <iostream>
#include <unistd.h>
#include <memory>
#include "include.h"
Car  car[CAR_NUM],car_sorted[CAR_NUM];
Road road[ROAD_NUM],road_sorted[ROAD_NUM];
Magic_garage garage[ROAD_NUM];
Cross cross[CROSS_NUM],cross_sorted[CROSS_NUM];

Road map[CROSS_NUM][CROSS_NUM];
bool research_best_way(struct MGraph &dijk_graph,Car* car_array_,std::vector<int>&car_dict_,
		       std::vector<int>&road_dict_,Road* road_array_,Cross* cross_array_,
		       std::vector<int>&cross_dict_,Road map_[][CROSS_NUM]
		      ,int T_, struct System_Para &para_,struct DIJK_map &dijk_map);
int main(int argc,char** argv)
{
 
       bool block_flag=true;
      //神奇车库数组 存放每条道路处的神奇车库

      int T=0; 
      // 路口到路口的权重网络
      int weight_net[CROSS_NUM][CROSS_NUM];
      memset(car,0,sizeof(Car));memset(car_sorted,0,sizeof(Car));memset(cross,0,sizeof(Cross));
      memset(road,0,sizeof(Road));memset(road_sorted,0,sizeof(road_sorted));memset(cross_sorted,0,sizeof(Cross));
      memset(garage,0,sizeof(Magic_garage));memset(map,0,sizeof(Road));memset(cross_sorted,0,sizeof(Cross));
      memset(weight_net,0,sizeof(weight_net));
      int min_car_id=0,max_car_id=0;
      int min_road_id=0,max_road_id=0;
      int min_cross_id=0,max_cross_id=0;
      int max_roadlength=0,min_roadlength=0;
      print_time("Begin");
      // 道路（边）总数 路口（节点）数 车辆数
       int road_num=0,cross_num=0,car_num=0;
      std::cout << "Begin" << std::endl;
      if(argc < 5)
	{
	  std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
	  exit(1);
	}
      std::string carPath(argv[1]);
      std::string roadPath(argv[2]);
      std::string crossPath(argv[3]);
      std::string answerPath(argv[4]);


      std::vector<int>car_dict;
      std::vector<int>road_dict;
      std::vector<int>cross_dict;
      
      if(!read_file(crossPath,cross,cross_sorted,&min_cross_id,&max_cross_id,
	            roadPath,road,road_sorted,&min_road_id,&max_road_id,
                    carPath,car,car_sorted,&min_car_id,&max_car_id,map,
		    car_dict,road_dict,cross_dict,
		    &min_roadlength,&max_roadlength
 		  ))    return 0;
      out("here");
      road_num=road_dict.size(),
      cross_num=cross_dict.size(),
      car_num=car_dict.size();
      std::cout<<"Car: "<< car_num

	  <<" Road: "<< road_num

	  <<" Cross: "<< cross_num<<std::endl;  
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/

      System_Para para;
      MGraph dijk_g;
      DIJK_map dijk_map;
      dijk_g.cross_num=cross_num;
      dijk_g.road_num=road_num;
      init_MGraph(dijk_g);
      init_Para(para);
      map_matrix(cross,cross_dict,road_dict,road,map,dijk_g,max_roadlength,min_roadlength,para);   
/*********************************A-Star算法  + 神奇车库*********************************/
      int has_find=0; 

      print_time("Begin");
      
      memset(dijk_map.map,-1,sizeof(dijk_map.map));

     for(int i=0;i<car_num;i++)
     {
	int start_to_sub=cross_tosub(car[i].set,cross_dict);
	int end_to_sub=cross_tosub(car[i].goal,cross_dict);   

	int best_next=dijk_search(dijk_g,start_to_sub,end_to_sub,&car[i],
				  road_dict,cross_dict,road,map,para);
	
	
	int rod_sub= road_tosub(map[start_to_sub][best_next].id,road_dict);
	for(unsigned int k=0;k<cross_dict.size();k++)
	{
	  if(car[i].cross_path[k+1]==0)  break;
	  
	  int cr1_sub=cross_tosub(car[i].cross_path[k],cross_dict); 
	  int cr2_sub=cross_tosub(car[i].cross_path[k+1],cross_dict); 
	  int rosub=road_tosub(map[cr1_sub][cr2_sub].id,road_dict);
	  int new_w =  dijk_g.edges[cr1_sub][cr2_sub] 
		      +   (road[rod_sub].road_length*(float)road[rod_sub].lane_num)              * para.T1_roadlenghtspace_w
	              +   map[start_to_sub][best_next].car_willonroad                            * para.car_willonroad 
				  ;
	  if(new_w<0) {std::cout<<"warning ";new_w=road[rosub].road_length*0.5;}			 
	  dijk_insert(dijk_g,cr1_sub,cr2_sub,new_w );
	}
	car_sorted[i].route=car[i].route;
	has_find++;
     }
     //deal with the data
     deal_with_car(car_dict,car,car_sorted);
     std::cout<<"find "<< has_find<< " solutions of  "<<car_num<<" car"<<std::endl;
     //安排神奇车库
     ready_garage(car_dict,road_dict,cross_dict, cross,garage,road,car,car_sorted,map);
/******************************车辆调度规则执行******************************/
	  //记录时刻 
	  
	  //等待表;
	  std::vector<int> wait_list; 
	  //检锁表
          std::vector<int> block_list; 
	  int wait_num=0;                        
	  block_flag=false;
	  int reached_car=0;                     
	  T=0;
	  srand(time(NULL));
	  unsigned int wait_car=0;
	  int wait_sametime=0;
	  //检查是否全部到终点
	  std::cout<<"---------------------------------------------------------"<<std::endl;
	  std::cout<<"---------------------------------------------------------"<<std::endl;
// 	  sleep(2);
	  while(!All_car_isreached(car,car_num))
	  {
	    //开始走表 
		T++;      
	    //将所有终止态车变为等待态 并加入等待表
	      chang_completed_towait(car,wait_list,car_dict);
	    //调度全地图等待车车 直到等待表为空
	      while(!wait_list.empty())
	      {
// 		wait_num=wait_list.size();
	      if(wait_car==wait_list.size())
	       {
		 wait_sametime++;  
		 if(wait_sametime>200) 
		 { out("BLOCKED"); 
		   debug_dir_tocross(road_dict, road,cross_dict,cross,dijk_g); 
		   block_flag=true;break;}
	        }else  wait_sametime=0; 
		block_flag=sch_allcross_drive(car,car_dict,
					      cross, cross_dict,
				              road,road_dict,
				              garage,map,T,wait_list,block_list,&reached_car,&wait_num,dijk_g);
		wait_car=wait_list.size();
		if(block_flag) break;
	      }    
	      if(block_flag){ std::cout<<"SCH out block!!!"<<std::endl;  break;} 
	      //正常跳出while 表明 无车等待 尝试调度车库
              sch_allcross_garage(car,cross,cross_dict,road,road_dict,car_dict,
				  garage,map,T,&wait_num,&reached_car,car_num,dijk_g,
				  &min_roadlength,&max_roadlength,para
				 );
	       if(T>1)
	        {
	          research_best_way(dijk_g,car,car_dict,road_dict,road,cross,cross_dict,map,T,para,dijk_map);
		  std::cout<<"On road : "<<(wait_num-reached_car)<<" || " 
		  <<T<<" || "<<(wait_num)<<"      |"<<(reached_car) 
		  <<std::endl<< std::endl; 
		}
	  } 

	    std::cout<<"Time Schdule: "<<" | "<<T<<" | "<<std::endl; 
            if(!write_output(answerPath,car,car_num,map,cross_dict)) return 0;
            print_time("End");
/******************************车辆调度规则执行******************************/

	//调试路口调度时 驶向路口道路信息
// 	debug_dir_tocross(road_dict, road,cross_dict,cross);  
 

  
      return 0;
}
bool research_best_way(struct MGraph &dijk_graph,Car* car_array_,std::vector<int>&car_dict_,
		       std::vector<int>&road_dict_,Road* road_array_,Cross* cross_array_,
		       std::vector<int>&cross_dict_,Road map_[][CROSS_NUM],int T_, struct System_Para &para_
		      ,struct DIJK_map &dijk_map)
{
//   calculate all map route
  memset(dijk_map.map,-1,sizeof(dijk_map.map));
  for(unsigned int from=0;from<cross_dict_.size();from++)
  {
    for(unsigned int to=0;to<cross_dict_.size();to++)
    {
      if(from!=to)  
      {
      update_map(dijk_map,from,to,dijk_graph,para_);
      }
    }
  }
  
  int re_num=0;
  for(unsigned int car_sub=0;car_sub<car_dict_.size();car_sub++)
  {
 
    if((car_array_[car_sub].state==still_stored)||(car_array_[car_sub].state==reached)) continue; 
      else if(car_array_[car_sub].next_road==car_array_[car_sub].now_road)  continue;
    if(car_array_[car_sub].not_allow_research==true) continue;
    //reschedule the route:
    if(((car_array_[car_sub].now_road>road_dict_[0])&&car_array_[car_sub].now_road<road_dict_[road_dict_.size()-1])
	&&((car_array_[car_sub].next_road>road_dict_[0])&&car_array_[car_sub].next_road<road_dict_[road_dict_.size()-1]))
      {
	int now_cross=-1;
	int now_roadsub = road_tosub(car_array_[car_sub].now_road,road_dict_);
	int next_roadsub =road_tosub(car_array_[car_sub].next_road,road_dict_);
	if(road_array_[now_roadsub].end==road_array_[next_roadsub].start) now_cross=road_array_[now_roadsub].end;
	else if(road_array_[now_roadsub].end==road_array_[next_roadsub].end) now_cross=road_array_[now_roadsub].end;
	else if(road_array_[now_roadsub].start==road_array_[next_roadsub].end) now_cross=road_array_[now_roadsub].start;
	else if(road_array_[now_roadsub].start==road_array_[next_roadsub].start) now_cross=road_array_[now_roadsub].start;
	else  {  continue;}
	int pre_cross=now_cross;
	int next_cross=-1;

         int ori_sub=0;
	for(int k=car_array_[car_sub].route-1;k>=0;k--)
	{
	  if(car_array_[car_sub].cross_path[k]==0){ 
	    std::cout<<"error"<<std::endl;
	    break;
	  }
	  
	 if(car_array_[car_sub].cross_path[k]==now_cross) 
	  { 
	   ori_sub=k;
	   next_cross=car_array_[car_sub].cross_path[k+1];
	   pre_cross=car_array_[car_sub].cross_path[k-1];
	   break;
	  }
        }
	  
	if(next_cross==car_array_[car_sub].goal) continue;
	int pre_cross_sub = cross_tosub(pre_cross,cross_dict_);
	int temp_w_save;

	int now_cross_sub = cross_tosub(now_cross,cross_dict_);  
	int goal_sub = 	cross_tosub(car_array_[car_sub].goal,cross_dict_);
	

	  temp_w_save =dijk_graph.edges[now_cross_sub][pre_cross_sub]; 
 
	// not allowed back
	  dijk_graph.edges[now_cross_sub][pre_cross_sub]=INF; 
	
          dijk_research(dijk_map,pre_cross_sub,dijk_graph,now_cross_sub,goal_sub,now_roadsub,&car_array_[car_sub],
		      cross_array_,road_array_,road_dict_,cross_dict_,map_,ori_sub,para_);
	
	  dijk_graph.edges[now_cross_sub][pre_cross_sub]=temp_w_save;

      }
      
      re_num++;
  }



  return true;
}
