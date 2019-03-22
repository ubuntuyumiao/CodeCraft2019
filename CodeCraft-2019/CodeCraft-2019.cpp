#include <iostream>
#include <unistd.h>
#include "include.h"
Car  car[car_to_sub(MAX_CAR_ID+1)],car_sorted[car_to_sub(MAX_CAR_ID+1)];
Road road[road_to_sub(MAX_ROAD_ID+1)],road_sorted[road_to_sub(MAX_ROAD_ID+1)];
Magic_garage garage[road_to_sub(MAX_ROAD_ID+1)];
Cross cross[MAX_CROSS],cross_sorted[MAX_CROSS];

Road map[MAX_CROSS][MAX_CROSS];
//神奇车库数组 存放每条道路处的神奇车库

// 道路（边）总数 路口（节点）数 车辆数
int road_num=0,cross_num=0,car_num=0;
// 路口到路口的权重网络
int weight_net[MAX_CROSS][MAX_CROSS];

int main(int argc,char** argv)
{
      int min_car_id=0,max_car_id=0;
      int min_road_id=0,max_road_id=0;
      int min_cross_id=0,max_cross_id=0;
      print_time("Begin");
 
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

      if(!read_file(crossPath,cross,cross_sorted,&min_cross_id,&max_cross_id,
	            roadPath,road,road_sorted,&min_road_id,&max_road_id,
                    carPath,car,car_sorted,&min_car_id,&max_car_id,map))    return 0;
      
      road_num=max_road_id-min_road_id+1,
      cross_num=max_cross_id-min_cross_id+1,
      car_num=max_car_id-min_car_id+1;
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/
      map_matrix(cross,min_cross_id,max_cross_id,weight_net,road,map);    
/*********************************A-Star算法  + 神奇车库*********************************/ 
      int has_find=0; 
      quickSortOfCpp(car_sorted,car_to_sub(min_car_id),car_to_sub(max_car_id));
     for(int i=min_car_id;i<=max_car_id;i++)
     {
       if( Astar_search(&car[car_to_sub(i)], road,road_to_sub(min_road_id),road_to_sub(max_road_id),
	                cross,min_cross_id,max_cross_id,weight_net,map))
            has_find++;
     }
     //安排神奇车库
     ready_garage(car_to_sub(min_car_id),car_to_sub(max_car_id),garage,road,car, car_sorted, map);
/*********************************A-Star算法  + 神奇车库*********************************/
     

/******************************车辆调度规则执行******************************/
	  //记录时刻   
	  int T=0;  
	  //等待表;
	  std::vector<int> wait_list; 
	  //检锁表
          std::vector<int> block_list; 
	  bool block_flag=false;
	  int wait_num=0; 
	  //检查是否全部到终点
	  while(!All_car_isreached(car,car_to_sub(min_car_id),car_to_sub(max_car_id)))
	  {
	    //开始走表 
		T++;      
	    //将所有终止态车变为等待态 并加入等待表
	      chang_completed_towait(car_to_sub(min_car_id),car_to_sub(max_car_id),car,wait_list);
	      wait_num=wait_list.size();
	    //调度全地图等待车车 直到等待表为空
	      while(!wait_list.empty())
	      {
		block_flag=sch_allcross_drive(car,car_to_sub(min_car_id),car_to_sub(max_car_id),
					      cross, min_cross_id, max_cross_id,
				              road_to_sub(min_road_id),road_to_sub(max_road_id),
				              road,garage,map,T,wait_list,block_list);
		if(block_flag) break;
	      }
	      if(block_flag){ std::cout<<"SCH out block!!!"<<std::endl;  break;} 
// 	      if(T==2) break;
	      //正常跳出while 表明 无车等待 尝试调度车库
              sch_allcross_garage(car,cross, min_cross_id, max_cross_id,road,garage,map,T,&wait_num);
	  } 
	  
/******************************车辆调度规则执行******************************/
	//调试发车时 驶离开路口道路信息
// 	debug_dir_leavecross(road,min_cross_id,max_cross_id,cross);
	//调试路口调度时 驶向路口道路信息
// 	debug_dir_tocross(road,min_cross_id,max_cross_id,cross);  
      //写输出文件
      if(!write_output(answerPath,car,car_to_sub(min_car_id), car_to_sub(max_car_id),map)) return 0;
      print_time("End");
      return 0;
}

