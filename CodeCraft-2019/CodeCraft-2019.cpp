#include <iostream>
#include <unistd.h>
#include "include.h"

Car  car[CAR_NUM],car_sorted[CAR_NUM];
Road road[ROAD_NUM],road_sorted[ROAD_NUM];
Magic_garage garage[ROAD_NUM];
Cross cross[CROSS_NUM],cross_sorted[CROSS_NUM];

Road map[CROSS_NUM][CROSS_NUM];
//神奇车库数组 存放每条道路处的神奇车库


// 路口到路口的权重网络
int weight_net[CROSS_NUM][CROSS_NUM];

int main(int argc,char** argv)
{
      int min_car_id=0,max_car_id=0;
      int min_road_id=0,max_road_id=0;
      int min_cross_id=0,max_cross_id=0;
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
		    car_dict,road_dict,cross_dict
 		  ))    return 0;
      road_num=road_dict.size(),
      cross_num=cross_dict.size(),
      car_num=car_dict.size();
      std::cout<<"Car: "<< car_num

	  <<" Road: "<< road_num

	  <<" Cross: "<< cross_num<<std::endl;  
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/
      map_matrix(cross,cross_dict,road_dict,weight_net,road,map);   
/*********************************A-Star算法  + 神奇车库*********************************/
      int has_find=0; 
      quickSortOfCpp(car_sorted,car_num);

     for(int i=0;i<car_num;i++)
     {
       if( Astar_search(&car[i], road,road_dict,
	                cross,cross_dict,weight_net,map))
            has_find++;
//        print_map(8,&car[i]);
//        sleep(1);
     }
     std::cout<<"find "<< has_find<< " solutions of  "<<car_num<<" car"<<std::endl;
  
     //安排神奇车库
     ready_garage(car_dict,road_dict,cross_dict, garage,road,car,car_sorted,map);
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
	  int reached_car=0;
	  srand(time(NULL));
	  //检查是否全部到终点
	  while(!All_car_isreached(car,car_num))
	  {
	    //开始走表 
		T++;      
	    //将所有终止态车变为等待态 并加入等待表
	      chang_completed_towait(car,wait_list,car_dict);
	    //调度全地图等待车车 直到等待表为空
	      while(!wait_list.empty())
	      {
		wait_num=wait_list.size();
		block_flag=sch_allcross_drive(car,car_dict,
					      cross, cross_dict,
				              road,road_dict,
				              garage,map,T,wait_list,block_list,&reached_car,&wait_num);
		if(block_flag) break;
	      }
	      if(block_flag){ std::cout<<"SCH out block!!!"<<std::endl;  break;} 
	      //正常跳出while 表明 无车等待 尝试调度车库
              sch_allcross_garage(car,cross,cross_dict,road,road_dict,car_dict,
				  garage,map,T,&wait_num,&reached_car,car_num);
	  } 
	  
/******************************车辆调度规则执行******************************/
	//调试发车时 驶离开路口道路信息
// 	debug_dir_leavecross(road,min_cross_id,max_cross_id,cross);
	//调试路口调度时 驶向路口道路信息
// 	debug_dir_tocross(road_dict, road,cross_dict,cross);  
      //写输出文件
      std::cout<<"Time Schdule: "<<" | "<<T<<" | "<<std::endl; 
      if(!write_output(answerPath,car,car_num,map,cross_dict)) return 0;
      print_time("End");
      return 0;
}

