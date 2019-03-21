#include <iostream>
#include <unistd.h>
#include "include.h"
Road road[MAX_ROAD],road_sorted[MAX_ROAD];
Car  car[MAX_CAR],car_sorted[MAX_CAR];
Cross cross[MAX_CROSS],cross_sorted[MAX_CROSS];
Road map[MAX_CROSS][MAX_CROSS];
//神奇车库数组 存放每条道路处的神奇车库
Magic_garage garage[MAX_ROAD];
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
      if(!read_file(argv[3],cross,cross_sorted,&min_cross_id,&max_cross_id,
	          argv[2],road,road_sorted,&min_road_id,&max_road_id,
                 argv[1],car,car_sorted,&min_car_id,&max_car_id,
	          map))    return 0;
      road_num=max_road_id-min_road_id+1,
      cross_num=max_cross_id-min_cross_id+1,
      car_num=max_car_id-min_car_id+1;
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/
      map_matrix(cross,min_cross_id,max_cross_id,weight_net,road,map);    
       
/*********************************A-Star算法  + 神奇车库*********************************/ 
      int has_find=0; 
      quickSortOfCpp(car_sorted,min_car_id,max_car_id);
     for(int i=min_car_id;i<max_car_id+1;i++)
     {
       if( Astar_search(&car[i], road,min_road_id,max_road_id,cross,min_cross_id,max_cross_id,weight_net,map))
           has_find++;
     }
     //安排神奇车库
     ready_garage( min_car_id, max_car_id,garage,road,car, car_sorted, map);
     printf("\nget %d solution of %d car\n\n",has_find,max_car_id-min_car_id+1);
     if(!write_output(argv[4],car, min_car_id, max_car_id,map)) return 0;
/*********************************A-Star算法  + 神奇车库*********************************/

/*********************************神奇车库  测试输出*********************************/
//      for(int i=5005;i<=5005;i++)
//      {
//        for(int j=0;j<=1;j++)
//        {
// 	std::vector<int>  copy_garage(garage[i].garage[j]);
// 	 if(j==0) printf(" 道路 %d 正向出发的车： ",i );
//   	  else  printf("      反向出发的车： ");
//       while(copy_garage.size()>0)
// 	  {
// 	   std::cout<<copy_garage[0]<< "  ";
// 	    copy_garage.erase(copy_garage.begin());
// 	  }
//        }
//        	    std::cout<<std::endl;
//      }
/*********************************神奇车库*********************************/

/******************************车辆调度规则执行******************************/
	  //记录时刻   
	  int T=0;  
	  //等待表;
	  std::vector<int> wait_list; 
	  //检锁表
          std::vector<int> block_list; 
	  bool block_flag=false;
	  while(!All_car_isreached(car,min_car_id,max_car_id)) //检查是否全部到终点
	  {
	    //开始走表 
		T++;    
	    //将所有终止态车变为等待态 并加入等待表
	      chang_completed_towait(min_car_id,max_car_id,car,wait_list);
	    //调度全地图等待车车 直到等待表为空
	      while(!wait_list.empty())
	      {
		if(wait_list.size()==car_num)
		{
		  write_output(argv[4],car, min_car_id, max_car_id,map);
		  return 0;
		}
// 		sleep(1);
		block_flag=sch_allcross_drive(car,min_car_id,max_car_id,
					      cross, min_cross_id, max_cross_id,min_road_id,max_road_id,
				              road,garage,map,T,wait_list,block_list);
		if(block_flag) break;
	      }
	      if(block_flag){ std::cout<<"SCH out block!!!"<<std::endl;  break;} 
// 	      if(T==2) break;
	      //正常跳出while 表明 无车等待 尝试调度车库
              sch_allcross_garage(car,cross, min_cross_id, max_cross_id,road,garage,map,T);
	  } 
	  
/******************************车辆调度规则执行******************************/
	//调试发车时 驶离开路口道路信息
// 	debug_dir_leavecross(road,min_cross_id,max_cross_id,cross);
	//调试路口调度时 驶向路口道路信息
// 	debug_dir_tocross(road,min_cross_id,max_cross_id,cross);  
      //写输出文件
      if(!write_output(argv[4],car, min_car_id, max_car_id,map)) return 0;
      print_time("End");
      return 0;
}

