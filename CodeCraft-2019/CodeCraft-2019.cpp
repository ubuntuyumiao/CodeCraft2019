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
  print_time("Begin");
    std::ifstream fin_car(argv[1]);
    std::ifstream fin_road(argv[2]);
    std::ifstream fin_cross(argv[3]);
    
    if(!fin_road.is_open())  {std::cout<<"/**ERROR:No Road input file**/"<<std::endl;return 0;} 
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Car input file**/"<<std::endl;return 0;}
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Cross input file**/"<<std::endl;return 0;}
    std::string s;
    int count=1;
    int min_car_id,max_car_id;
    int min_road_id,max_road_id;
    int min_cross_id,max_cross_id;
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
	   
	    std::sscanf(str_tr,"(%d,%d,%d,%d,%d,%d,%d)",   &(road[Subscript].id),
						    &(road[Subscript].road_length),
						    &(road[Subscript].limit_speed),
						    &(road[Subscript].lane_num),
						    &(road[Subscript].start),
						    &(road[Subscript].end),
						    &(road[Subscript].flag_twoway));
	  if(first_line) 
	  {
	    first_line=false; Subscript=road[Subscript].id;
	    min_road_id= Subscript;
	    road_sorted[Subscript] = road[Subscript]=road[0];
	  }
	road[Subscript].completed=false;
	road_sorted[Subscript] = road[Subscript];
	Subscript++;
	count++;
         }
       }
       fin_road.close();
       max_road_id = Subscript-1;
       road_num=count-1;
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
	   
	   std::sscanf(str_tc,"(%d,%d,%d,%d,%d)",   &(car[Subscript].id),
						    &(car[Subscript].set),
						    &(car[Subscript].goal),
						    &(car[Subscript].max_speed),
						    &(car[Subscript].set_time));
	  if(first_line) 
	  {
	    first_line=false; Subscript=car[Subscript].id;
	    min_car_id = Subscript;
	    car_sorted[Subscript] = car[Subscript] = car[0];
	  } 
	car_sorted[Subscript] = car[Subscript];
	car[Subscript].state = still_stored;
	car[Subscript].wait_anthor =false;
	car[Subscript].now_road= car[Subscript].next_road =-1;
	Subscript++;
	count++;
         }
       }
       fin_car.close();
       max_car_id= Subscript-1;
       car_num=count-1;
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
	   
	   std::sscanf(str_ts,"(%d,%d,%d,%d,%d)",   &(cross[Subscript].id),
						    &(cross[Subscript].road_id[0]),
						    &(cross[Subscript].road_id[1]),
						    &(cross[Subscript].road_id[2]),
						    &(cross[Subscript].road_id[3]));
	  if(first_line) 
	  {
	    first_line=false; Subscript=cross[Subscript].id;
	    min_cross_id = Subscript;
	    cross_sorted[Subscript] =cross[Subscript] = cross[0];
	  } 
	cross_sorted[Subscript] =cross[Subscript];
	Subscript++;
	count++;
         }
       }
       cross_num=count-1;
       fin_cross.close();
       max_cross_id = Subscript-1;
       std::cout<<"read finished"<<std::endl;
       int l=min_car_id;
       int m=min_cross_id;
       int n=min_road_id;
       printf("MinNumber %d  car_info: %d  %d  %d  %d  %d \n",l,car[l].id,car[l].set,car[l].goal,car[l].max_speed,car[l].set_time);
       printf("MinNumber %d  cross: %d  %d  %d  %d  %d \n",m,cross[m].id,cross[m].road_id[0],cross[m].road_id[1],cross[m].road_id[2],cross[m].road_id[3]);
       printf("MinNumber %d  road: %d  %d  %d  %d  %d  %d  %d\n",n,(road[n].id),
						    (road[n].road_length),
						    (road[n].limit_speed),
						    (road[n].lane_num),
						    (road[n].start),
						    (road[n].end),
						    (road[n].flag_twoway));
       
       printf("road_num: %d  car_num: %d  cross_num: %d \n",max_road_id-min_road_id+1,max_car_id-min_car_id+1,max_cross_id-min_cross_id+1);
      
       //初始化地图 将所有路口连接置为-1 表示无连接 共有1～(cross_num-1) 个路口
       //数组以0下表开始 直观起见 map[i][j]直接指第i个路口到第j个路口信息
       
       for(int i=min_cross_id;i<=max_cross_id;i++)
       {
	 //初始化路口的十字连通关系
	 cross[min_cross_id+i-1].up_cross_id = 
	    cross[min_cross_id+i-1].right_cross_id =
	      cross[min_cross_id+i-1].down_cross_id =
		cross[min_cross_id+i-1].left_cross_id=-1;
	 for(int j=min_cross_id;j<=max_cross_id;j++)         
	    map[i][j].id= -1;
       }
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/

       for(int i=min_cross_id;i<max_cross_id+1;i++)
       {
	  weight_net[i][i]=INF;
	 for(int j=i+1;j<max_cross_id+1;j++)
	 {
	   if((cross[i].road_id[0]!=-1)&&(cross[i].road_id[0]==cross[j].road_id[2])) 
	      {
		weight_net[i][j]=10;                              
		//先初始化所有权重为10
		map[i][j]=road[cross[i].road_id[0]];
		cross[i].up_cross_id=j;
		if(road[cross[i].road_id[0]].flag_twoway==1) {   
		  weight_net[j][i]=10;
		  //如果是双向道路 （j,i）元素值与(i,j)处相等 下同
		  map[j][i]=road[cross[i].road_id[0]];
		  cross[j].down_cross_id=i;
		}
	      }
	   if((cross[i].road_id[1]!=-1)&&(cross[i].road_id[1]==cross[j].road_id[3]))
	      { 
		weight_net[i][j]=10;                              
		//先初始化所有权重为10
		map[i][j]=road[cross[i].road_id[1]];
		cross[i].right_cross_id=j;
		if(road[cross[i].road_id[1]].flag_twoway==1){
		  weight_net[j][i]=10;
		  map[j][i]=road[cross[i].road_id[1]];
		  cross[j].left_cross_id=i;
		}
	      }
	   if((cross[i].road_id[2]!=-1)&&(cross[i].road_id[2]==cross[j].road_id[0])) 
	      {
		weight_net[i][j]=10;                              
		//先初始化所有权重为10
		map[i][j]=road[cross[i].road_id[2]];
		cross[i].down_cross_id=j;
		if(road[cross[i].road_id[2]].flag_twoway==1){
		  weight_net[j][i]=10;
		  map[j][i]=road[cross[i].road_id[2]];
		  cross[j].up_cross_id=i;
		}
	      }
	   if((cross[i].road_id[3]!=-1)&&(cross[i].road_id[3]==cross[j].road_id[1])) 
	      { 
		weight_net[i][j]=10;                              
		//先初始化所有权重为10
		map[i][j]=road[cross[i].road_id[3]];
		if(road[cross[i].road_id[3]].flag_twoway==1){
		  weight_net[j][i]=10;
		  cross[i].left_cross_id=j;
		  map[j][i]=road[cross[i].road_id[3]];
		  cross[j].right_cross_id=i;
		}
	      }
	 }
       }
       
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/      
       
   

/*********************************A-Star算法  + 神奇车库*********************************/ 
      int has_find=0; 
      quickSortOfCpp(car_sorted,min_car_id,max_car_id);
     for(int i=min_car_id;i<max_car_id+1;i++)
     {
       if( Astar_search(&car[i], road,min_road_id,max_road_id,cross,min_cross_id,max_cross_id,weight_net))
           has_find++;
     }
      std::ofstream fout(argv[4], std::ios::app);
      if(!fout.is_open()) { std::cout<< "No output file" <<std::endl; return 0;}
      std::fstream file(argv[4], std::ios::out);
      fout <<"#(carId,StartTime,road...)"<<std::endl;
     for(int i=min_car_id;i<=max_car_id;i++)
     {
       {
       int road_path,ga_roadpath;
       fout <<"("<<car[i].id<<","<<car[i].set_time<<",";          
       for(int j=0;j<MAX_CROSS;j++)
       {   
	 //为考虑特殊情况 就一条路径？？
	 if(car[i].cross_path[2]==0)
	 {
	   road_path= map[car[i].cross_path[j]][car[i].cross_path[j+1]].id ;
	  fout << road_path<<")"<<std::endl;
	   break;
	 }
	  if(car[i].cross_path[j+1]==0) break;
	  road_path= map[car[i].cross_path[j]][car[i].cross_path[j+1]].id ;
	  fout << road_path; 
	    if(j==0)  
	    {
	      ga_roadpath = map[car[car_sorted[i].id].cross_path[j]][car[car_sorted[i].id].cross_path[j+1]].id;
	      int cur_dup = not_equal(car[car_sorted[i].id].set,road[ga_roadpath].start);
	      garage[ga_roadpath].garage[cur_dup].push_back(car_sorted[i].id); 
	    }
	  if(car[i].cross_path[j+2]!=0) fout <<",";
	    else fout <<")"<<std::endl; 
        }   
     for(int j=0;j<MAX_CROSS;j++)
       {
	 if(car[i].cross_path[j+1]==0) 
	 { 
	   fout <<car[i].cross_path[j];break;
	 }
	   else fout <<car[i].cross_path[j]<<"--->";
       }
        fout <<std::endl;
	fout <<std::endl;}
      }	fout.close();
      
     printf("\nget %d solution of %d car\n\n",has_find,car_num);
     
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
// 		sleep(1);
		block_flag=sch_allcross_drive(car,min_car_id,max_car_id,
					      cross, min_cross_id, max_cross_id,
				road,garage,map,T,wait_list,block_list);
		 if(block_flag) break;
		 std::cout<<"Wait Size: "<<wait_list.size() << std::endl<<std::endl;
// 		  if(T==2)  break;
	      }
	      if(block_flag){ std::cout<<"SCH out block!!!"<<std::endl;  break;} 
// 	      if(T==2) break;
	      //正常跳出while 表明 无车等待 尝试调度车库
              sch_allcross_garage(car,cross, min_cross_id, max_cross_id,road,garage,map,T);
// 	     if(T==2) debug_dir_tocross(road,min_cross_id,max_cross_id,cross);
	  } 
	  
/******************************车辆调度规则执行******************************/
	//调试发车时 驶离开路口道路信息
// 	debug_dir_leavecross(road,min_cross_id,max_cross_id,cross);
	//调试路口调度时 驶向路口道路信息
// 	debug_dir_tocross(road,min_cross_id,max_cross_id,cross);  


      print_time("End");
      return 0;
}

