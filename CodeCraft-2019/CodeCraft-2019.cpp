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

void out(std::string s)
{
  std::cout << std::endl;
  std::cout << RED <<s <<std::endl;
}
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
	road[Subscript].car_on_road=0;
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
	cross_sorted[Subscript].up_cro_to_me_isempty=
				  cross_sorted[Subscript].right_cro_to_me_isempty=
				  cross_sorted[Subscript].down_cro_to_me_isempty=
				  cross_sorted[Subscript].left_cro_to_me_isempty=true;
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
       
       for(int i=1;i<cross_num+1;i++)
       {
	 //初始化路口的十字连通关系
	 cross[min_cross_id+i-1].up_cross_id = cross[min_cross_id+i-1].right_cross_id =cross[min_cross_id+i-1].down_cross_id =cross[min_cross_id+i-1].left_cross_id=-1;
	 for(int j=1;j<cross_num+1;j++)         
	    map[i][j].id= -1;
       }
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/
/*	             1      2      3       .....  (cross_num)
 *                ------------------------------------------------> matrix_cols
 *                '   
 *              1 ' -1  |  R2  |   R3
 *                '
 *                ' R2' |  -1  |   R4
 *                '
 *              2 ' R3' |  R4' |   -1
 *                '
 *                '
 *                '
 *              3 '
 *                '
 *              . '
 *              . '
 *              . '
 *              . '
 *                '
 *    (cross_num) '
 *                '
 *                '
 *  matrix_rows   '
 *  
 */       
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
      int cur_node=0;
      int next_node=0;
      quickSortOfCpp(car_sorted,min_car_id,max_car_id);
     for(int i=min_car_id;i<max_car_id+1;i++)
     {
       if( Astar_search(&car[i], road,min_road_id,max_road_id,cross,min_cross_id,max_cross_id,weight_net))
           has_find++;
  
 /*********************************每规划一台小车的路径，就将小车经过的车道权重加10*********************************/       
	 for(int j=0;j<max_cross_id+1;j++)
	 {
	   if(car[i].cross_path[j+1]!=0)
	   {
	   weight_net[car[i].cross_path[j]][car[i].cross_path[j+1]]=weight_net[car[i].cross_path[j]][car[i].cross_path[j+1]]+10;
	   cur_node=car[i].cross_path[j];
	   next_node=car[i].cross_path[j+1];
	   std::cout <<cur_node<<"to"<< next_node<<"is"<<weight_net[car[i].cross_path[j]][car[i].cross_path[j+1]] << std::endl;
	   }
	}
/*********************************每规划一台小车的路径，就将小车经过的车道权重加10*********************************/
     }
     
//      //输出每条路径的权值
//      for(int i=0;i<36;i++)
//      {
//        for(int j=0;j<36;j++)
//        {
// 	 std::cout <<"the weight of "<<i<<"Row"<<j<<"column is" << weight_net[car[i].cross_path[j]][car[i].cross_path[j+1]]<<std::endl;
//        } 
//      }
/**********************************输出文件*********************************/    
      std::ofstream fout(argv[4], std::ios::app);
      if(!fout.is_open()) { std::cout<< "No output file" <<std::endl; return 0;}
      std::fstream file(argv[4], std::ios::out);
      fout <<"#(car_id,starttime,road...)"<<std::endl;
     for(int i=min_car_id;i<=max_car_id;i++)
     {
       int road_path,ga_roadpath;
       fout <<"("<<car[i].id<<","<<car[i].set_time<<",";
       for(int j=0;j<MAX_CROSS;j++)
       { 
       if(car[i].cross_path[j+1]==0) break;
//        std::cout << car[i].cross_path[j] << "-->";
       road_path= map[car[i].cross_path[j]][car[i].cross_path[j+1]].id ;
       fout << road_path; 
       if(j==0)  
       {
	   ga_roadpath = map[car[car_sorted[i].id].cross_path[j]][car[car_sorted[i].id].cross_path[j+1]].id;
	   garage[ga_roadpath].garage[car_sorted[i].set=road[ga_roadpath].start?0:1].push_back(car_sorted[i].id); 
       }
       if(car[i].cross_path[j+2]!=0) fout <<",";
         else fout <<")"<<std::endl; 
	 }
      }
      fout.close();
     printf("\nget %d solution of %d car\n\n",has_find,car_num);
     
/*********************************A-Star算法  + 神奇车库*********************************/
/*********************************神奇车库 将在该路口出发的车压入向量中*********************************/
     for(int i=min_road_id;i<=max_road_id;i++)
     {
       for(int j=0;j<=1;j++)
       {
	 if(j==0) printf(" 道路 %d 正向出发的车： ",i );
  	  else  printf(" \n     反向出发的车： ");
      while(garage[i].garage[j].size()>0)
	  {
	   std::cout<<garage[i].garage[j][0]<< "  ";
	    garage[i].garage[j].erase(garage[i].garage[j].begin());
	  }
       }
       	    std::cout<<std::endl<<std::endl;
     }
/*********************************神奇车库*********************************/
             

/******************************车辆调度规则执行******************************/
//        //记录时刻     
//        int T=0;  
//        //已上路车的等待态表;
//        std::vector<int> wait_list; 
//        while(!All_car_iscompleted(car,min_car_id,max_car_id).all_car_iscompleted)
// 	{
// 	    int car_inroad_iscompleted = All_car_iscompleted(car,min_car_id,max_car_id).car_inroad_iscompleted;
// 	    int sch_init_cross=min_cross_id;
// 	    int sch_cur_road;
// 	    for(int sch_cur_cross=sch_init_cross;sch_cur_cross<=max_cross_id;sch_cur_cross++) 
// 	    {
// 	      /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
// 	      int cur_cross_road[4];
// 	      std::stack<int> cross_shoulebe;
// 	      std::memcpy(cur_cross_road,cross[sch_cur_cross].road_id,sizeof(cross[sch_cur_cross].road_id));
// 	      int array_offset=3;
// 	    for(int i=0;i<4;i++)
// 	      if(cur_cross_road[i]!=-1)
// 	        if(road[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口为入口 则此道路值为-1
// 	          if(sch_cur_cross!=road[cur_cross_road[i]].end)  cur_cross_road[i]=-1;
// 	      std::sort(cur_cross_road,cur_cross_road+4);    
// 	     //去掉不存在的道路 或者不调度 不进入该交叉口的道路
// 	    while(cur_cross_road[array_offset]>0){                      
// 	      cross_shoulebe.push(cur_cross_road[array_offset]); 
// 	      array_offset--;
// 	      if(array_offset<0) 
// 		break;
// 	    }  
// 	      /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
// 	      for(int road_offset=array_offset;road_offset<4;road_offset++)
// 	      {
// 		sch_cur_road=cur_cross_road[road_offset];   //即将进行调度的道路
// 		road_empty empty_Condition = check_road_empty(&cross[sch_cur_cross],&road[sch_cur_road]);
// 		bool road_is_empty=empty_Condition.is_empty;	//查看道路是否为空 非空的话可加塞最高优先级车道是？ 余量是？ （可以是0 ！=-1）
// 		if(road_is_empty)          //道路上无车 为空
// 		{
// 		  //等待表非空 调度下一条道路
//                     if(!wait_list.empty())   continue;
// 			else{
// 			      //等待表为空  调度神奇车库 判断神奇车库是否有车 等同于已上路的车全为终止态.
// 			      //该条道路的神奇车库无车 调度下一条道路  
// 				if(!check_garage(&cross[sch_cur_cross],&road[sch_cur_road],&garage[sch_cur_road]))                 
// 				 {
// 				  if(road_is_empty) continue;
// 				    else
// 				    {
// 				      // 如果该道路在位置上最靠前 且为终止态 ，且下一个时刻即将过路口，将其信息发送到其下一个路口公告字段
// 				      update_to_cross(car,&road[sch_cur_road],&cross[sch_cur_cross]);
// 				      continue;
// 				    }
// 				      
// 				 }  
// 				 //神奇车库有车  empty_Condition.lane  empty_Condition.offset
// 				 else 
// 				 {
// 				   // 往可以加塞的车道驶入
// 				   for(int sch_cur_lane=empty_Condition.lane;sch_cur_lane<road[sch_cur_road].lane_num;sch_cur_lane++)
// 				   {
// // 				     if()
// 				     
// 				   }
// 				   
// 				 }
// 				  
// 			    }
// 		    
// 			
// 		  //
// 		}
// 		else                      //道路上有车 非空
// 		{
// 		  //empty_Condition.lane  empty_Condition.offset  为道路非空时 最先优先级车道 以及其余量
// 		  int wait_car_num=has_car_wait_inroad(&cross[sch_cur_cross],&road[sch_cur_road],car);
// 		  //无车等待 全为终止态
// 		  if(wait_car_num==0)
// 		  {
// 		    //等待表飞空
// 		    if(!wait_list.empty())   continue;
// 		      else 
// 		      {
// 			
// 		      }
// 		  }
// 		    else
// 		     {
// 		    
// 		     }
// 		  
// 		  
// 		}
// 		
// 		
// 	      }
// 	      
// 	      
// 	      
// 	    }
// 	  
// 	  
// 
// 	 T++;            //一轮调度结束 道路上所有车都为终止态 调度时间往后累加
// 	}
// 	
/******************************车辆调度规则执行******************************/

       
  print_time("End");
  return 0;
}