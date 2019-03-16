#include <iostream>
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
  std::cout<< "Debug: "<<s <<std::endl;
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
      quickSortOfCpp(car_sorted,min_car_id,max_car_id);
     for(int i=min_car_id;i<max_car_id+1;i++)
     {
       if( Astar_search(&car[i], road,min_road_id,max_road_id,cross,min_cross_id,max_cross_id,weight_net))
           has_find++;
     }
      std::ofstream fout(argv[4], std::ios::app);
      if(!fout.is_open()) { std::cout<< "No output file" <<std::endl; return 0;}
      std::fstream file(argv[4], std::ios::out);
      fout <<"#(road_id,starttime,road...)"<<std::endl;
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
	
	int cur_dup = not_equal(car[car_sorted[i].id].set,road[ga_roadpath].start);
	garage[ga_roadpath].garage[cur_dup].push_back(car_sorted[i].id); 
       }
       if(car[i].cross_path[j+2]!=0) fout <<",";
         else fout <<")"<<std::endl; 
	 }
     for(int j=0;j<MAX_CROSS;j++)
       {
	 if(car[i].cross_path[j+1]==0) { fout <<car[i].cross_path[j];break;}
	   else fout <<car[i].cross_path[j]<<"--->";
       }
        fout <<std::endl;
	fout <<std::endl;
      }
      fout.close();
     printf("\nget %d solution of %d car\n\n",has_find,car_num);
     
/*********************************A-Star算法  + 神奇车库*********************************/
/*********************************神奇车库  测试输出*********************************/
     for(int i=min_road_id;i<=max_road_id;i++)
     {
       for(int j=0;j<=1;j++)
       {
	std::vector<int>  copy_garage(garage[i].garage[j]);
	 if(j==0) printf(" 道路 %d 正向出发的车： ",i );
  	  else  printf("      反向出发的车： ");
      while(copy_garage.size()>0)
	  {
	   std::cout<<copy_garage[0]<< "  ";
	    copy_garage.erase(copy_garage.begin());
	  }
       }
       	    std::cout<<std::endl;
     }
/*********************************神奇车库*********************************/

/******************************车辆调度规则执行******************************/

       //记录时刻     
       int T=0;  
       //已上路车的等待态表;
       std::vector<int> wait_list; 
       while(!All_car_isreached(car,min_car_id,max_car_id)) //检查是否全部到终点
	{
	    int car_inroad_iscompleted = All_car_iscompleted(car,min_car_id,max_car_id).car_inroad_iscompleted; 
	    
	    if(car_inroad_iscompleted==true)   //路上不存在等待态的车  直接调度每条路的神奇车库
	      {
	         for(int sch_cross_garage=min_cross_id;sch_cross_garage<=max_cross_id;sch_cross_garage++)
		 {
		   std::cout<<std::endl;
	            //因为道路上的所有车都已是终止态 所以先调度那个方向的都可以 如果发车的可行驶距离超过道路长度 直接放到路口
		    /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
		  int cur_cross_road[4];
		  std::memcpy(cur_cross_road,cross[sch_cross_garage].road_id,sizeof(cross[sch_cross_garage].road_id));
		  int array_offset=3;
		for(int i=0;i<4;i++)
		  if(cur_cross_road[i]!=-1)
		    if(road[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口不是起点
		      if(sch_cross_garage!=road[cur_cross_road[i]].start)  cur_cross_road[i]=-1;
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
		    //std::cout<<" "<< sch_road_garage;	
		    //判断该路是否有车调度
		    int cur_dup=not_equal(sch_cross_garage,road[sch_road_garage].start);
		    
		     if(garage[sch_road_garage].garage[cur_dup].empty()){ continue;}
		      else
			 {   
// 			std::cout<<"|"<<sch_road_garage<< "|";
			//路上的车已不需调动  所以调度哪个神奇车库都没关系;但是需满足假设：所有道路的长度<=limit_speed
			//该方向是否有道路 只调度从该路口驶出的车
		        //方向也符合 查看车库是否有车
			if((road[sch_road_garage].flag_twoway==0)&&(sch_cross_garage!=road[sch_road_garage].start)) continue;
			else
			    {   
			      //检查道路最高优先级车道 以及偏移量
			      road_space space_Condition = check_road_space(&cross[sch_cross_garage],&road[sch_road_garage]);
			      //车道非空且车库有车
			      int cur_dup=not_equal(sch_cross_garage,road[sch_road_garage].start);
			      //循环直到道路填满 或者 车库为空
			      while((space_Condition.lane!=-1)||(!garage[road[sch_road_garage].id].garage[cur_dup].empty()))
			      {
				    int cur_dup=not_equal(sch_cross_garage,road[sch_road_garage].start);
				    std::vector<int> car_garage(garage[road[sch_road_garage].id].garage[cur_dup]); 
				    //向量拷贝 只是为了让后面看起来短一点 没什么太大软用
				    //判断发车时刻.<=T
				    if(car[car_garage[0]].set_time>T) 
				    {printf("%d 号车发车时刻为 %d 现在时刻为 %d\n",car_garage[0],car[car_garage[0]].set_time,T); break;}
// 			            std::cout<<car_garage[0]<< "  ";  //将要出发车辆的id
				    //判断该车速度与可行驶距离大小 
				    //D=min（car_speed,offset）
				    int how_far = min(road[sch_road_garage].road_length-space_Condition.offset,car[car_garage[0]].max_speed);
				    how_far = road[sch_road_garage].road_length-how_far;
				    /**** 终于将车安排上了 难受****/
				    road[sch_road_garage].load[cur_dup][space_Condition.lane][how_far]=car_garage[0];
				    /**** 终于将车安排上了 难受****/
				    
				    /**** 上路后还有一系列操作 比如 ***/
				    /***如果下一时刻即将过路口 则更新路口公共字段****/
				    /***写car结构体中的state now_road next_road move_ori settime ****/
				    
				    
				    //发车成功  将其从车库中删掉
			  	   garage[road[sch_road_garage].id].
				    garage[cur_dup].erase(garage[road[sch_road_garage].id].
					garage[cur_dup].begin());
				      
				space_Condition = check_road_space(&cross[sch_cross_garage],&road[sch_road_garage]);
			      }
			      //如果道路已经无车位驶入 调度下一道路车库
			       continue;
		    } 
		  }
		  }
		}
	      }
	      else
	      { 
		 //  调度路上的车  直到路上所有车全是终止态  让等待表为空
	  while(!wait_list.empty())
	  {
	    int sch_init_cross=min_cross_id;
	    int sch_cur_road;
	    for(int sch_cur_cross=sch_init_cross;sch_cur_cross<=max_cross_id;sch_cur_cross++) 
	    {
	      /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
		  int cur_cross_road[4];
		  std::memcpy(cur_cross_road,cross[sch_cur_cross].road_id,sizeof(cross[sch_cur_cross].road_id));
		  int array_offset=3;
		for(int i=0;i<4;i++)
		  if(cur_cross_road[i]!=-1)
		    if(road[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口为入口 则此道路值为-1
		      if(sch_cur_cross!=road[cur_cross_road[i]].end)  cur_cross_road[i]=-1;
		  std::sort(cur_cross_road,cur_cross_road+4);    
		//去掉不存在的道路 或者不调度 不进入该交叉口的道路
		for(;array_offset>=0;array_offset--)
		{
		  if((array_offset==0)&&(cur_cross_road[array_offset]!=-1)) break;
		  if(cur_cross_road[array_offset]==-1) { array_offset+=1;  break;}
		}
	      /******需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset******/
	      
	      for(int road_offset=array_offset;road_offset<4;road_offset++)
	      {
		sch_cur_road=cur_cross_road[road_offset];   //即将进行调度的道路
		road_empty empty_Condition = check_road_empty(&cross[sch_cur_cross],&road[sch_cur_road]);
		bool road_is_empty=empty_Condition.is_empty;	//查看道路是否为空 非空的话可加塞最高优先级车道是？ 余量是？ （可以是0 ！=-1）
		if(road_is_empty)          //道路上无车 为空
		{
		  //等待表非空 调度下一条道路
                    if(!wait_list.empty())   continue;
		       else { std::cout << YELLOW << "Logitic warning" <<std::endl; continue;}
		}
		else                      //道路上有车 非空
		{
		  //empty_Condition.lane  empty_Condition.offset  为道路非空时 最先优先级车道 以及其余量
		  int *road_contain_car;
		  road_contain_car=has_car_wait_inroad(&cross[sch_cur_cross],&road[sch_cur_road],car);
		  //返回两个元素的数组  分别为道路上等待调度的车的数量 以及 最高优先级的车辆id
		  if(*road_contain_car==0)
		  {
		    //等待表飞空
		    if(!wait_list.empty())   continue;
		      else 
		      {
			
		      }
		  }
		  //调度该车
		    else
		     {   //尝试调度该路上所有车辆
		           while(*(road_contain_car+1)>0)
			   {
			    
			     
			     
			     //再次获得该道路车辆情况
			     road_contain_car=has_car_wait_inroad(&cross[sch_cur_cross],&road[sch_cur_road],car);
			   }
		     }
		  
		  
		}
		
		
	      }
	      
	    }      
	  }
	            //路上等待态的车全部调度完成  再去检查车库是否有车调度车库
	            //for()
	      }
         break;
	 T++;            //一轮调度结束 道路上所有车都为终止态 调度时间往后累加
	}
	
/******************************车辆调度规则执行******************************/

       
  print_time("End");
  return 0;
}