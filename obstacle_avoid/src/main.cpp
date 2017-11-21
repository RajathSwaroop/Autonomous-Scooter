#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180)



typedef struct
{
    float startangle;
    float endangle;
    float startdist;
    float enddist;
    int b;
}MyLidar;


MyLidar get_angle(const sensor_msgs::LaserScan::ConstPtr& scan,int start, int end)
{
    float dist=0;
    float val = 0;
    float deg=0;
    float startangle = 0;
    float endangle=0;
    char flag;
    int count = scan->scan_time / scan->time_increment;
    float startdist=0;
    float enddist=0;
    MyLidar ml;
    if(start==-30) flag='F';
    if(start==30) flag='R';
    if(start==-90) flag='L';
    
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	val = scan->ranges[i];
	if(degree < end && degree > start){
		//std::cout<<"gone inside one"<<std::endl;
		if(degree<0){
			deg = -degree;//*(-1.0);
		}else{
			deg = degree;
		}
		if(flag=='F'){
		if(degree>13 or degree< -13){		
		dist = ((float)(0.4572)*(float)(sin(DEG2RAD(90-deg))))/((float)(sin(DEG2RAD(deg))));
		} else{
		dist = 2.0;	
		}
		
		if(dist<0){
			dist = dist* -1.0;
		}
		//std::cout<<"gone inside one:\t"<<dist<<"@"<<val<<std::endl;
		//":\t"<<sin(DEG2RAD(90-deg))<<":\t"<<((float)(0.4572)*(float)(sin(DEG2RAD(90-deg))))<<":"
		if(val < dist){
		if(startdist==0)
		{
			startdist=val;
			startangle=degree;
		//ROS_INFO("object at %f,\t %f:\n",startdist,startangle);		  
		}
		else if(deg<30)
		{
			enddist=val;
			endangle=degree;
		//ROS_INFO("object at %f,\t %f:\n",enddist,endangle);		  
		}
		}
		}
		else if(flag=='R'){
		
		dist = 3.0;	
		
		//std::cout<<"gone inside one:\t"<<dist<<"@"<<val<<std::endl;
		//":\t"<<sin(DEG2RAD(90-deg))<<":\t"<<((float)(0.4572)*(float)(sin(DEG2RAD(90-deg))))<<":"
		if(val < dist){
		startdist=val;
		startangle=degree;
		//ROS_INFO("left object at %f,\t %f:\n",startdist,startangle);		  
		//ROS_INFO("object at %f, %f:\n",degree, val);	
		break;	
		}	
		}
		else if(flag=='L'){
		
		dist = 3.0;	
		
		//std::cout<<"gone inside one:\t"<<dist<<"@"<<val<<std::endl;
		//":\t"<<sin(DEG2RAD(90-deg))<<":\t"<<((float)(0.4572)*(float)(sin(DEG2RAD(90-deg))))<<":"
		if(val < dist){
		if(degree<-30)
		{
		enddist=val;
		endangle=degree;
		//ROS_INFO("right object at %f,\t %f:\n",enddist,endangle);		  
		}
		//ROS_INFO("object at %f, %f:\n",degree, val);	
			
		}	
		}
		
	}
       // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
ml.startangle = startangle;
ml.endangle=endangle;
ml.startdist=startdist;
ml.enddist=enddist;
//ROS_INFO("object at %f,\t %f,\t %f,\t %f:\n",ml.startdist,ml.enddist,ml.startangle,ml.endangle);		  
return (ml);
}

MyLidar right(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float startdist=0;
    float enddist=0;
    float startangle = 0;
    float endangle=0;
    MyLidar MR;
    int start=30;
    int end = 90;
    MR = get_angle(scan,start,end);
//ROS_INFO("Right case with object at %f,\t %f,\t %f,\t %f:\n",MR.startdist,MR.enddist,MR.startangle,MR.endangle);		  
return MR;
}

MyLidar left(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float startdist=0;
    float enddist=0;
    float startangle = 0;
    float endangle=0;
    MyLidar ML;
    int start=-90;
    int end = -30;
    ML = get_angle(scan,start,end);
//ROS_INFO("left case with object at %f,\t %f,\t %f,\t %f:\n",ML.startdist,ML.enddist,ML.startangle,ML.endangle);		  
     
return ML;
}
MyLidar forward(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float startdist=0;
    float enddist=0;
    float startangle = 0;
    float endangle=0;
    MyLidar MF;
    int start=-30;
    int end = 30;
    MF = get_angle(scan,start,end);
   // ROS_INFO("forward case with object at %f,\t %f,\t %f,\t %f:\n",MF.startdist,MF.enddist,MF.startangle,MF.endangle);		  
return MF;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float FSangle=0;
    float FEangle=0;
    float Langle=0;
    float Rangle=0;
    float FSdist=0;
    float FEdist=0;
    float Ldist=0;
    float Rdist=0;
    MyLidar MF;
    MyLidar MR;
    MyLidar ML;
    int change=0;
    ML = left(scan);
    MF = forward(scan);
    MR = right(scan);
 /*   if(ML.enddist!=0.0){
    std::cout<<"Left case:"<<ML.endangle<<"\t"<<ML.enddist<<"\n"<<std::endl; 
	}

    if(MF.startdist!=0 or MF.enddist!=0){ 
    std::cout<<"Forward case:"<<MF.startangle<<"\t"<<MF.startdist<<"\t"<<MF.endangle<<"\t"<<MF.enddist<<"\n"<<std::endl;  
	} 

    if(MR.startdist!=0.0){
    std::cout<<"Right case:"<<MR.startangle<<"\t"<<MR.startdist<<"\n"<<std::endl;
    }  */
    if(MF.startdist==0.0){
	//std::cout<<"forward"<<std::endl;
	}
	else{
	std::cout<<"stop"<<std::endl;
    std::cout<<"case:"<<MF.startangle<<"\t"<<MF.endangle<<"\t"<<MR.startangle<<"\t"<<ML.endangle<<"\n"<<std::endl;  
	 if(MF.endangle!=0.0)
	{
	if(MF.endangle-MR.startangle > 60.0){
	change++;
	std::cout<<"change is:"<<change<<std::endl;
	} else if(ML.endangle-MF.startangle > 60.0){
	change--;
	std::cout<<"change is:"<<change<<std::endl;
	}
	}
    else
    {
	if(MF.startangle-MR.startangle > 60.0){
	change++;
	std::cout<<"change is:"<<change<<std::endl;
	} else if(ML.endangle-MF.startangle > 60.0){
	change--;
	std::cout<<"change is:"<<change<<std::endl;
	}
	}
	}
}

int main(int argc, char **argv){

    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}
