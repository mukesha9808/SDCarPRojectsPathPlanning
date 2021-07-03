#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <map>
#include <string>
#include <vector>

// Get lane number based on d position in frenet coordinate
int getlanenumber(double car_d) {
  //each lane width 4m wide
  int lane= floor(car_d/4);
  
  if (lane < 0){
    lane=0;
  } else if (lane > 2) {
    lane=2;
  }
  
  return lane;
}



//Determine cost if car is driven on same lane
double keepLane_cost(double car_s, double car_d, double car_v, int lane, 
                     vector<vector<double>> &sensorFusion, int prev_size)
{
  //Initialise cost and bias cost
  double cost=0;
  double bias=0;
  bool biasadded=false;
  
  //Loop through each sensor fusion measurement to 
  for(int i=0; i< sensorFusion.size(); ++i) {
    
    double d=sensorFusion[i][6];
    double check_s=sensorFusion[i][5];
    
    double car_s0=car_s-(0.02*prev_size*car_v/2.24);
    
    //sensed car is in same lane of ego car
    if((d> lane*4)  && (d < (lane+1)*4)) {
      
      double vx=sensorFusion[i][3];
      double vy=sensorFusion[i][4];
      double check_speed= sqrt(vx*vx + vy*vy);
      
      double check_s0=check_s;
      
      
      //Estimates cars s coordind at end of path planned previously
      check_s+= (0.02*prev_size*check_speed);

      double gap=check_s - car_s;
		std::cout<< "gap  " << gap << std::endl;
      //If car is far ahead or car is too much behind of end of path
      //it could be possible collision if in future car is just behind
      double tempcost;
      if ((check_s0> car_s0) && (check_s0 < car_s) ){
        std::cout<< "cars  " << car_s << "  checks0  "<<check_s0 << " checks  "<<check_s << "  cars0  "<< car_s0<< std::endl;
      std::cout<< "card " << car_d << " currd "<< d << std::endl;
       tempcost=0.8; 
      }
      else if((gap < 0) || (gap > 100) ){
        tempcost=0; // cost is 0 if no object in path
      }
      else if (gap < 20) {
        tempcost=0.75; 
      }
      else  {
        tempcost= 1-exp(-pow((24/gap),2));
      }
      
      //store the highest cost for keeping on lane
      if(tempcost >cost){
        cost=tempcost;
      } 
    }
    else if (biasadded && (fabs(car_d -d) < 4 ) && ((check_s > car_s0) && (car_s >check_s) ))
    {
      //If car is side too close to ego car add bias cost
      //bias=0.2+(0.5 - 0.25*fabs(car_d -d));
      bias= 1-exp(-pow((3/fabs(car_d -d)),2));
      biasadded=true;
      std::cout<< "cars  " << car_s << " checks  "<<check_s << "  card  "<< car_d << "  vehd  " << d<<std::endl;
      std::cout<< "biasadded " << biasadded << "  biascost  "<<bias <<  "  for dist  " << fabs(car_d -d) << std::endl;
    }
  }
  std::cout<< "lane keep cost " << cost+bias << std::endl;
  return cost+bias;
}



//Determine cost for changing lane
double laneChangeCost(double car_s, double car_d, double car_v, int finallane, 
                     vector<vector<double>> &sensorFusion, int prev_size) 
{
  //Initialise cost
  double cost=0;
  double drivingcost = keepLane_cost(car_s, (finallane*4 +2),  car_v, finallane, sensorFusion, prev_size);
  //Loop through each sensor fusion measurement 
  for(int i=0; i< sensorFusion.size(); ++i) {
    // d coordinate for sensed car
    float d=sensorFusion[i][6];
    double check_s0=sensorFusion[i][5];
    
    double tempcost;
    if((d > (finallane*4))  && (d < ((finallane+1)*4))) {
      
      double vx=sensorFusion[i][3];
      double vy=sensorFusion[i][4];
      double check_speed= sqrt(vx*vx + vy*vy);
      double check_s=sensorFusion[i][5];
      
      //Estimates cars s coordind at end of path planned previously
      check_s+= (0.02*prev_size*check_speed);
      
      double gap= car_s -check_s;
      std::cout<< "lcgap  " << gap << std::endl;
      //If car really behind or too ahead thens assign  min cost. 
      //Min cost for lane change is 0.25 and not 0 to discourage lane change if not needed
  
     if (( gap <10) && (gap >-10)) {
       // Gap is too less than cost is maximum i.e. 1 
       tempcost=1; 
      } 
      else if((gap >100) || (gap < -10)){
        tempcost=0.05;
      }
      else {
        //Cost function in region
        tempcost= 0.5*(1-exp(-pow((15/gap),2)));
      }
      
      // Take highest cost
      if(tempcost >cost) {
        cost=tempcost;
      }
    }
  }
  

  return cost+ drivingcost;
}


//Determine future trajectory state 
vector<string> determinefutureTrajectory(int lane) {
  
  vector<string> futurestates;
  if(lane ==0) //if car is on left most lane
  {
    futurestates.push_back("KL");
    futurestates.push_back("LCR");
  } 
  else if (lane ==1) //if car is on middle lane
  {
    futurestates.push_back("KL");
    futurestates.push_back("LCR");
    futurestates.push_back("LCL");
  }
  else //if car is on right most
  {
    futurestates.push_back("KL");
    futurestates.push_back("LCL");
  }
  
  return futurestates;  
}


/* Determing Target state. Target state has for possible states
0- Keep on same lane and slow down
1- Keep on same lane and maintain speed
2- Change lane to right
3- Change lane to left

9- Emergency braking
*/
int target_state(double car_s, double car_d, double car_vel, 
                vector<vector<double>> &sensorFusion, int prev_size) 
{
  //determine car's current lane
  int lane= getlanenumber(car_d);
  
        
  //determine cars possible trajectories
  vector<string> futurestates=determinefutureTrajectory(lane);
  
  //Initialise final cost and targetstate
  double cost=1;
  int target=0; // slowdown
  
  //Loop through all possible trajectory
  for(int i=0; i< futurestates.size(); ++i)
  {
    double tempcost;
    
    //when future trajectory is change lane to left
    if(futurestates[i].compare("LCL")==0) {
      //Determine cost for lane change
      tempcost=laneChangeCost(car_s, car_d, car_vel, lane-1, 
                     sensorFusion, prev_size); 
      
      // cost is least update target state and cost
      if (tempcost < cost)
      {
        target=3; //turn left
        cost=tempcost;
      }
      std::cout<< "LCL Cost  " << tempcost << std::endl;
    }
    else if (futurestates[i].compare("LCR")==0) {
      //when future trajectory is change lane to right
      tempcost=laneChangeCost(car_s, car_d, car_vel, lane+1, 
                              sensorFusion, prev_size); 

      // cost is least update target state and cost
      if (tempcost < cost)
      {
        target=2; //turn right
        cost=tempcost;
      }
      std::cout<< "LCR Cost  " << tempcost << std::endl;
    }
    else
    {
      //when future trajectory is keep on same lane
      double keeplanecost,prevCost;
 
      //Determine cost for keeping on same lane
      keeplanecost =keepLane_cost(car_s, car_d, car_vel, lane, 
                                  sensorFusion, prev_size);
		std::cout<< "LCK unbiased Cost  " << keeplanecost << std::endl;
      //When cost is least
      if( (keeplanecost < cost) )
      {
        //If cost is sufficiently less then
        //Keep on driving and maintain speed
        if (keeplanecost < 0.7) {
          target=1;
        } 
        else {
          
          if(keeplanecost>0.75)
          {
          target=9;
          }
          else{
          
          //When cost for same lane is higher
          //Add bias cost to avoid collision
          keeplanecost += 0.25;
          
          //when cost is higher and increasing
          //Target state is slow down
          if(prevCost < keeplanecost)
          {
            target=0;
          } 
          else {
            //when cost is reducing speedup and try to maintain speed
            target=1;
          }   
          }
        }
        
        //Saturate cost for same lane to 0.9
        if(keeplanecost>0.9)
        {
          keeplanecost=0.9;
          target=9;
        }
        prevCost=keeplanecost;
        cost=keeplanecost;
        std::cout<< "LCK Cost  " << keeplanecost << std::endl;
      }   
    }
  }
  
  return target;          
}


//create desired trajectory
std::pair<vector<double>,vector<double>> create_trajectory(int final_lane , const vector<double> &maps_s, 
                                 const vector<double> &maps_x, const vector<double> &maps_y,
                                 const vector<double> &PrevPathX, const vector<double> &PrevPathY, 
                                 double car_x, double car_y, double car_s, double car_yaw, double car_vel) {
  vector<double> ptsx,ptsy;
  
  //Detrmine d point for target lane
  // bias is added outer lane so that car does not go out of lane
  double target_d;
  switch (final_lane){
    case 0:
      target_d=2.02;
      break;
    case 1:
      target_d=6;
      break;
    case 2:
      target_d=9.98;
      break;
  }
  
  //Start createing point pipeline for spline
  double ref_x=car_x;
  double ref_y=car_y;
  double ref_yaw=deg2rad(car_yaw);
  
  // If prious path is empty create input points for spline 
  //using car's coordinates
  if (PrevPathX.size() < 2) {
    
    //create a point in past in direction of driving 
    double prev_X=car_x -cos(ref_yaw);
    double prev_Y=car_y -sin(ref_yaw);
    
    ptsx.push_back(prev_X);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(prev_Y);
    ptsy.push_back(ref_y); 
  } 
  else {
    
    //when previous path is available use points from that
    double prev_X=PrevPathX[PrevPathX.size()-2];
    double prev_Y=PrevPathY[PrevPathX.size()-2];
    
    ref_x=PrevPathX[PrevPathX.size()-1];
    ref_y=PrevPathY[PrevPathX.size()-1];
    
    //Determine yaw angle for rotation
    ref_yaw=atan2(ref_y-prev_Y,ref_x-prev_X);
    
    ptsx.push_back(prev_X);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(prev_Y);
    ptsy.push_back(ref_y);   
  }
  
  //Add points from final lane in spline pipeline
  vector<double> next_wp0=getXY(car_s+30, target_d,maps_s,maps_x,maps_y);
  vector<double> next_wp1=getXY(car_s+60, target_d,maps_s,maps_x,maps_y);
  vector<double> next_wp2=getXY(car_s+90, target_d,maps_s,maps_x,maps_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  //Shift and rotate coordinates to transform points
  //at car_s as origin in vehicle frame
  for (int i=0; i<ptsx.size();++i){
    double shift_x=ptsx[i] -ref_x;
    double shift_y=ptsy[i] -ref_y;
    
    ptsx[i] =shift_x*cos(0-ref_yaw) -shift_y*sin(0-ref_yaw);
    ptsy[i] =shift_x*sin(0-ref_yaw) +shift_y*cos(0-ref_yaw);
  }
  
  //create spline
  tk::spline s;
  s.set_points(ptsx,ptsy);
  
  //Find trajectory points in next 30m
  //x distance is divided into equally spaced points
  //to create future x points
  double trgt_dist=distance(0, 0, 30, s(30));
  double N=trgt_dist/(0.02*car_vel/2.24);
  
  vector<double>  trjactory_x;
  vector<double>  trjactory_y;
  
  //Loop thoruog each points and transform into global frame again
  for (int i=0; i<N;++i){
    double x_point=(i+1)*30/N;
    double y_point=s(x_point);
    
    double x_ref=x_point;
    double y_ref=y_point;
    x_point =x_ref*cos(ref_yaw) -y_ref*sin(ref_yaw);
    y_point=x_ref*sin(ref_yaw) +y_ref*cos(ref_yaw);
    
    x_point+=ref_x;
    y_point+= ref_y; 
    
    trjactory_x.push_back(x_point);
    trjactory_y.push_back(y_point);
  }
  
  return std::make_pair(trjactory_x,trjactory_y);
}

#endif  // PATHPLANNER_H