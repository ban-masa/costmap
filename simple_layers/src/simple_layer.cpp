#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  size_t i=0;
  double angle_min = msg->angle_min;
  double angle_max = msg->angle_max;
  double angle_increment = msg->angle_increment;

  double range_min = msg->range_min;
  double range_max = msg->range_max;
  double r[1000];
  double x[1000];
  double y[1000];
  double theta[1000];

  size_t n=0;
  double new_r[1000];
  double new_x[1000];
  double new_y[1000];
  double new_theta[1000];
  size_t end_p=0;

  size_t ro=0;
  size_t lo=0;
  size_t center_p=0;
  size_t occ_lp[100];
  size_t occ_rp[100];

//ファイルを開く
  FILE *fp;
  fp =fopen("URG_Data.dat","w");

 //std::cout << "range_max: " << range_max << std::endl;　

  size_t data_length = (angle_max - angle_min) / angle_increment;
  for ( i = 0; i < data_length; i++) {

     //1点の位置を計算
    r[i] = std::min(range_max, std::max(range_min, (double)msg->ranges[i]));
    x[i] = r[i] * std::cos(angle_min + i * angle_increment);
    y[i] = r[i] * std::sin(angle_min + i * angle_increment);
    theta[i] = angle_min + i * angle_increment;

//座標を表示
  //std::cout <<"x"<<i <<":"<< x[i] << std::endl;
  //std::cout <<"y"<<i <<":"<< y[i] << std::endl;


//20cm以下の点を排除して、再定義
   if(r[i]>=0.2){
    new_r[n]=r[i];
    new_x[n]=x[i]; 
    new_y[n]=y[i];
    new_theta[n]=theta[i];
fprintf(fp,"%zu %lf %lf %lf %lf \n",n,new_r[n],new_x[n],new_y[n],new_theta[n]);
    n=n+1;
     }
  }//全点の位置計測と再定義終わり

//中心の検出
for(i=1; i<n; i++){
center_p = i;
if(theta[i]>0){break;}
}

fclose(fp);//ファイル閉じる

//ファイルを開く
fp =fopen("URG_Occ.dat","w");

//右オクルージョンの判定
for(i=30; i<center_p-1; i++){
if((new_r[i+1]-new_r[i])>1||(new_r[i]-new_r[i+1])>1){
 occ_rp[ro]=i;
fprintf(fp,"%zu \n",occ_rp[ro]);
 ro=ro+1;
}}

//左オクルージョンの判定
for(i=center_p; i<n-30; i++){
if((new_r[i+1]-new_r[i])>1||(new_r[i]-new_r[i+1])>1){
 occ_lp[lo]=i+1;
 fprintf(fp,"%zu \n",occ_lp[lo]);
 lo=lo+1;
}}

//右危険領域の中心
double dan_rx[100];
double dan_ry[100];
for(i=0; i<ro; i++){
  dan_rx[i]=new_x[occ_rp[i]];
  dan_ry[i]=new_y[occ_rp[i]]+0.4* std::tan(new_theta[occ_rp[i]]);
  fprintf(fp,"%lf %lf \n",dan_rx[i],dan_ry[i]);
}

//左危険領域の中心
double dan_lx[100];
double dan_ly[100];
for(i=0; i<lo; i++){
  dan_lx[i]=new_x[occ_lp[i]];
  dan_ly[i]=new_y[occ_lp[i]]+0.4* std::tan(new_theta[occ_lp[i]]);
  fprintf(fp,"%lf %lf \n",dan_lx[i],dan_ly[i]);
}

fclose(fp);
    
}//callback終わり


//サブスクライブの実装
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lasertest_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1, scanCallback);
  ros::spin();
  return 0;
}

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

 //mark_rx[0]=dan_rx[0]*cos(robot_yaw)-dan_ry[0]*sin(robot_yaw)+robot_x;
 //mark_ry[0]=dan_rx[0]*sin(robot_yaw)+dan_ry[0]*cos(robot_yaw)+robot_y;

  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

} // end namespace
