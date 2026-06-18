#include <jgl_dwa_local_planner/pursuit.h>
#define PI 3.1415926
VehicleState::VehicleState(){
    
}
VehicleState::VehicleState(double a, double b, double c, double d)
{
    x = a;
    y = b;
    yaw = c;
    v = d;
}

Pursuit::Pursuit(double a, double b)
{
    kk = a;
    Lfc = b;
}

// 计算两个点之间的向量
geometry_msgs::PoseStamped Pursuit::getDirectionVector(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
    geometry_msgs::PoseStamped vector;
    vector.pose.position.x = p1.pose.position.x - p2.pose.position.x;
    vector.pose.position.y = p1.pose.position.y - p2.pose.position.y;
    vector.pose.position.z = p1.pose.position.z - p2.pose.position.z;
    return vector;
}

// 归一化向量
geometry_msgs::PoseStamped Pursuit::normalizeVector(const geometry_msgs::PoseStamped& vector) {
    double length = sqrt(pow(vector.pose.position.x, 2) + pow(vector.pose.position.y, 2) + pow(vector.pose.position.z, 2));
    geometry_msgs::PoseStamped normalizedVector;
    if (length > 0) {
        normalizedVector.pose.position.x = vector.pose.position.x / length;
        normalizedVector.pose.position.y = vector.pose.position.y / length;
        normalizedVector.pose.position.z = vector.pose.position.z / length;
    }
    return normalizedVector;
}

// 计算目标点
geometry_msgs::PoseStamped Pursuit::calculateTargetPoint(const std::vector<geometry_msgs::PoseStamped>& distPoint, double distance) {
    // 检查点的数量是否足够
    if (distPoint.size() < 2) {
        throw std::invalid_argument("至少需要两个点来计算方向向量");
    }

    // 获取倒数第二个点和第二个点
    const geometry_msgs::PoseStamped& secondLastPoint = distPoint[distPoint.size() - 2];
    const geometry_msgs::PoseStamped& secondPoint = distPoint[1];

    // 获取最后一个点
    const geometry_msgs::PoseStamped& lastPoint = distPoint.back();

    // 计算方向向量
    geometry_msgs::PoseStamped directionVector = getDirectionVector(secondLastPoint, secondPoint);

    // 归一化方向向量
    geometry_msgs::PoseStamped normalizedVector = normalizeVector(directionVector);

    // 计算目标点
    geometry_msgs::PoseStamped targetPoint;
    targetPoint.pose.position.x = lastPoint.pose.position.x + normalizedVector.pose.position.x * distance;
    targetPoint.pose.position.y = lastPoint.pose.position.y + normalizedVector.pose.position.y * distance;
    targetPoint.pose.position.z = lastPoint.pose.position.z + normalizedVector.pose.position.z * distance;

    return targetPoint;
}



int Pursuit::calc_target_index(VehicleState state, std::vector<geometry_msgs::PoseStamped> distPoint)
{

    double min = 100;
    int ind = 0;
    //从目标点找出一个离当前点最近的点
    for (int i = 0; i < distPoint.size(); i++)
    {
        double distence = (state.x - distPoint[i].pose.position.x) * (state.x - distPoint[i].pose.position.x) + (state.y - distPoint[i].pose.position.y) * (state.y - distPoint[i].pose.position.y);
        if (distence <min)
        {
            min = distence;
            ind = i;
        }
    }


    // int max_index=distPoint.size()-1;
    // if(distPoint.back().pose.position.z<0){
    //     max_index=2*max_index;
    // }

    //计算离这个点距离最接近前视距离的点
    double sum = 0;
    double Lf = kk * state.v + Lfc;
    while (Lf > sum && ind <distPoint.size()-1)
    {
        sum = sum + sqrt((distPoint[ind + 1].pose.position.x - distPoint[ind].pose.position.x) * (distPoint[ind + 1].pose.position.x - distPoint[ind].pose.position.x) +
                         (distPoint[ind + 1].pose.position.y - distPoint[ind].pose.position.y) * (distPoint[ind + 1].pose.position.y - distPoint[ind].pose.position.y));
        ind++;

    }
  //  std::cout<<"total="<<distPoint.size()<<std::endl;
  //  std::cout<<"current ind= "<<ind<<std::endl;
    if(distPoint.size()- ind <=3)
    {
        nearEnd = true;
        refPosition = Pursuit::calculateTargetPoint(distPoint, 0.3);
  //      std::cout<<"near end X Y "<<refPosition.pose.position.x << "  "<<refPosition.pose.position.y<<std::endl;
    }
    else{
        nearEnd = false;
    }


    return ind;
}

struct Result Pursuit::pure_pursuit(VehicleState state, std::vector<geometry_msgs::PoseStamped> distPoint, int ind)
{

    // int ind = calc_target_index(state, distPoint);
    double alpha, tx,ty;
if(ind<distPoint.size())
{

    if(nearEnd == false)
    {
        tx = distPoint[ind].pose.position.x;
        ty = distPoint[ind].pose.position.y;
    }
    else{
        tx = refPosition.pose.position.x;
        ty = refPosition.pose.position.y;
    }


    double distance=sqrt((tx-state.x)*(tx-state.x)+(ty-state.y)*(ty-state.y));
    alpha = atan2(ty - state.y, tx - state.x) - state.yaw;

    if(alpha<=-PI){
        alpha=alpha+2*PI;
    }else if(alpha>=PI){
        alpha=alpha-2*PI;
    }
}



    struct Result ret;
    ret.x = alpha;
    ret.y = ind;
    return ret;
}
