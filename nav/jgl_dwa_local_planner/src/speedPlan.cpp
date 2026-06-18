#include <jgl_dwa_local_planner/speedPlan.h>
SpeedPlan::SpeedPlan(double v, double am)
{
        vmax = v;
        amax= am;
        
}
void SpeedPlan::speedComputeLine(double d, double length, std::vector<double> &fov_speed, const float resolution)
{
        double map_resolution = resolution;
        double s1 = vmax * vmax / (2 * amax);
        double speed;
        double s2 = d * map_resolution;
        double len = length * map_resolution;
       
       std::cout <<"len s1 s2: "<< len << "," << s1 << "," << s2 << ",";
        if(s2>len){
                speed=0.5*(s2-len);
                std::cout << speed <<std::endl;
                if(speed<0.12)
                {
                  speed = 0.12;
                }
                fov_speed.push_back(speed);
                return;
        }
        // if(abs(d-length)<0.06){
        //         std::cout<<"接近终点时设置终点速度为0"<<std::endl;
        //         fov_speed.push_back(0);
        //         return;30]: Timed out waiting for transform from base_link to map to become available before running costmap, tf error: canTransform: target_frame map does not exist. canTransform: source_frame base_link does not exist.. canTransform returned after 0.100204 timeout was 0.1.
//[ WARN] [1732214281.343711043]: Timed out waiting for transform from base_link to map to become available be
        // }
        if(s2<0.2){
                 std::cout<<"起步时给一个最低速度0.1"<<std::endl;
                 fov_speed.push_back(0.15);//最低速度
                 return ;
         }
        
        if (len >= 2 * s1) //长直线段
        {
                if (s2 < s1)
                        speed = sqrt(2 * amax * s2);
                else if (s2 >= s1 && s2 <= len - s1)
                        speed = vmax;
                else if (s2 > len - s1)
                {
                        if (len != s2)
                        {
                                speed = sqrt(2 * amax * (len - s2));
                        }

                        else
                                speed = 0;
                }
        }
        else
        {
                if (2 * s2 <= len)
                        speed = sqrt(2 * amax * s2);
                else
                {

                        speed = sqrt(2 * amax * (len - s2));

                }
        }
        if(speed < 0.12)
        {
                speed = 0.12;
        }
        std::cout << "0speed------------------" << speed <<std::endl;
        fov_speed.push_back(speed);
}
