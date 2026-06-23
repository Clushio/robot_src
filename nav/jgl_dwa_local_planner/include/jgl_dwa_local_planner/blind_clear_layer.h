#ifndef JGL_DWA_LOCAL_PLANNER_BLIND_CLEAR_LAYER_H_
#define JGL_DWA_LOCAL_PLANNER_BLIND_CLEAR_LAYER_H_

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace jgl_dwa_local_planner
{

class BlindClearLayer : public costmap_2d::Layer
{
public:
  BlindClearLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;
  void matchSize() override;

private:
  bool isInBlindZone(double world_x, double world_y) const;

  double min_angle_;
  double max_angle_;
  double clear_range_;
  double keep_radius_;
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
};

}  // namespace jgl_dwa_local_planner

#endif  // JGL_DWA_LOCAL_PLANNER_BLIND_CLEAR_LAYER_H_
