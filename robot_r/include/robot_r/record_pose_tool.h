#ifndef ROBOT_R_RECORD_POSE_TOOL_H
#define ROBOT_R_RECORD_POSE_TOOL_H

#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>

namespace robot_r
{

class RecordPoseTool : public rviz_default_plugins::tools::GoalTool
{
  Q_OBJECT

public:
  RecordPoseTool();

protected:
  void onInitialize() override;
};

}  // namespace robot_r

#endif  // ROBOT_R_RECORD_POSE_TOOL_H
