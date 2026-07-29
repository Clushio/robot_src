#ifndef ROBOT_R_RECORD_POSE_TOOL_H
#define ROBOT_R_RECORD_POSE_TOOL_H

#include <rviz/default_plugin/tools/goal_tool.h>

namespace robot_r
{

class RecordPoseTool : public rviz::GoalTool
{
  Q_OBJECT

public:
  RecordPoseTool();

protected:
  void onInitialize() override;
};

}  // namespace robot_r

#endif  // ROBOT_R_RECORD_POSE_TOOL_H
