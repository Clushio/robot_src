#include <robot_r/record_pose_tool.h>

#include <pluginlib/class_list_macros.h>

namespace robot_r
{

RecordPoseTool::RecordPoseTool()
{
  shortcut_key_ = 'r';
}

void RecordPoseTool::onInitialize()
{
  rviz::GoalTool::onInitialize();
  setName(QString::fromUtf8("记录点位"));
  setDescription(QString::fromUtf8(
      "在 map 中拖出位置和朝向，仅发布到 /anav/record_pose，不触发车辆导航。"));
}

}  // namespace robot_r

PLUGINLIB_EXPORT_CLASS(robot_r::RecordPoseTool, rviz::Tool)
