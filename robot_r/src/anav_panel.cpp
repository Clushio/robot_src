#include <robot_r/anav_panel.h>

#include <QCheckBox>
#include <QDateTime>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.h>
#include <rviz/config.h>

namespace robot_r
{

ANavPanel::ANavPanel(QWidget* parent) : rviz::Panel(parent)
{
  sim_mode_checkbox_ = new QCheckBox("仿真模式", this);
  sim_mode_checkbox_->setChecked(false);
  sim_mode_checkbox_->setEnabled(false);

  start_setting_button_ = makeButton("设置导航点");
  stop_setting_button_ = makeButton("结束设置");
  QPushButton* sim_button = makeButton("打开仿真器");
  QPushButton* add_point_button = makeButton("添加点");
  QPushButton* add_workstation_button = makeButton("添加工位");
  QPushButton* show_point_button = makeButton("显示点位");
  start_location_button_ = makeButton("开始定位");
  quit_location_button_ = makeButton("退出定位");
  auto_nav_button_ = makeButton("AutoNAV");
  QPushButton* move_base_button = makeButton("MoveBase");
  quit_nav_button_ = makeButton("退出Nav");
  QPushButton* nav_start_button = makeButton("开始");
  QPushButton* nav_pause_button = makeButton("暂停");
  QPushButton* nav_resume_button = makeButton("继续");

  QGridLayout* grid = new QGridLayout;
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(6);
  grid->addWidget(start_setting_button_, 0, 0, 1, 2);
  grid->addWidget(stop_setting_button_, 0, 2, 1, 2);
  grid->addWidget(sim_button, 0, 4);
  grid->addWidget(add_point_button, 1, 0);
  grid->addWidget(add_workstation_button, 1, 1);
  grid->addWidget(show_point_button, 1, 2);
  grid->addWidget(start_location_button_, 2, 0, 1, 2);
  grid->addWidget(quit_location_button_, 2, 2, 1, 2);
  grid->addWidget(auto_nav_button_, 3, 0, 1, 2);
  grid->addWidget(move_base_button, 3, 2);
  grid->addWidget(quit_nav_button_, 3, 3);
  grid->addWidget(nav_start_button, 4, 0);
  grid->addWidget(nav_pause_button, 4, 1);
  grid->addWidget(nav_resume_button, 4, 2);

  setting_buttons_ << add_point_button << add_workstation_button << show_point_button;
  nav_shortcut_buttons_ << nav_start_button << nav_pause_button << nav_resume_button;

  const int point_row = 5;
  for (int i = 0; i < 5; ++i)
  {
    QPushButton* button = makeButton(QString("导航\n点%1").arg(i + 1));
    connect(button, &QPushButton::clicked, this, [this, i]() { goToPoint(i); });
    grid->addWidget(button, point_row, i);
    nav_shortcut_buttons_ << button;
  }

  for (int i = 0; i < 3; ++i)
  {
    QPushButton* button = makeButton(QString("W%1").arg(i + 1));
    connect(button, &QPushButton::clicked, this, [this, i]() { goToWorkstation(i + 1); });
    grid->addWidget(button, point_row + 1, i);
    nav_shortcut_buttons_ << button;
  }

  QLabel* y_label = new QLabel("Y=", this);
  y_input_ = new QLineEdit("0.00", this);
  y_input_->setValidator(new QDoubleValidator(-10.0, 10.0, 3, y_input_));
  QPushButton* y_send_button = makeButton("发送Y");
  y_value_label_ = new QLabel("当前Y=0.00 m", this);

  grid->addWidget(y_label, 7, 0);
  grid->addWidget(y_input_, 7, 1);
  grid->addWidget(y_send_button, 7, 2);
  grid->addWidget(y_value_label_, 7, 3, 1, 2);

  const QList<double> y_values = {-0.10, 0.00, 0.07, 0.10, 0.20};
  for (int i = 0; i < y_values.size(); ++i)
  {
    const double value = y_values[i];
    QPushButton* button = makeButton(QString("Y=%1").arg(value, 0, 'f', 2));
    connect(button, &QPushButton::clicked, this, [this, value]() { setYValue(value, true); });
    grid->addWidget(button, 8, i);
  }

  QPushButton* can_button = makeButton("CANStart");
  QPushButton* base_button = makeButton("BaseStart");
  QPushButton* joy_button = makeButton("JOY");
  QPushButton* tag_button = makeButton("TAG");
  QPushButton* stop_all_button = makeButton("停止全部");
  grid->addWidget(can_button, 9, 0);
  grid->addWidget(base_button, 9, 1);
  grid->addWidget(joy_button, 9, 2);
  grid->addWidget(tag_button, 9, 3);
  grid->addWidget(stop_all_button, 9, 4);

  status_label_ = new QLabel("未启动", this);
  log_view_ = new QTextEdit(this);
  log_view_->setReadOnly(true);
  log_view_->setMaximumHeight(160);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(sim_mode_checkbox_);
  layout->addLayout(grid);
  layout->addWidget(status_label_);
  layout->addWidget(log_view_);
  setLayout(layout);

  connect(sim_button, &QPushButton::clicked, this, &ANavPanel::startSimulator);
  connect(start_setting_button_, &QPushButton::clicked, this, &ANavPanel::startSetLocation);
  connect(stop_setting_button_, &QPushButton::clicked, this, &ANavPanel::stopSetLocation);
  connect(add_point_button, &QPushButton::clicked, this, &ANavPanel::addPoint);
  connect(add_workstation_button, &QPushButton::clicked, this, &ANavPanel::addWorkstation);
  connect(show_point_button, &QPushButton::clicked, this, &ANavPanel::showPoints);
  connect(start_location_button_, &QPushButton::clicked, this, &ANavPanel::startRelocalization);
  connect(quit_location_button_, &QPushButton::clicked, this, &ANavPanel::quitRelocalization);
  connect(auto_nav_button_, &QPushButton::clicked, this, &ANavPanel::startAutoNav);
  connect(move_base_button, &QPushButton::clicked, this, &ANavPanel::startMoveBase);
  connect(quit_nav_button_, &QPushButton::clicked, this, &ANavPanel::stopAutoNav);
  connect(nav_start_button, &QPushButton::clicked, this, &ANavPanel::navStart);
  connect(nav_pause_button, &QPushButton::clicked, this, &ANavPanel::navPause);
  connect(nav_resume_button, &QPushButton::clicked, this, &ANavPanel::navResume);
  connect(y_send_button, &QPushButton::clicked, this, &ANavPanel::sendYInput);
  connect(y_input_, &QLineEdit::returnPressed, this, &ANavPanel::sendYInput);
  connect(can_button, &QPushButton::clicked, this, &ANavPanel::startCan);
  connect(base_button, &QPushButton::clicked, this, &ANavPanel::startBase);
  connect(joy_button, &QPushButton::clicked, this, &ANavPanel::startJoy);
  connect(tag_button, &QPushButton::clicked, this, &ANavPanel::startTag);
  connect(stop_all_button, &QPushButton::clicked, this, &ANavPanel::stopAll);

  setSettingButtonsVisible(false);
  setNavShortcutsVisible(false);
  stop_setting_button_->setEnabled(false);
  quit_location_button_->setEnabled(true);
  quit_nav_button_->setEnabled(true);
}

QPushButton* ANavPanel::makeButton(const QString& text)
{
  QPushButton* button = new QPushButton(text, this);
  button->setMinimumHeight(34);
  return button;
}

void ANavPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  sim_mode_checkbox_->setChecked(false);
}

void ANavPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("SimMode", false);
}

void ANavPanel::startSimulator()
{
  if (!sim_mode_checkbox_->isChecked())
  {
    appendLog("实车模式下不启动仿真器");
    return;
  }

  startLaunch("simulator", {"robot_r", "ranger_mini_sim.launch"});
}

void ANavPanel::startSetLocation()
{
  startLaunch("setting_location", {"robot_r", "3settinglocation.launch"});
  start_setting_button_->setEnabled(false);
  stop_setting_button_->setEnabled(true);
  setSettingButtonsVisible(true);
}

void ANavPanel::stopSetLocation()
{
  stopProcess("setting_location");
  stopProcessByPattern("3settinglocation");
  start_setting_button_->setEnabled(true);
  stop_setting_button_->setEnabled(false);
  setSettingButtonsVisible(false);
}

void ANavPanel::startRelocalization()
{
  startLaunch("localization", {"robot_r", "3startlocation.launch", "rviz_enable:=false"});
  start_location_button_->setEnabled(false);
}

void ANavPanel::quitRelocalization()
{
  stopProcess("localization");
  stopProcessByPattern("3startlocation");
  start_location_button_->setEnabled(true);
}

void ANavPanel::startAutoNav()
{
  startLaunch("auto_nav", {"robot_r", "3navlocations.launch"});
  auto_nav_button_->setEnabled(false);
  setNavShortcutsVisible(true);
}

void ANavPanel::stopAutoNav()
{
  stopProcess("auto_nav");
  stopProcessByPattern("3navlocations");
  auto_nav_button_->setEnabled(true);
  setNavShortcutsVisible(false);
}

void ANavPanel::startMoveBase()
{
  startLaunch("move_base", {"robot_r", "5nav.launch"});
}

void ANavPanel::startBase()
{
  startLaunch("base", {"ranger_bringup", "ranger_mini_v2.launch"});
}

void ANavPanel::startJoy()
{
  startLaunch("joy", {"x2bot_teleop", "x2bot_joy_PXN.launch"});
}

void ANavPanel::startTag()
{
  startTerminal("tag_mm3v", "roslaunch robot_r 6tagReadAndCtl_mm3v.launch; exec bash");
  startTerminal("tag_tcp", "python3 tcpserver.py; exec bash");
}

void ANavPanel::startCan()
{
  startTerminal("can", "echo '1' | sudo -S ip link set can0 up type can bitrate 500000; exec bash");
}

void ANavPanel::navStart()
{
  sendJoy("nav_start", {0, 0, 0, 1, 0, 0});
}

void ANavPanel::navPause()
{
  sendJoy("nav_pause", {1, 0, 0, 0, 0, 0});
}

void ANavPanel::navResume()
{
  sendJoy("nav_resume", {0, 0, 1, 0, 0, 0});
}

void ANavPanel::addPoint()
{
  sendJoy("add_point", {0, 0, 1, 0, 0, 0});
}

void ANavPanel::addWorkstation()
{
  sendJoy("add_workstation", {0, 0, 0, 0, 1, 0});
}

void ANavPanel::showPoints()
{
  sendJoy("show_points", {0, 0, 0, 0, 0, 1});
}

void ANavPanel::sendYInput()
{
  bool ok = false;
  const double value = y_input_->text().toDouble(&ok);
  if (!ok)
  {
    y_value_label_->setText("当前Y=输入错误");
    appendLog("Y 输入错误");
    return;
  }

  setYValue(value, true);
}

void ANavPanel::stopAll()
{
  const QStringList keys = processes_.keys();
  for (const QString& key : keys)
  {
    stopProcess(key);
  }
  stopProcessByPattern("3settinglocation");
  stopProcessByPattern("3startlocation");
  stopProcessByPattern("3navlocations");
  appendLog("已发送停止命令");
  start_setting_button_->setEnabled(true);
  stop_setting_button_->setEnabled(false);
  start_location_button_->setEnabled(true);
  auto_nav_button_->setEnabled(true);
  setSettingButtonsVisible(false);
  setNavShortcutsVisible(false);
  updateStatus();
}

void ANavPanel::setSettingButtonsVisible(bool visible)
{
  for (QPushButton* button : setting_buttons_)
  {
    button->setVisible(visible);
  }
}

void ANavPanel::setNavShortcutsVisible(bool visible)
{
  for (QPushButton* button : nav_shortcut_buttons_)
  {
    button->setVisible(visible);
  }
}

void ANavPanel::setYValue(double value, bool send)
{
  current_target_y_ = value;
  y_input_->setText(QString::number(value, 'f', 2));
  y_value_label_->setText(QString("当前Y=%1 m").arg(value, 0, 'f', 2));
  if (send)
  {
    callService("set_y", {"call", "/set_target_y", QString::number(value, 'f', 3)});
  }
}

void ANavPanel::goToPoint(int id)
{
  callService("go_to_point", {"call", "/plan_path_and_go", QString::number(id), QString::number(current_id_), "1"});
  current_id_ = id;
}

void ANavPanel::goToWorkstation(int station_id)
{
  goToPoint(-station_id);
}

void ANavPanel::callService(const QString& key, const QStringList& arguments)
{
  startProgram(key, "rosservice", arguments);
}

void ANavPanel::sendJoy(const QString& key, const QList<int>& buttons)
{
  QStringList button_values;
  for (int value : buttons)
  {
    button_values << QString::number(value);
  }

  const QString payload = QString("{axes: [0.0, 0.0, 0.0], buttons: [%1]}").arg(button_values.join(", "));
  startProgram(key, "rostopic", {"pub", "-1", "/joy", "sensor_msgs/Joy", payload});
}

void ANavPanel::startLaunch(const QString& key, const QStringList& arguments)
{
  startProgram(key, "roslaunch", arguments);
}

void ANavPanel::startProgram(const QString& key, const QString& program, const QStringList& arguments)
{
  if (processes_.contains(key) && processes_[key]->state() != QProcess::NotRunning)
  {
    appendLog(key + " 已经在运行");
    return;
  }

  QProcess* process = new QProcess(this);
  process->setProgram(program);
  process->setArguments(arguments);
  process->setProcessChannelMode(QProcess::MergedChannels);

  connect(process, &QProcess::readyReadStandardOutput, this, [this, process]() {
    const QString output = QString::fromLocal8Bit(process->readAllStandardOutput()).trimmed();
    if (!output.isEmpty())
    {
      appendLog(output);
    }
  });

  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this, key](int exit_code, QProcess::ExitStatus) {
            appendLog(key + " 已退出，code=" + QString::number(exit_code));
            updateStatus();
          });

  processes_[key] = process;
  process->start();

  appendLog("启动 " + modeLabel() + ": " + program + " " + arguments.join(" "));
  updateStatus();
}

void ANavPanel::startTerminal(const QString& key, const QString& command)
{
  startProgram(key, "gnome-terminal", {"--", "bash", "-c", command});
}

void ANavPanel::stopProcess(const QString& key)
{
  QProcess* process = processes_.value(key, nullptr);
  if (!process || process->state() == QProcess::NotRunning)
  {
    return;
  }

  process->terminate();
  if (!process->waitForFinished(2500))
  {
    process->kill();
  }
}

void ANavPanel::stopProcessByPattern(const QString& pattern)
{
  QProcess::execute("pkill", {"-15", "-f", pattern});
}

void ANavPanel::appendLog(const QString& message)
{
  const QString time = QDateTime::currentDateTime().toString("HH:mm:ss");
  log_view_->append("[" + time + "] " + message);
}

void ANavPanel::updateStatus()
{
  QStringList running;
  for (auto it = processes_.cbegin(); it != processes_.cend(); ++it)
  {
    if (it.value() && it.value()->state() != QProcess::NotRunning)
    {
      running << it.key();
    }
  }

  status_label_->setText(running.isEmpty() ? "未启动" : "运行中: " + running.join(", "));
}

QString ANavPanel::modeLabel() const
{
  return "实车";
}

}  // namespace robot_r

PLUGINLIB_EXPORT_CLASS(robot_r::ANavPanel, rviz::Panel)
