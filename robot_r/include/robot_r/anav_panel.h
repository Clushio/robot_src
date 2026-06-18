#ifndef ROBOT_R_ANAV_PANEL_H
#define ROBOT_R_ANAV_PANEL_H

#include <QList>
#include <QMap>
#include <QProcess>
#include <QPushButton>
#include <QTextEdit>
#include <QWidget>

#include <rviz/panel.h>

class QCheckBox;
class QLabel;
class QLineEdit;

namespace robot_r
{

class ANavPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit ANavPanel(QWidget* parent = nullptr);

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void startSimulator();
  void startSetLocation();
  void stopSetLocation();
  void startRelocalization();
  void quitRelocalization();
  void startAutoNav();
  void stopAutoNav();
  void startMoveBase();
  void startBase();
  void startJoy();
  void startTag();
  void startCan();
  void navStart();
  void navPause();
  void navResume();
  void addPoint();
  void addWorkstation();
  void showPoints();
  void sendYInput();
  void stopAll();

private:
  QPushButton* makeButton(const QString& text);
  void setSettingButtonsVisible(bool visible);
  void setNavShortcutsVisible(bool visible);
  void setYValue(double value, bool send);
  void goToPoint(int id);
  void goToWorkstation(int station_id);
  void callService(const QString& key, const QStringList& arguments);
  void sendJoy(const QString& key, const QList<int>& buttons);
  void startLaunch(const QString& key, const QStringList& arguments);
  void startProgram(const QString& key, const QString& program, const QStringList& arguments);
  void startTerminal(const QString& key, const QString& command);
  void stopProcess(const QString& key);
  void stopProcessByPattern(const QString& pattern);
  void appendLog(const QString& message);
  void updateStatus();
  QString modeLabel() const;

  QCheckBox* sim_mode_checkbox_;
  QLabel* status_label_;
  QLabel* y_value_label_;
  QLineEdit* y_input_;
  QTextEdit* log_view_;
  QPushButton* start_setting_button_;
  QPushButton* stop_setting_button_;
  QPushButton* start_location_button_;
  QPushButton* quit_location_button_;
  QPushButton* auto_nav_button_;
  QPushButton* quit_nav_button_;
  QList<QPushButton*> setting_buttons_;
  QList<QPushButton*> nav_shortcut_buttons_;
  QMap<QString, QProcess*> processes_;
  int current_id_ = 0;
  double current_target_y_ = 0.0;
};

}  // namespace robot_r

#endif  // ROBOT_R_ANAV_PANEL_H
