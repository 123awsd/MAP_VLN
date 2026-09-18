#include "waypoint_mission/waypoint_recorder_panel.hpp"

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Empty.h>

namespace mission_planner {

WaypointRecorderPanel::WaypointRecorderPanel(QWidget* parent) : rviz::Panel(parent) {
    capture_pub_ = nh_.advertise<std_msgs::Empty>("/waypoint_recorder/capture", 1);
    capture_stop_pub_ = nh_.advertise<std_msgs::Empty>("/waypoint_recorder/capture_stop", 1);
    finalize_pub_ = nh_.advertise<std_msgs::Empty>("/waypoint_recorder/finalize", 1);
    undo_pub_ = nh_.advertise<std_msgs::Empty>("/waypoint_recorder/undo", 1);
    origin_pub_ = nh_.advertise<std_msgs::Empty>("/waypoint_recorder/set_origin", 1);
    clear_pub_ = nh_.advertise<std_msgs::Empty>("/waypoint_recorder/clear", 1);
    load_last_pub_ = nh_.advertise<std_msgs::Empty>("/waypoint_recorder/load_last", 1);
    map_capture_toggle_pub_ = nh_.advertise<std_msgs::Empty>(
        "/waypoint_recorder/toggle_map_capture", 1);

    auto* root = new QVBoxLayout;
    auto* title = new QLabel("Waypoint recorder");
    title->setStyleSheet("font-weight: bold; font-size: 14px;");
    root->addWidget(title);
    auto* help = new QLabel("Wait until the UAV is still, then record its measured EKF pose:");
    help->setWordWrap(true);
    root->addWidget(help);

    auto* buttons = new QGridLayout;
    addButton("PASS", "background:#1976d2; color:white; font-weight:bold;", capture_pub_, buttons, 0, 0);
    addButton("STOP", "background:#ef6c00; color:white; font-weight:bold;", capture_stop_pub_, buttons, 0, 1);
    addButton("LOAD LAST", "background:#455a64; color:white;", load_last_pub_, buttons, 1, 0);
    addButton("CLEAR POINTS", "background:#c62828; color:white;", clear_pub_, buttons, 1, 1);
    addButton("UNDO", "background:#8e2430; color:white;", undo_pub_, buttons, 2, 0);
    addButton("MAP PAUSE/RESUME", "background:#6a1b9a; color:white;", map_capture_toggle_pub_, buttons, 2, 1);
    addButton("ORIGIN", "background:#7b1fa2; color:white;", origin_pub_, buttons, 3, 0);
    addButton("FINISH & SAVE MAP", "background:#2e7d32; color:white; font-weight:bold;", finalize_pub_, buttons, 3, 1);
    root->addLayout(buttons);
    auto* note = new QLabel("Blue: optional pass. Orange: fixed yaw + dwell. Pause map capture while walking beside people; LOAD LAST replaces the current points.");
    note->setWordWrap(true);
    root->addWidget(note);
    setLayout(root);
}

void WaypointRecorderPanel::addButton(const QString& text, const QString& style,
                                      ros::Publisher& publisher, QGridLayout* layout,
                                      const int row, const int column,
                                      const int row_span, const int column_span) {
    auto* button = new QPushButton(text);
    button->setMinimumHeight(32);
    button->setStyleSheet(style);
    connect(button, &QPushButton::clicked, this, [this, &publisher]() { publish(publisher); });
    layout->addWidget(button, row, column, row_span, column_span);
}

void WaypointRecorderPanel::publish(ros::Publisher& publisher) {
    publisher.publish(std_msgs::Empty());
}

}  // namespace mission_planner

PLUGINLIB_EXPORT_CLASS(mission_planner::WaypointRecorderPanel, rviz::Panel)
