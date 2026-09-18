#ifndef MISSION_PLANNER_WAYPOINT_RECORDER_PANEL_HPP
#define MISSION_PLANNER_WAYPOINT_RECORDER_PANEL_HPP

#include <ros/ros.h>
#include <rviz/panel.h>

class QPushButton;
class QString;
class QGridLayout;

namespace mission_planner {

// A fixed RViz dock panel is deliberately used instead of scene-attached
// interactive markers: it remains clickable while the UAV and camera move.
class WaypointRecorderPanel : public rviz::Panel {
public:
    explicit WaypointRecorderPanel(QWidget* parent = nullptr);

private:
    void publish(ros::Publisher& publisher);
    void addButton(const QString& text, const QString& style, ros::Publisher& publisher,
                   QGridLayout* layout, int row, int column,
                   int row_span = 1, int column_span = 1);

    ros::NodeHandle nh_;
    ros::Publisher capture_pub_;
    ros::Publisher capture_stop_pub_;
    ros::Publisher finalize_pub_;
    ros::Publisher undo_pub_;
    ros::Publisher origin_pub_;
    ros::Publisher clear_pub_;
    ros::Publisher load_last_pub_;
    ros::Publisher map_capture_toggle_pub_;
};

}  // namespace mission_planner

#endif
