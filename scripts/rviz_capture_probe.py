import sys
from python_qt_binding import QtWidgets, QtCore
from rviz import bindings as rviz
app = QtWidgets.QApplication(sys.argv)
frame = rviz.VisualizationFrame()
frame.setSplashPath('')
frame.initialize()
config = rviz.Config()
rviz.YamlConfigReader().readFile(config, '/workspace/falcon_ws/src/pre_map_bridge/config/stage1.rviz')
frame.load(config)
frame.setMenuBar(None)
frame.setStatusBar(None)
frame.setHideButtonVisibility(False)
for w in frame.findChildren(QtWidgets.QDockWidget): w.hide()
for w in frame.findChildren(QtWidgets.QToolBar): w.hide()
frame.resize(1000, 600)
frame.show()
def inspect():
    for w in frame.findChildren(QtWidgets.QWidget):
        if w.width()>600 and w.height()>350:
            print(w.metaObject().className(),w.objectName(), w.width(), w.height(), int(w.winId()), flush=True)
    frame.screen().grabWindow(int(frame.winId())).save('/workspace/shared/outputs/rviz_probe.png')
    app.quit()
QtCore.QTimer.singleShot(3000,inspect)
app.exec_()
