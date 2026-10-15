#!/usr/bin/env python3
"""Controlled FSM failure and both collision entrances; short cases only."""
import json,time,sys
from pathlib import Path
import rospy
from std_msgs.msg import Int32
out=Path(sys.argv[1]);rospy.init_node('controlled_execution_faults',anonymous=True)
pub=rospy.Publisher('/preview/motion_blocked',Int32,queue_size=1)
began=time.monotonic();done=set()
while time.monotonic()-began<65 and not rospy.is_shutdown():
 elapsed=time.monotonic()-began
 try:command=json.loads((out/'bridge/command.json').read_text())
 except (OSError,ValueError):time.sleep(.1);continue
 if elapsed>12 and 'fail' not in done:rospy.set_param('/consistency/inject_plan_failure',1);rospy.set_param('/consistency/request_replan',True);done.add('fail')
 if elapsed>25 and 'safety' not in done:rospy.set_param('/consistency/force_safety_stop',True);done.add('safety')
 if elapsed>40 and 'blocked' not in done:pub.publish(Int32(data=command['trajectory_id']));done.add('blocked')
 time.sleep(.05)
(out/'injected_faults.json').write_text(json.dumps(sorted(done)))
