#!/usr/bin/env python

import rospy, fysom
import jsk_rviz_plugins.msg, std_msgs.msg
import std_srvs.srv, hrpsys_ros_bridge_jvrc.srv

class GaitStateManager():
    def __init__(self):
        # ROS
        rospy.init_node("gait_state_manager", anonymous=True)
        self.gait_state_text_pub = rospy.Publisher("gait_state_text", jsk_rviz_plugins.msg.OverlayText, queue_size=1)
        rospy.Service('call_gait_state_event', hrpsys_ros_bridge_jvrc.srv.StringRequest, self.__callGaitStateEvent)
        # Finite State Machine : document `pydoc fysom`
        self.fsm = fysom.Fysom({
            'initial': 'biped standby',
            'events' : [
                {'name': 'biped walk',         'src': 'biped standby',          'dst': 'biped walking'},
                {'name': 'biped stop',         'src': 'biped walking',          'dst': 'biped standby'},
                {'name': 'to quadruped',       'src': 'biped standby',          'dst': 'quadruped transforming'},
                {'name': 'finish quadruped',   'src': 'quadruped transforming', 'dst': 'quadruped standby'},
                {'name': 'to biped',           'src': 'quadruped standby',      'dst': 'biped transforming'},
                {'name': 'finish biped',       'src': 'biped transforming',     'dst': 'biped standby'},
                {'name': 'quadruped walk',     'src': 'quadruped standby',      'dst': 'quadruped walking'},
                {'name': 'quadruped stop',     'src': 'quadruped walking',      'dst': 'quadruped standby'}
            ]
            })
        # ??? I don't know why, but this works.
        self.fsm.onchangestate = self.__change_state_cb
        for dst in self.fsm.get_all_state():
            for src in self.fsm.get_all_state():
                self.fsm.add_event({'events':[{'name': dst, 'src': src, 'dst': dst}]})
        self.fsm.fatal_event = []
        self.fsm.warn_event = ['biped walk', 'to quadruped', 'to biped', 'quadruped walk']

    def __callGaitStateEvent(self, req):
        eve = req.data
        result = False
        if self.fsm.can(eve):
            rospy.loginfo("[%s] %s has been fired in %s", rospy.get_name(), eve, self.fsm.current)
            getattr(self.fsm, eve)()
            result = True
        else:
            rospy.logwarn("[%s] cannot be fired %s in %s", rospy.get_name(), eve, self.fsm.current)
        return hrpsys_ros_bridge_jvrc.srv.StringRequestResponse(result)

    def __change_state_cb(self, e):
        print e
        text = jsk_rviz_plugins.msg.OverlayText()
        text.text = 'GaitState: ' + e.dst
        text.top = 10
        text.left = 10
        text.width = 750
        text.height = 50
        text.bg_color = std_msgs.msg.ColorRGBA(r = 0.9, g = 0.9, b = 0.9, a = 0.1)
        if e.event in self.fsm.fatal_event:
            text.fg_color = std_msgs.msg.ColorRGBA(r = 0.8, g = 0.3, b = 0.3, a = 1.0)
        elif e.event in self.fsm.warn_event:
            text.fg_color = std_msgs.msg.ColorRGBA(r = 0.8, g = 0.5, b = 0.1, a = 1.0)
        else:
            text.fg_color = std_msgs.msg.ColorRGBA(r = 0.3, g = 0.8, b = 0.3, a = 1.0)
        text.line_width = 1
        text.text_size = 30
        self.gait_state_text_pub.publish(text)

    def main(self):
        # start
        rospy.loginfo("[%s] start main loop", rospy.get_name())
        rospy.spin()

# extension of Fysom ###############################
## get_all_state
def get_all_state(self):
    def flatten(nested_list):
        flat_list = []
        fringe = [nested_list]
        while len(fringe) > 0:
            node = fringe.pop(0)
            if isinstance(node, list):
                fringe = node + fringe
            else:
                flat_list.append(node)
        return flat_list
    tmp = flatten(self._map.values())
    return list(set(flatten([[i.keys() for i in tmp], [i.values() for i in tmp]])))

## add_event
def add_event(self, cfg):
    events = cfg['events'] if 'events' in cfg else []
    callbacks = cfg['callbacks'] if 'callbacks' in cfg else {}
    tmap = self._map
    def add(e):
        src = [e['src']] if isinstance(e['src'], basestring) else e['src']
        if e['name'] not in tmap:
            tmap[e['name']] = {}
        for s in src:
            tmap[e['name']][s] = e['dst']
    for e in events:
        add(e)
    for name in tmap:
        setattr(self, name, self._build_event(name))
    for name in callbacks:
        setattr(self, name, callbacks[name])



if __name__ == "__main__":
    setattr(fysom.Fysom, "get_all_state", get_all_state)
    setattr(fysom.Fysom, "add_event", add_event)
    ins = GaitStateManager()
    ins.main()
