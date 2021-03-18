import rospy

from std_msgs.msg import String
from webots_ros.srv import get_bool, set_int, set_float

import twizy_webots.robot as robot


class Robot:
    def __init__(self, model=None):
        self.model = self._fetch_model() if model is None else model

    def _fetch_model(self, timeout=None):
        return rospy.wait_for_message('/model_name', String, timeout).data

    def service_name(self, srv):
        return f'{self.model}/{srv}'

    def topic_name(self, topic):
        return f'{self.model}/{topic}'

    def service(self, srv, srv_type, wait=True, persistent=False):
        srv_name = self.service_name(srv)
        proxy = rospy.ServiceProxy(srv_name, srv_type, persistent)
        if wait:
            proxy.wait_for_service()
        return proxy

    def call_enable(self, device, ups):
        mspt = int(round(1000.0 / ups))
        return self.service(f'{device}/enable', set_int)(mspt)
    
    def set_position(self, device, wait=True, persistent=False):
        srv_name = f'{device}/set_position'
        return self.service(srv_name, set_float, wait, persistent)
    
    def set_velocity(self, device, wait=True, persistent=False, initial=0):
        self.set_position(device)(float('inf'))
        srv_name = f'{device}/set_velocity'
        proxy = self.service(srv_name, set_float, wait, persistent)
        if proxy and initial is not None:
            proxy(initial)
        return proxy
