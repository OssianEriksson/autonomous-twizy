import rospy

from std_msgs.msg import String
from webots_ros.srv import get_bool, set_int

import twizy_webots.robot as robot


class Robot:
    def __init__(self, model=None):
        self.model = self._fetch_model() if model is None else model

    def _fetch_model(self, timeout=None):
        return rospy.wait_for_message('/model_name', String, timeout).data

    def service_name(self, srv):
        return f'{self.model}/{srv}'

    def service(self, srv, srv_type, wait=True, persistent=False):
        srv_name = self.service_name(srv)
        proxy = rospy.ServiceProxy(srv_name, srv_type, persistent)
        if wait:
            proxy.wait_for_service()
        return proxy

    def topic_name(self, topic):
        return f'{self.model}/{topic}'

    def enable(self, device, ups):
        mspt = int(round(1000.0 / ups))
        return self.service(f'{device}/enable', set_int)(mspt)