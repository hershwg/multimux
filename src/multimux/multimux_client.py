import rospy
from topic_tools.srv import MuxSelect

class MultimuxClient:
    def __init__(self, mux_name, prefix):
        self.mux_service = '%s/select' % mux_name
        self.prefix = prefix
        self.prev_topic = None

    def __enter__(self):
        rospy.loginfo('Acquiring mux %s' % self.mux_service)
        self.prev_topic = self.set_mux(self.prefix)
    
    def __exit__(self):
        rospy.loginfo('Releasing mux %s' % self.mux_service)
        if self.prev_topic is None:
            raise ValueError('Tried to release mux when not holding it')
        self.set_mux(self.prev_topic)
        self.prev_topic = None

    def set_mux(self, prefix):
        # create proxy for mux service
        rospy.wait_for_service(self.mux_service)
        mux_proxy = rospy.ServiceProxy(self.mux_service, MuxSelect)

        # set the mux to teleop
        try:
            return mux_proxy(prefix).prev_topic
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
            raise


