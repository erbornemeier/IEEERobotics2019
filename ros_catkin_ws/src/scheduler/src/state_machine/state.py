import rospy

class State(object):
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return self.name

    def start(self):
        #rospy.loginfo("[{!s}] Entering State".format(self))
        pass

    def run(self):
        assert 0, str(self) + " run not implemented"

    def finish(self):
        #rospy.loginfo("[{!s}] Exiting State".format(self))
        pass
