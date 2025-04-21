"""
This is the implementation of the OGN node defined in ROS1PublishContactSensor.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy
import rospy

from std_msgs.msg import Bool

class ROS1PublishContactSensor:
    """
         Accept in_contact(bool) and value(double), and publish contact ros msg
    """
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            # inputs are :
            #     inputs.inContact
            #     inputs.nodeNamespace
            #     inputs.queueSize
            #     inputs.timeStamp
            #     inputs.topicName
            #     inputs.value

            inContact     = db.inputs.inContact
            nodeNamespace = db.inputs.nodeNamespace
            queueSize     = db.inputs.queueSize
            timeStamp     = db.inputs.timeStamp
            topicName     = db.inputs.topicName
            value         = db.inputs.value

            # check if roscore is running
            if not rospy.core.is_initialized():
                print("roscore is not running")
                return False
            
            # publish inContact
            pub = rospy.Publisher(nodeNamespace + "/" + topicName, Bool, queue_size=queueSize)
            pub.publish(inContact)

            # With the compute in a try block you can fail the compute by raising an exception
            pass
        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True
