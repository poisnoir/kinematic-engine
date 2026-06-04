from spine import Subscriber, Publisher
from mad import MadType
import numpy as np
from kinematics import forward_kinematics, inverse_kinematics

class Robot:

    def __init__(
        self,
        name: str,
        input_source: Subscriber,
        output_source: Publisher, 
    ):
        self.name: str = name
        self.input_source = input_source
        self.output_source = output_source

        self.current_joints = self.current_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)


    def run(self):
        while True:
            print( tuple(self.current_joints) )
            self.output_source.publish(tuple(self.current_joints))
            input = self.input_source.get_data()
            displacement = np.array(input, dtype=np.float64)
            current_position = forward_kinematics(self.current_joints)
            goal = current_position @ displacement
            result = inverse_kinematics(goal, self.current_joints)
            if not result.success:
                continue
            self.current_joints = result.q

