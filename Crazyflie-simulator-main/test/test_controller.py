from context import model
import controller
import trajGen
from model.quadcopter import Quadcopter
import numpy as np
import unittest

class TestController(unittest.TestCase):
    def test_run(self):
        pos = (0,0,0)
        attitude = [0,0,np.pi/6]
        quad = Quadcopter(pos, attitude)
        time = 5
        des_state = trajGen.genLine(time)
        F, M = controller.run(quad, des_state)
        print("desired state", des_state)  # desired state
        print("total thrust", F) # total thrust
        print("Moment matrix: ", M) # moment matrix

if __name__ == '__main__':
    unittest.main()
