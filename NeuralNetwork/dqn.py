# python      3.7.0
# matplotlib  3.3.2

# 제목 : Neural Network Package v1.1.0
# 날짜 : 2021.12.24
# 내용 : 다른 프로젝트와 코드 통합

import os
import math
import random

class DQN():

    def __init__(self, input_node, hidden_node, output_node):
        self.input_node = input_node
        self.hidden_node = hidden_node
        self.output_node = output_node

        self.NETi = []
        self.NETh1 = []
        self.NETh2 = []

    def sigmoidFunction(self, x):
        if x > 10:
            return 1.0
        elif x < -10:
            return 0.0
        else:
            return 1.0 / (1.0 + math.exp(-1 * x))
    
    def tanHFunction(self, x):
        if x > 10:
            return 1.0
        elif x < -10:
            return -1.0
        else:
            return math.tanh(x)
    
    def softSignFunction(self, x):
        return x / (1.0 + abs(x))