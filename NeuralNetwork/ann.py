# python      3.7.0
# matplotlib  3.3.2

# 제목 : Neural Network Package v1.1.1
# 날짜 : 2021.01.04
# 내용 : Sigmoid Function 분리

import os
import math
import random

import matplotlib.pyplot as plt

class ANN():
    
    def __init__(self, input_node, hidden_node, output_node, axis):
        self.input_node = input_node
        self.hidden_node = hidden_node
        self.output_node = output_node
        self.p = axis
        
        self.NETz = []
        self.z = []
        self.NETy = []
        self.y = []
        self.dy = []
        self.dz = []

        for h in range(0, self.hidden_node):
            self.NETz.append(0.0)
            self.z.append(0.0)
            self.dz.append(0.0)
        for o in range(0, self.output_node):
            self.NETy.append(0.0)
            self.y.append(0.0)
            self.dy.append(0.0)        
                
        self.V = []
        self.W = []
        for i in range(0, self.input_node):
            self.V.append([])
            for h in range(0, self.hidden_node):
                self.V[i].append(0.0)
        for h in range(0, self.hidden_node):
            self.W.append([])
            for o in range(0, self.output_node):
                self.W[h].append(0.0)

        self.input_min = 0
        self.input_weight = 0
        self.output_min = 0
        self.output_weight = 0

    def setNormalizeInputList(self, input_data):
        temp = []
        temp_max = 0

        # 단계 1. 배열의 원소 중 최소값을 구한다.
        self.input_min = min(min(input_data))

        # 단계 2. 배열에서 최소값을 뺀다
        for p in range(0, self.p):
            temp.append([])
            for i in range(0, self.input_node):
                data = input_data[p][i] - self.input_min
                temp[p].append(data)
        
        # 단계 3. 배열에서 최대값을 구한다.
        temp_max = max(max(temp))
        
        # 단계 4. 가중치를 구한다.
        self.input_weight = 1 / temp_max

        # 단계 5. normalization을 한다.
        for p in range(0, self.p):
            for i in range(0, self.input_node):
                temp[p][i] = ((temp[p][i] * self.input_weight) * 0.8) + 0.1
        
        return temp

    def setNormalizeOutputList(self, output_data):
        temp = []
        temp_max = 0

        # 단계 1. 배열의 원소 중 최소값을 구한다.
        self.output_min = min(min(output_data))
        
        # 단계 2. 배열에서 최소값을 뺀다
        for p in range(0, self.p):
            temp.append([])
            for o in range(0, self.output_node):
                data = output_data[p][o] - self.output_min
                temp[p].append(data)
        
        # 단계 3. 배열에서 최대값을 구한다.
        temp_max = max(max(temp))
        
        # 단계 4. 가중치를 구한다.
        self.output_weight = 1 / temp_max

        # 단계 5. normalization을 한다.
        for p in range(0, self.p):
            for o in range(0, self.output_node):
                temp[p][o] = ((temp[p][o] * self.output_weight) * 0.8) + 0.1
        
        return temp
    
    def setNormalizeInput(self, data):
        return (((data - self.input_min) * self.input_weight) * 0.8) + 0.1

    def setNormalizeOutput(self, data):
        return (((data - self.output_min) * self.output_weight) * 0.8) + 0.1

    def setLocalizeInput(self, data):
        return (((data - 0.1) / 0.8) / self.input_weight) + self.input_min
    
    def setLocalizeOutput(self, data):
        return (((data - 0.1) / 0.8) / self.output_weight) + self.output_min

    def sigmoidFunction(self, x):
        if x > 10:
            return 1.0
        elif x < -10:
            return 0.0
        else:
            return 1.0 / (1.0 + math.exp(-1 * x))

    def setTrainingData(self, x, d):
        # 단계 1. 학습시킬 p개의 학습 패턴쌍(입력 패턴 x, 목표치 d)을 선정한다.
        #self.p = p  # Class 초기화때 입력 됨.
        self.x = x
        self.d = d
    
    def setOrignalWeight(self):
        # 단계 2. 연결 강도 V와 W를 임의의 작은값으로 초기화한다.
        self.V.clear()
        self.W.clear()
        self.V = [[-0.19720, 0.57243, 0.09768, -0.24579], [-0.41968, -0.63147, 0.28397, 0.19976]]
        self.W = [[0.03251, 0.21748], [0.34263, -0.33052], [-0.15177, -0.19681], [-0.03149, -0.45499]]

    def setRandomWeight(self):
        # 단계 3. 연결 강도 V와 W를 임의의 작은값으로 초기화한다.
        for i in range(0, 2):
            for h in range(0, 4):
                self.V[i][h] = round(random.uniform(-10000, 10000) / 10000, 5)
        for h in range(0, 4):
            for o in range(0, 2):
                self.W[h][o] = round(random.uniform(-10000, 10000) / 10000, 5)

    def train(self, learning_rate=1, target_error=0.00001, epoch=1):
        # 단계 4. 적절한 학습률 n과 오차의 최대 한계치 Emax를 결정한다.
        self.n = learning_rate      # 학습률
        self.Emax = target_error    # 목표 제곱 오차
        self.E = 0.0                # 제곱 오차
        self.epoch = epoch          # 반복 횟수
        self.epoch_count = 0        # 현재 반복된 횟수

        # 단계 5. 연결 강도를 변경하기 위해 학습 패턴쌍을 차례로 입력한다.
        while self.epoch_count < self.epoch:
            # 변수 초기화
            self.E = 0.0
            self.epoch_count += 1

            for p in range(0, self.p):
                # 변수 초기화
                e_sum = 0.0
                for h in range(0, self.hidden_node):
                    self.NETz[h] = 0.0
                    self.z[h] = 0.0
                    self.dz[h] = 0.0
                for o in range(0, self.output_node):
                    self.NETy[o] = 0.0
                    self.y[o] = 0.0
                    self.dy[0] = 0.0

                # 단계 6. 은닉층의 입력 가중합 NETz를 구한 다음 시그모이드 함수로 출력 z를 구한다.
                for h in range(0, self.hidden_node):
                    for i in range(0, self.input_node):
                        self.NETz[h] += self.x[p][i] * self.V[i][h]
                    self.z[h] = self.sigmoidFunction(self.NETz[h])
                
                # 단계 7. 출력층의 입력 가중합 NETy를 구한 다음 시그모이드 함수로 최종 출력 y를 구한다.
                for o in range(0, self.output_node):
                    for h in range(0, self.hidden_node):
                        self.NETy[o] += self.z[h] * self.W[h][o]
                    self.y[o] = self.sigmoidFunction(self.NETy[o])
                
                # 단계 8. 목표치 d와 최종 출력 y를 비교하여 제곱오차 E를 계산한다.
                for o in range(0, self.output_node):
                    a = self.d[p][o] - self.y[o]
                    e_sum += math.pow(a , 2) / 2

                # 단계 9. 출력층의 오차 신호 dy를 구한다.
                for o in range(0, self.output_node):
                    for h in range(0, self.hidden_node):
                        self.dy[o] = (self.d[p][o] - self.y[o]) * self.y[o] * (1 - self.y[o])
                
                # 단계 10. 은닉층에 전파되는 오차 신호 dz를 구한다.
                for h in range(0, self.hidden_node):
                    temp = 0
                    for o in range(0, self.output_node):
                        temp += self.dy[o] * self.W[h][o]
                    self.dz[h] = self.z[h] * (1 - self.z[h]) * temp
                
                # 단계 11. 은닉층과 출력층 간의 연결 강도 변화량 dw를 계산하여 연결강도를 구한다.
                for o in range(0, self.output_node):
                    for h in range(0, self.hidden_node):
                        dw = self.n * self.dy[o] * self.z[h]
                        self.W[h][o] += dw
                
                # 단계 12. 입력층과 은닉층 간의 연결 강도 변화량 dv를 계산하여 연결강도를 구한다.
                for h in range(0, self.hidden_node):
                    for i in range(0, self.input_node):
                        dv = self.n * self.dz[h] * self.x[p][i]
                        self.V[i][h] += dv

            # 단계 13. 오차 E가 특정 범위 Emax보다 작아지면 학습을 종료한다.
            self.E = e_sum / p
            if self.E < self.Emax:
                self.epoch_count += self.epoch
                break
    
    def predict(self, input_data):
        # 변수 초기화
        x = input_data
        for h in range(0, self.hidden_node):
            self.NETz[h] = 0.0
            self.z[h] = 0.0
            self.dz[h] = 0.0
        for o in range(0, self.output_node):
            self.NETy[o] = 0.0
            self.y[o] = 0.0
            self.dy[0] = 0.0

        # 은닉층의 입력 가중합 NETz를 구한 다음 시그모이드 함수로 출력 z를 구한다.
        for h in range(0, self.hidden_node):
            for i in range(0, self.input_node):
                self.NETz[h] += x[i] * self.V[i][h]
            self.z[h] = self.sigmoidFunction(self.NETz[h])
        
        # 출력층의 입력 가중합 NETy를 구한 다음 시그모이드 함수로 최종 출력 y를 구한다.
        for o in range(0, self.output_node):
            for h in range(0, self.hidden_node):
                self.NETy[o] += self.z[h] * self.W[h][o]
            self.y[o] = self.sigmoidFunction(self.NETy[o])
        
        return self.y