# Python  3.7.0

# 제목 : Robotics Package v1.0
# 날짜 : 2020.12.24
# 내용 : 다른 프로젝트와 코드 통합

import math

class Kinematics():

    # default
    # link1 = 64
    # link2 = 44
    # link3 = 210
    def __init__(self, link1=64, link2=44, link3=210):
        self.link1 = link1
        self.link2 = link2
        self.link3 = link3
        self.last_step = [0, 0, 0]
    
    def forward(self, step1, step2, step3):
        angle = [0, 0, 0]
        radian = [0, 0, 0]
        coordinates = [0, 0, 0]

        angle[0] = self.stepToDegree(512 - step1)
        angle[1] = self.stepToDegree(512 - step2)
        angle[2] = self.stepToDegree(512 - step3)

        radian[0] = self.degreeToRadian(90 + angle[0])
        radian[1] = self.degreeToRadian(angle[1])
        radian[2] = self.degreeToRadian(angle[2])

        coordinates[0] = (self.link2 * math.sin(radian[1]) + self.link3 * math.sin(radian[1] + radian[2])) * math.cos(radian[0])
        coordinates[1] = (self.link2 * math.sin(radian[1]) + self.link3 * math.sin(radian[1] + radian[2])) * math.sin(radian[0])
        coordinates[2] = (self.link2 * math.cos(radian[1]) + self.link3 * math.cos(radian[1] + radian[2])) + self.link1

        for i in range(0, 3):
            coordinates[i] = round(coordinates[i])
        
        return coordinates
    
    def invers(self, x, y, z):
        steps = [0, 0, 0]
        angle = [0, 0, 0]
        theta = [0, 0, 0]
    
        temp = math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z - self.link1, 2))
        cos_a = (math.pow(self.link2, 2) + math.pow(temp, 2) - math.pow(self.link3, 2)) / (2 * self.link2 * temp)
        cos_b = (math.pow(self.link2, 2) + math.pow(self.link3, 2) - math.pow(temp, 2)) / (2 * self.link2 * self.link3)

        if (-1 <= cos_a and cos_a <= 1) or (-1 <= cos_b and cos_b <= 1):
            pi = math.atan2(z - self.link1, math.sqrt(math.pow(x, 2) + math.pow(y, 2)))
            a = math.atan2(math.sqrt(1 - math.pow(cos_a, 2)), cos_a)
            b = math.atan2(math.sqrt(1 - math.pow(cos_b, 2)), cos_b)

            theta[0] = math.atan2(y, x)
            theta[1] = math.pi / 2 - pi - a
            theta[2] = math.pi - b

            angle[0] = self.radianToDegree(theta[0])
            angle[1] = self.radianToDegree(theta[1])
            angle[2] = self.radianToDegree(theta[2])
            
            steps[0] = 512 - round(self.degreeToStep(90 - angle[0]))  # 1번째 축
            steps[1] = 512 - round(self.degreeToStep(angle[1]))  # 2번째 축
            steps[2] = 512 - round(self.degreeToStep(angle[2]))  # 3번째 축
            self.last_step = steps
        
        return self.last_step
    
    def radianToDegree(self, radian):
        return (radian * 180.0) / math.pi
    
    def degreeToRadian(self, degree):
        return (degree * math.pi) / 180.0

    def stepToDegree(self, step):
        return step * 0.29296875
    
    def degreeToStep(self, degree):
        return degree / 0.29296875