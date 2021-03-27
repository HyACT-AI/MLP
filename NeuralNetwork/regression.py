# python      3.7.0
# matplotlib  3.3.2

# 제목 : Neural Network Package v1.1.0
# 날짜 : 2021.12.24
# 내용 : 다른 프로젝트와 코드 통합

import os
import math
import random

from keras import utils, datasets, models, layers, optimizers, callbacks
import matplotlib.pyplot as plt
import numpy as np

class Regression():

    def __init__(self):
        self.bb = 0

    def makeNormalizeInputData(self, data):
        self.input_min = np.min(data)
        temp = data - self.input_min
        temp_max = np.max(temp)
        self.input_weight = 1 / temp_max
        self.input_data = (((data - self.input_min) * self.input_weight) * 0.6) + 0.2

        return self.input_data
    
    def makeNormalizeOutputData(self, data):
        self.output_min = np.min(data)
        temp = data - self.output_min
        temp_max = np.max(temp)
        self.output_weight = 1 / temp_max
        self.output_data = (((data - self.output_min) * self.output_weight) * 0.6) + 0.2

        return self.output_data

    def normalizeInputData(self, data):
        return (((data - self.input_min) * self.input_weight) * 0.6) + 0.2
    
    def normalizeOutputData(self, data):
        return (((data - self.output_min) * self.output_weight) * 0.6) + 0.2

    def localizeInputData(self, data):
        return (((data - 0.2) / 0.6) / self.input_weight) + self.input_min
    
    def localizeOutputData(self, data):
        return (((data - 0.2) / 0.6) / self.output_weight) + self.output_min
    
    def build(self, input_shape=(4, 3), hidden_shape = (64, 64), output_shape=3, activation1='sigmoid', activation2='sigmoid', learning_rate=0.001):
        self.input_shape = input_shape
        self.hidden_shape = hidden_shape
        self.output_shape = output_shape
        self.activation1 = activation1
        self.activation2 = activation2
        self.learning_rate = learning_rate

        self.model = models.Sequential()
        self.model.add(layers.Dense(self.input_shape[1], activation=self.activation1))
        for i in range(0, len(self.hidden_shape)):
            self.model.add(layers.Dense(self.hidden_shape[i], activation=self.activation1))
        self.model.add(layers.Dense(self.output_shape, activation=self.activation2))
        self.model.compile(optimizer=optimizers.Adam(learning_rate=self.learning_rate), loss='mse', metrics=['mae'])
        #self.model.compile(optimizer=optimizers.rmsprop(learning_rate=self.learning_rate), loss='mse', metrics=['mae'])
        #self.model.summary()

    def makeCallBack(self, epoch, progress_bar, text_edit):
        #self.early_stop_callback = callbacks.EarlyStopping(monitor='loss', patience=20)
        self.custom_call_back = Callback(epoch, progress_bar, text_edit)
    
    def train(self, batch_size=4, epoch=1):
        self.batch_size = batch_size
        self.epoch = epoch
        self.history = self.model.fit(
            x=self.input_data,
            y=self.output_data,
            batch_size=self.batch_size,
            epochs=self.epoch,
            #callbacks=[self.early_stop_callback, self.custom_call_back]
            callbacks=[self.custom_call_back]
            )

        self.saveModelParameter()
        self.model.save('model.h5')
    
    def saveModelParameter(self):
        f = open('parameters.txt', 'w')
        f.write(str(self.input_min))
        f.write('\n')
        f.write(str(self.input_weight))
        f.write('\n')
        f.write(str(0.6))
        f.write('\n')
        f.write(str(0.2))
        f.write('\n')
        f.write(str(self.output_min))
        f.write('\n')
        f.write(str(self.output_weight))
        f.write('\n')
        f.write(str(0.6))
        f.write('\n')
        f.write(str(0.2))
        f.write('\n')
        f.close()
    
    def result(self):
        loss = self.history.history['loss']
        mae = self.history.history['mae']
        epochs = range(1, len(loss) + 1)
        plt.subplot(211)
        plt.plot(epochs, loss, 'r', label='loss')
        plt.legend()
        plt.subplot(212)
        plt.plot(epochs, mae, 'b', label='mae')
        plt.legend()
        plt.xlabel('Epochs')
        plt.show()

    def loadModel(self):
        self.model = models.load_model('model.h5')

class Callback(callbacks.Callback):

    def __init__(self, setting_epoch, progress_bar, text_edit):
        self.setting_epoch = setting_epoch
        self.progress_bar = progress_bar
        self.text_edit = text_edit

    def on_epoch_end(self, epoch, logs={}):
        p = round((epoch / self.setting_epoch) * 100, 2)
        self.progress_bar.setValue(p)
        if (epoch % 500) == 0:
            loss = logs['loss']
            mae = logs['mae']
            self.text_edit.append('epoch : %d/%d,   loss : %2.6f,   mae : %2.6f'%(epoch, self.setting_epoch, loss, mae))
        if epoch+1 == self.setting_epoch:
            loss = logs['loss']
            mae = logs['mae']
            self.text_edit.append('==================================================')
            self.text_edit.append('epoch : %d/%d,   loss : %2.10f,   mae : %2.10f'%(epoch+1, self.setting_epoch, loss, mae))
            self.text_edit.append('==================================================')