import pickle
import cv2
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='3'

from keras.datasets import mnist
from keras.models import Sequential, load_model
from keras.layers import Dense, Dropout, Activation
from keras.utils import np_utils

with open('outputs.pickle', 'rb') as handle:
    nn_inputs = pickle.load(handle) 

X = np.array([x.flatten() for x in nn_inputs.values()])
y = np.array([ord(y[0])-ord('A') for y in nn_inputs.keys()])
data_len = len(X)
image_size = len(X[0])
X_train, X_test = X[:int(data_len*0.7)], X[int(data_len*0.7):]
y_train, y_test = y[:int(data_len*0.7)], y[int(data_len*0.7):]


#get input and normalize
#(X_train, y_train), (X_test, y_test) = 
X_train = X_train.reshape(len(X_train), image_size)
X_test = X_test.reshape(len(X_test), image_size)
X_train = X_train.astype('float32') 
X_test = X_test.astype('float32') 
X_train= X_train / 255.
X_test = X_test  / 255.

# one-hot encoding using keras' numpy-related utilities
n_classes = 6 
Y_train = np_utils.to_categorical(y_train, n_classes)
Y_test = np_utils.to_categorical(y_test, n_classes)

# building a linear stack of layers with the sequential model
model = Sequential()
model.add(Dense(512, input_shape=(image_size,)))
model.add(Activation('relu'))                            
model.add(Dropout(0.2))

model.add(Dense(512))
model.add(Activation('relu'))
model.add(Dropout(0.2))

model.add(Dense(6))
model.add(Activation('softmax'))

# compiling the sequential model
model.compile(loss='categorical_crossentropy', metrics=['accuracy'], optimizer='adam')

history = model.fit(X_train, Y_train,
          batch_size=128, epochs=20,
          verbose=2,
          validation_data=(X_test, Y_test))

# saving the model
save_dir = "results/"
model_name = 'letter_recognizer.h5'
model_path = os.path.join(save_dir, model_name)
model.save(model_path)

#eval
mnist_model = load_model(save_dir + model_name)
loss_and_metrics = mnist_model.evaluate(X_test, Y_test, verbose=2)

print("Test Loss", loss_and_metrics[0])
print("Test Accuracy", loss_and_metrics[1])
