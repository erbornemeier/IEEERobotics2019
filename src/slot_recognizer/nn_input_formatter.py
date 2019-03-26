import cv2
import pickle
import numpy as np
import random

def sp_noise(image,prob):
    '''
    Add salt and pepper noise to image
    prob: Probability of the noise
    '''
    output = np.zeros(image.shape,np.uint8)
    thres = 1 - prob 
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            rdn = random.random()
            if rdn > thres:
                output[i][j] = int(random.random() * 255) 
            else:
                output[i][j] = image[i][j]
    return output

abc_photos = []
def_photos = []
num_pics = 50
noisify_each = 5 
for pic in range(1,num_pics+1): 
    photo_index = "robotNNABC/ABC_{}.jpg".format(str(pic).rjust(2, '0'))
    print(photo_index)
    abc_photos.append(photo_index)
    photo_index = "robotNNDEF/DEF_{}.jpg".format(str(pic).rjust(2, '0'))
    print(photo_index)
    def_photos.append(photo_index)

outputs = dict()

for p, image_index in enumerate(abc_photos):
    img = cv2.imread(image_index)
    img = cv2.resize(img, (0,0), fx=0.2, fy=0.2)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #adding noise
    for i in range(noisify_each):
        noisy = sp_noise(gray,0.05)
        key = "ABC:{}:{}".format(p,i)
        outputs[key] = cv2.resize(noisy, (100,100))

    print ("Finished: " + image_index)
    '''
    while(True):
        cv2.imshow('original', noisy)
        key = cv2.waitKey(0)
        if key == ord('q'):
            cv2.destroyAllWindows()
            break
    '''
for p, image_index in enumerate(def_photos):
    img = cv2.imread(image_index)
    img = cv2.resize(img, (0,0), fx=0.2, fy=0.2)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #adding noise
    for i in range(noisify_each):
        noisy = sp_noise(gray,0.05)
        key = "DEF:{}:{}".format(p,i)
        outputs[key] = cv2.resize(noisy, (100,100))

    print ("Finished: " + image_index)
    '''
    while(True):
        cv2.imshow('original', noisy)
        key = cv2.waitKey(0)
        if key == ord('q'):
            cv2.destroyAllWindows()
            break
    '''

print(outputs.keys())

with open('outputs.pickle', 'wb') as handle:
    pickle.dump(outputs, handle, protocol=pickle.HIGHEST_PROTOCOL)


