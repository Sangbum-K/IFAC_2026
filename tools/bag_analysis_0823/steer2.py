import pickle, numpy as np
D=pickle.load(open('d43.pkl','rb'))
ak=D['/ackermann_cmd']; au=D['/autonomy/ackermann_cmd']
print("ack cols",ak.shape,"auto",au.shape)
print("ack head\n",ak[:3])
print("auto head\n",au[:3])
import collections
print("auto steer min/max",au[:,1].min(),au[:,1].max(), au[:,2].min(), au[:,2].max())
