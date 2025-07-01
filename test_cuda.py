import torch

if torch.cuda.is_available() == True:
    print('Cuda Available')