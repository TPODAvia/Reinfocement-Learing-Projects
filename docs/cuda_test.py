import torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Device used:")
print(device)