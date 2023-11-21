import torch.nn as nn
import torch.nn.functional as F
from tqdm import tqdm
import sys
# import gymnasium
# pip install dm_tree
# pip install gym[accept-rom-license]
# pip install ale-py

class ConvBlock(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(4, 64, 5, padding=2, stride=2)
        self.conv2 = nn.Conv2d(64, 128, 5, padding=2, stride=2)
        self.bn2 = nn.BatchNorm2d(128)
        self.conv3 = nn.Conv2d(128, 128, 5, padding=2, stride=2)
        self.conv4 = nn.Conv2d(128, 256, 5, padding=2, stride=2)
        self.bn4 = nn.BatchNorm2d(256)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.conv3(x))
        x = F.relu(self.bn4(self.conv4(x)))
        x = x.mean((-1, -2))  # Global average pooling
        return x
    


from ncps.torch import CfC

class ConvCfC(nn.Module):
    def __init__(self, n_actions):
        super().__init__()
        self.conv_block = ConvBlock()
        self.rnn = CfC(256, 64, batch_first=True, proj_size=n_actions)

    def forward(self, x, hx=None):
        batch_size = x.size(0)
        seq_len = x.size(1)
        # Merge time and batch dimension into a single one (because the Conv layers require this)
        x = x.view(batch_size * seq_len, *x.shape[2:])
        x = self.conv_block(x)  # apply conv block to merged data
        # Separate time and batch dimension again
        x = x.view(batch_size, seq_len, *x.shape[1:])
        x, hx = self.rnn(x, hx)  # hx is the hidden state of the RNN
        return x, hx
    
import gym
import ale_py
from ray.rllib.env.wrappers.atari_wrappers import wrap_deepmind
import numpy as np

env = gym.make("ALE/Breakout-v5")
# We need to wrap the environment with the Deepmind helper functions
env = wrap_deepmind(env)
import torch
from torch.utils.data import Dataset
import torch.optim as optim
from ncps.datasets.torch import AtariCloningDataset

train_ds = AtariCloningDataset("breakout", split="train")
val_ds = AtariCloningDataset("breakout", split="val")
trainloader = torch.utils.data.DataLoader(
    train_ds, batch_size=32, num_workers=4, shuffle=True
)
valloader = torch.utils.data.DataLoader(val_ds, batch_size=32, num_workers=4)

def run_closed_loop(model, env, num_episodes=None):
    obs = env.reset()
    device = next(model.parameters()).device
    hx = None  # Hidden state of the RNN
    returns = []
    total_reward = 0
    with torch.no_grad():
        matrix, _ = obs
        while True:
            # print(len(obs))
            # print(obs)
            # PyTorch require channel first images -> transpose data
            obs = np.transpose(matrix, [2, 0, 1]).astype(np.float32) / 255.0
            # add batch and time dimension (with a single element in each)
            obs = torch.from_numpy(obs).unsqueeze(0).unsqueeze(0).to(device)
            pred, hx = model(obs, hx)
            # remove time and batch dimension -> then argmax
            action = pred.squeeze(0).squeeze(0).argmax().item()
            obs, r, done, _, _ = env.step(action)
            total_reward += r
            if done:
                obs = env.reset()
                hx = None  # Reset hidden state of the RNN
                returns.append(total_reward)
                total_reward = 0
                if num_episodes is not None:
                    # Count down the number of episodes
                    num_episodes = num_episodes - 1
                    if num_episodes == 0:
                        return returns
                    

def train_one_epoch(model, criterion, optimizer, trainloader):
    running_loss = 0.0
    pbar = tqdm(total=len(trainloader))
    model.train()
    device = next(model.parameters()).device  # get device the model is located on
    for i, (inputs, labels) in enumerate(trainloader):
        inputs = inputs.to(device)  # move data to same device as the model
        labels = labels.to(device)

        # zero the parameter gradients
        optimizer.zero_grad()
        # forward + backward + optimize
        outputs, hx = model(inputs)
        labels = labels.view(-1, *labels.shape[2:])  # flatten
        outputs = outputs.reshape(-1, *outputs.shape[2:])  # flatten
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        pbar.set_description(f"loss={running_loss / (i + 1):0.4g}")
        pbar.update(1)
    pbar.close()

def eval(model, valloader):
    losses, accs = [], []
    model.eval()
    device = next(model.parameters()).device  # get device the model is located on
    with torch.no_grad():
        for inputs, labels in valloader:
            inputs = inputs.to(device)  # move data to same device as the model
            labels = labels.to(device)

            outputs, _ = model(inputs)
            outputs = outputs.reshape(-1, *outputs.shape[2:])  # flatten
            labels = labels.view(-1, *labels.shape[2:])  # flatten
            loss = criterion(outputs, labels)
            acc = (outputs.argmax(-1) == labels).float().mean()
            losses.append(loss.item())
            accs.append(acc.item())
    return np.mean(losses), np.mean(accs)


# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# model = ConvCfC(n_actions=env.action_space.n).to(device)
# criterion = nn.CrossEntropyLoss()
# optimizer = optim.Adam(model.parameters(), lr=0.0001)

# for epoch in range(50):  # loop over the dataset multiple times
#     train_one_epoch(model, criterion, optimizer, trainloader)

#     # Evaluate model on the validation set
#     val_loss, val_acc = eval(model, valloader)
#     print(f"Epoch {epoch+1}, val_loss={val_loss:0.4g}, val_acc={100*val_acc:0.2f}%")

#     # Apply model in closed-loop environment
#     returns = run_closed_loop(model, env, num_episodes=10)
#     print(f"Mean return {np.mean(returns)} (n={len(returns)})")



# # Visualize Atari game and play endlessly
# env = gym.make("ALE/Breakout-v5", render_mode="human")
# env = wrap_deepmind(env)
# run_closed_loop(model, env)


if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = ConvCfC(n_actions=env.action_space.n).to(device)
    model.load_state_dict(torch.load('model_Breakout.pth'))
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.0001)

    # for epoch in range(50):  # loop over the dataset multiple times
    #     train_one_epoch(model, criterion, optimizer, trainloader)

    #     # Evaluate model on the validation set
    #     val_loss, val_acc = eval(model, valloader)
    #     print(f"Epoch {epoch+1}, val_loss={val_loss:0.4g}, val_acc={100*val_acc:0.2f}%")

    #     # Save the model
    #     torch.save(model.state_dict(), 'model_Breakout.pth')
    #     torch.save(model.state_dict(), 'model_Breakout_m.pt')


    #     # Apply model in closed-loop environment
    #     returns = run_closed_loop(model, env, num_episodes=10)
    #     print(f"Mean return {np.mean(returns)} (n={len(returns)})")

    # Visualize Atari game and play endlessly
    env = gym.make("ALE/Breakout-v5", render_mode="human")
    env = wrap_deepmind(env)
    run_closed_loop(model, env)