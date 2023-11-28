Hello everyone, this is offtopic and nothing related to our HSE program. I want to share a project I’d like to work on. It is really fascinating to watch how animals and humans adapt to their environments and wondering if we can replicate the evolution of nature's brain. The term "Artificial Intelligence" is currently just a marketing strategy, and no one has successfully created a real general AI, also known as Сильный искусственный интеллект.
Most systems are based on deep neural networks that perform specific tasks like computer vision, computer hearing, natural language processing, and image generation. The broader tasks like making a dish or tidying the apartment are still really challenging for ML systems. Humans are still exploring new methods to create and mimic the human brain.
What fascinates me is watching children learning to walk. Achieving bipedal walking is incredibly complex and almost impossible using traditional inverse kinematic approaches.
In recent times, the technology known as "reinforcement learning" has been evolving at a rapid pace. This technology is closely related with the philosophy of the brain and mind, but it is one of the hardest methods to implement in real applications, even for experienced programmers. 





























My plan is to implement this technology to train some simple and interesting games like Chrome Dino, Mario, Doom, and Hotline Miami. Then, I will train it in my own environment using the Unity engine and implement it in a real-world system. This approach is more interesting than traditional ML methods and can even beat some games. 

















For anyone who has experience in ML the pipeline I want to implement looks like this. 

I hope this explanation is clear and easy to understand. If anyone here wants to push programming skills in Machine Learning and Artificial Intelligence to the limit, you’re welcome to collaborate on this project with me. Again, this is just my hobby and not related to our study. There’s no requirement for deep math knowledge. Just need to know the basic architectures and when they are needed. The list are here: Imitation learning, Behavior cloning, Deep Q-Learning, Proximal Policy Optimization and Convolutional neural network
```
# System manipulations
import os
import sys
# Computer vision
import cv2
# RL framework
import ray
from stable_baselines3 import DQN, PPO
# Deep neural network
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
# game engine
import mlagent # Requires the Unity to be installed
import pygame
import gymnasium as gym
# math calculations
import random
from enum import Enum
from collections import namedtuple
import numpy as np
# annotation plotter
import tensorboard
import matplotlib.pyplot as plt
from IPython import display
```


