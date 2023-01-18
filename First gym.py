import gym
###Classic Control
#env = gym.make("CartPole-v1")
#env = gym.make("MountainCarContinuous-v0")
#env = gym.make("Pendulum-v0")

###Box2D
env = gym.make("LunarLander-v2")
#env = gym.make("BipedalWalker-v2")
#env = gym.make("CarRacing-v0")
#env.render("human")

###Mujoco
#env = gym.make("Ant-v2")

env.action_space.seed()

observation = env.reset()
info = 0

for _ in range(1000):
    env.render()
    observation, reward, terminated, info = env.step(env.action_space.sample())

    if terminated:
        env.reset()

env.close()