Behavior cloning, also known as imitation learning, is a technique in machine learning where the model learns by mimicking the behavior of an expert. To add behavior cloning to the existing agent, we need to make some changes to the existing code. We will add an `Expert` class and modify the `Agent` class to learn from the expert's actions. We will also need to modify the training loop to incorporate expert training.

Let's start with the `Expert` class. This class will be responsible for providing expert actions given a game state. The expert's behavior can be hardcoded based on the game's rules or it could be a pre-trained model. In this case, we will use a simple hardcoded expert for the Snake game. The expert will follow the rule of always moving towards the food unless it leads to a collision.

```python
class Expert:
    def get_action(self, state):
        """
        Given a state, this method returns the action that the expert would take.
        Here, the 'state' parameter is the same as the one used in the Agent's get_state() function.
        """
        final_move = [0,0,0]
        
        # If food is to the left and no danger to the left
        if state[10] and not state[2]:
            final_move = [1, 0, 0]
        # If food is to the right and no danger to the right
        elif state[11] and not state[0]:
            final_move = [0, 0, 1]
        # If food is upwards and no danger upwards
        elif state[12] and not state[3]:
            final_move = [0, 1, 0]
        # If food is downwards and no danger downwards
        elif state[13] and not state[1]:
            final_move = [0, 0, 1]
        else:
            # If going towards the food leads to a collision, take a random safe direction
            safe_directions = [i for i in range(3) if state[i]==0]
            if safe_directions:
                final_move[random.choice(safe_directions)] = 1
            else:
                # If no safe direction, take a random direction
                final_move[random.randint(0, 2)] = 1
                
        return final_move
```
[Source 0](https://medium.com/@sthanikamsanthosh1994/imitation-learning-behavioral-cloning-using-pytorch-d5013404a9e5)

Next, we need to modify the `Agent` class to incorporate the expert's actions during training. We'll add an `__init__` parameter `imitation_learning` to control whether to use imitation learning or not. If `imitation_learning` is `True`, the agent will follow the expert's actions with a certain probability (`expert_prob`) during training. Over time, the agent will depend less on the expert and more on its own learned policy.

```python
class Agent:
    def __init__(self, imitation_learning=False, expert_prob=0.5):
        ...
        self.imitation_learning = imitation_learning
        self.expert_prob = expert_prob
        self.expert = Expert() if imitation_learning else None

    def get_action(self, state):
        ...
        if self.imitation_learning and random.random() < self.expert_prob:
            # Follow the expert's action
            final_move = self.expert.get_action(state)
        else:
            ...
        return final_move
```
[Source 0](https://medium.com/@sthanikamsanthosh1994/imitation-learning-behavioral-cloning-using-pytorch-d5013404a9e5)

Finally, we need to gradually decrease `expert_prob` during training so that the agent can transition from imitation learning to reinforcement learning. We can do this in the training loop:

```python
def train():
    ...
    agent = Agent(imitation_learning=True)
    ...
    while True:
        ...
        if done:
            ...
            # Decrease expert_prob
            agent.expert_prob *= 0.99
    ...
```
[Source 0](https://medium.com/@sthanikamsanthosh1994/imitation-learning-behavioral-cloning-using-pytorch-d5013404a9e5)

This way, the agent starts by mostly imitating the expert's actions and gradually starts to rely more on its own policy as it learns from its experiences.