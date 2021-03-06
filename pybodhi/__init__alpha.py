# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import seaborn as sb, matplotlib.pyplot as plt, pandas as pd, numpy as np
from matplotlib.colors import LinearSegmentedColormap

class Node(object):

    def __init__(self, tasks):

        # assign actions, conditions, or nodes
        self.tasks = tasks

        # convert functionals to leaves
        for i in range(len(self.tasks)):
            if not isinstance(self.tasks[i], Node):
                self.tasks[i] = Leaf(self.tasks[i])

        # assign ids
        self.id = [0]
        self._id()

        # set all response to 3 (off)
        self.reset()

    def _id(self):

        # insert id into tasks' ids
        for i in range(len(self.tasks)):

            # set child id
            self.tasks[i].id = self.id + [i]

            # recursion
            if isinstance(self.tasks[i], Node):
                self.tasks[i]._id()

    def reset(self):

        # extract actions and conditions
        self.response = dict()
        for task in self.tasks:

            # retrieve response of subsequent node
            if isinstance(task, Node):
                self.response.update(task.response)

            # update response of leaves
            else:
                self.response[task.__name__] = 3

class Fallback(Node):

    def __init__(self, tasks):

        # initialise as node
        Node.__init__(self, tasks)

    def __call__(self):

        # reset response
        self.reset()

        # loop through tasks
        for task in self.tasks:

            # compute status
            status = task()

            # retrieve node response
            if isinstance(task, Node):
                self.response.update(task.response)

            # append leaf response
            else:
                self.response[task.__name__] = status


            # if failed
            if status is 0:
                continue

            # if succesful
            elif status is 1:
                return 1

            # if running
            elif status is 2:
                return 2

        # all tasks returned False
        return 0

class Sequence(Node):

    def __init__(self, tasks):

        # initialise as node
        Node.__init__(self, tasks)

    def __call__(self):

        # reset response
        self.reset()

        # loop through tasks
        for task in self.tasks:

            # compute status
            status = task()

            # retrieve node response
            if isinstance(task, Node):
                self.response.update(task.response)

            # append leaf response
            else:
                self.response[task.__name__] = status

            # if failed
            if status is 0:
                return 0

            # if succesful
            elif status is 1:
                continue

            # if running
            elif status is 2:
                return 2

        # all tasks returned true
        return 1

class Leaf(object):

    def __init__(self, function):

        # bound method
        self.function = function
        self.__name__ = function.__name__
        self.id = [0]

    def __call__(self):
        status = self.function()
        self.msg = {
            'id': self.id,
            'status': status,
            'type': type(self).__name__,
            'children': []
        }
        return status


class Tree(object):

    def __init__(self, node):

        # parent node
        self.node = node

        # response record
        self.responses = list()

    def reset(self):

        # reset response record
        self.responses = list()

    def __call__(self):

        # compute tree response
        status = self.node()
        self.response = self.node.response

        # record response
        self.responses.append(self.response)

        return self.response

    def plot(self, ax=None, duplicates=False):

        # create axis if not provided
        if ax is None:
            fig, ax = plt.subplots(1)

        # function names
        cols = list(self.response.keys())

        # data frame
        data = pd.DataFrame(self.responses, columns=cols)

        # remove consecutive duplicates
        if duplicates is False:
            data = data[cols].loc[(data[cols].shift() != data[cols]).any(axis=1)]

        # make time sequence horizontally
        data = data.T

        # colormap
        cmap = sb.color_palette("Paired", 4)

        # discretise colors for responses
        cmap = LinearSegmentedColormap.from_list('Custom', cmap, len(cmap))

        # make heatmap
        ax = sb.heatmap(data, ax=ax, cmap=cmap, linewidth=0.1, cbar_kws=dict(use_gridspec=False, location='top'))

        # set colorbar tick spacing
        cb = ax.collections[0].colorbar
        cb.set_ticks(np.linspace(0, 3, len(self.response.keys()))[1::2])

        # set tick labels
        cb.set_ticklabels(["Failure", "Sucesss", "Running", "Off"])
        ax.set_ylabel('Leaves')

        plt.show()

if __name__ == "__main__":

    tree = Fallback([lambda: 1, lambda: 1])
    tree = Sequence([tree, lambda: 1])
    tree()
    print(tree.tasks[0].msg)
    print(type(tree).__name__)
