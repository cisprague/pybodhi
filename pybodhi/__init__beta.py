# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np

class Node(object):

    def __init__(self, tasks, name=None):

        if name is not None:
            self.__name__ = name
        else:
            self.__name__ = type(self).__name__

        # convert functionals to leaves
        for i in range(len(tasks)):
            if not isinstance(tasks[i], Node):
                tasks[i] = Leaf(tasks[i])

        # assign tasks
        self.tasks = tasks

        # assign ids
        self.id = [0]
        self._id()

        # reset responses
        self._reset()

    def _id(self):

        # insert parent id
        i = 0
        for task in self.tasks:

            # set child id
            task.id = self.id + [i]

            # recursion
            if isinstance(task, Node):
                task._id()
            i += 1

    def _reset(self):

        # set to off
        self.status = 3

        # recursion
        for task in self.tasks:
            task.status = 3
            if isinstance(task, Node):
                task._reset()

    def _status(self):
        return {
            'id': self.id,
            'status': ["FAILURE", "SUCCESS", "RUNNING", "None"][self.status],
            'type': "FB" if isinstance(self, Fallback) else "SEQ",
            'children': [task._status() for task in self.tasks],
            'label': self.__name__
        }

    def _newick(self):
        return "(" + str([task._newick() for task in self.tasks)]).replace("'","").replace("\\", "").replace('"', "").replace("[","").replace("]", "").replace("Fallback", "?").replace("Sequence", "→") + ")" + self.__name__


class Leaf(object):

    def __init__(self, function):
        self.function = function
        self.id = [0]
        self.status = 3

    def _status(self):
        return {
            'id': self.id,
            'status': ["FAILURE", "SUCCESS", "RUNNING", "None"][self.status],
            'type': "CON",
            'children': [],
            'label': self.function.__name__
        }

    def __call__(self):

        self.status = self.function()
        print(self.function.__name__, self.status)
        return self.status

    def _newick(self):
        return self.function.__name__

class Sequence(Node):

    def __init__(self, tasks, name=None):

        # initialise as node
        Node.__init__(self, tasks, name)

    def __call__(self):

        # reset responses
        self._reset()

        # loop through tasks
        for task in self.tasks:

            # compute status
            self.status = task()

            # if failed
            if self.status == 0:
                return 0

            # if succesful
            elif self.status == 1:
                continue

            # if running
            elif self.status == 2:
                return 2

        # all tasks succesful
        return 1

class Fallback(Node):

    def __init__(self, tasks, name=None):

        # initialise as node
        Node.__init__(self, tasks, name)

    def __call__(self):

        # reset response
        self._reset()

        # loop through tasks
        for task in self.tasks:

            # compute status
            self.status = task()

            # if failed
            if self.status == 0:
                continue

            # task succesful
            elif self.status == 1:
                return 1

            # task running
            elif self.status == 2:
                return 2

        # all tasks failed
        return 0

class Tree(object):

    def __init__(self, node):

        # parent node
        self.node = node

        # responses
        self.responses = list()

    def reset(self):

        # responses
        self.responses = list()

        # reset responses
        self.node._reset()

    def __call__(self):

        # reset node
        self.node._reset()

        # compute tree response
        response = self.node()

        # statuses
        self.status = self.node._status()

        return response

    def newick(self):

        newick = self.node._newick().replace("'","").replace("\\", "").replace('"', "").replace("[","").replace("]", "").replace("Fallback", "?").replace("Sequence", "→")
        newick = "(" + newick + ")Ø;"
        return newick


if __name__ == "__main__":

    tree = Fallback([lambda: 1, lambda: 1])
    tree = Sequence([tree, lambda: 1])
    tree = Tree(tree)
    print(tree.node._status())
    tree()
    print(tree.node._status())
