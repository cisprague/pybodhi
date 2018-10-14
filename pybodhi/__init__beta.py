# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np

class Node(object):

    def __init__(self, tasks):

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
            'status': self.status,
            'type': type(self).__name__,
            'children': [task._status() for task in self.tasks]
        }

class Leaf(object):

    def __init__(self, function):
        self.function = function
        self.id = [0]
        self.status = 3

    def _status(self):
        return {
            'id': self.id,
            'status': self.status,
            'type': type(self).__name__,
            'children': []
        }


def Sequence(Node):

    def __init__(self, tasks):

        # initialise as node
        Node.__init__(self, tasks)

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

class Fallback(object):

    def __init__(self, tasks):

        # initialise as node
        Node.__init__(self, tasks)

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


if __name__ == "__main__":

    tree = Node([lambda: 1, lambda: 1])
    tree = Node([tree, lambda: 1])

    print(tree._status())
