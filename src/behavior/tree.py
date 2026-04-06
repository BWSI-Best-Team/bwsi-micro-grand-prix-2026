from enum import Enum, auto


class Status(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()


class Node:
    def tick(self, ctx):
        raise NotImplementedError


class Sequence(Node):
    def __init__(self, *children):
        self.children = children

    def tick(self, ctx):
        for child in self.children:
            s = child.tick(ctx)
            if s != Status.SUCCESS:
                return s
        return Status.SUCCESS


class Fallback(Node):
    def __init__(self, *children):
        self.children = children

    def tick(self, ctx):
        for child in self.children:
            s = child.tick(ctx)
            if s != Status.FAILURE:
                return s
        return Status.FAILURE


class Condition(Node):
    def __init__(self, fn):
        self._fn = fn

    def tick(self, ctx):
        return Status.SUCCESS if self._fn(ctx) else Status.FAILURE


class Action(Node):
    def __init__(self, fn):
        self._fn = fn

    def tick(self, ctx):
        return self._fn(ctx)
