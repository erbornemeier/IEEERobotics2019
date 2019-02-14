class State(object):
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return self.name

    def start(self):
        assert 0, str(self) + " start not implemented"

    def run(self):
        assert 0, str(self) + " run not implemented"

    def finish(self):
        assert 0, str(self) + " finish not implemented"
