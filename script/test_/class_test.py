class object:
    def __init__(self):
        dummy = 1

    def go(self):
        self.type = 'obstacle'
        self.pos = [0.0, 0.0]
        self.A = 0
        self.BT = 1234
        self.b = 0
        self.ORC = []

if __name__=="__main__":
    a = object()
    a.go()

    print a.BT