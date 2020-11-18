class CandidateInfo:
    def __init__(self):



        self.pos = [0.0, 0.0]
        self.A = 0
        self.BT = 1
        self.b = 0
        self.ORC = []

    def show(self):
        print "\nCandidate Info"
        print "position :", self.pos
        if self.A == 1:
            print "accessible"
        else:
            print "not accessible"
            print "to access, need to remove", self.ORC
        if self.BT == 1:
            print "blocking the target"
        else:
            print "not blocking the target"
        print "b value is :", self.b


if __name__=="__main__":
    c = []
    for i in range(10):
        c.append(CandidateInfo())

    print "c position", c[0].pos
    print c