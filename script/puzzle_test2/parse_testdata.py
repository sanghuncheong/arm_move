def str_list_to_float_list(slist):
    out_list = []
    for s in slist:
        out_list.append(float(s))
    return out_list

def str_list_to_int_list(slist):
    out_list = []
    for s in slist:
        out_list.append(int(s))
    return out_list


def read_and_parse_testdata(file_path, line_ind):
    print("Parsing " + str(line_ind)+"th data")

    file = open(file_path,'r')

    slist = file.readlines()

    print("file read")

    #parse second line

    separate1 = slist[line_ind].split("\n")
    separate2 = separate1[0].split(":")
    # N target X Y R H
    N = int(separate2[0])
    target = int(separate2[1])

    X = str_list_to_float_list(separate2[2].split(","))
    Y = str_list_to_float_list(separate2[3].split(","))
    R = str_list_to_float_list(separate2[4].split(","))
    H = str_list_to_float_list(separate2[5].split(","))
    pose_idx_list = str_list_to_int_list(separate2[6].split(","))

    file.close()

    return N, target, X, Y, R, H, pose_idx_list, slist[line_ind]


class testdata:
    #constructor.
    def __init__(self, file_path, data_ind):
        if(file_path != ""):
            N, target, X, Y, R, H, pose_idx_list, raw = read_and_parse_testdata(file_path, data_ind)
            self.N = N
            self.target = target
            self.X = X
            self.Y = Y
            self.R = R
            self.H = H
            self.pose_idx_list = pose_idx_list
            self.raw = raw
        else:
            self.N = -1
            self.target = -1
            self.X = []
            self.Y = []
            self.R = []
            self.H = []
            self.pose_idx_list = []
            self.raw = ""