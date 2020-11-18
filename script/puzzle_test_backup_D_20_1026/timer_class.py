#Jeeho made
#2020.10.22
import time

class timer_kit:
    #constructor. set to start counting as soon as it is created
    def __init__(self, name):
        self.name = name
        self.start = time.time()
        self.end = 0
        self.is_counting = True
        print("Time Count Started: " + name)

    #just in case the same object is used to count again
    def start_count(self):
        self.start = time.time()
        self.end = 0
        self.is_counting = True

    #stop time count
    def stop_count(self):
        if(self.is_counting == True):
            self.end = time.time()
            self.is_counting = False
        else:
            print("time count not started")

    #just in case the same object is used to count again
    def clear(self):
        self.start = 0.0
        self.end = 0.0
        self.is_counting = False

    #return delta time
    def get_record(self):
        return self.end - self.start

class time_list:
    def __init__(self):
        self.list = []

    def add(self, time_count):
        #only add if time_count is a timer_kit type of data
        if(isinstance(time_count, timer_kit)):
            self.list.append(time_count)
        else:
            print("Not a timer_kit data")

    def get_total_time(self):
        sigma = 0
        for n in self.list:
            sigma += n.get_record()
        return sigma

    def add_to_file(self, path, current_time):
        file = open(path,'a')        
        file.write("*** Total Time: " + str(self.get_total_time()) + " at " + current_time +"\n")
        for n in self.list:
            file.write(n.name + ",")
        file.write("\n")
        for v in self.list:
            file.write(str(v.get_record()) + ",")

        file.write("\n")
        file.close()
