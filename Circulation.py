
class Circulation:
    
    def __init__(self):
        self.pump1Mode = 0
        self.pump2Mode = 0

    
    def turnOffPump(self):
        self.pump1Mode = 0
        self.pump2Mode = 0

    def turnOnPump(self):
        if self.pump1Mode == 0 and self.pump2Mode==0:
            self.pump1Mode = 1
        else:
            print("The pumps are already on")
        


    def swichPump(self):
        if self.pump1Mode == 0 and self.pump2Mode == 1:
            self.pump1Mode = 1
            self.pump2Mode = 0
        elif self.pump1Mode == 1 and self.pump2Mode==0:
            self.pump1Mode = 0
            self.pump2Mode = 1
        else:
            print("Turn on pumps before switch")
