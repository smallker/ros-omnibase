class Data:
    time:int
    left_speed:float
    right_speed:float
    pos_x:float
    pos_y:float
    pos_th:float
    compass:float
    def __init__(self, data:str) -> None:
        obj = data.replace('\n','').split(',')
        self.time = obj[0]
        self.left_speed = obj[1]
        self.right_speed = obj[2]
        self.pos_x = obj[3]
        self.pos_y = obj[4]
        self.pos_th = obj[5]
        self.compass = obj[6]