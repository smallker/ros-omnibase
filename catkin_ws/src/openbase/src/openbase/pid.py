class Pid:
    i_err:float = 0
    d_err:float = 0
    last_err:float = 0
    pos:float = 0
    sp:float = 0
    windup:float = 0.4
    def __init__(self, kp=0, ki=0, kd=0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self) -> float:
        if abs(self.i_err) > self.windup:
            self.i_err = 0
        err = self.sp - self.pos
        self.d_err = err - self.last_err
        self.last_err = err
        self.i_err = self.i_err + err
        result_pid = (self.kp * err) + (self.ki * self.i_err) + (self.kd * self.d_err)
        return result_pid

    def compute_from_err(self, err):
        if abs(self.i_err) > self.windup:
            self.i_err = 0
        self.d_err = err - self.last_err
        self.last_err = err
        self.i_err = self.i_err + err
        result_pid = (self.kp * err) + (self.ki * self.i_err) + (self.kd * self.d_err)
        return result_pid

    def reset_err(self):
        self.i_err = 0
        self.d_err = 0
        self.last_err = 0
# from time import sleep
# p = Pid(kp=0.1, ki=0, kd=0)
# p.sp = 0.3
# while True:
#     p.pos += p.pid()
#     print(p.pos)
#     sleep(0.1)