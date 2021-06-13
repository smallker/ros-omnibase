class Pid:
    i_err = 0
    d_err = 0
    last_err = 0
    pos = 0
    sp = 0
    
    def __init__(self, kp=0, ki=0, kd=0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def pid(self) -> float:
        err = self.sp - self.pos
        self.d_err = err - self.last_err
        self.last_err = err
        self.i_err = self.i_err + err
        result_pid = (self.kp * err) + (self.ki * self.i_err) + (self.kd * self.d_err)
        return result_pid

    def reset_err(self):
        self.i_err = 0
        self.d_err = 0
        self.last_err = 0
# p = Pid()
# p.kp = 0.1
# p.sp = 10
# while True:
#     p.pos += p.pid()
#     print(p.pos)
#     sleep(0.1)