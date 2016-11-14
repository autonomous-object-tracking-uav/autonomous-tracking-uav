import numpy as np

class Pid():
    def __init__(self, Kp, Ki, Kd, n=10):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.n = n
        self.store = np.zeros(n)
        self.iteration = 0
        self.limit = None
        self.r = 0

    def set_limit(self, limit):
        self.limit = limit

    def set_reference(self, r):
        self.r = r

    def get_output(self, y):
        e = self.r - y
        self.store[self.iteration % self.n] = e

        P = self.Kp * e
        I = self.get_i()
        D = self.get_d()

        PID = P + I + D
        if self.limit and PID < -self.limit: PID = -self.limit
        if self.limit and PID >  self.limit: PID =  self.limit

        self.iteration += 1
        return PID

    def get_i(self):
        return self.Ki * sum(self.store)

    def get_d(self):
        if self.iteration >= self.n:
            tmp = self.iteration % self.n
            if tmp < self.n / 2:
                a = sum(self.store[tmp + 1:tmp + 1 + self.n / 2])
                b = sum(self.store) - a
            else:
                b = sum(self.store[tmp + 1 - self.n / 2:tmp + 1])
                a = sum(self.store) - b
            return self.Kd * (a - b)
        else:
            return 0
