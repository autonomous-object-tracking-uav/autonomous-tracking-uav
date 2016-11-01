class Pid():
	def __init__(self, Kp, Ki, Kd, lim=None, integrator_lim=10, dt=0.01):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.lim = lim
		self.integrator_lim = (-integrator_lim, integrator_lim)
		self.dt = dt
		self.integrator = 0
		self.derivator = 0

	def set_reference(self, r):
		self.r = r

	def get_u(self, y):
		e = self.r - y
		P = self.Kp * e
		D = self.Kd * (e - self.derivator) / self.dt
		self.derivator = e
		self.integrator += e
		if   self.integrator < self.integrator_lim[0]: self.integrator = self.integrator_lim[0]
		elif self.integrator > self.integrator_lim[1]: self.integrator = self.integrator_lim[1]
		I = self.integrator * self.Ki
		PID = P + I + D
		if self.lim and PID < self.lim[0]: PID = self.lim[0]
		if self.lim and PID > self.lim[1]: PID = self.lim[1]
		return PID
