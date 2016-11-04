class Pid():
	def __init__(self, Kp, Ki, Kd, limit=[-10, 10], integration_limit=[-10, 10], dt=0.05):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.limit = limit
		self.integration_limit = integration_limit
		self.dt = dt
		self.integrator = 0
		self.derivator = 0

	def set_limit(self, limit):
		self.limit = limit

	def set_reference(self, r):
		self.r = r

	def get_output(self, y):
		e = self.r - y
		P = self.Kp * e
		D = self.Kd * (e - self.derivator) / self.dt
		self.derivator = e
		self.integrator += e
		if   self.integrator < self.integration_limit[0]: self.integrator = self.integration_limit[0]
		elif self.integrator > self.integration_limit[1]: self.integrator = self.integration_limit[1]
		I = self.integrator * self.Ki
		PID = P + I + D
		if self.limit and PID < self.limit[0]: PID = self.limit[0]
		if self.limit and PID > self.limit[1]: PID = self.limit[1]
		return PID
