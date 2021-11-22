import numpy as np
import math
import scipy.integrate
import tf.transformations


def pwm_to_rpm(x):
    return -0.003918 * pow(x, 2) + 20.77 * x - 15650


class Propeller:
    def __init__(self, prop_dia, prop_pitch):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.speed = 0  # RPM
        self.thrust = 0

    def set_speed(self, speed):
        self.speed = speed
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * math.pow(self.dia, 3.5) / (math.sqrt(self.pitch))
        self.thrust = self.thrust * (4.23e-4 * self.speed * self.pitch)


class Quadcopter:
    # State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
    # From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
    def __init__(self, quads):
        self.quads = quads
        self.r = 0
        self.p = 0
        self.y = 0
        self.ode = scipy.integrate.ode(self.state_dot).set_integrator('vode', nsteps=500, method='bdf')
        for key in self.quads:
            self.quads[key]['state'] = np.zeros(3)
            self.quads[key]['m1'] = Propeller(self.quads[key]['prop_size'][0], self.quads[key]['prop_size'][1])
            self.quads[key]['m2'] = Propeller(self.quads[key]['prop_size'][0], self.quads[key]['prop_size'][1])
            self.quads[key]['m3'] = Propeller(self.quads[key]['prop_size'][0], self.quads[key]['prop_size'][1])
            self.quads[key]['m4'] = Propeller(self.quads[key]['prop_size'][0], self.quads[key]['prop_size'][1])

    def set_rotation(self, orient):
        (self.r, self.p, self.y) = tf.transformations.euler_from_quaternion([
            orient.x,
            orient.y,
            orient.z,
            orient.w,
        ])

    def state_dot(self, time, state, key):
        state_dot = np.zeros(3)

        # The acceleration
        aprop = (self.quads[key]['m1'].thrust + self.quads[key]['m2'].thrust + self.quads[key]['m3'].thrust +
                 self.quads[key]['m4'].thrust) / self.quads[key]['weight']

        dragx = 0.008
        dragy = 0.008

        state_dot[0] = aprop * abs(math.sin(self.p)) - dragx * pow(self.quads[key]['state'][0], 2)
        state_dot[1] = aprop * abs(math.sin(self.r)) - dragy * pow(self.quads[key]['state'][1], 2)

        return state_dot

    def update(self, dt):
        for key in self.quads:
            self.ode.set_initial_value(self.quads[key]['state'], 0).set_f_params(key)
            self.quads[key]['state'] = self.ode.integrate(self.ode.t + dt)

    def set_motor_speeds(self, quad_name, speeds):
        self.quads[quad_name]['m1'].set_speed(pwm_to_rpm(speeds[0]))
        self.quads[quad_name]['m2'].set_speed(pwm_to_rpm(speeds[1]))
        self.quads[quad_name]['m3'].set_speed(pwm_to_rpm(speeds[2]))
        self.quads[quad_name]['m4'].set_speed(pwm_to_rpm(speeds[3]))

    def get_linear_rate(self, quad_name):
        return self.quads[quad_name]['state']
