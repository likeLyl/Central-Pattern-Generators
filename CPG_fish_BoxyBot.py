# Reimplemented from [1]
# [1]Crespi, A., Lachat, D., Pasquier, A. and Ijspeert, A.J., 2008. Controlling swimming and crawling in a fish robot
#    using a central pattern generator. Autonomous Robots, 25(1), pp.3-13.
# University of York, Yunlong Lian
# April, 2021

import matplotlib.pyplot as plt
import numpy as np

class oscillator():
    def __init__(self, ss, ar, ax, R, X, r_0, x_0):
        self.stepsize = ss
        self.ax = ax
        self.ar = ar

        self.R = R
        self.r_dy1 = 0
        self.r_dy2 = 0
        self.r = r_0

        self.X = X
        self.x_dy1 = 0
        self.x_dy2 = 0
        self.x = x_0

    def update_r_2(self, r_position, r_dy1, stepsize):
        pos = r_position
        dy1 = r_dy1
        dt = stepsize

        # Get the r''
        dy2 =  ((self.ar * self.ar) / 4) * (self.R - pos) - self.ar * dy1
        # Get the r'
        dy1 += dy2 * dt
        # Get the r
        pos += dy1 * dt

        # update all parameters
        self.r = pos
        self.r_dy1 = dy1
        self.r_dy2 = dy2

        return pos

    def update_x_2(self, x_position, x_dy1, stepsize):
        pos = x_position
        dy1 = x_dy1
        dt = stepsize

        # Get the r''
        dy2 =  ((self.ax * self.ax) / 4) * (self.R - pos) - self.ax * dy1
        # Get the r'
        dy1 += dy2 * dt
        # Get the r
        pos += dy1 * dt

        # update all parameters
        self.x = pos
        self.x_dy1 = dy1
        self.x_dy2 = dy2

        return pos

    def update_r(self, p):
        pos = p
        # dy1 = r_dy1
        # times = stepsize

        # Get the r''
        # self.r_dy2 = (((self.ar*self.ar)/4) * (self.R-pos) - self.ar*self.r_dy1) * self.stepsize
        self.r_dy2 = (((self.ar * self.ar) / 4) * (self.R - pos) - self.ar * self.r_dy1)
        # Get the r'
        self.r_dy1 += self.r_dy2 * self.stepsize
        # Get the r
        pos += self.r_dy1 * self.stepsize
        self.r = pos

        return pos

    def update_x(self, p):
        pos = p
        # Get the r''
        self.x_dy2 = ((self.ax * self.ax) / 4) * (self.X-pos) - self.ax*self.x_dy1
        # Get the r'
        self.x_dy1 += self.x_dy2 * self.stepsize
        # Get the r
        pos += self.x_dy1 * self.stepsize
        # update the r's position
        self.x = pos
        return pos

class Fish_cpg1():
    def __init__(self, omega, ss, ar, ax, R, X, r, x, phi):
        """
        :param omega: (list)  desired frequency
        :param ss:    (number) step size, 1/dt
        :param ar:    (number) constant positive gains
        :param ax:    (number) constant positive gains
        :param R:     (list)   desired amplitude
        :param X:     (list)   desired offset
            For example:
            R = [R0, R1, R2]
            X = [X0, X1, X2]
        :param r:     (list)   state variable
        :param x:     (list)   state variable
            The r and x variables are initial positions of amplitude and offset,
            which will asymptotically and monotonically converge to R and X.
            For example:
            r = [r0, r1, r2]
            x = [x0, x1, x2]
        :param phi:   (list)   state variable
        """
        self.stepsize = ss
        # Three oscillators for three fins
        self.osc = [0,1,2]
        self.osc[0] = oscillator(ss, ar, ax, R[0], X[0], r[0], x[0])
        self.osc[1] = oscillator(ss, ar, ax, R[1], X[1], r[1], x[1])
        self.osc[2] = oscillator(ss, ar, ax, R[2], X[0], r[2], x[2])

        self.theta = [0,0,0]

        # state variable
        self.phi = phi

        self.omega = omega

        self.coupling_w = [[0,0.5,0.5],
                           [0.5,0,0.5],
                           [0.5,0.5,0]]
        self.coupling_phi = [[0,0,0],
                             [0,0,0],
                             [0,0,0]]

    def update_r(self):
        for osc in self.osc:
            osc.update_r(osc.r)
        all_osc_r = [self.osc[0].r, self.osc[0].r, self.osc[0].r]
        return all_osc_r

    def update_x(self):
        for osc in self.osc:
            osc.update_x(osc.x)
        all_osc_x = [self.osc[0].x, self.osc[0].x, self.osc[0].x]
        return all_osc_x

    def update_phi(self):
        """
        before update phi, the "r" and "x" should be updated firstly
        """

        dy_phi = [0,0,0]
        sum = [0,0,0]

        # calculate all dy_phi
        for i in range(len(self.phi)):
            for j in range(len(self.phi)):
                sum[i] += self.coupling_w[i][j] * self.osc[j].r * \
                        np.sin(self.phi[j] - self.phi[i] - self.coupling_phi[i][j])
            dy_phi[i] = self.omega[i] + sum[i]

        # update all the phi
        for i in range(len(self.phi)):
            self.phi[i] += dy_phi[i] * self.stepsize

        return self.phi

    def update_setpoints(self):
        """
        before update setpoints, we should update phi firstly

        """

        for i in range(len(self.theta)):
            self.theta[i] = self.osc[i].x + self.osc[i].r * np.cos(self.phi[i])
        return self.theta

    def set_parameters(self, desired_phi, desired_R, desired_X):
        """
        :param desired_phi: list -- length 3
        :param desired_R: list -- length 3
        :param desired_X: list -- length 3
        :return: None
        """
        self.phi = desired_phi
        for i in range(len(phi)):
            self.osc[i].R = desired_R[i]
            self.osc[i].X = desired_X[i]
            self.phi[i] = phi[i]


    def update_all(self, phi, ri, xi):
        """
        update all things and output all setpoints as a list
        """
        if len(phi) == len(ri) == len(xi) == 3:
            for i in range(len(phi)):
                self.osc[i].r = ri[i]
                self.osc[i].x = xi[i]
                self.phi[i] = phi[i]

        self.update_r()
        self.update_x()
        self.update_phi()
        setpoints = self.update_setpoints()

        return setpoints

    def plot_result(self, y):
        l = len(y)
        t_interval  = 1 / (10 ** (len(str(l))-1))
        # print(t_interval)
        t_end = int(l * t_interval)
        # print(t_end)
        t = np.arange(0,t_end,t_interval)
        # print(l,':', t_end, ":", t_interval)
        plt.figure()
        plt.plot(t, y)
        plt.show()

    def plot_results(self, y1, y2, y3, num, label, title):
        l = len(y1[0])
        t_interval = 1 / (10 ** (len(str(l)) - 1))
        # print(t_interval)
        t_end = int(l * t_interval)
        # print(t_end)
        t = np.arange(0, t_end, t_interval)
        # print(l,':', t_end, ":", t_interval)
        plt.figure(figsize=(8,6))
        plt.subplot(num + 0)
        plt.plot(t, y1[0], color='red', label=label[0][0])
        plt.plot(t, y1[1], color='orange', label=label[0][1])
        plt.plot(t, y1[2], color='blue', label=label[0][2])
        plt.axis([0, t_end, -3, 3])
        plt.title(title[0])
        plt.legend(loc = "right")
        plt.subplot(num + 1)
        plt.plot(t, y2[0], color='red', label=label[1][0])
        plt.plot(t, y2[1], color='orange', label=label[1][1])
        plt.plot(t, y2[2], color='blue', label=label[1][2])
        plt.axis([0, t_end, 0, 2.5])
        plt.title(title[1])
        plt.legend(loc = "right")
        plt.subplot(num + 2)
        plt.plot(t, y3[0], color='red', label=label[2][0])
        plt.plot(t, y3[1], color='orange', label=label[2][1])
        plt.plot(t, y3[2], color='blue', label=label[2][2])
        plt.axis([0, t_end, -1, 60])
        plt.title(title[2])
        plt.legend(loc = "right")
        # plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    # Test oscillator
    osc1 = oscillator(0.001, 20, 20, 1, 0, 2, 0)
    omeg = [np.pi*4, np.pi*4, np.pi*4]
    R_list = [1,1,1]
    X_list = [0,0,0]

    ri = [2, 1.2, 0.2]
    xi = [0.2, -1, -2]

    phi = [0,0,0]
    test_cpg = Fish_cpg1(omeg, 0.001, 20, 20, R_list, X_list, ri, xi, phi)
                    # omega, stpesize, ar, ax, R, X,
    pos = 2
    y = []
    y_phi = [[],[],[]]
    y_r = [[],[],[]]
    theta = [[],[],[]]
    # for i in range(4000):
    #     if i == 2000:
    #         osc1.r = 2
    #     osc1.update_r(osc1.r)
    #     print(osc1.r)
    #     y.append(osc1.r)
    #print(y)
    # test_cpg.plot_result(y)
    for i in range(4000):
        if i == 2000:
            test_cpg.osc[0].r = 2
            test_cpg.osc[1].r = 1.2
            test_cpg.osc[2].r = 0.2
        # First method to update
        # This is to plot the result
        test_cpg.update_x()
        test_cpg.update_r()
        phi_pos = test_cpg.update_phi()
        theta_v = test_cpg.update_setpoints()

        # Second method to update
        # controller send the control parameters using set_parameters()
        # test_cpg.set_parameters(Phi, R, X)
        # receive the state variables to update next motion
        # theta_v = test_cpg.update_all(phi, r, x)

        y_r[0].append(test_cpg.osc[0].r)
        y_r[1].append(test_cpg.osc[1].r)
        y_r[2].append(test_cpg.osc[2].r)

        y_phi[0].append(test_cpg.phi[0])
        y_phi[1].append(test_cpg.phi[1])
        y_phi[2].append(test_cpg.phi[2])

        theta[0].append(theta_v[0])
        theta[1].append(theta_v[1])
        theta[2].append(theta_v[2])

    label = [['θ 0', 'θ 1', 'θ 2'],
             ['r 0', 'r 1', 'r 2'],
             ['Φ 0', 'Φ 1', 'Φ 2']]
    title = ['θ', 'r', 'Φ']
    test_cpg.plot_results(theta, y_r, y_phi, 311, label, title)
    # 311: three picutres(3), one picture per line(1), first picture number(1)