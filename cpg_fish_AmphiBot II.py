# Reimplemented from [1]
# [1]Crespi, A. and Ijspeert, A.J., 2006. AmphiBot II: An amphibious snake robot that crawls and swims using a central
#    pattern generator. In Proceedings of the 9th international conference on climbing and walking robots (CLAWAR 2006)
#    (No. CONF, pp. 19-27).
# University of York, Yunlong Lian
# April, 2021

import numpy as np
import matplotlib.pyplot as plt

class oscillator():
    """
    One of oscillator in CPG
    Oscillator Class only provide a computing process, it doesn't have any variables.
    """
    def __init__(self, timestep):
        """
        :param timestep:  step size used to calculate the integral, it can be regarded as how many seconds to update the joint angle
        """
        self.t = timestep

    def dy_r2(self, a_i, R_i, pos, dy1):
        dy2 = a_i*(a_i/4*(R_i - pos)-dy1)
        return dy2

    def dy_r1(self, dy1, dy2):
        new_dy1 = dy1 + dy2 * self.t
        return new_dy1

    def update_r(self, pos, dy1):
        p = pos + dy1 * self.t
        return p

    def dy_theta(self, m, n, v_i, weights, thetas, phis):
        sum = 0
        # coupling: between neighbour oscillators are set to delta_phi for the descending connections,
        #           and to -delta_phi for ascending connections
        # m, n is the index of a oscillator
        for i in range(max(0, m-1), min(m+2, len(weights))):
            if i > m:
                phi = -phis[i][n]
            elif i < m:
                phi = phis[i][n]
            else:
                continue
            sum += weights[i][n] * np.sin(thetas[i][n] - thetas[m][n] + phi)
        if n == 0:
            phi = -phis[m][1]
            x = 1
        else:
            phi = phis[m][0]
            x = 0
        sum += weights[m][n] * np.sin(thetas[m][x] - thetas[m][n] + phi)
        dy = 2 * np.pi * v_i + sum
        return dy

    def update_theta(self, theta, dy1):
        new_theta = theta + dy1 * self.t
        return new_theta

    def update_x(self, r_i, theta_i):
        xi = r_i * (1 + np.cos(theta_i))
        return xi

class cpg_fish(oscillator):
    def __init__(self, a_matrix, R_matrix, weights, phase_biases, timestep = 0.001):
        super(cpg_fish, self).__init__(timestep)

        self.t = timestep
        self.a = a_matrix
        self.R = R_matrix

        self.w = weights
        self.phi = phase_biases

        self.r_dy1 =  [[0 for i in range(len(self.w[0]))] for j in range(len(self.w))]
        self.x = [[0 for i in range(len(self.w[0]))] for j in range(len(self.w))]
        self.output_angles = [0 for i in range(len(self.w))]
        # print(self.output_angles)
        # print(self.r_dy1)

    def update_joint(self, i, j, v, r_matrix, theta_matrix):
        """
        To update one joint's oscillator
        :param i:  row index of the oscillator
        :param j:  column index of the oscillator
        :param v:  frequency parameters
        :param r_matrix:  all positions of oscillators
        :param theta_matrix:  all theta of oscillators
        :return: The new value of r[i][j] and theta[i][j]
        """
        # calculate the new position of r[i][j]
        r_dy2 = self.dy_r2(self.a[i][j], self.R[i][j], r_matrix[i][j], self.r_dy1[i][j])
        self.r_dy1[i][j] = self.dy_r1(self.r_dy1[i][j], r_dy2)
        r_ij = self.update_r(r_matrix[i][j], self.r_dy1[i][j])
        # calculate the new position of theta[i][j]
        theta_dy = self.dy_theta(i, j, v, self.w, theta_matrix, self.phi)
        theta_ij = self.update_theta(theta_matrix[i][j], theta_dy)
        # using new r and theta to calculate the new positive output signal
        self.x[i][j] = self.update_x(r_ij, theta_ij)
        return r_ij, theta_ij

    def update_joints(self, r_matrix, theta_matrix, v_matrix):
        """
        To update each oscillator
        :param r_matrix:
        :param theta_matrix:
        :param v_matrix:
        :return: The new values of all r and theta
        """
        for i in range(len(r_matrix)):
            for j in range(len(r_matrix[i])):
                # update each r and theta.
                r_matrix[i][j], theta_matrix[i][j] = self.update_joint(i, j, v_matrix[i][j], r_matrix, theta_matrix)
        return r_matrix, theta_matrix

    def update_angles(self):
        """
        After updated all oscillator, the final thing is to calculate all setpoints according to oscillators
        :return: all setpoints
        """
        for i in range(len(self.x)):
            self.output_angles[i] = self.x[i][0] - self.x[i][1]
        return self.output_angles

    def update_cpg(self, r_matrix, theta_matrix, v_matrix):
        """
        to update the entire CPG, and return state variables, setpoints
        :param r_matrix: state variables of each oscillator
        :param theta_matrix: state variables of each oscillator
        :param v_matrix: control variables, speed
        :return: all r, theta, setpoints
        """
        new_r, new_theta = self.update_joints(r_matrix, theta_matrix, v_matrix)
        output_angles = self.update_angles()
        return new_r, new_theta, output_angles

    def plot_result(self, y, title = None, timestep = None):
        l = len(y)
        if timestep == None:
            t_interval  = 1 / (10 ** (len(str(l))-1))
        else:
            t_interval = timestep
        # print(t_interval)
        t_end = int(l * t_interval)
        # print(t_end)
        t = np.arange(0,t_end,t_interval)
        # print(l,':', t_end, ":", t_interval)
        plt.figure(figsize= (15, 6))
        plt.plot(t, y)
        if title != None:
            plt.title(str(title))
        plt.show()

    def plot_results(self, y, label, ylim, timestep = None):
        l = len(y[0])
        if timestep == None:
            t_interval  = 1 / (10 ** (len(str(l))-1))
        else:
            t_interval = timestep
        # print(t_interval)
        t_end = int(l * t_interval)
        # print(t_end)
        t = np.arange(0, t_end, t_interval)
        num = len(y)
        plt.figure(figsize=(12, 8))
        for i in range(len(y)):
            plt.subplot(num, 1, i+1)
            plt.plot(t, y[i], label=label[i])
            plt.axis([0, t_end, ylim[0], ylim[1]])
            plt.legend(loc = "lower right")
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    # In the paper,
    # They chose all R,v, weights, phase baises to be equal for all oscillators

    a_matrix = [[100 for i in range(2)] for j in range(7)]

    R_matrix = [[1 for i in range(2)] for j in range(7)]

    # standing
    R_matrix_standing = [[0 for i in range(2)] for j in range(7)]

    v_matrix = [[1 for i in range(2)] for j in range(7)]

    w_matrix = [[4 for i in range(2)] for j in range(7)]

    phi_matrix = [[1 for i in range(2)] for j in range(7)]

    r_matrix = [[0.5 for i in range(2)] for j in range(7)]

    theta_matrix = [[1 for i in range(2)] for j in range(7)]

    # print(theta_matrix)
    cpg = cpg_fish(a_matrix, R_matrix, w_matrix, phi_matrix, 0.001)
    # Update the CPG firstly
    # r, theta, angles = cpg.update_cpg(r_matrix, theta_matrix)
    # Then, we can get all joints angle
    # print(angles)
    # print("r_dy1: ", cpg.r_dy1)

    old_r = r_matrix
    old_theta = theta_matrix
    angs = [[] for j in range(7)]
    xs = [[] for j in range(14)]
    thetas = [[] for j in range(14)]
    r0 = []
    for i in range(25000):
        if i == 5000:
            v_matrix = [[2 for i in range(2)] for j in range(7)]
            cpg.R = R_matrix
            # cpg.R = R_matrix_standing
        elif i == 10000:
            R_matrix = [[0.5+i for i in range(2)] for j in range(7)]
            v_matrix = [[1 for i in range(2)] for j in range(7)]
            # print(R_matrix)
            cpg.R = R_matrix
            # cpg.R = R_matrix_standing
        elif i == 15000:
            R_matrix = [[1 for i in range(2)] for j in range(7)]
            phi_matrix = [[-2 for i in range(2)] for j in range(7)]
            cpg.R = R_matrix
            cpg.phi = phi_matrix
        elif i == 20000:
            R_matrix = [[0.5 for i in range(2)] for j in range(7)]
            phi_matrix = [[1 for i in range(2)] for j in range(7)]
            cpg.R = R_matrix
            cpg.phi = phi_matrix
        r, theta, angles = cpg.update_cpg(old_r, old_theta, v_matrix)
        # print("theta: ", theta)
        old_r = r
        old_theta = theta
        old_angles = angles
        # perturbation
        if i == 12500:
            old_angles[0] = 1.5
            old_angles[1] = 1.5
            old_angles[2] = 1.5
            old_angles[3] = 1.5
        for k in range(7):
            angs[k].append(angles[k])
            for m in range(2):
                xs[k+m*7].append(cpg.x[k][m])
                thetas[k+m*7].append(theta[k][m])
        r0.append(r[0][0])
        #print(angles)

    labels = ['joint angle 0', 'joint angle 1', 'joint angle 2', 'joint angle 3',
              'joint angle 4', 'joint angle 5', 'joint angle 6']
    ylim = [-3, 3]
    cpg.plot_results(angs, labels, ylim, 0.001)

    # print(angs[0])
    # cpg.plot_result(angs[0], 'angle[0]', 0.001)
    # cpg.plot_result(xs[0], 'x[0]', 0.001)
    # cpg.plot_result(r0, 'r[0]', 0.001)

    # labels = ['x[0]', 'x[1]', 'x[2]', 'x[3]','x[4]', 'x[5]', 'x[6]',
    #           'x[7]', 'x[8]', 'x[9]', 'x[10]','x[11]', 'x[12]', 'x[13]']
    # labels = ['x[0]', 'x[1]', 'x[2]', 'x[3]', 'x[4]', 'x[5]', 'x[6]']
    # ylim = [-0.5, 2.5]
    # cpg.plot_results(xs[:7], labels, ylim, 0.001)
    #
    # labels = ['theta[0]', 'theta[1]', 'theta[2]', 'theta[3]', 'theta[4]', 'theta[5]', 'theta[6]']
    # ylim = [-20, 150]
    # cpg.plot_results(thetas[:7], labels, ylim, 0.001)
