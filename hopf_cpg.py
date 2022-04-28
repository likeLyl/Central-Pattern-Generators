# Reimplemented from [1]
# [1]Righetti, L. and Ijspeert, A.J., 2008, May.
#    Pattern generators with sensory feedback for the control of quadruped locomotion.
#    In 2008 IEEE International Conference on Robotics and Automation (pp. 819-824). IEEE.
# University of York, Yunlong Lian
# April, 2021

import math as m
import numpy as np
import matplotlib.pyplot as  plt

class cpg_hopf():
    def __init__(self,alpha, beta, mu, omega_stance, omega_swing, b):
        self.alpha = alpha
        self.beta = beta
        self.mu = mu
        self.omega_stance = omega_stance
        self.omega_swing = omega_swing
        self.b = b
        self.pos = [1,1]

    def hopf(self,x, y, steps = 0.001):
        self.r_2 = x ** 2 + y ** 2
        #print(self.b*y)
        omega1 = self.omega_stance / (m.exp(-self.b * y) + 1)
        omega2 = self.omega_swing / (m.exp(self.b * y) + 1)
        # print(m.exp(-self.b * y))

        self.omega = omega1 + omega2
        dx = self.alpha * (self.mu - self.r_2) * x - self.omega * y
        dy = self.beta * (self.mu - self.r_2) * y + self.omega * x
        return [x + dx*steps, y + dy*steps]

    def get_trajectory(self, pos, data_number = 5000):
        x = []
        y = []
        px = pos[0]
        py = pos[1]
        for i in range(data_number):
            position = self.hopf(px, py)
            px = position[0]
            py = position[1]
            x.append(position[0])
            y.append(position[1])
        return x,y

class cpg(object):
    def __init__(self,alpha, beta, mu, omega_stance, omega_swing, b):
        self.cm = [[0, -1, -1, 1], [-1, 0, 1, -1],  # coupling_matrix
                   [-1, 1, 0, -1], [1, -1, -1, 0]]
        self.alpha = alpha
        self.beta = beta
        self.mu = mu
        self.omega_stance = omega_stance
        self.omega_swing = omega_swing
        self.b = b
        self.pos = [1,1]

    def hopf(self,x, y, steps = 0.001):
        self.r_2 = x ** 2 + y ** 2
        #print(self.b*y)
        omega1 = self.omega_stance / (m.exp(-self.b * y) + 1)
        omega2 = self.omega_swing / (m.exp(self.b * y) + 1)
        # print(m.exp(-self.b * y))
        self.omega = omega1 + omega2

        dx = self.alpha * (self.mu - self.r_2) * x - self.omega * y
        dy = self.beta * (self.mu - self.r_2) * y + self.omega * x

        return [x + dx*steps, y + dy*steps]

    def coupling(self, cells, steps = 0.001):
        # all_legs are defined as following:
        # all_legs = [[leg0_x,leg0_y], [leg1_x,leg1_y],
                    # [leg2_x,leg2_y], [leg3_x,leg3_y]]
        x = []
        y = []
        new_cells = []
        for i in range(len(cells)):
            x.append(cells[i][0])
            y.append(cells[i][1])
        # print("x: ", x)
        # print("y: ", y)
        if len(cells) != len(self.cm):
            raise ValueError("Error, the number of legs should be equal to the column number of coupling matrics")

        for i in range(len(cells)):
            for j in range(len(self.cm[0])):
                cells[i][1] += self.cm[i][j]*cells[j][1]*steps        # yi
            new_cells.append(cells[i])

        return new_cells

    def get_trajectory(self, pos, data_number = 5000):
        x = []
        y = []
        px = pos[0]
        py = pos[1]
        for i in range(data_number):
            position = self.hopf(px, py)
            px = position[0]
            py = position[1]
            x.append(position[0])
            y.append(position[1])
        return x,y

    def get_all_trajectory(self, data_number = 5000):
        cells = []
        cell = []
        # l1_x = pos[0]
        # l1_y = pos[1]
        # l2_x = pos[0]
        # l2_y = pos[1]
        # l3_x = pos[0]
        # l3_y = pos[1]
        # l4_x = pos[0]
        # l4_y = pos[1]

        l1_x = 1
        l1_y = 0
        l2_x = -1
        l2_y = 0
        l3_x = -1
        l3_y = 0
        l4_x = 1
        l4_y = 0

        cell.append([l1_x,l1_y])
        cell.append([l2_x,l2_y])
        cell.append([l3_x,l3_y])
        cell.append([l4_x,l4_y])
        cells.append(cell)

        for i in range(data_number-1):
            l1_pos = self.hopf(l1_x, l1_y)
            l2_pos = self.hopf(l2_x, l2_y)
            l3_pos = self.hopf(l3_x, l3_y)
            l4_pos = self.hopf(l4_x, l4_y)
            cell[0] = l1_pos
            cell[1] = l2_pos
            cell[2] = l3_pos
            cell[3] = l4_pos
            new_cell = self.coupling(cell)
            # print(new_cell)
            l1_x = new_cell[0][0]
            l1_y = new_cell[0][1]
            l2_x = new_cell[1][0]
            l2_y = new_cell[1][1]
            l3_x = new_cell[2][0]
            l3_y = new_cell[2][1]
            l4_x = new_cell[3][0]
            l4_y = new_cell[3][1]

            cells.append(new_cell)
        return cells

if __name__ == '__main__':
    # coupling_matrix = [[0,-1,-1, 1], [-1, 0, 1, -1], [-1, 1, 0, -1], [1, -1, -1, 0]]

    # test hopf oscillator
    # cpg = cpg_hopf(100, 100, 9, np.pi*2, np.pi*2, 50)
    # x1, x2 = cpg.get_trajectory([1,1])
    # cpg2 = cpg_hopf(100, 100, 1, np.pi*2, np.pi*2, 50)
    # x3, x4 = cpg2.get_trajectory([1, 1])
    # # plot the values of two oscillators
    # t = np.arange(0, 5, 0.001)
    # fig1 = plt.figure()
    # plt.plot(t, x1)  # x of the first oscillator
    # plt.plot(t, x2)  # y of the first oscillator
    # plt.plot(t, x3)  # x of the second oscillator
    # plt.show()
    # fig3 = plt.figure()
    # plt.axis('equal')
    # plt.plot(x1,x2)
    # plt.show()

    # test for coupling
    # legs = [[1,2], [3,4],
    #         [5,6], [7,8]]
    # print(legs)
    # cpg_coupling = cpg(100, 100, 9, np.pi*2, np.pi*2, 50)
    # c = cpg_coupling.coupling(legs, 0.001)
    # print(c)

    # cpg1 = cpg(100, 100, 9, np.pi*2, np.pi*2, 50)
    # x1, x2 = cpg1.get_trajectory([1,1])
    # cpg2 = cpg(100, 100, 1, np.pi*2, np.pi*2, 50)
    # x3, x4 = cpg2.get_trajectory([1, 1])
    # # plot the values of two oscillators
    # t = np.arange(0, 5, 0.001)
    # fig1 = plt.figure()
    # plt.plot(t, x1)  # x of the first oscillator
    # plt.plot(t, x2)  # y of the first oscillator
    # plt.plot(t, x3)  # x of the second oscillator
    # plt.show()
    # fig3 = plt.figure()
    # plt.axis('equal')
    # plt.plot(x1,x2)
    # plt.show()

    # test_cpg = cpg(100, 100, 9, np.pi*2, np.pi*2, 50)
    # # test for coupling:
    # cells = []
    # l1 = test_cpg.hopf(1,1)
    # print(l1)
    # cells.append(l1)
    # l2 = test_cpg.hopf(1, 2)
    # print(l2)
    # cells.append(l2)
    # l3 = test_cpg.hopf(1, 3)
    # print(l3)
    # cells.append(l3)
    # l4 = test_cpg.hopf(1, 4)
    # print(l4)
    # cells.append(l4)
    # coupled = test_cpg.coupling(cells)
    # print("coupled: ", coupled)

    # test for coupled cpg
    ccpg = cpg(100, 100, 1, np.pi*2, np.pi, 50)
    cells_trace = ccpg.get_all_trajectory(10000)
    # print(cells_trace[0][0][0])
    # print(len(cells_trace[0][0]))
    x1 = []
    x2 = []
    x3 = []
    x4 = []
    t = np.arange(0, 10, 0.001)
    for j in range(len(cells_trace)):
        x1.append(cells_trace[j][0][0])
        x2.append(cells_trace[j][1][0])
        x3.append(cells_trace[j][2][0])
        x4.append(cells_trace[j][3][0])

    fig0 = plt.figure()
    plt.subplot(4,1,1)
    plt.plot(t, x1, color='red', label='x1')
    plt.axis([0, 10, -1.1, 1.1])
    plt.subplot(4, 1, 2)
    plt.plot(t, x2, color='orange', label='x2')
    plt.axis([0, 10, -1.1, 1.1])
    plt.subplot(4, 1, 3)
    plt.plot(t, x3, color='blue', label='x3')
    plt.axis([0, 10, -1.1, 1.1])
    plt.subplot(4, 1, 4)
    plt.plot(t, x4, color='green', label='x4')
    plt.axis([0, 10, -1.1, 1.1])
    # plt.axis([0, 6.3, -2, 2])
    # plt.legend(loc="upper left")
    plt.tight_layout()
    plt.savefig('trot gait.png')
    plt.show()


    # fig1 = plt.figure()
    # plt.plot(t, x1, color = 'red',label = 'x1')
    # plt.axis([0, 6.3, -2, 2])
    # plt.legend(loc = "upper left")
    # plt.show()
    # fig2 = plt.figure()
    # plt.plot(t, x2, color = 'orange', label = 'x2')
    # plt.axis([0, 6.3, -2, 2])
    # plt.legend(loc = "upper left")
    # plt.show()
    # fig3 = plt.figure()
    # plt.plot(t, x3, color = 'blue', label = 'x3')
    # plt.axis([0, 6.3, -2, 2])
    # plt.legend(loc = "upper left")
    # plt.show()
    # fig4 = plt.figure()
    # plt.plot(t, x4, color = 'green', label = 'x4')
    # plt.axis([0, 6.3, -2, 2])
    # plt.legend(loc = "upper left")
    # plt.show()

    # fig5 = plt.figure()
    # plt.plot(t, x1, color = 'red', label = 'x1')
    # plt.plot(t, x2, color = 'orange', label = 'x2')
    # plt.axis([0, 6.3, -2, 2])
    # plt.legend(loc = "upper left")
    # plt.show()
    # fig6 = plt.figure()
    # plt.plot(t, x3, color = 'blue', label = 'x3')
    # plt.plot(t, x4, color = 'green', label = 'x4')
    # plt.axis([0, 6.3, -2, 2])
    # plt.legend(loc = "upper left")
    # plt.show()
