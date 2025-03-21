from control import *
from control.matlab import *  # MATLAB-like functions
import numpy as np
from matplotlib import pyplot as plt
from .car_model import CarModel
# from car_model import CarModel


class StateSpaceSimulation:
    def __init__(self):
        self.sample_size = 0
        self.num_states = 0
        self.num_outputs = 0
        self.x = np.zeros([1, 1], dtype=float)
        self.y = np.zeros([1, 1], dtype=float)
        self.lqr_car = CarModel()
        self.sysd = 0

    def build_system(self, Vx):
        self.sysd = self.lqr_car.build_error_model(Vx)

    def ss_simulation(self, sys, x0, u):
        self.sysd = sys
        [A, B, C, D] = ssdata(self.sysd)
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        D = np.array(D)
        x0 = np.array(x0)
        u = np.array([u])
        self.sample_size = u.size
        self.num_states = A.shape[0]
        self.num_outputs = D.shape[0]
        self.x = np.zeros([self.num_states, self.sample_size], dtype=float)
        self.y = np.zeros([self.num_outputs, self.sample_size], dtype=float)
        self.x[:, 0] = x0.transpose()
        print("here1")
        for k in range(0, self.sample_size):
            print("here2")
            self.y[:, k] = np.add(np.dot(C, self.x[:, k]).reshape(self.num_states, 1), np.dot(D, u[k]))
            print("here3")
            print(f"np.dot(A, self.x[:, k]).reshape(self.num_states, 1): {np.dot(A, self.x[:, k]).reshape(self.num_states, 1)}")
            print("here4")
            print(f"np.dot(B, u[k])).transpose(): {np.dot(B, u[k]).transpose()}")
            print("here5")
            print(f"np.add(np.dot(A, self.x[:, k]).reshape(self.num_states, 1), np.dot(B, u[k])): {np.add(np.dot(A, self.x[:, k]).reshape(self.num_states, 1), np.dot(B, u[k])).transpose()}")
            self.x[:, k + 1] = np.add(np.dot(A, self.x[:, k]).reshape(self.num_states, 1), np.dot(B, u[k]))
            print(f"np.dot(C, self.x[:, k]).reshape(self.num_states, 1): {np.dot(C, self.x[:, k]).reshape(self.num_states, 1)}")
        return self.x, self.y
    
    def ss_simulation_step(self, sys, x0, u):
        self.sysd = sys
        [A, B, C, D] = ssdata(self.sysd)
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        D = np.array(D)
        x0 = np.array(x0)
        u = np.array([u])
        self.sample_size = u.size
        self.num_states = A.shape[0]
        self.num_outputs = D.shape[0]
        self.x = np.zeros([self.num_states, self.sample_size], dtype=float)
        self.y = np.zeros([self.num_outputs, self.sample_size], dtype=float)
        self.x[:, 0] = x0.transpose()
        # print("here1")
        # print(f"y: {self.y}")
        # print(f"np.dot(C, x0): {np.dot(C, x0)}")
        # print(f"np.dot(D, u): {np.dot(D, u).reshape(self.num_states, 1)}")
        # print(f"y add {self.y}")
        # self.y = np.add(np.dot(C, x0).reshape(self.num_states, 1), np.dot(D, u))
        # self.x = np.add(np.dot(A, self.x).reshape(self.num_states, 1), np.dot(B, u))
        self.y = np.add(np.dot(C, x0), np.dot(D, u).reshape(self.num_states, 1))
        self.x = np.add(np.dot(A, self.x), np.dot(B, u))
        return self.x, self.y
    
    def get_output(self,sys, x0, u):
        self.x, self.y = self.ss_simulation_step(sys, x0, u)
        return self.y

    def plot_ss_results(self, t):
        legend_state_labels = []
        legend_output_labels = []
        plt.subplot(1, 2, 1)
        for k in range(0, self.num_states):
            plt.plot(t, self.x[k, :])
            legend_state_labels.append("x" + str(k+1))
        plt.xlabel("Samples (k)")
        plt.ylabel(f"State values")
        plt.legend(legend_state_labels)
        plt.subplot(1, 2, 2)
        for k in range(0, self.num_outputs):
            plt.plot(t, self.y[k, :])
            legend_output_labels.append("y" + str(k+1))
        plt.xlabel("Samples (k)")
        plt.ylabel(f"Output values")
        plt.legend(legend_output_labels)
        plt.show()


def plot_ss_sol_example():
    v = 5  # m/s
    my_sim = StateSpaceSimulation()
    my_sim.build_system(v)
    x0 = np.array([[0.34],
                  [-0.12],
                  [0.42],
                  [0]])
    # delta1 = .5 * np.ones(10, dtype=float)
    # delta2 = np.zeros(10, dtype=float)
    # delta = np.concatenate((delta1, delta2), axis=None)
    delta = 0.19
    # tsim = linspace(0, delta.shape[0], delta.shape[0])
    # xsim, ysim = my_sim.ss_simulation(my_sim.sysd, x0, delta)
    xsim, ysim = my_sim.ss_simulation_step(my_sim.sysd, x0, delta)
    ysim2 = my_sim.get_output(my_sim.sysd, x0, delta)
    print(f"my_sim.sysd.C: {my_sim.sysd.C}")
    print(f"my_sim.sysd.D: {my_sim.sysd.D}")
    print(f"x,y: {xsim}, {ysim}")
    print(f"y {ysim2}")
    # print(f"y {len(ysim2)}")
    # print(f"y {len(ysim2[0])}")
    # my_sim.plot_ss_results(tsim)


if __name__ == '__main__':
    plot_ss_sol_example()
