from control.matlab import *  # MATLAB-like functions
import numpy as np
import copy
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
from .car_model import CarModel
# from car_model import CarModel


class LQRDesign:
    def __init__(self, car_model):
        self.sysd = car_model.build_error_model(0.1)
        self.num_states = self.sysd.A.shape[0]
        self.num_inputs = self.sysd.B.shape[1]
        self.Ts = self.sysd.dt
        self.Q = np.identity(1, dtype=float)
        self.R = np.identity(1, dtype=float)
        # self.q1_range = [0.1, 0.3, 1.0]
        # self.q2_range = [0.1, 0.3, 1.0]
        # self.q3_range = [10, 30.0, 100.0]
        # self.q4_range = [1.0, 3.0, 10.0]

        # didnt crash, slight ossc
        # self.Vx_range = [0.1, 0.5, 5.0]
        # self.q1_range = [20.0, 60.0, 200.0]
        # self.q2_range = [0.1, 0.3, 1.0]
        # self.q3_range = [0.1, 0.3, 1.0]
        # self.q4_range = [0.1, 0.3, 1.0]
        #
        
        self.Vx_range = [0.5, 1.5, 5.0]
        self.q1_range = [8.0, 24.0, 80.0]
        self.q2_range = [0.1, 0.3, 1.0]
        self.q3_range = [0.1, 0.3, 1.0]
        self.q4_range = [10.0, 30.0, 100.0]
        self.q1_coef = []
        self.q2_coef = []
        self.q3_coef = []
        self.q4_coef = []
        self.r1 = 1.0E-2
        self.compute_weight_functions()
        
    def compute_weight_functions(self):
        self.q1_coef = self.q_curve_fit(self.Vx_range, self.q1_range)
        self.q2_coef = self.q_curve_fit(self.Vx_range, self.q2_range)
        self.q3_coef = self.q_curve_fit(self.Vx_range, self.q3_range)
        self.q4_coef = self.q_curve_fit(self.Vx_range, self.q4_range)
        # print(f"q1_coef, q3_coef: [{self.q1_coef}], [{self.q3_coef}]")
        
    def q_curve_fit(self, x_input, y_output):
        popt, _ = curve_fit(self.curve_fit_power_func, x_input, y_output, maxfev=100000)
        a, b, c = popt
        return [a, b, c]
    
    def q_power_func(self, x_input, coefficients):
        a = coefficients[0]
        b = coefficients[1]
        c = coefficients[2]
        result = a * np.power(x_input, b) + c
        return result

    def build_system(self, car_model, Vx):
        self.sysd = car_model.build_error_model(Vx)

    def compute_weights(self, vel):
        self.q1_val = float(self.q_power_func(vel, self.q1_coef))
        self.q2_val = float(self.q_power_func(vel, self.q2_coef))
        self.q3_val = float(self.q_power_func(vel, self.q3_coef))
        self.q4_val = float(self.q_power_func(vel, self.q4_coef))
        self.Q = np.diag([self.q1_val, self.q2_val, self.q3_val, self.q4_val])  # FIXME: update to vary as function of Vx
        self.R = np.diag([self.r1])

    def compute_weights_identity(self):
        self.Q = np.identity(self.num_states, dtype=float)  # weights for outputs (states)
        self.R = np.identity(self.num_inputs, dtype=float)  # weights for inputs

    def compute_single_gain_sample(self, vel, sysd=None):
        if sysd is not None:
            self.sysd = sysd
        self.compute_weights(vel)
        K, S, E = dlqr(self.sysd, self.Q * self.Ts, self.R / self.Ts)
        return K

    def compute_sim_gain_samples(self, car_model, vx_vec):
        k_mat = np.empty((0, 4))
        for vx in vx_vec:
            self.build_system(car_model, vx)
            k = self.compute_single_gain_sample(vx, self.sysd)
            k_mat = np.append(k_mat, k, axis=0)
        return k_mat

    def curve_fit_power_func(self, x_input, a, b, c):
        result = a * np.power(x_input, b) + c
        return result

    def my_curve_fit(self, x_input, y_output):
        popt, _ = curve_fit(self.curve_fit_power_func, x_input, y_output, maxfev=100000)
        a, b, c = popt
        y_fit = self.curve_fit_power_func(x_input, a, b, c)
        return y_fit


def lqr_example():
    num_sims = 20
    V_max = 5
    V_min = 0.1
    Vx_vec = linspace(V_min, V_max, num_sims)
    my_car = CarModel()
    my_sys = my_car.build_error_model(V_min,2,1)
    my_sys_mod = copy.deepcopy(my_sys)
    my_sys_mod.B = my_sys.B[:,0]
    print(my_sys.B)
    print(my_sys_mod.B)
    my_lqr = LQRDesign(my_car)
    K_s = my_lqr.compute_single_gain_sample(V_min, my_sys_mod).flat
    K_mat = my_lqr.compute_sim_gain_samples(my_car, Vx_vec)
    x_hat = np.array([[1.24059389],
                      [5.64647673],
                      [1.21848635],
                      [1.79247224]])
    
    xy_hat = np.array([[1.24059389],
                      [5.64647673],
                      [1.21848635],
                      [1.79247224]])
    xy_hat[:] = 0
    K_mat_shape = K_mat.shape
    print(f"\nmy_sys: {my_sys}"
          f"\n my_sys_mod: {my_sys_mod}"
          f"\nnum_states: {my_sys.A.shape[0]}"
          f"\nnum_inputs: {my_sys.B.shape[1]}"
          f"\nK_mat[0]: {K_mat.flat[0]}"
          f"\nK_mat[1]: {K_mat.flat[1]}"
          f"\nK_mat[2]: {K_mat.flat[2]}"
          f"\nK_mat[3]: {K_mat.flat[3]}"
          f"\nK_s: {K_s}"
          f"\nK_s.flat[0]: {K_s[0]}"
          f"\nK_s.flat[1]: {K_s[1]}"
          f"\nK_s.flat[2]: {K_s[2]}"
          f"\nK_s.flat[3]: {K_s[3]}"
          f"\nKs dot x: {np.dot(K_s, x_hat.flat)}"
          # f"\nKs dot x flat: {-np.dot(K_s[0], x_hat).flat[0]}"
        #   f"\nK_mat: {K_mat}"
        #   f"\nx_hat: {x_hat}"
        #   f"\nx_hat.shape: {x_hat.shape}"
        #   f"\nxy_hat: {xy_hat[0][0]}"
        #   f"\nxy_hat: {xy_hat[1][0]}"
        #   f"\nxy_hat: {xy_hat[2][0]}"
        #   f"\nxy_hat flat: {xy_hat}"
          )
    try:
        for K in range(0, K_mat_shape[1]):
            plt.subplot(2, 2, K + 1)
            plt.xlabel("Velocity (m/s)")
            plt.ylabel(f"K{K + 1} Gain")
            k_flat = list(np.concatenate(K_mat[:, K]).flat)
            # print(k_flat)
            plt.scatter(Vx_vec, k_flat)
            K_fit = my_lqr.my_curve_fit(Vx_vec, k_flat)
            plt.plot(Vx_vec, K_fit)
    except ValueError or RuntimeError:
        pass
    plt.show()


if __name__ == '__main__':
    lqr_example()
