from control import *
from control.matlab import *  # MATLAB-like functions
import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
import yaml
import math


class CarModel:
    def __init__(self, car_parameter_input_path=None):
        self.Ts = 1/20
        self.mu = 0.01  # FIXME coeff of friction of driving surface
        self.g = 9.81  # gravity acceleration
        self.m = 5  # FIXME total mass (kg)
        self.L = 0.325  # FIXME wheel base (meters)
        self.Lf = self.L * (7/11)  # distance from CG to front axel
        self.Lr = self.L * (6/11)  # distance from CG to rear axel
        self.mf = self.m * (1 - self.Lf / self.L)  # FIXME mass on front axel
        self.mr = self.m * (1 - self.Lr / self.L)  # FIXME mass on rear axel
        self.Iz = (self.Lf**2 * self.mf) + (self.Lr**2 * self.mr) # moment of inertia
        self.cf = 0.65  # FIXME front tire cornering stiffness
        self.cr = 3.0  # FIXME rear tire cornering stiffness
        self.sysd = 0
        self.Kv = (self.mf / self.cf) - (self.mr / self.cr)  # understeer gradient
        # self.Kv = (self.m / (self.Lr + self.Lf)) * (self.Lr / self.cf) - (self.Lf / self.cf)  # understeer gradient
        self.car_parameter_input_path = car_parameter_input_path
        if self.car_parameter_input_path is not None:
            self.car_parameter_input_dictionary = {}
            self.update_parameters()

    def update_parameters(self):
        with open(self.car_parameter_input_path, "r") as car_parameter_file:
            car_inputs = yaml.load(car_parameter_file, Loader=yaml.FullLoader)
            self.car_parameter_input_dictionary = eval(car_inputs)
            for key in self.car_parameter_input_dictionary:
                value = self.car_parameter_input_dictionary[key]
                setattr(self, key, value)

    def build_error_model(self, Vx, measure_model=None, input_model=None):
        """
        states:
        x1 - ecg: cross-trackk error from center of gravity (cg) --- = (pose_error_y * delta_x_path - pose_error_x * delta_y_path) / (delta_x_path^2 + delta_y_path^2)
        x2 - ecg_dot: cross-trackk error rate from cg --- = vy + vx * sin(theta_error)
        x3 - theta_e: heading error --- = path_angle - car_yaw_angle
        x4 - theta_e_dot: heading error rate --- = (theta_e_k - theta_e_km1) / self.Ts # theta_e_k = heading error at sample k AND theta_e_km1 = heading error at sample k - 1
        """
        if Vx <= 0.1:
            Vx = 0.1

        a11 = 0
        a12 = 1
        a13 = 0
        a14 = 0
        a21 = 0
        a22 = -(self.cf + self.cr) / (self.m * Vx)
        a23 = (self.cf + self.cr) / self.m
        a24 = (self.Lr * self.cr - self.Lf * self.cf) / (self.m * Vx)
        a31 = 0
        a32 = 0
        a33 = 0
        a34 = 1
        a41 = 0
        a42 = (self.Lr * self.cr - self.Lf * self.cf) / (self.Iz * Vx)
        a43 = (self.Lf * self.cf - self.Lr * self.cr) / self.Iz
        a44 = -((self.Lr ** 2) * self.cr + (self.Lf ** 2) * self.cf) / (self.Iz * Vx)

        A = np.matrix(
            [[a11, a12, a13, a14],
             [a21, a22, a23, a24],
             [a31, a32, a33, a34],
             [a41, a42, a43, a44]]
        )

        # Input matrix
        B1 = np.matrix(
            [[0],
            [self.cf / self.m],
            [0],
            [self.Lf * self.cf / self.Iz]]
        )

        if input_model == 1:
            # Input matrix (feed forward) (used in Kalman filter, not in LQR)
            B_ff = np.matrix(
                [[0],
                [a24 - Vx],
                [0],
                [a44]]
            )
            B = np.concatenate([B1, B_ff],axis=1)
        else:
            # Input matrix
            B = B1

        if measure_model is not None:
            # Output matrix
            if measure_model == 1:
                # measure ecg
                C = np.matrix(
                    [[1, 0, 0, 0]
                     ])
                # Feed-Forward matrix
                # measure on only 1 state
                # D = np.matrix(
                #     [[0]
                #      ])
            if measure_model > 1:
                if measure_model == 2:
                    # measure ecg & theta_e states
                    C = np.matrix(
                        [[1, 0, 0, 0],
                         [0, 0, 1, 0]
                         ])
                elif measure_model == 3:
                    # measure ecg & theta_e_dot states
                    C = np.matrix(
                        [[1, 0, 0, 0],
                         [0, 0, 1, 0]
                         ])

                # Feed-Forward matrix
                # measure on only 2 states
                # D = np.matrix(
                #     [[0],
                #      [0]
                #      ])
        else:
            # Output matrix
            # measure all states
            C = np.matrix(
                [[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1],
                 ])

            # Feed-Forward matrix
            # measure on all states
            # D = np.matrix(
            #     [[0],
            #      [0],
            #      [0],
            #      [0]
            #      ])
        # Feed-Forward matrix
        D = np.zeros([C.shape[0],B.shape[1]])

        sys = ss(A, B, C, D)
        self.sysd = c2d(sys, self.Ts, method='zoh')
        return self.sysd

    def build_2d_bicycle_model(self, delta, psi, Vx, Vy):
        """
        states:
        x1 - Vx: longitudinal velocity (m/s) ------- WRT: BODY FRAME
        x2 - Vy: lateral velocity (m/s) ------------ WRT: BODY FRAME
        x3 - psi: yaw angle (radians) -------------- WRT: WORLD FRAME
        x4 - psi_dot: yaw angle rate (radians/s) --- WRT: WORLD FRAME
        x5 - x: coordinate (m) --------------------- WRT: WORLD FRAME
        x6 - y: coordinate (m) --------------------- WRT: WORLD FRAME

        inputs:
        u1 - delta: steering angle (rad) ----------- WRT: BODY FRAME
        u2 - ax: longitudinal acceleration --------- WRT: BODY FRAME
        """
        a11 = -self.mu * self.g / Vx
        a12 = -(self.cf / self.m) * (math.sin(delta) / Vx)
        a13 = 0
        a14 = Vy - (self.cf * self.Lf / self.m) * (math.sin(delta) / Vx)
        a15 = 0
        a16 = 0
        a21 = 0
        a22 = -(self.cf * math.cos(delta) + self.cr) / (self.m * Vx)
        a23 = 0
        a24 = ((self.Lr * self.cr - self.Lf * self.cf * math.cos(delta)) / (self.m * Vx)) - Vx
        a25 = 0
        a26 = 0
        a31 = 0
        a32 = 0
        a33 = 0
        a34 = 1
        a35 = 0
        a36 = 0
        a41 = 0
        a42 = (self.Lr * self.cr - self.Lf * self.cf * math.cos(delta)) / (self.Iz * Vx)
        a43 = 0
        a44 = -((self.Lr ** 2) * self.cr + (self.Lf ** 2) * self.cf * math.cos(delta)) / (self.Iz * Vx)
        a45 = 0
        a46 = 0
        a51 = math.cos(psi)
        a52 = -math.sin(psi)
        a53 = 0
        a54 = 0
        a55 = 0
        a56 = 0
        a61 = math.sin(psi)
        a62 = math.cos(psi)
        a63 = 0
        a64 = 0
        a65 = 0
        a66 = 0

        b11 = -(self.cf * math.sin(delta)) / self.m
        b21 = self.cf * math.cos(delta) / self.m
        b31 = 0
        b41 = self.Lf * self.cf * math.cos(delta) / self.Iz
        b51 = 0
        b61 = 0

        b12 = 1
        b22 = 0
        b32 = 0
        b42 = 0
        b52 = 0
        b62 = 0

        A = np.matrix(
            [[a11, a12, a13, a14, a15, a16],
             [a21, a22, a23, a24, a25, a26],
             [a31, a32, a33, a34, a35, a36],
             [a41, a42, a43, a44, a45, a46],
             [a51, a52, a53, a54, a55, a56],
             [a61, a62, a63, a64, a65, a66]]
        )

        # Input matrix
        B = np.matrix(
            [[b11, b12],
             [b21, b22],
             [b31, b32],
             [b41, b42],
             [b51, b52],
             [b61, b62]]
        )

        # Output matrix
        C = np.matrix(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]
             ])

        # Feed-Forward matrix
        D = np.matrix(
            [[0, 0],
             [0, 0],
             [0, 0],
             [0, 0],
             [0, 0],
             [0, 0]]
        )

        sys = ss(A, B, C, D)
        self.sysd = c2d(sys, self.Ts, method='zoh')
        np.set_printoptions(precision=3)
        return self.sysd

    def build_general_discrete_model(self, A, B, C, D, Ts):
        A = np.matrix(A)  # System dynamics/behavior model
        B = np.matrix(B)  # Input matrix
        C = np.matrix(C)  # Output matrix
        D = np.matrix(D)  # Feed-Forward matrix

        sys = ss(A, B, C, D)
        self.sysd = c2d(sys, Ts, method='zoh')
        return self.sysd

    def calc_ecg(self, car_xy, path_xy):
        delta_x_path = path_xy[1][0] - path_xy[0][0]
        delta_y_path = path_xy[1][1] - path_xy[0][1]
        pose_error_x = car_xy[0] - path_xy[0][0]
        pose_error_y = car_xy[1] - path_xy[0][1]
        ecg = (pose_error_y * delta_x_path - pose_error_x * delta_y_path) / (delta_x_path**2 + delta_y_path**2)
        return ecg

    def calc_theta_e_dot(self, car_yaw, path_yaw):
        theta_e_dot = path_yaw - car_yaw
        return theta_e_dot

    def calc_output(self, state_vector):
        y = np.dot(self.sysd.C, state_vector)
        return y

    def ctrb_test(self, sysd):
        result = False
        rank = np.linalg.matrix_rank(ctrb(sysd.A, sysd.B))
        if rank == sysd.A.shape[0]:
            result = True
        else:
            result = False
        print(f"Is system controlable? {result}, rank of ctrb matrix: {rank}")
        return result

    def obsv_test(self, sysd):
        result = False
        rank = np.linalg.matrix_rank(obsv(sysd.A, sysd.C))
        if rank == sysd.A.shape[0]:
            result = True
        else:
            result = False
        print(f"Is system observable? {result}, rank of obsv matrix: {rank}")
        return result


def build_model_example():
    V_x = 3
    delta = 1
    psi = 0
    Vx = 3
    Vy = 0
    x0 = np.array([[0.1], [0.2], [0.3], [0.4]])
    my_car_model = CarModel()
    my_sys = my_car_model.build_error_model(V_x,2,1)
    y = my_car_model.calc_output(x0)
    # my_sys = my_car_model.build_2d_bicycle_model(delta, psi, Vx, Vy)
    [A, B, C, D] = ssdata(my_sys)
    print(f"my_sys.A: {my_sys.A}" 
          f"\n my_sys.Ts: {my_sys.dt}"
          f"\n A: {A}"
          f"\n rank(A): {np.linalg.matrix_rank(A)}"
          f"\n B: {B}"
          f"\n C: {C}"
          f"\n D: {D}"
          f"\n y: {y}"
          )
    my_car_model.ctrb_test(my_sys)
    my_car_model.obsv_test(my_sys)


if __name__ == '__main__':
    build_model_example()
