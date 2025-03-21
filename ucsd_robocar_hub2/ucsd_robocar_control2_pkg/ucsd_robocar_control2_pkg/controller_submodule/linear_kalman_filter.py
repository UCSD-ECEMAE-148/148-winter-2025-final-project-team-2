from control import *
from control.matlab import *  # MATLAB-like functions
import numpy as np
from .car_model import CarModel
# from car_model import CarModel


class LinearKalmanFilter:
    def __init__(self):
        self.sample_size = []  # length of input vector
        self.Pp = []  # error covariance
        self.P_mat = []  # storage for Pp over time
        self.K_mat = []  # K gain matrix
        self.xhat = []  # optimal state estimate
        self.yhat = []  # optimal output estimate
        self.xhat_mat = []  # storage for xhat over time
        self.yhat_mat = []  # storage for yhat over time
        self.lqr_car = CarModel()
        self.sysd = 0
        self.debug = False

    def build_system(self, Vx):
        self.sysd = self.lqr_car.build_error_model(Vx,measure_model=2,input_model=1)

    def lkf(self, sys, x0, u, y, P0, Qo, Ro):
        self.sysd = sys
        [A, B, C, D] = ssdata(self.sysd)
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        D = np.array(D)
        x0 = np.array(x0)
        u = np.array(u)
        y = np.array(y)
        P0 = np.array(P0)
        Qo = np.array(Qo)
        Ro = np.array(Ro)

        a_num_rows, a_num_cols = A.shape
        d_num_rows, d_num_cols = D.shape
        self.Pp = P0
        self.xhat = x0
        self.sample_size = u.shape[1]
        self.xhat_mat = np.zeros([a_num_rows, self.sample_size], dtype=float)
        self.yhat_mat = np.zeros([d_num_rows, self.sample_size], dtype=float)
        self.K_mat = np.zeros([a_num_rows, self.sample_size], dtype=float)

        A_t = A.transpose()
        C_t = C.transpose()
        num_states = A.shape[0]
        # print(f"sample size: {self.sample_size}")

        for k in range(0, self.sample_size):

            try:
                self.xhat_mat[:, k] = self.xhat.transpose()  # store the estimates
                
                # Kalman predictor gain
                K = np.dot(np.dot(np.dot(A, self.Pp), C_t), np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro))) 

                # Time update
                self.xhat = np.add(np.dot(A, self.xhat).reshape(num_states, 1),np.dot(B, u[:,[k]]))  # predicted state estimate
                self.Pp = np.add(np.dot(np.dot(A, self.Pp), A_t), Qo)  # covariance

                # print(f"lkf step5: {np.dot(B, u[:,[k]])}")
                # print(f"lkf step6: {K}")
                # print(f"lkf step7: {y[:, k]}")
                # print(f"lkf step8: {np.dot(K, y[:, k])}")
                self.xhat = np.add(np.dot((np.subtract(A, np.dot(K, C))), self.xhat).reshape(num_states, 1), np.add(np.dot(B, u[:,[k]]), np.dot(K, y[:, [k]]).reshape(num_states, 1)))
                self.Pp = np.subtract(self.Pp, np.dot(np.dot(np.dot(np.dot(self.Pp, C_t), np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro))), C), self.Pp))
                
                # filtered output prediction
                self.yhat = np.dot(C, self.xhat)

                # self.xhat_mat[:, k] = self.xhat.transpose()  # store the estimates
                # # Time update
                # self.xhat = np.add(np.dot(A, self.xhat).reshape(num_states, 1),np.dot(B, u[k]))  # predicted state estimate
                # self.Pp = np.add(np.dot(np.dot(A, self.Pp), A_t), Qo)  # covariance
                # # Kalman gain
                # K = np.dot(\
                #     np.dot(self.Pp, C_t), \
                #     np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro))) 
                # # Measurement update
                # self.xhat = np.add(\
                #     (self.xhat).reshape(num_states, 1), \
                #     np.dot(K, np.subtract(y[:, k], np.dot(C, self.xhat))).reshape(num_states, 1))
                # self.Pp = np.subtract(self.Pp, np.dot(np.dot(np.dot(np.dot(self.Pp, C_t), np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro))), C), self.Pp))
                # # filtered output
                # self.yhat = np.dot(C, self.xhat)
                if self.debug:
                    print(f"A: {A}")
                    # print(f"B: {B}")
                    # print(f"C: {C}")
                    # print(f"D: {D}")
                    # print(f"self.x0: {x0}")
                    # print(f"u: {u}")
                    # print(f"y: {y}")
                    # print(f"P0: {P0}")
                    # print(f"y: {y}")
                    # print(f"self.xhat: {self.xhat}")
                    # print(f"u: {u}")
                    # print(f"u[0]: {u[0]}")
                    # print(f"np.dot(B, u[k]): {np.dot(B, u)}")
                    # print(f"K: {K}")
                    # print(f"y[:, k]: {y[:, 0]}")
                    # print(f"np.dot(K, y[:, k]): {np.dot(K, y[:, 0])}")
                    # print(f"step7: {np.dot(K, y[:, 0]).reshape(num_states, 1)}")
                    # print(f"self.xhat_mat: {self.xhat_mat}")
                    # print(f"self.xhat: {self.xhat}")
                    # print(f"A: {A}")
                    # print(f"self.xhat: {self.xhat}")
                    # print(f"np.dot(A, self.xhat) 2: {np.dot(A, self.xhat)}")
                    # print(f"B: {B}")
                    # print(f"u[k]: {u[k]}")
                    # print(f"K_mat: {self.K_mat}")
                    # print(f"K: {K}")
                    # print(f"y: {self.yhat}")
                    # print(f"step9: {self.yhat_mat}")
                    # print(f"step1: {np.dot(C, self.Pp)}")
                    # print(f"step2: {np.dot(np.dot(C, self.Pp), C_t)}")
                    # print(f"step3: {np.add(np.dot(np.dot(C, self.Pp), C_t), Ro)}")
                    # print(f"step4: {np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro))}")
                    # # print(f"u[:,[k]]: {u[:,[k]]}")
                    # print(f"lkf step5: {np.dot(B, u[:,[k]])}")
                    # print(f"lkf step1 sim: {K}")
                    # print(f"lkf step2 sim: {y[:, [k]]}")
                    # print(f"lkf step3 sim: {np.dot(K, y[:, [k]])}")
            except np.linalg.LinAlgError:
                print("Singluar Matrix Detected")
        return self.xhat, self.Pp
    
    def lkf_step(self, sys, x0, u, y, P0, Qo, Ro):
        self.sysd = sys
        [A, B, C, D] = ssdata(self.sysd)
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        D = np.array(D)
        x0 = np.array(x0)
        u = np.array(u)
        y = np.array(y)
        P0 = np.array(P0)
        Qo = np.array(Qo)
        Ro = np.array(Ro)

        a_num_rows, a_num_cols = A.shape
        d_num_rows, d_num_cols = D.shape
        self.Pp = P0
        self.xhat = x0

        A_t = A.transpose()
        C_t = C.transpose()
        num_states = A.shape[0]

        try:
            # Kalman predictor gain
            K = np.linalg.multi_dot([A, self.Pp, C_t, np.linalg.inv(np.add(np.linalg.multi_dot([C, self.Pp, C_t]), Ro))])

            # filtered state estimate
            self.xhat = np.add(np.dot((np.subtract(A, np.dot(K, C))), self.xhat).reshape(num_states, 1), np.add(np.dot(B, u), np.dot(K, y).reshape(num_states, 1)))

            # Predicted error covariance
            self.Pp = \
                np.add(\
                    np.subtract(\
                        np.linalg.multi_dot([A, self.Pp, A_t]), \
                        (np.linalg.multi_dot([A, self.Pp, C_t, np.linalg.inv(np.add(np.linalg.multi_dot([C, self.Pp, C_t]), Ro)), C, self.Pp, A_t])) \
                ),Qo)
            
            # Kalman predictor gain
            # K = np.dot(np.dot(np.dot(A, self.Pp), C_t), np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro))) 
            # Predicted error covariance
            # self.Pp = \
            #     np.subtract(\
            #         np.dot(A, np.dot(self.Pp, A_t), \
            #         np.add(\
            #             np.dot(np.dot(np.dot(A, np.dot(self.Pp, C_t)), np.linalg.inv(np.add(np.dot(C, np.dot(self.Pp, C_t))), Ro), np.dot(C, np.dot(self.Pp, A_t)))),
            #             Qo)
            #         Pp = A * Pp * A' - (A * Pp * C') / (C * Pp * C' + Ro) * C * Pp * A' + Qo;
            
            #         Pp_1 = A * Pp * A'
            #         Pp_2 = (A * Pp * C') / (C * Pp * C' + Ro) * C * Pp * A'
            #         Pp_3 = Qo
            
            #         Pp = Pp_1 - Pp_2 + Pp_3
            
            #         Pp = [A * Pp * A'] 
            #              -
            #              [(A * Pp * C') 
            #              * (C * Pp * C' + Ro)^-1 
            #              * C * Pp * A'
            #              + Qo];
            
            # filtered output prediction
            self.yhat = np.dot(C, self.xhat)
        except np.linalg.LinAlgError:
            print("Singluar Matrix Detected")
            
        if self.debug:
            print(f"A: {A}"
            f"\n B: {B}"
            f"\n C: {C}"
            f"\n D: {D}"
            # f"\n self.x0: {x0}"
            # f"\n u: {u}"
            # f"\n y: {y}"
            # f"\n P0: {P0}"
            # f"\n y: {y}"
            # f"\n self.xhat: {self.xhat}"
            # f"\n u[0]: {u}"
            # f"\n np.dot(B, u[k]): {np.dot(B, u)}"
            # f"\n K: {K}"
            # f"\n y[:, k]: {y[:, 0]}"
            # f"\n np.dot(K, y[:, k]): {np.dot(K, y)}"
            # f"\n np.dot(K, y[:, k]).reshape(num_states, 1)): {np.dot(K, y).reshape(num_states, 1)}"
            # f"\n np.add(np.dot(B, u[k]), np.dot(K, y[:, k]).reshape(num_states, 1))): {np.add(np.dot(B, u), np.dot(K, y).reshape(num_states, 1))}"
            # f"\n self.xhat_mat: {self.xhat_mat}"
            # f"\n self.xhat: {self.xhat}"
            # f"\n A: {A}"
            # f"\n self.xhat: {self.xhat}"
            # f"\n np.dot(A, self.xhat) 2: {np.dot(A, self.xhat)}"
            # f"\n B: {B}"
            # f"\n u[k]: {u}"
            # f"\n np.dot(B, u[k]): {np.dot(B, u)}"
            # f"\n K_mat: {self.K_mat}"
            # f"\n step1 step: K: {K}"
            # f"\n step2 step: y: {self.yhat}"
            # f"\n step3 step: y_mat: {self.yhat_mat}"
            f"\n step1 step: {K}"
            f"\n step2 step: {y}"
            f"\n step3 step: {np.dot(K, y)}"
            )
        return self.xhat, self.Pp


def main():
    my_kalman = LinearKalmanFilter()
    my_kalman.build_system(0.5)
    # x0 = [0, 0, 0, 0]
    x0 = np.array([[0.34],
                  [-0.12],
                  [0.42],
                  [0]])
    u = 0.19
    u = np.array([[0.19], [0.1]])

    y = np.array([[1.878],
                  [0.0267]])

    # y = np.array([[1.878],
    #               [0.34],
    #               [0.121],
    #               [0.0267]])
    P0 = np.diag([1.0, 1.0, 1.0, 1.0])
    Qo = np.diag([0.1, 0.1, 0.1, 0.1])
    # Ro = [0.1]
    Ro = np.diag([1.0E-3, 5.0E-2])
    my_kalman.debug = True
    # print(
    #       f"\n" \
    #       f"\n A: {my_kalman.sysd.A}"\
    #       f"\n B: {my_kalman.sysd.B}"\
    #       f"\n C: {my_kalman.sysd.C}"\
    #       f"\n D: {my_kalman.sysd.D}"\
    #       f"\n" \
    #       f"\n y: {y}" \
    #       f"\n" \
    #       f"\n xhat_type: {type(x0)}" \
    #       f"\n u: {type(u)}" \
    #       f"\n y: {type(y)}" \
    #       f"\n" \
    #       f"\n" \
    #       f"\n" \
    #       )
    my_kalman.lkf(my_kalman.sysd, x0, u, y, P0, Qo, Ro)
    # print(f"\n A: {my_kalman.sysd.A}"\
    #       f"\n" \
    #       f"\n y: {y}" \
    #       f"\n" \
    #       f"\n xhat: {my_kalman.xhat}" \
    #       f"\n xhat_type: {type(x0)}" \
    #       f"\n u: {type(u)}" \
    #       f"\n y: {type(y)}" \
    #       f"\n" \
    #       f"\n" \
    #       f"\n" \
    #       )
    my_kalman.lkf_step(my_kalman.sysd, x0, u, y, P0, Qo, Ro)
    # print(f"\n "\
    #       f"\n LKF Step: " \
    #       f"\n A: {my_kalman.sysd.A}"\
    #       f"\n y: {y}" \
    #       f"\n" \
    #       f"\n xhat: {my_kalman.xhat}" \
    #       f"\n xhat_type: {type(x0)}" \
    #       f"\n u: {type(u)}" \
    #       f"\n y: {type(y)}" \
    #       f"\n" \
    #       f"\n" \
    #       f"\n" \
    #       )


if __name__ == '__main__':
    main()

