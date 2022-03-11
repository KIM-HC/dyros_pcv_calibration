#!/usr/bin/env python3

from simulate_pcv import PCVSimulator, DEG2RAD, RAD2DEG
from calibrate_pcv import CalibratePCV
import numpy as np
import rospkg
import yaml

if __name__ == "__main__":
    no_error = False
    num_test = 10000
    sim_rot_speed = 0.5 #0.4
    sim_dt = 0.05 #0.01
    sim_lap = 1
    is_auto_cali = True
    cal_section = 1

    # TODO: do not increase file_number reuse 0 1 2 3
    # TODO: always use simulation_basic_0.yaml
    # TODO: 

    # basic_file: 'simulation/simulation_basic_0.yaml'
    # num_section: 3
    # sweep_angle: 110.0 # DEGREE
    # loop_index: 0

    e_pnt = 5           # mm
    e_bet = 8.0*DEG2RAD # radian
    e_off = 3           # mm
    e_rad = 3           # mm

    out_data = []


    for iter in range(num_test):
        if iter == 0:
            e_pnt = 0
            e_bet = 0
            e_off = 0
            e_rad = 0

        error_point = []
        for module in range(4):
            rn = e_pnt * np.random.random_sample(3) - e_pnt/2
            rn[2] = 0.0
            error_point.append(rn)
        error_beta = (e_bet * np.random.random_sample(4) - e_bet/2).tolist()
        error_offset = (e_off * np.random.random_sample(4) - e_off/2).tolist()
        error_radius = (e_rad * np.random.random_sample(4) - e_rad/2).tolist()

        param_set = {}
        param_set['steer_point']  = [np.array([215,125,0.0]), np.array([215,-125,0.0]),np.array([-215,-125,0.0]),np.array([-215,125,0.0])]
        param_set['beta']         = [  0,   0,   0,   0]
        param_set['wheel_offset'] = [-20, -20, -20, -20]
        param_set['wheel_radius'] = [ 55,  55,  55,  55]

        for module in range(4):
            param_set['steer_point'][module]  += error_point[module]
            param_set['beta'][module]         += error_beta[module]
            param_set['wheel_offset'][module] += error_offset[module]
            param_set['wheel_radius'][module] += error_radius[module]

        pkg_path = rospkg.RosPack().get_path('pcv_calibration')
        dumper = {}
        dumper['loop_index'] = {}
        dumper['loop_index'][0] = []
        dumper['is_mocap'] = True
        dumper['is_simulation'] = True
        dumper['parameter_file'] = '/setting/output/sim_param_0.yaml'

        for module in range(4):
            pcv_sim = PCVSimulator(stationary_set=module, param_set=param_set)
            pcv_sim.make_data()
            dumper['set_{0}'.format(module)], lp_idx = pcv_sim.export_data()
            dumper['loop_index'][0].append(lp_idx)

        ## for calibrating
        with open(pkg_path + '/setting/simulation/validate_calibration_basic_0.yaml', 'w') as f:
            yaml.dump(dumper, f)

        param_dumper = {}
        for module in range(4):
            ddd = {}
            ddd['angle_error_deg'] = param_set['beta'][module] * RAD2DEG
            ddd['angle_error_rad'] = param_set['beta'][module]
            ddd['steer_point'] = (param_set['steer_point'][module][0:2] / 1000.0).tolist()
            ddd['wheel_offset'] = param_set['wheel_offset'][module] / 1000.0
            ddd['wheel_radius'] = param_set['wheel_radius'][module] / 1000.0
            param_dumper[module] = ddd

        # TODO: perform calibration and read cal param
        pcv_cal = CalibratePCV(yaml_path='validate_calibration.yaml')
        param_cal = pcv_cal.export_data()
        

        # TODO: compare param and save MSE
        for module in range(4):
            pass





    # TODO: make mean and std data

