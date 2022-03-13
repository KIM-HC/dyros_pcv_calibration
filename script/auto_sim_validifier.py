#!/usr/bin/env python3

from simulate_pcv import PCVSimulator, DEG2RAD, RAD2DEG
from calibrate_pcv import CalibratePCV
import numpy as np
import rospkg
import yaml

if __name__ == "__main__":
    num_module = 4

    tot_num_test = 10000
    sim_rot_speed = 0.6 #0.4
    sim_dt = 0.06 #0.01
    sim_lap = 1.01 #3.03

    out_data = {}
    out_data['steer_point'] = []
    out_data['angle_error'] = []
    out_data['wheel_offset'] = []
    out_data['wheel_radius'] = []

    ## +- 5%(5DEG) error
    eae  = 10.0 * DEG2RAD   # rad
    espx = 21.5             # mm
    espy = 12.5             # mm
    ewo  = 2.0              # mm
    ewr  = 5.5              # mm

    for iter in range(tot_num_test):
        if iter % 50 == 0: print('iteration {0}'.format(iter))
        if iter == 0:
            e_pntx = 0
            e_pnty = 0
            e_bet = 0
            e_off = 0
            e_rad = 0
        else:
            e_pntx = espx
            e_pnty = espy
            e_bet = eae
            e_off = ewo
            e_rad = ewr

        dumper = {}
        dumper['loop_index'] = {}
        dumper['loop_index'][0] = []
        dumper['is_mocap'] = True
        dumper['is_simulation'] = True
        dumper['parameter_file'] = '/setting/output/sim_param_0.yaml'

        param_set = {}
        param_set['steer_point']  = [np.array([215.0,125.0,0.0]),  np.array([215.0,-125.0,0.0]),
                                     np.array([-215.0,-125.0,0.0]),np.array([-215.0,125.0,0.0])]
        param_set['beta']         = [  0.0,   0.0,   0.0,   0.0]
        param_set['wheel_offset'] = [-20.0, -20.0, -20.0, -20.0]
        param_set['wheel_radius'] = [ 55.0,  55.0,  55.0,  55.0]

        # error_point = []
        error_beta = (e_bet * np.random.random_sample(num_module) - e_bet/2).tolist()
        error_offset = (e_off * np.random.random_sample(num_module) - e_off/2).tolist()
        error_radius = (e_rad * np.random.random_sample(num_module) - e_rad/2).tolist()

        for module in range(num_module):
            rn = np.zeros(3)
            rn[0] = e_pntx * np.random.random_sample() - e_pntx/2
            rn[1] = e_pnty * np.random.random_sample() - e_pnty/2
            # error_point.append(rn)
            # param_set['steer_point'][module]  += error_point[module]
            param_set['steer_point'][module]  += rn
            param_set['beta'][module]         += error_beta[module]
            param_set['wheel_offset'][module] += error_offset[module]
            param_set['wheel_radius'][module] += error_radius[module]

        for module in range(num_module):
            pcv_sim = PCVSimulator(stationary_set=module, param_set=param_set, is_validifier=True,
                                   rot_speed=sim_rot_speed, tot_lap=sim_lap, del_time=sim_dt)
            pcv_sim.make_data()
            dumper['set_{0}'.format(module)], lp_idx = pcv_sim.export_data()
            dumper['loop_index'][0].append(lp_idx)

        pkg_path = rospkg.RosPack().get_path('pcv_calibration')
        with open(pkg_path + '/setting/simulation/validate_calibration_basic_0.yaml', 'w') as f:
            yaml.dump(dumper, f)

        pcv_cal = CalibratePCV(yaml_path='validate_calibration.yaml', is_save_data=False)
        param_cal = pcv_cal.export_data()

        for module in range(num_module):
            asp = np.linalg.norm(param_set['steer_point'][module][0:2] - param_cal[module]['steer_point'])
            aae = abs(param_set['beta'][module] - param_cal[module]['angle_error_rad'])
            awo = abs(param_set['wheel_offset'][module] - param_cal[module]['wheel_offset'])
            awr = abs(param_set['wheel_radius'][module] - param_cal[module]['wheel_radius'])
            out_data['steer_point'].append(asp)
            out_data['angle_error'].append(aae)
            out_data['wheel_offset'].append(awo)
            out_data['wheel_radius'].append(awr)

    out_data['steer_point'] = np.array(out_data['steer_point'])
    out_data['angle_error'] = np.array(out_data['angle_error'])
    out_data['wheel_offset'] = np.array(out_data['wheel_offset'])
    out_data['wheel_radius'] = np.array(out_data['wheel_radius'])

    mae_sp = np.average(out_data['steer_point'])
    mae_ae = np.average(out_data['angle_error'])
    mae_wo = np.average(out_data['wheel_offset'])
    mae_wr = np.average(out_data['wheel_radius'])

    var_sp = np.std(out_data['steer_point']) ** 2
    var_ae = np.std(out_data['angle_error']) ** 2
    var_wo = np.std(out_data['wheel_offset']) ** 2
    var_wr = np.std(out_data['wheel_radius']) ** 2

    param_dumper = {}
    param_dumper['total steer_point absolute error'] = out_data['steer_point']
    param_dumper['total angle_error absolute error'] = out_data['angle_error']
    param_dumper['total wheel_offset absolute error'] = out_data['wheel_offset']
    param_dumper['total wheel_radius absolute error'] = out_data['wheel_radius']

    param_dumper['_MAE steer_point']  =  mae_sp
    param_dumper['_MAE angle_error']  =  mae_ae
    param_dumper['_MAE wheel_offset'] = mae_wo
    param_dumper['_MAE wheel_radius'] = mae_wr

    param_dumper['_VAR steer_point']  =  var_sp
    param_dumper['_VAR angle_error']  =  var_ae
    param_dumper['_VAR wheel_offset'] = var_wo
    param_dumper['_VAR wheel_radius'] = var_wr

    with open(pkg_path + '/setting/output/sim_validation_result.yaml', 'w') as f:
        yaml.dump(param_dumper, f)

    # print('steer_point:\n{0}'.format(out_data['steer_point']))
    # print('angle_error:\n{0}'.format(out_data['angle_error']))
    # print('wheel_offset:\n{0}'.format(out_data['wheel_offset']))
    # print('wheel_radius:\n{0}'.format(out_data['wheel_radius']))

    print('MAE steer_point:  {0}'.format(mae_sp))
    print('MAE angle_error:  {0}'.format(mae_ae))
    print('MAE wheel_offset: {0}'.format(mae_wo))
    print('MAE wheel_radius: {0}'.format(mae_wr))

    print('VAR steer_point:  {0}'.format(var_sp))
    print('VAR angle_error:  {0}'.format(var_ae))
    print('VAR wheel_offset: {0}'.format(var_wo))
    print('VAR wheel_radius: {0}'.format(var_wr))

