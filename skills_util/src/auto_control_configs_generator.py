#!/usr/bin/env python3

import rospy
import sys
import itertools
from std_srvs.srv import Empty


def control_configs_generation(srv):
    print('-------------------------------------------------------------------Service start--------------------------------------------------------')

    configuration_manager_name = sys.argv[1]

    if (not rospy.has_param('auto_control_configurations')):
        print('Not auto_control_configurations param')
        return []

    param = rospy.get_param('auto_control_configurations')

    new_configurations = []

    config_sets = []

#    Creation of single configuration for single robot
    if 'robots' in param:
        if 'configurations' in param:
            for robot_name in param['robots'].keys():
                robot_configs = []
                for config_name in param['robots'][robot_name]:
                    if config_name in param['configurations']:
                        configuration = {}
                        configuration['name'] = robot_name + '_' + config_name
                        robot_configs.append(configuration['name'])
                        config_param = param['configurations'][config_name]
                        if 'components' in config_param:
                            configuration['components'] = []
                            for component in config_param['components']:
                                comp = component.copy()
                                comp['hardware_interface'] = robot_name + '_' + comp['hardware_interface']
                                configuration['components'].append(comp)
                        else:
                            print('Not components param in configuration ' + config_name + ' config')
                            continue
                        if 'depends' in config_param:
                            configuration['depends'] = []
                            for depend in config_param['depends']:
                                configuration['depends'].append(robot_name + '_' + depend)
                        for other_param in config_param.keys():
                            if (other_param == 'components' or other_param == 'depends'):
                                continue
                            configuration[other_param] = config_param[other_param]
                        new_configurations.append(configuration)
                    else:
                        print('Not ' + config_name + ' config in configurations')
                        continue
                config_sets.append(robot_configs)
        else:
            print('Not configurations param in auto_control_configurations')
            return []
    else:
        print('Not robots param in auto_control_configurations')
        return []

#    Creation of config combination
    configs_combinations = list(itertools.product(*config_sets))
    for combination in configs_combinations:
        configuration = {}
        configuration['name'] = ''
        for config_name in combination:
            configuration['name'] = configuration['name'] + config_name + '_'
        configuration['name'] = configuration['name'][:-1]
        configuration['depends'] = list(combination)
        configuration['components'] = []
        new_configurations.append(configuration)

    rospy.set_param('/' + configuration_manager_name + '/control_configurations', new_configurations)
    print('/' + configuration_manager_name + '/control_configurations set')

    return []


def server_launch():
    rospy.Service('/control_configurations_generation', Empty, control_configs_generation)
    print("Ready to set.")
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('auto_control_configs_generator', anonymous=True)
    print('-------------------------------------------------------------------auto_control_configs_generator--------------------------------------------------------')
    server_launch()
