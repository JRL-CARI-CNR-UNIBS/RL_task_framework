#!/usr/bin/env python3

import pymongo
import pprint
import rospy

from std_msgs.msg import String
from pymongo import MongoClient
from kr_50_r2500_tests.srv import SetParamByMongo
from kr_50_r2500_tests.srv import SetMongoByParam

def callback(data):
    return data.data

def set_mongo_by_param(srv):
    print('set_mongo_by_param')

    if rospy.has_param("/RL_params"):
        RL_params = rospy.get_param("/RL_params")
        if RL_params:
            skill_params_mongo = []
            action_params_mongo = []
            for action in RL_params.keys():
                action_param = {}
                for skill in RL_params[action].keys():
                    data = RL_params[action][skill]
                    if type(data) is dict:
#                        print(type(data))
                        data["action_name"] = action
                        data["skill_name"]  = skill
                        skill_params_mongo.append(data)
                    else:
                        action_param[skill] = RL_params[action][skill]
                action_param["action_name"] = action
                action_params_mongo.append(action_param)
    else:
        print("No RL_params")

    skills_collection.delete_many({})
    skills_collection.insert_many(skill_params_mongo)
    actions_collection.delete_many({})
    actions_collection.insert_many(action_params_mongo)

    if rospy.has_param("/policy_params"):
        policy_params = rospy.get_param("/policy_params")
        if policy_params:
            policy_params_mongo = []
            for action in policy_params.keys():
                data = policy_params[action]
                data["action_name"] = action
                policy_params_mongo.append(data)

        policy_collection.delete_many({})
        policy_collection.insert_many(policy_params_mongo)
    else:
        print("No policy_params")

    if rospy.has_param("/arbitrator_params"):
        arbitrator_params = rospy.get_param("/arbitrator_params")
        if arbitrator_params:
            arbitrator_params_mongo = []
            for action in arbitrator_params.keys():
                    for skill in arbitrator_params[action].keys():
                            data = arbitrator_params[action][skill]
                            data["action_name"] = action
                            data["skill_name"] = skill
                            arbitrator_params_mongo.append(data)
        arbitrator_collection.delete_many({})
        arbitrator_collection.insert_many(arbitrator_params_mongo)
    else:
        print("No arbitrator_params")

    if rospy.has_param("/learning_params"):
        learning_params = rospy.get_param("/learning_params")
        if learning_params:
            learning_params_mongo = []
            for action in learning_params.keys():
                    for skill in learning_params[action].keys():
                            data = learning_params[action][skill]
                            data["action_name"] = action
                            data["skill_name"]  = skill
                            learning_params_mongo.append(data)
        learning_collection.delete_many({})
        learning_collection.insert_many(learning_params_mongo)
    else:
        print("No learning_params")

    print('set_mongo_by_param finisced')
    return 'true'

def set_param_by_mongo(srv):

    print('set_param_by_mongo')

    action_params    = list(actions_collection.find())
    skill_params     = list(skills_collection.find())
    policy_params     = list(policy_collection.find())
    arbitrator_params = list(arbitrator_collection.find())
    learning_params   = list(learning_collection.find())

    for action_param in action_params:
        str = "/"
        ns_str = "RL_params"
        action_name = action_param["action_name"]
        del action_param["_id"]
        del action_param["action_name"]
        for key in action_param.keys():
            param_str = str+ns_str+str+action_name+str+key
            print('Set ' + param_str)
            rospy.set_param(param_str, action_param[key])
    print('action_parmas setted')

    for skill_param in skill_params:
        str = "/"
        ns_str = "RL_params"
        action_name = skill_param["action_name"]
        skill_name  = skill_param["skill_name"]
        del skill_param["_id"]
        del skill_param["action_name"]
        del skill_param["skill_name"]
        for key in skill_param.keys():
            param_str = str+ns_str+str+action_name+str+skill_name+str+key
            print('Set ' + param_str)
            rospy.set_param(param_str, skill_param[key])
    print('skill_params setted')

    for arbitrator_param in arbitrator_params:
        str = "/"
        arbitrator_str = "arbitrator_params"
        action_name = arbitrator_param["action_name"]
        skill_name  = arbitrator_param["skill_name"]
        del arbitrator_param["_id"]
        del arbitrator_param["action_name"]
        del arbitrator_param["skill_name"]
        for key in arbitrator_param.keys():
            param_str = str+arbitrator_str+str+action_name+str+skill_name+str+key
            rospy.set_param(param_str, arbitrator_param[key])
    print('learning_params setted')

    for learning_param in learning_params:
        str = "/"
        learning_str = "learning_params"
        action_name = learning_param["action_name"]
        skill_name  = learning_param["skill_name"]
        del learning_param["_id"]
        del learning_param["action_name"]
        del learning_param["skill_name"]
        for key in learning_param.keys():
            param_str = str+learning_str+str+action_name+str+skill_name+str+key
            rospy.set_param(param_str, learning_param[key])
    print('learning_params setted')

    for policy_param in policy_params:
        str = "/"
        policy_str = "policy_params"
        action_name = policy_param["action_name"]
        del policy_param["_id"]
        del policy_param["action_name"]
        for key in policy_param.keys():
            param_str = str+policy_str+str+action_name+str+key
            rospy.set_param(param_str, policy_param[key])
    print('policy_params setted')

    return 'true'


def clear_param(srv):
    actions_collection.delete_many({})
    skills_collection.delete_many({})
    policy_collection.delete_many({})
    arbitrator_collection.delete_many({})
    learning_collection.delete_many({})
    print('Params cleared!')
    return 'true'

def server_launch():
    s = rospy.Service('/set_mongo_by_param', SetMongoByParam, set_mongo_by_param)
    s = rospy.Service('/set_param_by_mongo', SetParamByMongo, set_param_by_mongo)
    s = rospy.Service('clear_param_on_mongo', SetMongoByParam, clear_param)
    print("Ready to set.")
    rospy.spin()


client = MongoClient('localhost', 27017)

db_param = rospy.get_param("/collection_name")
db = client[db_param]

actions_collection       = db.action_params
skills_collection       = db.skill_params
policy_collection     = db.policy_param
arbitrator_collection = db.arbitrator_param
learning_collection   = db.learning_param

if __name__ == '__main__':
    rospy.init_node('prova', anonymous=True)
    server_launch()
