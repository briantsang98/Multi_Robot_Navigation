#!/usr/bin/env python3.6
# import service
from multi_robot_collision_avoidance.srv import EvalWaypoint,EvalWaypointResponse
from multi_robot_collision_avoidance.MapHack import MapHack
from geometry_msgs.msg import Pose

import rospy

import os
import numpy as np

import torch
import gpytorch
import function.utils
import matplotlib.pyplot as plt

# gpytorch GP model
class ExactGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

def pose2arr(pose):
    return np.array([pose.position.x,pose.position.y])
# FIX
def sample_valid_point(mapImage, segment, chick_pix, bold_pix):
    hallway_idx = [6,18,21,26,29]

    xmin, ymin = np.maximum(chick_pix-[0,60], 0)
    xmax, ymax = np.minimum(chick_pix+60, mapImage.shape)
    samples = np.random.uniform([ymin, xmin], [ymax, xmax], (1000,2)).astype(int).T
    y,x = samples
    idx = np.argwhere(np.isin(segment[y,x], hallway_idx)).reshape(-1)
    samples = samples[:,idx]
    y,x = samples
    idx = np.argwhere(mapImage[y,x] != 0).reshape(-1)
    samples = samples[:,idx]
    return samples

def handle_eval_waypoint(req):
    chick_pos = pose2arr(req.chicken_pose)
    bold_pos  = pose2arr(req.bold_pose)
    # Sample vaild waypoints
    mapImage = plt.imread('/home/brian/Desktop/3ne.pgm')
    segment  = plt.imread('/home/brian/Desktop/1.6m_color_coded_map.pgm')

    st = rospy.Time.now()
    chick_pix = mh.pos2pixConverter(chick_pos)
    waypoints = sample_valid_point(mapImage, segment, chick_pix, bold_pos).T
    waypoints = waypoints.reshape(-1,2)
    print("sampling took {:.3f}s".format((rospy.Time.now() - st).to_sec()))
    st = rospy.Time.now()
    # Use position information from request; chicken_pos, bold_pos
    # to measure the features (4 distances) from given map
    test_x = []
    for wp in waypoints:
        dists = mh.extractFeatures(chick_pos, bold_pos, wp)
        if(dists[2]>0.5 and dists[3]>0.5 and dists[2]+dists[3] > 1.4):
            test_x.append(dists)

    print("feature extraction took {:.3f}s".format((rospy.Time.now() - st).to_sec()))
    st = rospy.Time.now()
    test_x = torch.from_numpy(np.array(test_x)).type(torch.float)

    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        expected_reward = likelihood(model(test_x))

    print("GP model took {:.3f}s".format((rospy.Time.now() - st).to_sec()))
    st = rospy.Time.now()
    resp = EvalWaypointResponse()

    best_reward = np.amax(expected_reward)
    best_idx = np.argwhere(expected_reward == best_reward).reshape(-1)
    print(best_idx.shape)
#    sample_reward = expected_reward.sample().numpy()
#    best_reward = np.amax(sample_reward)
#    best_idx = np.argwhere(sample_reward == best_reward).reshape(-1)

    x,y = mh.pix2posConverter(waypoints[best_idx][0][::-1])
    #print("Best expected rewards: {:.3f}".format(best_reward))
    print(best_reward)
    print("{}->[{:.3f},{:.3f}]".format(chick_pos, x,y))
    wpos = Pose()
    wpos.position.x = x
    wpos.position.y = y

    resp.waypoint = wpos
    #resp.E_reward = best_reward
    return resp

def eval_waypoint_server():
    rospy.init_node('eval_waypoint_server')

    # set maphack parameters manually
    map_dir = "/home/brian/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne/3ne.pgm"
    meta_dir = "/home/brian/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne/3ne.yaml"
    # Initialize MapHack function once with global varible to reduce computation
    global mh
    global likelihood
    global model

    mh = MapHack(map_dir, meta_dir)

    data_dir = os.path.join("/home/brian/catkin_ws/src/multi_robot_collision_avoidance/script","data")
    filenames = ["8m_train.txt"]
    feature_dim = 6
    fail_exp_id = [-997, -1001]
    # Read training data for the GP model
    features, reward = function.utils.readFromFiles(data_dir, filenames, feature_dim)
    train_x, train_y = function.utils.excludeFailure(features, reward, fail_exp_id)
    train_y          = function.utils.rewardClip(train_y, [-50,0])
    train_x = torch.from_numpy(train_x[:,2:]).type(torch.float)
    train_y = torch.from_numpy(train_y).type(torch.float)

    likelihood = gpytorch.likelihoods.GaussianLikelihood()
    model = ExactGPModel(train_x, train_y, likelihood)
    state_dict = torch.load("/home/brian/catkin_ws/src/multi_robot_collision_avoidance/script/model/ExactGPModel_0_prior_100_epoch.pth")
    model.load_state_dict(state_dict)

    # Activate evaluation mode and
    #Perform dummy evaluation to intialize torch network.
    model.eval()
    likelihood.eval()

    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        likelihood(model( torch.from_numpy(np.random.rand(1,4)).type(torch.float) ))

    s = rospy.Service('eval_waypoint', EvalWaypoint, handle_eval_waypoint)
    print("Ready to evaluate waypoint in map:\n{}.".format(map_dir))#map_dir))
    rospy.spin()

if __name__ == "__main__":
    eval_waypoint_server()
