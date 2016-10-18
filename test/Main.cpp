//  Copyright 2014 Andrea Romanoni
//
//  This file is part of manifoldReconstructor.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

#include <CameraPointsCollection.h>
#include <Logger.h>
#include <ReconstructFromSLAMData.h>
#include <types_config.hpp>
#include <types_reconstructor.hpp>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <utility>

#include "ros/ros.h"

#include <gamesh_bridge/GameshRays.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//#define USE_SFM
//#define PRODUCE_STATS
//#define SAVE_POINTS_TO_OFF_AND_EXIT

#define GAMESH_RAYS_TOPIC "gamesh_rays"

int numPointsPerCamera_ = 1000;

long unsigned int nextPointId_ = 0;
std::queue<CameraType*> newCameras_;
CameraPointsCollection cameraPoints_;

ros::Publisher globalPointCloudPublisher, globalBridgePointCloudPublisher;

void raysCallback(const gamesh_bridge::GameshRays::ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
	
	pcl::fromROSMsg(msg->pointcloud, pointCloud);
	
	long unsigned int cameraId = msg->camera_id;
	
	CameraType* camera = new CameraType();
	
	if (cameraPoints_.hasCamera(cameraId)) {
		std::cout << "raysCallback:\tcamera not updated" << std::endl;
		return; //TODO manage camera update
	}
	
	camera->idCam = cameraId;
	cameraPoints_.addCamera(camera);
	camera->center = glm::vec3(msg->camera_pose.position.x, msg->camera_pose.position.y, msg->camera_pose.position.z);
	
	/*** TEST PC2 PUBLISHER ***/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr usedPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	
	
	int numPointsAdded = 0;
//	for (auto pclPoint : pointCloud.points) {
	for(int i=0; i<pointCloud.points.size(); i++){
		if( i % (pointCloud.points.size()/numPointsPerCamera_) ) continue;
				
		auto pclPoint = pointCloud.points[i];
		
		long unsigned int pointId = nextPointId_++; //TODO manage identified points
		numPointsAdded++;
		
		PointType* point;
		
		if (cameraPoints_.hasPoint(pointId)) {
			point = cameraPoints_.getPoint(pointId);
			//std::cout << "UPDATE idPoint: "<<point->idPoint << "\tidReconstruction: "<<point->idReconstruction << "\tgetNunmberObservation: "<<point->getNunmberObservation() << std::endl;
		} else {
			point = new PointType();
			point->idPoint = pointId;
			cameraPoints_.addPoint(point);
		}
		
		point->position = glm::vec3(pclPoint.x, pclPoint.y, pclPoint.z);
		cameraPoints_.addVisibility(camera, point);
		
		/*** TEST PC2 PUBLISHER ***/
		usedPointCloud->points.push_back(pclPoint);
	}
	
	/*** TEST PC2 PUBLISHER ***/
	pcl::PCLPointCloud2::Ptr pc2m (new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*usedPointCloud, *pc2m);
	pc2m->header.frame_id = "world";
	globalPointCloudPublisher.publish(pc2m);
	
//	sensor_msgs::PointCloud2::Ptr gbpc2m (new sensor_msgs::PointCloud2());
	msg->pointcloud.header.frame_id = "world";
	globalBridgePointCloudPublisher.publish(msg->pointcloud);
	
	newCameras_.push(camera);
	std::cout << "newCameras length: \t" << newCameras_.size() << "\t numPointsAdded: \t" << numPointsAdded << std::endl;
}

int main(int argc, char **argv) {
	std::cout << "pwd: \t" << argv[0] << std::endl;
	
	utilities::Logger log;
	ManifoldReconstructionConfig confManif;
	int maxIterations_ = 0;
	
	ros::init(argc, argv, "gamesh_node");
	ros::NodeHandle n;
	ros::Subscriber raysSubscriber = n.subscribe<gamesh_bridge::GameshRays> (GAMESH_RAYS_TOPIC, 1000, raysCallback);
	
	/*** TEST PC2 PUBLISHER ***/
	globalPointCloudPublisher = n.advertise<pcl::PCLPointCloud2>("gamesh_points", 1);
	globalBridgePointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("gamesh_bridge_points", 1);
	
	
	if(!n.getParam("gamesh/inverse_conic_enabled", confManif.inverseConicEnabled)
	|| !n.getParam("gamesh/prob_or_vote_threshold", confManif.probOrVoteThreshold)
	|| !n.getParam("gamesh/ray_removal_threshold", confManif.rayRemovalThreshold)
	|| !n.getParam("gamesh/max_distance_cam_feature", confManif.maxDistanceCamFeature)
	|| !n.getParam("gamesh/enable_suboptimal_policy", confManif.enableSuboptimalPolicy)
	|| !n.getParam("gamesh/suboptimal_method", confManif.suboptimalMethod)
	|| !n.getParam("gamesh/w_1", confManif.w_1)
	|| !n.getParam("gamesh/w_2", confManif.w_2)
	|| !n.getParam("gamesh/w_3", confManif.w_3)
	|| !n.getParam("gamesh/w_m", confManif.w_m)
	|| !n.getParam("gamesh/steiner_grid_side_length", confManif.steinerGridSideLength)
	|| !n.getParam("gamesh/steiner_grid_step_length", confManif.steinerGridStepLength)
	|| !n.getParam("gamesh/manifold_update_every", confManif.manifold_update_every)
	|| !n.getParam("gamesh/initial_manifold_update_skip", confManif.initial_manifold_update_skip)
	|| !n.getParam("gamesh/save_manifold_every", confManif.save_manifold_every)
	|| !n.getParam("gamesh/primary_points_visibility_threshold", confManif.primary_points_visibility_threshold)
	|| !n.getParam("gamesh/num_points_per_camera", confManif.numPointsPerCamera)
	|| !n.getParam("gamesh/fake_points_multiplier", confManif.fake_points_multiplier)
	|| !n.getParam("gamesh/all_sort_of_output", confManif.all_sort_of_output)	
	|| !n.getParam("gamesh/time_stats_output", confManif.time_stats_output)
	|| !n.getParam("gamesh/update_points_position", confManif.update_points_position)
	|| !n.getParam("gamesh/output_folder", confManif.outputFolder)
	){
		std::cout << "Required parameters weren't specified" << std::endl;
		return 1;
	}
	
	numPointsPerCamera_ = confManif.numPointsPerCamera;
	std::cout << "max_iterations set to: " << maxIterations_ << std::endl;
	std::cout << confManif.toString() << std::endl;
	
	if (confManif.manifold_update_every <= 0 || confManif.save_manifold_every <= 0) {
		std::cerr << std::endl << "constraint: confManif.manifold_update_every > 0, confManif.save_manifold_every > 0" << std::endl;
		return 1;
	}
	
	ReconstructFromSLAMData m(confManif);
	
	m.setExpectedTotalIterationsNumber((maxIterations_) ? maxIterations_ + 1 : -1);
	
	ROS_INFO("Gamesh node initialised");
	
	// Main loop
	while(ros::ok()) {
		if(newCameras_.empty()){
			std::cout << "no new cameras to add" << std::endl;
			ros::Duration(0.1).sleep();
		}else{
			log.startEvent();
			
			while(!newCameras_.empty()){
				m.addCamera(newCameras_.front());
				newCameras_.pop();
			}
			
			m.updateManifold();
			//if (m.iterationCount && !(m.iterationCount % confManif.save_manifold_every)) m.saveManifold(confManif.outputFolder, "current");
			if (m.iterationCount && !(m.iterationCount % confManif.save_manifold_every)) m.saveManifold(confManif.outputFolder, std::to_string(m.iterationCount));
			
			log.endEventAndPrint("main loop\t\t\t\t\t\t", true);
			std::cout << std::endl;
			m.insertStatValue(log.getLastDelta());
		}
		
		if (maxIterations_ && m.iterationCount >= maxIterations_) break;
		
		ros::spinOnce();
	}
	
	// Do a last manifold update in case op.numCameras() isn't a multiple of confManif.manifold_update_every
	if (m.iterationCount > confManif.initial_manifold_update_skip) m.updateManifold();
	
	m.saveManifold(confManif.outputFolder, "final");
	
	log.endEventAndPrint("main\t\t\t\t\t\t", true);
	
	return 0;
}

