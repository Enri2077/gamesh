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
#include <Chronometer.h>
#include <ReconstructFromSLAMData.h>
#include <OutputManager.h>
#include <types_config.hpp>
#include <types_reconstructor.hpp>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <utility>
#include <omp.h>

#include <ros/ros.h>

#include <gamesh_bridge/GameshRays.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/Mesh.h>
#include <gamesh_bridge/GameshMesh.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

int numPointsAdded_ = 0;
long unsigned int iterationCounter_ = 0;
long unsigned int nextPointId_ = 0;
std::queue<CameraType*> newCameras_;
CameraPointsCollection cameraPoints_;
ManifoldReconstructionConfig config_;

ros::Publisher globalPointCloudPublisher, globalBridgePointCloudPublisher;

void raysCallback(const gamesh_bridge::GameshRays::ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr usedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::fromROSMsg(msg->pointcloud, pointCloud);

	// Check that if enableIdentifiedPoints is true, then the number of identifiers is the same of the number of points
	//TODO use GameshIdentifiedRays
//	if(confManif_.enableIdentifiedPoints && msg->point_ids.size() != pointCloud.points.size()) ROS_ERROR("If enable_identified_points is true, then the number of identifiers in gamesh_rays.point_ids should be the same of the number of points in gamesh_rays.pointcloud.points");

	long unsigned int cameraId = msg->camera_id;

	CameraType* camera;

	if (cameraPoints_.hasCamera(cameraId)) {
		std::cout << "raysCallback:\tcamera updated" << std::endl;
		camera = cameraPoints_.getCamera(cameraId);
	} else {
		camera = new CameraType();
		camera->idCam = cameraId;
		cameraPoints_.addCamera(camera);
	}

	camera->center = glm::vec3(msg->camera_pose.position.x, msg->camera_pose.position.y, msg->camera_pose.position.z);

	for (int i = 0; i < pointCloud.points.size(); i++) {
		// If the points in the input pointcloud are more than maxPointsPerCamera, subsample on a grid
		if (pointCloud.points.size() > config_.maxPointsPerCamera && (i % (pointCloud.points.size() / config_.maxPointsPerCamera))) continue;

		// Slide the grid to avoid taking always the same points from the pointcloud when the camera is not moving
		int gridSlide = iterationCounter_ * (pointCloud.points.size() / config_.maxPointsPerCamera / 3);

		long unsigned int pclPointId = (i + gridSlide) % pointCloud.points.size();

		auto pclPoint = pointCloud.points[pclPointId];

		long unsigned int pointId;
		if (config_.enableIdentifiedPoints) {
//			pointId = msg->point_ids[pclPointId].data; //TODO use GameshIdentifiedRays
			pointId = nextPointId_++; // TODO remove
		} else {
			pointId = nextPointId_++;
		}
		numPointsAdded_++;

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

		float minColor = 0.1, maxColor = 0.9;

		if (config_.generateColoredMesh) {
			point->r = minColor+(maxColor-minColor)*(pclPoint.r / 255.0);
			point->g = minColor+(maxColor-minColor)*(pclPoint.g / 255.0);
			point->b = minColor+(maxColor-minColor)*(pclPoint.b / 255.0);
			point->a = minColor+(maxColor-minColor)*(pclPoint.a / 255.0);
		} else {
			point->r = 0.5;
			point->g = 0.5;
			point->b = 0.5;
			point->a = 1.0;
		}

		cameraPoints_.addVisibility(camera, point);

		if (config_.publishUsedPointcloud) usedPointCloud->points.push_back(pclPoint);
	}
	iterationCounter_++;

	if (config_.publishUsedPointcloud) {
		pcl::PCLPointCloud2::Ptr pc2m(new pcl::PCLPointCloud2());
		pcl::toPCLPointCloud2(*usedPointCloud, *pc2m);
		pc2m->header.frame_id = "world";
		globalPointCloudPublisher.publish(pc2m);
	}

	if (config_.publishReceivedPointcloud) {
		msg->pointcloud.header.frame_id = "world";
		globalBridgePointCloudPublisher.publish(msg->pointcloud);
	}

	newCameras_.push(camera);
}

bool getParams(ros::NodeHandle& n) {

	if (!n.getParam("gamesh/enable_identified_points", config_.enableIdentifiedPoints) || !n.getParam(
			"gamesh/enable_inverse_conic", config_.enableInverseConic) || !n.getParam(
			"gamesh/enable_points_position_update", config_.enablePointsPositionUpdate) || !n.getParam(
			"gamesh/enable_ray_mistrust", config_.enableRayMistrust) || !n.getParam(
			"gamesh/enable_unused_vertex_removing", config_.enableUnusedVertexRemoving) || !n.getParam(
			"gamesh/enable_mesh_saving", config_.enableMeshSaving) || !n.getParam("gamesh/enable_mesh_publishing",
			config_.enableMeshPublishing) || !n.getParam("gamesh/generate_colored_mesh", config_.generateColoredMesh)

	|| !n.getParam("gamesh/free_vote_threshold", config_.freeVoteThreshold) || !n.getParam(
			"gamesh/non_conic_free_vote_threshold", config_.nonConicFreeVoteThreshold) || !n.getParam(
			"gamesh/ray_removal_threshold", config_.rayRemovalThreshold) || !n.getParam(
			"gamesh/unused_vertex_removal_threshold", config_.unusedVertexRemovalThreshold) || !n.getParam(
			"gamesh/primary_points_visibility_threshold", config_.primaryPointsVisibilityThreshold)

	|| !n.getParam("gamesh/max_points_per_camera", config_.maxPointsPerCamera) || !n.getParam(
			"gamesh/max_distance_camera_points", config_.maxDistanceCameraPoints) || !n.getParam(
			"gamesh/steiner_grid_step_length", config_.steinerGridStepLength) || !n.getParam("gamesh/w_1", config_.w_1) || !n.getParam(
			"gamesh/w_2", config_.w_2) || !n.getParam("gamesh/w_3", config_.w_3) || !n.getParam("gamesh/w_m",
			config_.w_m)

	|| !n.getParam("gamesh/save_mesh_every", config_.saveMeshEvery)
	|| !n.getParam("gamesh/check_integrity_when_finished", config_.checkIntegrityWhenFinished)
	|| !n.getParam("gamesh/time_stats_output", config_.timeStatsOutput) || !n.getParam("gamesh/debug_output",
			config_.debugOutput) || !n.getParam("gamesh/publish_received_pointcloud", config_.publishReceivedPointcloud) || !n.getParam(
			"gamesh/publish_used_pointcloud", config_.publishUsedPointcloud)

	|| !n.getParam("gamesh/input_topic", config_.inputTopic) || !n.getParam("gamesh/output_topic", config_.outputTopic)

	|| !n.getParam("gamesh/received_pointcloud_topic", config_.receivedPointcloudTopic) || !n.getParam(
			"gamesh/used_pointcloud_topic", config_.usedPointcloudTopic)

	|| !n.getParam("gamesh/output_folder", config_.outputFolder) || !n.getParam("gamesh/time_stats_folder",
			config_.timeStatsFolder) || !n.getParam("gamesh/count_stats_folder", config_.countStatsFolder)) {
		std::cout << "Required parameters weren't specified" << std::endl;
		return false;
	}

	return true;
}

int main(int argc, char **argv) {

//	int a, b = 0;
//#pragma omp parallel private(a) shared(b)
//	while (a < 50) {
//
//		int x = 1;
//
//		for (long long int i = 0; i < 100000000ll; i++)
//			if (x) x = 0;
//			else x = 1;
//
//#pragma omp atomic
//		b += a;
//
//		std::cout << b << "\t thread " << omp_get_thread_num() << " / " << omp_get_num_threads() << " \t" << x << std::endl;
//		++a;
//	}

	Chronometer chronoMain;
	chronoMain.start();
	int maxIterations_ = 0;

	std::cout << std::fixed << std::setprecision(3);

	ros::init(argc, argv, "gamesh_node");
	ros::NodeHandle n;

	getParams(n);

	ros::Subscriber raysSubscriber = n.subscribe<gamesh_bridge::GameshRays>(config_.inputTopic, 1000, raysCallback);

	ros::Publisher meshPublisher;
	if (config_.generateColoredMesh) meshPublisher = n.advertise<gamesh_bridge::GameshMesh>(config_.outputTopic, 1);
	else meshPublisher = n.advertise<shape_msgs::Mesh>(config_.outputTopic, 1);

	globalPointCloudPublisher = n.advertise<pcl::PCLPointCloud2>(config_.usedPointcloudTopic, 1);
	globalBridgePointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(config_.receivedPointcloudTopic, 1);

	std::cout << "max_iterations set to: " << maxIterations_ << std::endl;
	std::cout << config_.toString() << std::endl;

	if (config_.enableMeshSaving && config_.saveMeshEvery <= 0) {
		std::cerr << std::endl << "constraint: enable_mesh_saving implies save_mesh_every > 0" << std::endl;
		return 1;
	}

	ReconstructFromSLAMData m(config_);
	m.getOutputManager()->setMeshPublisher(meshPublisher);

	m.setExpectedTotalIterationsNumber((maxIterations_) ? maxIterations_ + 1 : -1);

	ROS_INFO("Gamesh node initialised");

	// Main loop
	while (ros::ok()) {
		if (newCameras_.empty()) {
			ros::Duration(0.1).sleep();
			ros::spinOnce();
		} else {
			Chronometer chronoMainLoop;
			chronoMainLoop.start();

			std::cout << "Input cameras buffer size: \t" << newCameras_.size() << "\t num points added: \t" << numPointsAdded_ << std::endl;
			numPointsAdded_ = 0;

			while (!newCameras_.empty()) {
				m.addCamera(newCameras_.front());
				newCameras_.pop();
			}

			m.update();

			if (config_.enableMeshSaving && ros::ok() && m.iterationCount && !(m.iterationCount % config_.saveMeshEvery)) {
//				m.getOutputManager()->writeMeshToOff("/home/enrico/gamesh_output/current.off");
				m.saveMesh(config_.outputFolder, "current");
			}

			if (config_.enableMeshPublishing) {
//				if (config_.generateColoredMesh) m.getOutputManager()->publishROSColoredMesh(meshPublisher);
//				else m.getOutputManager()->publishROSMesh(meshPublisher);
				if (config_.generateColoredMesh) m.getOutputManager()->publishROSColoredMesh();
				else m.getOutputManager()->publishROSMesh();
			}


			chronoMainLoop.stop();
			std::cout << "main loop\t\t\t\t\t\t" << chronoMainLoop.getSeconds() << std::endl;
			std::cout << std::endl;
			m.insertStatValue(chronoMainLoop.getSeconds());

			ros::spinOnce();

			if (newCameras_.empty()) std::cout << "Input cameras buffer empty. Waiting for new cameras" << std::endl;

		}

		if (maxIterations_ && m.iterationCount >= maxIterations_) break;

	}

	if (config_.enableMeshSaving && m.iterationCount){
//		m.getOutputManager()->writeMeshToOff("/home/enrico/gamesh_output/final.off");
		m.saveMesh(config_.outputFolder, "final");
	}
	if(config_.checkIntegrityWhenFinished) m.integrityCheck();

	chronoMain.stop();
	std::cout << "main\t\t\t\t\t\t" << chronoMain.getSeconds() << std::endl;

	return 0;
}

