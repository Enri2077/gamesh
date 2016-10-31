/*
 * OutputManager.h
 *
 *	TODO
 */

#ifndef OUTPUTMANAGER_H_
#define OUTPUTMANAGER_H_

#include <types_config.hpp>
#include <types_reconstructor.hpp>
#include <string>
#include <array>
#include <map>
#include <set>
#include <ros/ros.h>

class OutputManager {
public:

	OutputManager(
			std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>>& boundaryCellsSpatialMap, ManifoldReconstructionConfig conf);
	virtual ~OutputManager();


	void publishROSColoredMesh(ros::Publisher& meshPublisher);
	void publishROSMesh(ros::Publisher& meshPublisher);
	void writeMeshToOff(const std::string filename);

	void setPoints(std::vector<PointReconstruction>* points){
		points_ = points;
	}

private:

	std::array<Delaunay3::Vertex_handle, 3> faceIndexToVertices(Delaunay3::Cell_handle c, int faceIndex);

	ManifoldReconstructionConfig& conf_;
	std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>>& boundaryCellsSpatialMap_;
	std::vector<PointReconstruction>* points_ = NULL;
};

#endif /* OUTPUTMANAGER_H_ */
