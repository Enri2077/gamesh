/*
 * OutputCreator.cpp
 *
 *	TODO
 */

#include <OutputManager.h>
#include <Chronometer.h>
#include <fstream>
#include <iostream>

#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/Mesh.h>

OutputManager::OutputManager(
		std::map<index3,
				std::set<Delaunay3::Cell_handle,
						sortTetByIntersectionAndDefaultLess>>& boundaryCellsSpatialMap,
		ManifoldReconstructionConfig conf) :
		boundaryCellsSpatialMap_(boundaryCellsSpatialMap), conf_(conf) {

}

OutputManager::~OutputManager() {
}

void OutputManager::publishROSMesh(ros::Publisher& meshPublisher) {

//	shape_msgs::Mesh m0;
//	geometry_msgs::Point p0, p1, p2, p3, p4, p5, p6, p7;
//
//	shape_msgs::MeshTriangle triangle0, triangle1, triangle2;
//
//	p0.x = 0; p0.y = 0; p0.z = 0; m0.vertices.push_back(p0);
//	p1.x = 0; p1.y = 0; p1.z = 1; m0.vertices.push_back(p1);
//	p2.x = 0; p2.y = 1; p2.z = 0; m0.vertices.push_back(p2);
//	p3.x = 0; p3.y = 1; p3.z = 1; m0.vertices.push_back(p3);
//	p4.x = 1; p4.y = 0; p4.z = 0; m0.vertices.push_back(p4);
//	p5.x = 1; p5.y = 0; p5.z = 1; m0.vertices.push_back(p5);
//	p6.x = 1; p6.y = 1; p6.z = 0; m0.vertices.push_back(p6);
//	p7.x = 1; p7.y = 1; p7.z = 1; m0.vertices.push_back(p7);
//
//	triangle0.vertex_indices[0] = 2;
//	triangle0.vertex_indices[1] = 1;
//	triangle0.vertex_indices[2] = 0;
//	m0.triangles.push_back(triangle0);
//
//	triangle1.vertex_indices[0] = 1;
//	triangle1.vertex_indices[1] = 2;
//	triangle1.vertex_indices[2] = 3;
//	m0.triangles.push_back(triangle1);
//
//	triangle2.vertex_indices[0] = 1;
//	triangle2.vertex_indices[1] = 3;
//	triangle2.vertex_indices[2] = 5;
//	m0.triangles.push_back(triangle2);
//
//	meshPublisher.publish(m0);
//
//	return;

	shape_msgs::Mesh m;

	std::map<Delaunay3::Vertex_handle, int> vertexHandleToIndex;
//	std::vector<Delaunay3::Vertex_handle> vertexHandles;

	int facetToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
			{ 0, 2, 3 },	// facetIndex : 1
			{ 3, 1, 0 },	// facetIndex : 2
			{ 0, 1, 2 } 	// facetIndex : 3
	};

	// Populate the list of points, the list of vertex handles, and
	// the associative maps from vertex handle to vertex index.
	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container

//		std::cout << "i_lbc.second.size() \t\t" << i_lbc.second.size()
//				<< std::endl;

		for (auto c : i_lbc.second) { // For each boundary cell in the container

//			std::cout << "1\tFor each boundary cell in the container" << std::endl;

			for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell
//				std::cout << "2\tFor each face in the cell" << std::endl;

				// If the face is a boundary face (between the boundary cell and a non manifold cell)
				if (!c->neighbor(faceIndex)->info().iskeptManifold()) {

//					std::cout << "3\tIf the face is a boundary face (between the boundary cell and a non manifold cell)" << std::endl;

					std::array<Delaunay3::Vertex_handle, 3> triangleVertices =
							faceIndexToVertices(c, faceIndex);

					// Add the face's vertices to the vertices list (if they aren't already in it)
					for (auto v : triangleVertices) {

//						std::cout << "4\tAdd the face's vertices to the vertices list (if they aren't already in it)" << std::endl;

						if (!vertexHandleToIndex.count(v)) {


//							std::cout << "5\tAdd the face's vertices to the vertices list (if they aren't already in it)" << std::endl;

							geometry_msgs::Point p;
							p.x = v->point().x();
							p.y = v->point().y();
							p.z = v->point().z();

							m.vertices.push_back(p);
//							vertexHandles.push_back(v);
							vertexHandleToIndex[v] = m.vertices.size() - 1;
						}
					}

//					std::cout << "6\tAdd the face's triangle to the triangles list" << std::endl;

					// Add the face's triangle to the triangles list
					shape_msgs::MeshTriangle t;
					t.vertex_indices[0] =
							vertexHandleToIndex[triangleVertices[0]];
					t.vertex_indices[1] =
							vertexHandleToIndex[triangleVertices[1]];
					t.vertex_indices[2] =
							vertexHandleToIndex[triangleVertices[2]];

					m.triangles.push_back(t);

				}
			}
		}
	}

	meshPublisher.publish(m);

}

void OutputManager::writeMeshToOff(const std::string filename) {
	Chronometer chrono1, chrono2;
	chrono1.start();

	std::ofstream outfile;
	std::vector<PointD3> points;
	std::set<index3> triangles;
	std::map<Delaunay3::Vertex_handle, int> vertexHandleToIndex;
	std::vector<Delaunay3::Vertex_handle> vertexHandles;

	int facetToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
			{ 0, 2, 3 },	// facetIndex : 1
			{ 3, 1, 0 },	// facetIndex : 2
			{ 0, 1, 2 } 	// facetIndex : 3
	};

	outfile.open(filename.c_str());
	if (!outfile.is_open()) {
		std::cerr << "Unable to open file: " << filename << std::endl;
		return;
	}

	std::cout << "boundaryCellsSpatialMap_.size()\t\t "
			<< boundaryCellsSpatialMap_.size() << std::endl;

	// Populate the list of points, the list of vertex handles, and
	// the associative maps from vertex handle to vertex index).
	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container

		for (auto c : i_lbc.second) { // For each boundary cell in the container

			for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell

				// If the face is a boundary face (between the boundary cell and a non manifold cell)
				if (!c->neighbor(faceIndex)->info().iskeptManifold()) {

					std::array<Delaunay3::Vertex_handle, 3> triangleVertices =
							faceIndexToVertices(c, faceIndex);

//					bool containsSteinerPoint = false;
//					for(auto v : triangleVertices) if(v->info().getPointId() == -1) containsSteinerPoint = true;
//					if(containsSteinerPoint) continue;

					// Add the face's vertices to the vertices list (if they aren't already in it)
					for (auto v : triangleVertices) {
						if (!vertexHandleToIndex.count(v)) {
							points.push_back(v->point());
							vertexHandles.push_back(v);
							vertexHandleToIndex[v] = points.size() - 1;
						}
					}

					// Add the face's triangle to the triangles list
					triangles.insert(
							index3(vertexHandleToIndex[triangleVertices[0]],
									vertexHandleToIndex[triangleVertices[1]],
									vertexHandleToIndex[triangleVertices[2]]));

				}
			}
		}
	}

	chrono1.stop();
	chrono2.start();

	outfile << "OFF" << std::endl << points.size() << " " << triangles.size()
			<< " 0" << std::endl;

	for (auto p : points)
		outfile << static_cast<float>(p.x()) << " " << static_cast<float>(p.y())
				<< " " << static_cast<float>(p.z()) << " " << std::endl;

	for (auto t : triangles)
		outfile << "3 " << t.i << " " << t.j << " " << t.k << std::endl;

	outfile.close();

	chrono2.stop();
	std::cout << "writeMeshToOff collect:\t\t" << chrono1.getSeconds() << " s"
			<< std::endl;
	std::cout << "writeMeshToOff write  :\t\t" << chrono2.getSeconds() << " s"
			<< std::endl;

}

//
//std::array<int, 3> OutputManager::facetToTriangle(int facetIndex) {
//	int vertices[4];
//
//	if (facetIndex == 0) {
//		// Vertex handle order: 3 2 1
//		vertices[0] = 3;
//		vertices[1] = 2;
//		vertices[2] = 1;
//	} else if (facetIndex == 1) {
//		// Vertex handle order: 0 2 3
//		vertices[0] = 0;
//		vertices[1] = 2;
//		vertices[2] = 3;
//	} else if (facetIndex == 2) {
//		// Vertex handle order: 3 1 0
//		vertices[0] = 3;
//		vertices[1] = 1;
//		vertices[2] = 0;
//	} else { // f->second == 3
//		// Vertex handle order: 0 1 2
//		vertices[0] = 0;
//		vertices[1] = 1;
//		vertices[2] = 2;
//	}
//
//	return vertices;
//}

std::array<Delaunay3::Vertex_handle, 3> OutputManager::faceIndexToVertices(
		Delaunay3::Cell_handle c, int faceIndex) {
	std::array<Delaunay3::Vertex_handle, 3> vertices;

	int faceToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
			{ 0, 2, 3 },	// faceIndex : 1
			{ 3, 1, 0 },	// faceIndex : 2
			{ 0, 1, 2 } 	// faceIndex : 3
	};

	for (int i = 0; i < 3; i++)
		vertices[i] = c->vertex(faceToTriangleMatrix[faceIndex][i]);

	return vertices;
}

