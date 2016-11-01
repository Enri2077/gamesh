#ifndef MANIFOLDMANAGER_H_
#define MANIFOLDMANAGER_H_

//#include <Mesh.h>
//#include <tuple>
//#include <fstream>
#include <iostream>
#include <types_reconstructor.hpp>
#include <types_config.hpp>
#include <OutputCreator.h>
#include <OutputManager.h>
//#include <Logger.h>
#include <Chronometer.h>
/**
 * This class provides the basic tools to manage the actual manifold creation,
 * such as the region growing procedure, the manifoldness tests and the update of the
 * tetrahedra-based boundary
 *
 * */
class ManifoldManager {
public:
	ManifoldManager(Delaunay3& dt, ManifoldReconstructionConfig& conf);
	virtual ~ManifoldManager();

	size_t getBoundarySize() {
		long long int bSize = 0;
		for (auto i_lbc : boundaryCellsSpatialMap_) {
			bSize += i_lbc.second.size();
		}

		return bSize;
	}

	void shrinkManifold3(const std::set<index3>& enclosingVolumeMapIndices, const float& maxPointToPointDistance,
			const long currentEnclosingVersion);

	void shrinkSeveralAtOnce3(const std::set<index3>& enclosingVolumeMapIndices, const float& maxPointToPointDistance,
			const long currentEnclosingVersion);

	void regionGrowingBatch3(Delaunay3::Cell_handle& startingCell, const std::set<index3>& enclosingVolumeMapIndices);

	void regionGrowing3(const std::set<index3>& enclosingVolumeMapIndices);

	void growSeveralAtOnce3(const std::set<index3>& enclosingVolumeMapIndices);

	const std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> getBoundaryCells() const {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> boundaryCells;

		for (auto i_lbc : boundaryCellsSpatialMap_) {
			boundaryCells.insert(i_lbc.second.begin(), i_lbc.second.end());
		}

		return boundaryCells;
	}

//	const std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>>& getBoundaryCellsSpatialMap() const {
//		return boundaryCellsSpatialMap_;
//	}

	bool checkBoundaryIntegrity();

	OutputManager* getOutputManager() {
		return out_;
	}

	Chronometer chronoInsertInBoundary_, chronoRemoveFromBoundary_, chronoIsRegularOverall_, chronoIsRegularOverall2_, chronoSingleTestOverall_;

private:

	bool isInEnclosingVolume(Delaunay3::Cell_handle& c, const std::set<index3>& enclosingVolumeMapIndices,
			const long& currentEnclosingVersion, Chronometer& chronoEnclosingCache, Chronometer& chronoEnclosingCheck);

	void regionGrowingProcedure3(const std::set<index3>& enclosingVolumeMapIndices);

	/******************************************************/
	/**************Manifold check functions****************/
	/******************************************************/
	bool singleTetTest2(Delaunay3::Cell_handle& i);
	bool addSeveralAndCheckManifoldness2(Delaunay3::Vertex_handle curV);
	bool subSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& cellToTest1, int idxNeigh);
	bool isRegular(Delaunay3::Vertex_handle& v);
	bool isRegular2(Delaunay3::Vertex_handle& v);
	bool isRegularProfiled(Delaunay3::Vertex_handle& v);

	/******************************************************/
	/************Boundary update functions*****************/
	/******************************************************/

	void addTetAndUpdateBoundary2(Delaunay3::Cell_handle& cell);
	void subTetAndUpdateBoundary2(Delaunay3::Cell_handle& currentTet,
			std::vector<Delaunay3::Cell_handle>& newBoundaryTets);

	bool insertInBoundary(Delaunay3::Cell_handle& cellToTest);
	bool removeFromBoundary(Delaunay3::Cell_handle& cellToBeRemoved);

	bool isBoundaryCell(Delaunay3::Cell_handle& c);
	bool isBoundaryCell(Delaunay3::Cell_handle& c, std::vector<int>& neighNotManifold);
	bool isFreespace(Delaunay3::Cell_handle& cell);

	Delaunay3& dt_;
	std::map<index3, std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>> boundaryCellsSpatialMap_;

	ManifoldReconstructionConfig& conf_;
	OutputCreator* outputM_;
	OutputManager* out_ = NULL;

//	std::ofstream fileOut_;
//	utilities::Logger logger_;
	Chronometer chronoIsRegular_;
	long functionProfileCounter_isRegular_ = 0;
	int counter_ = 0;
};

#endif /* MANIFOLDMANAGER_H_ */
