/*
 * ManifoldManager.cpp
 *
 *  Created on: 26/giu/2015
 *      Author: andrea
 */

#include <ManifoldManager.h>
#include <OutputCreator.h>
#include <OutputManager.h>
#include <utilities.hpp>
#include <sstream>
#include <iostream>
#include <unordered_set>
#include <unordered_map>

using std::cout;
using std::cerr;
using std::endl;

ManifoldManager::ManifoldManager(Delaunay3& dt, ManifoldReconstructionConfig& conf) :
		dt_(dt), conf_(conf) {

	outputM_ = new OutputCreator(dt_);
	out_ = new OutputManager(boundaryCellsSpatialMap_, conf);

}

ManifoldManager::~ManifoldManager() {
	boundaryCellsSpatialMap_.clear();
}

bool ManifoldManager::isInEnclosingVolume(Delaunay3::Cell_handle& c, const std::set<index3>& enclosingVolumeMapIndices,
		const long& currentEnclosingVersion, Chronometer& chronoEnclosingCache, Chronometer& chronoEnclosingCheck) {
	bool inEnclosingVolume = false;

	chronoEnclosingCache.start();
	if (c->info().getEnclosingVersion() == currentEnclosingVersion) {
//		countSuccessfulEnclosingVersionCache++;
		inEnclosingVolume = c->info().isInEnclosingVolume();
		chronoEnclosingCache.stop();
	} else {
		chronoEnclosingCache.stop();

		chronoEnclosingCheck.start();
		for (int vertexId = 0; vertexId < 4; ++vertexId) {
			auto& p = c->vertex(vertexId)->point();

			int i = std::floor(p.x() / conf_.steinerGridStepLength);
			int j = std::floor(p.y() / conf_.steinerGridStepLength);
			int k = std::floor(p.z() / conf_.steinerGridStepLength);

			if (enclosingVolumeMapIndices.count(index3(i, j, k))) {
				inEnclosingVolume = true;
//				countIsInMinecraftEnclosingVolume++;
				break;
			}
		}
		chronoEnclosingCheck.stop();

		c->info().setInEnclosingVolume(inEnclosingVolume, currentEnclosingVersion);
	}
	return inEnclosingVolume;
}

void ManifoldManager::shrinkManifold3(const std::set<index3>& enclosingVolumeMapIndices,
		const float& maxPointToPointDistance, long currentEnclosingVersion) {
	// Stats
//	int countBoundaryInitCells;
	int countQueueInitCells = 0, countInEnclosingVolume = 0, countShrinked = 0, countTotal = 0, countIterations = 1;
	Chronometer chronoQueueInit, chronoQueueInserting, chronoQueuePopping, chronoInserting, chronoEnclosing,
			chronoEnclosingCache, chronoEnclosingCheck, chronoTesting, chronoShrinking;

	int countIsInMinecraftEnclosingVolume = 0;

	float maxPointToPointDistanceSquared = std::pow(maxPointToPointDistance, 2);

	bool fixedPoint = false;
	std::vector<Delaunay3::Cell_handle> tetNotCarved;
	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> tetsQueue;

	// Populate the list with the boundary tetrahedra
	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		tetsQueue.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

	countQueueInitCells = tetsQueue.size();

	chronoQueueInit.stop();

	while (!fixedPoint) {

		fixedPoint = true;

		// Shrink process
		while (!tetsQueue.empty()) {
			countTotal++;

			chronoQueuePopping.start();

			Delaunay3::Cell_handle currentTet = *tetsQueue.begin();
			tetsQueue.erase(tetsQueue.begin());

			chronoQueuePopping.stop();

			chronoEnclosing.start();
			bool inEnclosingVolume = isInEnclosingVolume(currentTet, enclosingVolumeMapIndices, currentEnclosingVersion,
					chronoEnclosingCache, chronoEnclosingCheck);

			chronoEnclosing.stop();

			if (inEnclosingVolume) countInEnclosingVolume++;
			else continue;

			if (!currentTet->info().getManifoldFlag() || !currentTet->info().getBoundaryFlag()) continue;

			if (!currentTet->info().getManifoldFlag()) cerr << "ManifoldManager::shrinkManifold3:\t wrong shrink order (trying to shrink non manifold cell)\t iteration: " << countIterations << endl;
			if (!currentTet->info().getBoundaryFlag()) cerr << "ManifoldManager::shrinkManifold3:\t wrong shrink order (non boundary cell in queue)\t iteration: " << countIterations << endl;
			if (currentTet->info().getManifoldFlag() && !currentTet->info().getBoundaryFlag()) cerr << "ManifoldManager::shrinkManifold3:\t inconsistent boundary and manifold (m and !b)\t iteration: " << countIterations << endl;
			if (!currentTet->info().getManifoldFlag() && currentTet->info().getBoundaryFlag()) cerr << "ManifoldManager::shrinkManifold3:\t inconsistent boundary and manifold (!m and b)\t iteration: " << countIterations << endl;

			chronoTesting.start();
			chronoSingleTestOverall_.start();
			if (singleTetTest2(currentTet)) {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				if (!currentTet->info().getManifoldFlag()) cerr << "ManifoldManager::shrinkManifold3:\t wrong shrink order\t iteration: " << countIterations << endl;

				countShrinked++;

				chronoShrinking.start();
				std::vector<Delaunay3::Cell_handle> newBoundaryTets;
				subTetAndUpdateBoundary2(currentTet, newBoundaryTets);
				chronoShrinking.stop();

				for (auto neighbour : newBoundaryTets) {
					if (neighbour->info().getManifoldFlag()) {
						chronoQueueInserting.start();

						tetsQueue.insert(neighbour);

						chronoQueueInserting.stop();
						fixedPoint = false;

					} else cerr << "ManifoldManager::shrinkManifold3:\t subTetAndUpdateBoundary return non inManifoldSet tet\t iteration: " << countIterations << endl;
				}

			} else {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				chronoInserting.start();
//				std::vector<Delaunay3::Cell_handle>::iterator it;
//				it = std::lower_bound(tetNotCarved.begin(), tetNotCarved.end(), currentTet, sortTetByIntersection());
				tetNotCarved.push_back(currentTet);
				chronoInserting.stop();

			}
		}

		if (tetsQueue.empty() && !fixedPoint) {
			countIterations++;
//			tetsQueue.insert(tetsQueue.begin(), tetNotCarved.begin(), tetNotCarved.end());
			tetsQueue.insert(tetNotCarved.begin(), tetNotCarved.end());
			tetNotCarved.clear();
		}
	}

	if (conf_.timeStatsOutput) {
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s\t / \t" << countQueueInitCells << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t queue inserting:\t\t\t" << chronoQueueInserting.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t inserting:\t\t\t\t" << chronoInserting.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t enclosing cache:\t\t\t" << chronoEnclosingCache.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t enclosing check:\t\t\t" << chronoEnclosingCheck.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t test:\t\t\t\t\t" << chronoTesting.getSeconds() << " s\t / \t" << countInEnclosingVolume << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkManifold3:\t\t\t\t shrink:\t\t\t\t" << chronoShrinking.getSeconds() << " s\t / \t" << countShrinked << endl;
		cout << "ManifoldManager::shrinkManifold3:\t\t\t\t number_of_finite_cells:\t\t" << dt_.number_of_finite_cells() << endl;
	}
}

void ManifoldManager::shrinkSeveralAtOnce3(const std::set<index3>& enclosingVolumeMapIndices,
		const float &maxPointToPointDistance, long currentEnclosingVersion) {
	chronoIsRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoQueueInit, chronoQueuePopping, chronoEnclosing, chronoEnclosingCache, chronoEnclosingCheck,
			chronoAddAndCheckManifoldness;

	float maxPointToPointDistanceSquared = std::pow(maxPointToPointDistance, 2);

	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> tetsQueue;

	// Populate the list with the boundary tetrahedra
	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		tetsQueue.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

//	std::sort(tetsQueue.begin(), tetsQueue.end());
	chronoQueueInit.stop();

	while (!tetsQueue.empty()) {
		chronoQueuePopping.start();
		Delaunay3::Cell_handle currentTet = *tetsQueue.begin();
		tetsQueue.erase(tetsQueue.begin());
		chronoQueuePopping.stop();

		chronoEnclosing.start();

		bool inEnclosingVolume = isInEnclosingVolume(currentTet, enclosingVolumeMapIndices, currentEnclosingVersion,
				chronoEnclosingCache, chronoEnclosingCheck);

		chronoEnclosing.stop();

		if (inEnclosingVolume) {
			// TODO check only non boundary vertices like in growSeveralAtOnce
			for (int curIdx = 0; curIdx < 4; ++curIdx) {
				if (!currentTet->info().getBoundaryFlag()) break;

				chronoAddAndCheckManifoldness.start();

				bool success = subSeveralAndCheckManifoldness2(currentTet, curIdx);

				chronoAddAndCheckManifoldness.stop();

				if (success) break;
			}
		}
	}

	if (conf_.timeStatsOutput) {
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t enclosing:\t\t\t\t" << chronoEnclosing.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t addAndCheckManifoldness:\t\t" << chronoAddAndCheckManifoldness.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::shrinkSeveralAtOnce3:\t\t\t\t isRegular:\t\t\t\t" << chronoIsRegular_.getSeconds() << " s\t / \t" << functionProfileCounter_isRegular_ << endl;
	}
}

void ManifoldManager::regionGrowingBatch3(Delaunay3::Cell_handle& startingCell,
		const std::set<index3>& enclosingVolumeMapIndices) {
	addTetAndUpdateBoundary2(startingCell);
	regionGrowingProcedure3(enclosingVolumeMapIndices);
}

void ManifoldManager::regionGrowing3(const std::set<index3>& enclosingVolumeMapIndices) {
	regionGrowingProcedure3(enclosingVolumeMapIndices);
}

bool ManifoldManager::checkBoundaryIntegrity() {
	std::cout <<   "ManifoldManager::checkBoundaryIntegrity: checking..." << endl;

	bool result = true;
	int count = 0;

	for (auto i_localBoundaryCells : boundaryCellsSpatialMap_) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = i_localBoundaryCells.second;

		for (auto boundaryCell : localBoundaryCells) {
			count++;

			if (!dt_.is_cell(boundaryCell)) {
				cerr << "\t\t dead cell" << endl;
				result = false;
				continue;
			}

			bool manifoldTest = true;
			for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++){
				Delaunay3::Vertex_handle v = boundaryCell->vertex(vertexIndex);
				if (!isRegular(v)) manifoldTest = false;
			}

			if (!manifoldTest || !isFreespace(boundaryCell) || !isBoundaryCell(boundaryCell) || !boundaryCell->info().getBoundaryFlag()) {
				result = false;
				cerr << "\t\t tests failed on cell:" << endl;

				if (!manifoldTest) cerr << "\t\t\t\t has vertex v st !isRegular(v)" << endl;
				if (!isFreespace(boundaryCell)) cerr << "\t\t\t\t !isFreespace(boundaryCell)" << endl;
				if (!isBoundaryCell(boundaryCell)) cerr << "\t\t\t\t !isBoundaryCell(boundaryCell)" << endl;
				if (!boundaryCell->info().getManifoldFlag()) cerr << "\t\t\t\t !boundaryCell->info().getManifoldFlag()" << endl;
				if (!boundaryCell->info().getBoundaryFlag()) cerr << "\t\t\t\t !boundaryCell->info().getBoundaryFlag()" << endl;

				cerr << "\t\t\t info on cell:" << endl;

				cout << "\t\t\t\t points:";
				for (int i = 0; i < 4; i++)
					cout << "\t" << boundaryCell->vertex(i)->info().getPointId();
				cout << endl;
			}

		}
	}

	cout << "\t\t checked " << count << ((count-1)?" cells":" cell") << endl;

	if(!result) cout << "#########      Integrity check FAILED!!!      #########" << endl << endl;
	else  cout <<       "#              Integrity check OK                     #" << endl << endl;

	return result;

}

void ManifoldManager::regionGrowingProcedure3(const std::set<index3>& enclosingVolumeMapIndices) {
	// Stats
	int countQueueInitCells, countBoundaryInitCells;
	int countGrowned = 0, countTotal = 0, countIterations = 0, maxIterations = 5;
	Chronometer chronoQueueInit, chronoQueueInserting, chronoQueuePopping, chronoInserting, chronoTesting,
			chronoGrowing;

	bool fixedPoint = false;

	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> tetsQueue;
	std::vector<Delaunay3::Cell_handle> tetNotCarved;
	std::set<Delaunay3::Cell_handle> localBoundaryCellsUnion;

	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		localBoundaryCellsUnion.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

	if (!localBoundaryCellsUnion.size()) {
		if (conf_.timeStatsOutput) cout << "ManifoldManager::regionGrowingProcedure3:\t\t localBoundaryCellsUnion was empty, searching in all boundary" << endl;

		for (auto i_localBoundaryCells : boundaryCellsSpatialMap_) {
			localBoundaryCellsUnion.insert(i_localBoundaryCells.second.begin(), i_localBoundaryCells.second.end());
		}

		if (conf_.timeStatsOutput && !localBoundaryCellsUnion.size()) {
			cout << "ManifoldManager::regionGrowingProcedure3:\t\t localBoundaryCellsUnion still empty. Aborting grow procedure" << endl;
			return;
		}
	}

	for (auto boundaryCell : localBoundaryCellsUnion) {

		if (!dt_.is_cell(boundaryCell)) {
			std::cerr << "ManifoldManager::regionGrowingProcedure3: \t\t dead cell found in boundary!" << endl;
			continue;
		}

		if (!boundaryCell->info().getManifoldFlag()) {
			std::cout << "ManifoldManager::regionGrowingProcedure3: \t\tnon manifold cell in boundary" << endl; //TODO Sometimes this happen
			std::cout << "ManifoldManager::regionGrowingProcedure3: \t\t     points:";
			for (int i = 0; i < 4; i++)
				cout << "\t" << boundaryCell->vertex(i)->info().getPointId();
			cout << endl;

			if (boundaryCell->info().getBoundaryFlag()) {
				std::cout << "ManifoldManager::regionGrowingProcedure3: \t\t     (boundary flag is fine)" << endl;
				boundaryCell->info().setManifoldFlag(true);
				if (isBoundaryCell(boundaryCell)) {
					std::cout << "ManifoldManager::regionGrowingProcedure3: \t\t     manifold flag corrected" << endl;
				} else {
					std::cout << "ManifoldManager::regionGrowingProcedure3: \t\t     manifold flag NOT corrected. Removing from boundary" << endl;
					boundaryCell->info().setManifoldFlag(false);
					removeFromBoundary(boundaryCell);
					continue;
				}
			} else {
				std::cout << "ManifoldManager::regionGrowingProcedure3: \t\t     boundary flag is wrong" << endl;
				boundaryCell->info().setManifoldFlag(true);
				boundaryCell->info().setBoundaryFlag(true);
				if (isBoundaryCell(boundaryCell)) {
					std::cout << "ManifoldManager::regionGrowingProcedure3: \t\t     boundary and manifold flags corrected" << endl;
				} else {
					std::cout << "ManifoldManager::regionGrowingProcedure3: \t\t     boundary and manifold flags NOT corrected. Removing from boundary" << endl;
					boundaryCell->info().setManifoldFlag(false);
					boundaryCell->info().setBoundaryFlag(false);
					removeFromBoundary(boundaryCell);
					continue;
				}
//				continue;
			}
		}

		for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
			Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(neighbourIndex);

			if (!neighbour->info().getManifoldFlag() && isFreespace(neighbour)) tetsQueue.insert(neighbour);
		}
	}

	chronoQueueInit.stop();

	countBoundaryInitCells = localBoundaryCellsUnion.size();
	countQueueInitCells = tetsQueue.size();

	while (!fixedPoint && countIterations < maxIterations) {
		fixedPoint = true;

		while (!tetsQueue.empty()) {
			countTotal++;

			//Single-tet-at-once
			chronoQueuePopping.start();

			Delaunay3::Cell_handle currentTet = *tetsQueue.rbegin();
			tetsQueue.erase(*tetsQueue.rbegin());

			chronoQueuePopping.stop();

			if (currentTet->info().getManifoldFlag() || currentTet->info().getBoundaryFlag()) continue;

			if (currentTet->info().getBoundaryFlag()) cerr << "ManifoldManager::regionGrowingProcedure3:\t wrong grow order (boundary cell in queue)\t iteration: " << countIterations << endl;
			if (currentTet->info().getManifoldFlag() && !currentTet->info().getBoundaryFlag()) cerr << "ManifoldManager::regionGrowingProcedure3:\t inconsistent boundary and manifold (m and !b)\t iteration: " << countIterations << endl;
			if (!currentTet->info().getManifoldFlag() && currentTet->info().getBoundaryFlag()) cerr << "ManifoldManager::regionGrowingProcedure3:\t inconsistent boundary and manifold (!m and b)\t iteration: " << countIterations << endl;
			if (currentTet->info().getManifoldFlag()) cerr << "ManifoldManager::regionGrowingProcedure3:\t wrong grow order\t iteration: " << countIterations << endl;

			chronoTesting.start();
			chronoSingleTestOverall_.start();
			if (singleTetTest2(currentTet)) {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				countGrowned++;

				chronoGrowing.start();
				addTetAndUpdateBoundary2(currentTet);
				chronoGrowing.stop();

				// Add the adjacent tets to the queue
				for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
					Delaunay3::Cell_handle neighbour = currentTet->neighbor(neighbourIndex);

					if (!neighbour->info().getManifoldFlag() && isFreespace(neighbour)) {
						chronoQueueInserting.start();

						tetsQueue.insert(neighbour);
						fixedPoint = false;

						chronoQueueInserting.stop();

					}
				}

			} else {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				chronoInserting.start();

				tetNotCarved.push_back(currentTet);

				chronoInserting.stop();
			}
		}

		tetsQueue.insert(tetNotCarved.begin(), tetNotCarved.end());
		tetNotCarved.clear();

		countIterations++;
	}

	tetsQueue.clear();

	if (conf_.timeStatsOutput) {
		cout << "ManifoldManager::regionGrowingProcedure3:\t\t\t countIterations:\t\t\t\t\t" << countIterations << endl;
		cout << "ManifoldManager::regionGrowingProcedure3:\t\t\t countBoundaryInitCells:\t\t\t\t" << countBoundaryInitCells << endl;
		cout << "ManifoldManager::regionGrowingProcedure3:\t\t\t countQueueInitCells:\t\t\t\t\t" << countQueueInitCells << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t queue inserting:\t\t\t" << chronoQueueInserting.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t inserting:\t\t\t\t" << chronoInserting.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t test:\t\t\t\t\t" << chronoTesting.getSeconds() << " s\t / \t" << countTotal << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::regionGrowingProcedure3:\t\t\t grow:\t\t\t\t\t" << chronoGrowing.getSeconds() << " s\t / \t" << countGrowned << endl;
		cout << "ManifoldManager::regionGrowingProcedure3:\t\t\t number_of_finite_cells:\t\t" << dt_.number_of_finite_cells() << endl;
	}
}

void ManifoldManager::growSeveralAtOnce3(const std::set<index3>& enclosingVolumeMapIndices) {
	chronoIsRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoQueueInit, chronoRemoveAndCheckManifoldness;
	int countTotal = 0, countSuccess = 0;

	// boundaryVertices will contain all the vertices on the boundary between the manifold and non manifold cells
	std::set<Delaunay3::Vertex_handle> boundaryVertices;
	std::set<Delaunay3::Cell_handle> localBoundaryCellsUnion;

	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		localBoundaryCellsUnion.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

	if (conf_.timeStatsOutput && !localBoundaryCellsUnion.size()) {
		cout << "ManifoldManager::growSeveralAtOnce3:\t\t\t localBoundaryCellsUnion was empty. Aborting grow several at once procedure" << endl;
		return;
	}

	for (auto boundaryCell : localBoundaryCellsUnion) {

		if (!dt_.is_cell(boundaryCell)) {
			std::cerr << "ManifoldManager::growSeveralAtOnce3: \t\t dead cell found in boundary!" << endl;
			continue;
		}

		if (!boundaryCell->info().getManifoldFlag()) {
			std::cout << "ManifoldManager::growSeveralAtOnce3: \t\tnon manifold cell in boundary" << endl; //TODO Sometimes this happen
			if (boundaryCell->info().getBoundaryFlag()) {
				std::cout << "ManifoldManager::growSeveralAtOnce3: \t\t     (boundary flag is fine)" << endl;
				boundaryCell->info().setManifoldFlag(true);
				if (isBoundaryCell(boundaryCell)) {
					std::cout << "ManifoldManager::growSeveralAtOnce3: \t\t     manifold flag corrected" << endl;
				} else {
					std::cout << "ManifoldManager::growSeveralAtOnce3: \t\t     manifold flag NOT corrected. Removing from boundary" << endl;
					boundaryCell->info().setManifoldFlag(false);
					removeFromBoundary(boundaryCell);
					continue;
				}
			} else {
				std::cout << "ManifoldManager::growSeveralAtOnce3: \t\talso  boundary flag is wrong" << endl;
				boundaryCell->info().setManifoldFlag(true);
				boundaryCell->info().setBoundaryFlag(true);
				if (isBoundaryCell(boundaryCell)) {
					std::cout << "ManifoldManager::growSeveralAtOnce3: \t\t     boundary and manifold flags corrected" << endl;
				} else {
					std::cout << "ManifoldManager::growSeveralAtOnce3: \t\t     boundary and manifold flags NOT corrected. Removing from boundary" << endl;
					boundaryCell->info().setManifoldFlag(false);
					boundaryCell->info().setBoundaryFlag(false);
					removeFromBoundary(boundaryCell);
					continue;
				}
				continue;
			}
		}

		// boundaryCell is a cell in the boundary
		for (int faceIndex = 0; faceIndex < 4; ++faceIndex) {
			Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(faceIndex);

			if (!neighbour->info().getManifoldFlag() && isFreespace(neighbour)) {
				// Here neighbour is a non manifold cell and free space, so it could be grown.
				// faceIndex is the index (in boundaryCell) of the face between boundaryCell and neighbour.
				// Thus, faceIndex identifies the faces on the boundary

				for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
					// The vertices of boundaryCell with index that satisfy the condition vertexIndex != faceIndex
					// are the ones composing the boundary face
					if (vertexIndex != faceIndex) boundaryVertices.insert(boundaryCell->vertex(vertexIndex));
				}

			}
		}
	}

	chronoQueueInit.stop();

	chronoRemoveAndCheckManifoldness.start();

	for (auto v : boundaryVertices) {
		countTotal++;

		bool success = addSeveralAndCheckManifoldness2(v);

		if (success) countSuccess++;
	}

	chronoRemoveAndCheckManifoldness.stop();

	if (conf_.timeStatsOutput) {
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t remove and check manifoldness:\t\t" << chronoRemoveAndCheckManifoldness.getSeconds() << " s" << endl;
		cout << std::fixed << std::setprecision(3) << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t isRegular:\t\t\t\t" << chronoIsRegular_.getSeconds() << " s\t / \t" << functionProfileCounter_isRegular_ << endl;
		cout << "ManifoldManager::growSeveralAtOnce3:\t\t\t\t successfully growned:\t\t\t" << countSuccess << "\t / \t" << countTotal << endl;
	}
}

bool ManifoldManager::addSeveralAndCheckManifoldness2(Delaunay3::Vertex_handle curV) {
	std::vector<Delaunay3::Cell_handle> incidentCells, cellsModified;
	std::vector<Delaunay3::Vertex_handle> incidentV;
//	Delaunay3::Vertex_handle curV = currentTet->vertex(curIdx);
	dt_.incident_cells(curV, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(curV, std::back_inserter(incidentV));

	bool testFailed = false;

	for (auto ic : incidentCells) {
		if (isFreespace(ic)) {
			if (!ic->info().getManifoldFlag()) {
				cellsModified.push_back(ic);
				ic->info().setManifoldFlag(true);
			}
		}
	}

	for (auto iv : incidentV) {
		if (!isRegularProfiled(iv)) {
			testFailed = true;
			break;
		}
	}

	if (!testFailed && !isRegularProfiled(curV)) {
		testFailed = true;
	}

	if (testFailed) {

		for (auto ic : cellsModified)
			ic->info().setManifoldFlag(false);

		return false;

	} else {

		for (auto ic : cellsModified)
			ic->info().setManifoldFlag(false);

		for (auto ic : cellsModified)
			addTetAndUpdateBoundary2(ic);

	}

	return true;
}

bool ManifoldManager::subSeveralAndCheckManifoldness2(Delaunay3::Cell_handle& currentTet, int curIdx) {

	std::vector<Delaunay3::Cell_handle> incidentCells, cellsModified;
	std::vector<Delaunay3::Vertex_handle> incidentV;
	Delaunay3::Vertex_handle curV = currentTet->vertex(curIdx);
	dt_.incident_cells(curV, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(curV, std::back_inserter(incidentV));

	for (auto ic : incidentCells) {
		if (ic->info().getManifoldFlag()) {
			cellsModified.push_back(ic);
			ic->info().setManifoldFlag(false);
		}
	}

	bool testFailed = false;

	for (auto iv : incidentV) {
//		if (!isRegular(iv)) {
		if (!isRegularProfiled(iv)) {
			testFailed = true;
			break;
		}
	}

//	if (!testFailed && !isRegular(curV)) {
	if (!testFailed && !isRegularProfiled(curV)) {
		testFailed = true;
	}

	if (testFailed) {

		for (auto ic : cellsModified)
			ic->info().setManifoldFlag(true);

		return false;

	} else {

		for (auto ic : cellsModified)
			ic->info().setManifoldFlag(true);

		for (auto ic : cellsModified) {
			std::vector<Delaunay3::Cell_handle> newBoundaryTets;
			subTetAndUpdateBoundary2(ic, newBoundaryTets);
		}

	}

	return true;
}

void ManifoldManager::addTetAndUpdateBoundary2(Delaunay3::Cell_handle& c) {

	// Add c to the manifold set and to the boundary if needed
	c->info().setManifoldFlag(true);

	std::vector<int> nonManifoldNeighbour;
	if (isBoundaryCell(c, nonManifoldNeighbour)) {
		insertInBoundary(c);
	} else {
		if (c->info().getBoundaryFlag()) {
			removeFromBoundary(c);
			cerr << "ManifoldManager::addTetAndUpdateBoundary2: \t !isBoundaryCell and isBoundary() == true" << endl;
		}
	}

	// Check if the neighbour of the added cell still belongs to the boundary
	for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
		Delaunay3::Cell_handle neighbour = c->neighbor(neighbourIndex);

		// If needed, remove the neighbour of c from the boundary

		// Note that isBoundary function returns the state flag of the cell, while
		// isBoundaryCell() checks for the current real state of the cell inside the triangulation after
		// the new cell has been added
		if (neighbour->info().getBoundaryFlag()) {

			if (!isBoundaryCell(neighbour)) {
				removeFromBoundary(neighbour);
			}

		} else {
			if (isBoundaryCell(neighbour)) {
				cerr << "ManifoldManager::addTetAndUpdateBoundary2: \t isBoundaryCell == true and isBoundary() == false" << endl; // This should not happen, unless the flag is uncongruent maybe
				insertInBoundary(neighbour);
			}
		}
	}
}

/*
 * 	Remove the current tetrahedron from the manifold set and from the boundary set,
 * 	and add or remove its neighbours from the boundary set accordingly.
 */
void ManifoldManager::subTetAndUpdateBoundary2(Delaunay3::Cell_handle& c,
		std::vector<Delaunay3::Cell_handle>& newBoundaryTets) {

	std::vector<int> notManifoldNeigh;
	isBoundaryCell(c, notManifoldNeigh);

	bool r = removeFromBoundary(c);

	if (!r) cout << "ManifoldManager::subTetAndUpdateBoundary2: \t\t setting manifold flag to false but cell not removed from boundary" << endl;

	c->info().setManifoldFlag(false);

	// Check for each neighbour whether it should belong to the boundary set
	for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
		Delaunay3::Cell_handle neighbour = c->neighbor(neighbourIndex);

		// If the neighbour wasn't in the boundary set before, it is now on the only condition of being in the manifold set,
		// in fact the remaining condition is satisfied. (Namely, it has at least a neighbour outside the manifold set, that is currentTet).
		if (!neighbour->info().getBoundaryFlag()) {

			if (neighbour->info().getManifoldFlag()) {
				insertInBoundary(neighbour);
				newBoundaryTets.push_back(neighbour);
			}

		} else {
			if (!neighbour->info().getManifoldFlag()) {
				cerr << "cell such that iskeptManifold() == false && isBoundary() == true" << endl;
				removeFromBoundary(neighbour);
			}
		}
	}
}

// Returns true if c is a boundary cell. That is if c is in the manifold set and some of its neighbours aren't.
bool ManifoldManager::isBoundaryCell(Delaunay3::Cell_handle& c) {
	std::vector<int> toThrowAway;

	return isBoundaryCell(c, toThrowAway);
}

// Returns true if c is a boundary cell. That is if c is in the manifold set and some of its neighbours aren't.
// If c is a boundary cell, then neighboursNotManifold contains the indeces of the neighbours outside the boundary (that are not in the manifold set).
bool ManifoldManager::isBoundaryCell(Delaunay3::Cell_handle& c, std::vector<int>& neighboursNotManifold) {

	if (!c->info().getManifoldFlag()) {
		return false;
	} else {
		bool neighNotManifoldFound = false;

		for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
			if (!c->neighbor(neighbourIndex)->info().getManifoldFlag()) {
				neighboursNotManifold.push_back(neighbourIndex);

				neighNotManifoldFound = true;
			}
		}
		return neighNotManifoldFound;
	}
}

/*
 *	If cellToBeAdded->info().isBoundary() is false, then insert cellToBeAdded in boundaryCells_ (maintaining the order) and set cellToBeAdded->info().isBoundary() to true.
 */
bool ManifoldManager::insertInBoundary(Delaunay3::Cell_handle& cellToBeAdded) {
	chronoInsertInBoundary_.start();

	if (!cellToBeAdded->info().getManifoldFlag()) cerr << "ManifoldManager::insertInBoundary: \t\t Violated precondition: Inserting a cell in boundary but iskeptManifold()==false" << endl;

	std::pair<std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>::iterator, bool> i;
	std::set<index3> mapIndices;
	for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
		auto v = cellToBeAdded->vertex(vertexIndex);

		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(v->point().x() / conf_.steinerGridStepLength);
		int j = std::floor(v->point().y() / conf_.steinerGridStepLength);
		int k = std::floor(v->point().z() / conf_.steinerGridStepLength);
		mapIndices.insert(index3(i, j, k));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		i = localBoundaryCells.insert(cellToBeAdded);
	}

	if (cellToBeAdded->info().getBoundaryFlag() && !i.second) {
		cerr << "ManifoldManager::insertInBoundary: \t Uncongruent flag: isBoundary() == true but cell was NOT in boundary" << endl;
		return false;
	}

	cellToBeAdded->info().setBoundaryFlag(true);

	chronoInsertInBoundary_.stop();
	return true;

}

/*
 *	If cellToBeRemoved->info().isBoundary() is set to true, removes cellToBeRemoved from boundaryCells_ and set cellToBeRemoved->info().isBoundary() to false,
 *	otherwise does nothing.
 */
bool ManifoldManager::removeFromBoundary(Delaunay3::Cell_handle& cellToBeRemoved) {
	chronoRemoveFromBoundary_.start();

	int e = 0;

	std::set<index3> mapIndices;
	for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
		auto v = cellToBeRemoved->vertex(vertexIndex);

		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(v->point().x() / conf_.steinerGridStepLength);
		int j = std::floor(v->point().y() / conf_.steinerGridStepLength);
		int k = std::floor(v->point().z() / conf_.steinerGridStepLength);
		mapIndices.insert(index3(i, j, k));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		e += localBoundaryCells.erase(cellToBeRemoved);
	}

	for(auto i_lbc : boundaryCellsSpatialMap_){
		if(i_lbc.second.count(cellToBeRemoved)) cerr << "ManifoldManager::removeFromBoundary: \t DIDN'T REMOVE ALL CELLS" << endl;
	}

	if (!cellToBeRemoved->info().getBoundaryFlag() && e > 0) {
		cerr << "ManifoldManager::removeFromBoundary: \t Uncongruent flag: isBoundary() == false but cell was in boundary\t" << e << endl;
		return false;
	}

	cellToBeRemoved->info().setBoundaryFlag(false);

	chronoRemoveFromBoundary_.stop();
	return true;
}

//bool ManifoldManager::checkManifoldness(Delaunay3::Cell_handle &cellToTest1, int idxNeigh) {
//	Delaunay3::Vertex_handle v = cellToTest1->vertex(idxNeigh);
//	Delaunay3::Cell_handle curTetNeigh = cellToTest1->neighbor(idxNeigh);
//
//	if (!isRegular(v)) return false;
//
//	for (int curNei = 0; curNei < 4; ++curNei) {
//		Delaunay3::Vertex_handle vC = curTetNeigh->vertex(curNei);
//
//		if (!isRegular(vC)) return false;
//	}
//	return true;
//}

bool ManifoldManager::isRegularProfiled(Delaunay3::Vertex_handle& v) {
	functionProfileCounter_isRegular_++;
	counter_++;

//	chronoIsRegular_.start();
//	chronoIsRegularOverall_.start();

	bool r = isRegular(v);

//	chronoIsRegularOverall_.stop();
//	chronoIsRegular_.stop();
//
//	if (r != isRegular2(v)) {
//		cerr << endl << endl << "#####################################" << endl << endl << "\tr != isRegular2(v)" << endl << endl << "#####################################" << endl << endl;
//		throw new std::exception();
//	}

	return r;
}

// TODO Merge first and second phase
bool ManifoldManager::isRegular(Delaunay3::Vertex_handle& v) {
	/*	Given |δV| as the manifold surface;
	 * 	The test is true if the triangles in δV including v can be ordered as t_0_ , ···, t_k−1_
	 * 	such that t_i_ ∩ t_(i+1)mod k_ is an edge, and such an edge is included in exactly two triangles t_i_ and t_j_
	 * 	In other words, the graph of the v-opposite edges in the δV triangles must form one and only one cycle.
	 *
	 * 	Based on this, three cases are possible:
	 * 	(a) There is no boundary surface passing through the vertex v because all incident cells are manifold,
	 * 	or (b) because no incident cell is manifold;
	 * 	(c)	There is a boundary surface passing through v because some incident cells are manifold and some others are not.
	 *
	 * 	In the first two cases the test is successful, in the third case it's necessary to test the following conditions on the graph:
	 * 	· (c_1) if the graph contains non binary vertices, implying the test is unsuccessful;
	 * 	· (c_2) if the graph contains multiple cycles, implying the test is unsuccessful;
	 * 	If none of these conditions are met, then the test is successful.
	 *
	 *
	 * 	Definitions:
	 * 	Binary Vertex (as in graph theory):
	 * 		A vertex that is part of exactly two edges.
	 * 	Binary Graph:
	 * 		A graph whose vertices are all binary vertices.
	 * 	v-opposite boundary edge (as in triangulation):
	 * 		An edge e such that the triangle t is composed of the vertices v, v1, v2. e is composed of v1 and v2.
	 * 		t ∈ δV, meaning that t is a face between two cells c1, c2 and c1 is in the manifold volume, c2 is not.
	 * 	v-opposite boundary vertices (as in triangulation):
	 * 		The vertices v1, v2 in the previous definition.
	 *
	 *	Note: only undirected graphs are considered.
	 */

	struct Edge {
		Delaunay3::Vertex_handle first, second;
		Edge(Delaunay3::Vertex_handle first, Delaunay3::Vertex_handle second) :
				first(first), second(second) {
		}
		Edge() {
			first = NULL;
			second = NULL;
		}
		inline bool operator<(const Edge& r) {
			Delaunay3::Vertex_handle x = std::min(first, second);
			Delaunay3::Vertex_handle y = std::min(r.first, r.second);
			if (x < y) return true;
			else if (y > x) return false;
			else return std::max(first, second) < std::max(r.first, r.second);
		}
		inline bool operator!=(const Edge& r) {
			return (first != r.first && first != r.second) || (second != r.second && second != r.first);
		}
		bool operator!() {
			return first == NULL; // second should be irrelevant, as either they both get assigned or neither
		}
		Delaunay3::Vertex_handle otherVertex(Delaunay3::Vertex_handle v1) {
			if (v1 != first) return first;
			else return second;
		}
	};

	// Can contain two edges like a pair, the contained edges are unique like in a set, the contained edges can be accessed as a vector.
	struct EdgePair {
		std::vector<Edge> edges;
		EdgePair() {
		}
		// Insert the Edge e.
		// Return false only when trying to insert a third edge different from the ones already present.
		// Returning false implies that a vertex is contained in three or more different edges
		// and means that a vertex is non binary.
		bool insert(Edge e) {
			if (edges.size() == 0) {
				edges.push_back(e);
				return true;
			} else if (edges.size() == 1) {
				if (e != edges[0]) {
					edges.push_back(e);
				}
				return true;
			} else {
				if (e != edges[0] && e != edges[1]) {
					edges.push_back(e);
					return false;
				}
				return true;
			}
		}
		// Precondition: The edges e1 and e2 are both and the only edges contained in this pair.
		// Given the edge e1, returns the other one.
		Edge otherEdge(Edge e1) {
			if (edges.size() != 2) cerr << "EdgePair not quite a pair: " << edges.size() << " edge" << endl;
			if (e1 != edges[0]) return edges[0];
			else return edges[1];
		}
	};

	chronoIsRegular_.start();
	chronoIsRegularOverall_.start();

	std::vector<Delaunay3::Cell_handle> vIncidentCells;
	dt_.incident_cells(v, std::back_inserter(vIncidentCells));

	// Verify conditions a and b:
	bool allIncidentCellsAreManifold = true;
	bool noIncidentCellIsManifold = true;

	for (auto c : vIncidentCells) {
		if (allIncidentCellsAreManifold && !c->info().getManifoldFlag()) {
			allIncidentCellsAreManifold = false;
		}
		if (noIncidentCellIsManifold && c->info().getManifoldFlag()) {
			noIncidentCellIsManifold = false;
		}
		if (!allIncidentCellsAreManifold && !noIncidentCellIsManifold) break;
	}

	// If conditions a or b are met the test is successful, otherwise condition c is true.
	if (allIncidentCellsAreManifold || noIncidentCellIsManifold) {
		chronoIsRegularOverall_.stop();
		chronoIsRegular_.stop();
		return true;
	}

	/*	The previous code is equivalent to
	 *
	 *	for (auto c : vIncidentCells)
	 *		if (!c->info().getManifoldFlag()) {
	 *			allIncidentCellsAreManifold = false;
	 *			break;
	 *		}
	 *	if (allIncidentCellsAreManifold) return true;
	 *
	 *	for (auto c : vIncidentCells)
	 *		if (c->info().getManifoldFlag()) {
	 *			noIncidentCellIsManifold = false;
	 *			break;
	 *		}
	 *	if (noIncidentCellIsManifold) return true;
	 *
	 */

	// Here condition c is met.
	// To test conditions c_1 and c_2 it's necessary to populate the graph.
	std::unordered_set<Delaunay3::Vertex_handle> vOppositeBoundaryVertices;
	std::unordered_map<Delaunay3::Vertex_handle, EdgePair> vertexToEdgesMap;

	// firstEdge and firstVertex are any of the edges and vertices in the graph such that firstEdge contains firstVertex
	Edge firstEdge;
	Delaunay3::Vertex_handle firstVertex;
	bool foundNonBinaryVertices = false;

	// Find all v-opposite boundary edges and vertices
	for (auto incidentCell : vIncidentCells) {
		int vIndex = incidentCell->index(v);

		for (int i = 0; i < 4; i++)
			if (i != vIndex) {
				if (incidentCell->info().getManifoldFlag() != incidentCell->neighbor(i)->info().getManifoldFlag()) {
					// Here incidentCell is manifold and i is the index (in incidentCell) of a face
					// between incidentCell and another cell incident to v that is not manifold.
					// So the face identified by i is a boundary face containing v.

					int j = 0, m, n;
					// The edge given by (incidentCell, m, n) with m, n the two indices different from i and vIndex,
					// is on the boundary and opposite to the vertex v

					for (; j == i || j == vIndex; j++)
						;
					m = j; // m is equal to the first index different from i and vIndex
					for (j++; j == i || j == vIndex; j++)
						;
					n = j; // n is equal to the second (and last) index different from i and vIndex

					Edge e(incidentCell->vertex(m), incidentCell->vertex(n));

					// While inserting the edges in the map, also check condition c_1
					foundNonBinaryVertices |= !vertexToEdgesMap[incidentCell->vertex(m)].insert(e);
					foundNonBinaryVertices |= !vertexToEdgesMap[incidentCell->vertex(n)].insert(e);

					// If there are non binary vertices condition c_1 is met and the test is unsuccesful
					if (foundNonBinaryVertices) {
						chronoIsRegularOverall_.stop();
						chronoIsRegular_.stop();
						return false;
					}

					if (!firstEdge) {
						firstEdge = e;
						firstVertex = incidentCell->vertex(m);
					}

					vOppositeBoundaryVertices.insert(incidentCell->vertex(m));
					vOppositeBoundaryVertices.insert(incidentCell->vertex(n));

					// To debug: boundaryTriangles.push_back(dt_.triangle(incidentCell, i)); ... outputM_->writeTrianglesToOFF("output/isRegular/v ", std::vector<int> { counter_ }, boundaryTriangles);
				}
			}
	}

	// Since condition c_1 is false, it's now guaranteed that the graph is binary, so all vertices must be part of a cycle.
	// Since conditions a and b weren't met, there has to be at least one cycle.
	// The test is succesful if there is exactly one cycle.
	// Given that all vertices are part of a cycle, to test whether there are multiple cycles (condition c_2),
	// it suffices to walk on a cycle and see if it contains all the vertices (the remaining vertices would be part of further cycles).

	if (!vOppositeBoundaryVertices.size()) {
		// <=> vOppositeBoundaryVertices.size() == 0 implies the graph is empty
		cerr << "\t\t\t NOT (allIncidentCellsAreManifold || noIncidentCellIsManifold) but graph is empty" << endl << "Violated hypothesis: allIncidentCellsAreManifold || noIncidentCellIsManifold <=> empty graph" << endl;
		chronoIsRegularOverall_.stop();
		chronoIsRegular_.stop();
		return false;
	}
	if (allIncidentCellsAreManifold || noIncidentCellIsManifold) {
		cerr << "\t\t\t (allIncidentCellsAreManifold || noIncidentCellIsManifold) but graph is NOT empty" << endl << "Violated hypothesis: allIncidentCellsAreManifold || noIncidentCellIsManifold <=> empty graph" << endl;
		chronoIsRegularOverall_.stop();
		chronoIsRegular_.stop();
		return false;
	}

	chronoIsRegularOverall_.stop();
	chronoIsRegularOverall2_.start();

	// Walk on the first cycle to count the number of its vertices and compare it to the total vertices
	Edge currentEdge = firstEdge;
	Delaunay3::Vertex_handle currentVertex = firstVertex;
	int verticesInThisCycleCount = 1;

	while (firstVertex != (currentVertex = currentEdge.otherVertex(currentVertex))) {

		// If the edges connected to the current vertex aren't exactly two
		// then this is not a cycle and the graph is not binary
		// This condition should not be possible
		if (vertexToEdgesMap[currentVertex].edges.size() != 2) {
			cerr << "Violated hypothesis: Non binary graph" << endl;
			chronoIsRegularOverall2_.stop();
			chronoIsRegular_.stop();
			return false;
		}

		// Count the new vertex in the path
		verticesInThisCycleCount++;

		// Select the edge that is connected to the new vertex and is different from the previous one
		currentEdge = vertexToEdgesMap[currentVertex].otherEdge(currentEdge);
	}

	chronoIsRegularOverall2_.stop();
	chronoIsRegular_.stop();

	// Condition c_2 is met if there are more vertices than the ones in the walked cycle
	// if c_2 is false the test is successful
	return verticesInThisCycleCount == vOppositeBoundaryVertices.size();

}

bool ManifoldManager::isRegular2(Delaunay3::Vertex_handle& v) {

	std::vector<Delaunay3::Cell_handle> incidentCells;
	std::vector<Delaunay3::Vertex_handle> incidentV;
	dt_.incident_cells(v, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(v, std::back_inserter(incidentV));

	std::vector<Delaunay3::Vertex_handle> pathVert;
	std::vector<Delaunay3::Edge> pathEdge;
	std::vector<Delaunay3::Edge> pathEdgeUnique;
	/*look for mesh edges concurring in each vertex (incidentV) incident to v*/
	for (auto incV : incidentV) {
		/*Find one tetrahedron with edge v->incV*/
		auto c = incidentCells.begin();
		bool found = false;
		Delaunay3::Edge curEdge;
		Delaunay3::Cell_handle startingCell;

		while (c != incidentCells.end() && !found) {
			if ((*c)->has_vertex(incV)) {
				Delaunay3::Edge e((*c), (*c)->index(incV), (*c)->index(v));
				curEdge = e;
				found = true;
				startingCell = (*c);
			}
			c++;
		}

		if (found) {
			Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, startingCell);
			Delaunay3::Cell_handle lastCell = cellCirc;
			bool lastManif = cellCirc->info().getManifoldFlag();
			/*look for edges of the mesh*/
			do {
				cellCirc++;
				if (lastManif == !cellCirc->info().getManifoldFlag()) {
					int curIdx;
					for (int curV = 0; curV < 4; ++curV) {/*look for the vertex along the mesh elements of the edge starting from incV*/
						Delaunay3::Vertex_handle curVonLast = cellCirc->vertex(curV);
						if (curV != cellCirc->index(v) && curV != cellCirc->index(incV) && lastCell->has_vertex(
								curVonLast)) {
							curIdx = curV;
						}
					}

					/*store the edge of the mesh*/
					pathEdge.push_back(Delaunay3::Edge(cellCirc, cellCirc->index(incV), curIdx));

				}
				lastManif = cellCirc->info().getManifoldFlag();
				lastCell = cellCirc;
			} while (cellCirc != startingCell);
		}
	}

	std::vector<bool> testE(pathEdge.size(), true);

	for (int idxExt = 0; idxExt < pathEdge.size(); ++idxExt) {

		if (testE[idxExt]) {
			Delaunay3::Vertex_handle vE1 = pathEdge[idxExt].first->vertex(pathEdge[idxExt].second);
			Delaunay3::Vertex_handle vE2 = pathEdge[idxExt].first->vertex(pathEdge[idxExt].third);

			pathVert.push_back(vE1);
			pathVert.push_back(vE2);
			pathEdgeUnique.push_back(pathEdge[idxExt]);
			//Remove duplicates
			for (int idxInt = idxExt + 1; idxInt < pathEdge.size(); ++idxInt) {
				Delaunay3::Vertex_handle vI1 = pathEdge[idxInt].first->vertex(pathEdge[idxInt].second);
				Delaunay3::Vertex_handle vI2 = pathEdge[idxInt].first->vertex(pathEdge[idxInt].third);

				if ((vI1 == vE1 && vI2 == vE2) || (vI1 == vE2 && vI2 == vE1)) {
					testE[idxInt] = false;
				}
			}

		}
	}

	if (pathEdgeUnique.empty()) {
		return true;
	}
	std::vector<bool> yetVisited(pathEdgeUnique.size(), false);

	int curEdgeIdx = 0;

	Delaunay3::Vertex_handle initV = pathEdgeUnique[0].first->vertex(pathEdgeUnique[0].second);
	Delaunay3::Vertex_handle curV = pathEdgeUnique[0].first->vertex(pathEdgeUnique[0].second);

//yetVisited[curEdgeIdx] = true;

	do {
		int countConcurr = 0;
		int testEdgeOK = 0;

		for (int testEdge = 0; testEdge < pathEdgeUnique.size(); testEdge++) {
			if (curV == pathEdgeUnique[testEdge].first->vertex(pathEdgeUnique[testEdge].second) || curV == pathEdgeUnique[testEdge].first->vertex(
					pathEdgeUnique[testEdge].third)) {

				countConcurr++;

				if (yetVisited[testEdge] == false) {
					testEdgeOK = testEdge;
				}
			}
		}

		if (countConcurr != 2) {
			return false;
		}

// std::cout << "Step2:(" << curV->point().x() << ", " << curV->point().y() << "," << curV->point().z() << ") " << std::endl;
		if (curV == pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].second)) {

			curV = pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].third);

		} else if (curV == pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].third)) {

			curV = pathEdgeUnique[testEdgeOK].first->vertex(pathEdgeUnique[testEdgeOK].second);
		} else {

			//std::cerr << "isRegular SOMETHING WRONG" << std::endl;
		}
//std::cout << "Step3:(" << curV->point().x() << ", " << curV->point().y() << "," << curV->point().z() << ") " << std::endl;

		yetVisited[testEdgeOK] = true;

	} while (curV != initV);

	for (auto v : yetVisited) {
		if (!v) {
			return false;
		}
	}
	return true;
}

bool ManifoldManager::isFreespace(Delaunay3::Cell_handle& cell) {
	bool value;
	if (!conf_.enableInverseConic) {
		value = cell->info().exceedsNonConicFreeVoteThreshold(conf_.nonConicFreeVoteThreshold);
	} else {
		value = cell->info().exceedsFreeVoteThreshold(conf_.freeVoteThreshold);
	}
	return value;
}

bool ManifoldManager::singleTetTest2(Delaunay3::Cell_handle& cell) {
	bool iskeptManif = cell->info().getManifoldFlag();

	// Count the number of Facets in the intersection between cell and the current manifold
	int faceIndexI, faceIndexJ;
	int numF = 0;
	for (int faceIndex = 0; faceIndex < 4; ++faceIndex) {
		if (cell->neighbor(faceIndex)->info().getManifoldFlag() != iskeptManif) {
			numF++;
			if (numF == 1) faceIndexI = faceIndex;
			if (numF == 2) faceIndexJ = faceIndex;
		}
	}

	// If numF == 0, then the test is true only if both numV and numE are 0.
	// If either numV or numE are greater than zero, the test is already determined and false.
	if (numF == 0) {

		// If even one vertex satisfies the condition, the test is false (numV > 0)
		for (int curVertexId = 0; curVertexId < 4; ++curVertexId) {
			std::vector<Delaunay3::Cell_handle> incidentCells;
			dt_.incident_cells(cell->vertex(curVertexId), std::back_inserter(incidentCells));

			for (auto c : incidentCells)
				if (c->info().getManifoldFlag() != iskeptManif) return false; // shortcut

		}

		// If even one edge satisfies the condition, the test is false (numE > 0)
		for (int curEdgeId1 = 0; curEdgeId1 < 4; ++curEdgeId1) {
			for (int curEdgeId2 = curEdgeId1 + 1; curEdgeId2 < 4; ++curEdgeId2) {
				Delaunay3::Edge curEdge(cell, curEdgeId1, curEdgeId2);

				Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, cell);
				Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(curEdge, cell);

				do {
					if (cellCirc->info().getManifoldFlag() != iskeptManif) return false; // shortcut
					cellCirc++;
				} while (cellCirc != cellCircInit);

			}
		}

		return true;

	} else if (numF == 1) {

		// If numF == 1, then the test is true only if both numV and numE are 3,
		// but given that the condition is true for one and only one face (called face i),
		// then the vertex opposite to the face i (that is, vertex i), determines if the test is true.
		// In fact, the remaining vertices are incident to the cell adjacent to the face that already satisfied the condition,
		// hence the condition on those vertices is already satisfied.

		std::vector<Delaunay3::Cell_handle> incidentCells;
		dt_.incident_cells(cell->vertex(faceIndexI), std::back_inserter(incidentCells));

		for (auto c : incidentCells) {
			if (c->info().getManifoldFlag() != iskeptManif) {
				return false;
			}
		}

		// If the condition is true for one and the only one face i,
		// then the condition must be true for all edges composing the face i and whether or not the condition is true for the vertex opposite to the face i (that is, vertex i),
		// it is redundant to check for the remaining edges, since the cells incident to the vertex i are also incident to them.
		// condition false for vertex i implies condition false for remaining edges (i, -)
		// condition true for vertex i implies the test is already determined (and false).

		return true;

	} else if (numF == 2) {

		// If two of the faces are adjacent the cells that make the condition true, then the condition is implied for all vertices and for all but one edge.
		// The only edge for which the condition needs to be tested is the one connecting the vertices opposite to the faces satisfing the condition.
		// Calling such faces i and j, said edge is the edge (i, j).

		Delaunay3::Edge edgeIJ(cell, faceIndexI, faceIndexJ);
		Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(edgeIJ, cell);
		Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(edgeIJ, cell);

		do {
			if (cellCirc->info().getManifoldFlag() != iskeptManif) {
				return false;
			}
			cellCirc++;
		} while (cellCirc != cellCircInit);

		return true;

	} else if (numF >= 3) {

		// If all or even just three faces satisfy the condition, then it is implied for all vertices and all edges to satisfy the condition.

		return true;

	}

}
