//  Copyright 2014 Andrea Romanoni
//
//  This file is part of edgePointSpaceCarver.
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

#ifndef TYPES_TEST_HPP_
#define TYPES_TEST_HPP_

#include <Eigen/Core>
#include <sstream>
#include <string>
#include <vector>



typedef struct {
  std::string folderImage;
  std::string baseNameImage;
  std::string imageExtension;
  int idxFirstFrame;
  int digitIdxLength; //if 0 no padding
  int idxLastFrame;
  int downsampleRate;
  int imageH;
  int imageW;
} VideoConfig;

typedef struct {
  int keyFramePeriod;
  int keyFramePeriodIteration;
  double cannyHighThreshold;
  int downsamplePeriod;
  int maxGaussNewtonIteration;
  int minDistBetweenTrackedPoints;
  double maxEpipolarDist;
  int minTrackLength;
  int manifoldPeriod;
  bool inverseConicEnabled;
  double probOrVoteThreshold;
  int edgePointEnabled;
  int firstGrowingFrame;
  double maxDistanceCamFeature;
  bool enableSuboptimalPolicy;
  int suboptimalMethod;
  float w_1;
  float w_2;
  float w_3;
} SpaceCarvingConfig;

typedef struct {
  std::string pathToSave;
  std::string pathLog;
  std::string pathLogPoints;
  std::string pathStats;
  std::string pathStatsManifold;
  bool enableSaveReconstr;
  bool enableSaveShrink;
} OutputSpaceCarving;
typedef struct {
  std::string pathInitPoints;
  std::string pathCamsPose;
} ManifoldRecConfig;


typedef struct {
} OutputManifoldConfig;

typedef struct {
  VideoConfig videoConfig;
  ManifoldRecConfig manifConfig;
  OutputManifoldConfig outputConfig;

  std::string toString() {
    std::stringstream totContent;

    totContent << "Video Config " << std::endl;
    totContent << "folderImage " << videoConfig.folderImage << std::endl;
    totContent << "baseNameImage " << videoConfig.baseNameImage << std::endl;
    totContent << "imageExtension " << videoConfig.imageExtension << std::endl;
    totContent << "idxFirstFrame " << videoConfig.idxFirstFrame << std::endl;
    totContent << "digitIdxLength " << videoConfig.digitIdxLength << std::endl;
    totContent << "idxLastFrame " << videoConfig.idxLastFrame << std::endl;
    totContent << "downsampleRate " << videoConfig.downsampleRate << std::endl;
    totContent << "imageH " << videoConfig.imageH << std::endl;
    totContent << "imageW " << videoConfig.imageW << std::endl;

    totContent << "manifConfig Config " << std::endl;
    totContent << "pathCamsPose " << manifConfig.pathCamsPose << std::endl;
    totContent << "pathInitPoints " << manifConfig.pathInitPoints << std::endl;

    totContent << "Output Config " << std::endl;
    return totContent.str();
  }
} ManifoldConfig;

typedef struct {
  VideoConfig videoConfig;
  SpaceCarvingConfig spaceCarvingConfig;
  OutputSpaceCarving outputSpaceCarving;

  std::string toString() {
    std::stringstream totContent;

    totContent << "Video Config " << std::endl;
    totContent << "folderImage " << videoConfig.folderImage << std::endl;
    totContent << "baseNameImage " << videoConfig.baseNameImage << std::endl;
    totContent << "imageExtension " << videoConfig.imageExtension << std::endl;
    totContent << "idxFirstFrame " << videoConfig.idxFirstFrame << std::endl;
    totContent << "digitIdxLength " << videoConfig.digitIdxLength << std::endl;
    totContent << "idxLastFrame " << videoConfig.idxLastFrame << std::endl;
    totContent << "downsampleRate " << videoConfig.downsampleRate << std::endl;
    totContent << "imageH " << videoConfig.imageH << std::endl;
    totContent << "imageW " << videoConfig.imageW << std::endl;

    totContent << "SpaceCarving Config " << std::endl;
    totContent << "keyFramePeriod " << spaceCarvingConfig.keyFramePeriod << std::endl;
    totContent << "keyFramePeriodIteration " << spaceCarvingConfig.keyFramePeriodIteration << std::endl;
    totContent << "cannyHighThreshold " << spaceCarvingConfig.cannyHighThreshold << std::endl;
    totContent << "downsamplePeriod " << spaceCarvingConfig.downsamplePeriod << std::endl;
    totContent << "maxGaussNewtonIteration " << spaceCarvingConfig.maxGaussNewtonIteration << std::endl;
    totContent << "minDistBetweenTrackedPoints " << spaceCarvingConfig.minDistBetweenTrackedPoints << std::endl;
    totContent << "maxEpipolarDist " << spaceCarvingConfig.maxEpipolarDist << std::endl;
    totContent << "minTrackLength " << spaceCarvingConfig.minTrackLength << std::endl;
    totContent << "manifoldPeriod " << spaceCarvingConfig.manifoldPeriod << std::endl;
    totContent << "inverseConicEnabled " << spaceCarvingConfig.inverseConicEnabled << std::endl;
    totContent << "probOrVoteThreshold " << spaceCarvingConfig.probOrVoteThreshold << std::endl;
    totContent << "edgePointEnabled " << spaceCarvingConfig.edgePointEnabled << std::endl;
    totContent << "firstGrowingFrame " << spaceCarvingConfig.firstGrowingFrame << std::endl;
    totContent << "maxDistanceCamFeature " << spaceCarvingConfig.maxDistanceCamFeature << std::endl;
    totContent << "enableSuboptimalPolicy " << spaceCarvingConfig.enableSuboptimalPolicy << std::endl;
    totContent << "suboptimalMethod " << spaceCarvingConfig.suboptimalMethod << std::endl;

    totContent << "Output Config " << std::endl;
    totContent << "enableSaveReconstr " << outputSpaceCarving.enableSaveReconstr << std::endl;
    totContent << "enableSaveShrink " << outputSpaceCarving.enableSaveShrink << std::endl;
    totContent << "pathToSave " << outputSpaceCarving.pathToSave << std::endl;
    totContent << "pathLog " << outputSpaceCarving.pathLog << std::endl;
    totContent << "pathLogPoints " << outputSpaceCarving.pathLogPoints << std::endl;
    totContent << "pathStats " << outputSpaceCarving.pathStats << std::endl;
    totContent << "pathStatsManifold " << outputSpaceCarving.pathStatsManifold << std::endl;
    return totContent.str();
  }
} Configuration;


typedef struct {
  bool inverseConicEnabled = true;
  float probOrVoteThreshold = 20.0;
  float maxDistanceCamFeature = 40.0;
  bool enableSuboptimalPolicy = false;
  int suboptimalMethod = 0;
  float w_1 = 10.0;
  float w_2 = 2.0;
  float w_3 = 1.0;
  float steinerGridSideLength = 280.0;
  float steinerGridStepLength = 20.0;

  int manifold_update_every = 4;
  int initial_manifold_update_skip = 2;
  int save_manifold_every = 32;
  int primary_points_visibility_threshold = 2;

  int fake_points_multiplier = 0;

  bool all_sort_of_output = false;
  bool update_points_position = false;


  std::string toString() {
    std::stringstream out;
    out << "inverseConicEnabled: " << inverseConicEnabled << std::endl;
    out << "maxDistanceCamFeature: " << maxDistanceCamFeature << std::endl;
    out << "probOrVoteThreshold: " << probOrVoteThreshold << std::endl;
    out << "enableSuboptimalPolicy: " << enableSuboptimalPolicy << std::endl;
    out << "suboptimalMethod: " << suboptimalMethod << std::endl;
    out << "w_1: " << w_1 << std::endl;
    out << "w_2: " << w_2 << std::endl;
    out << "w_3: " << w_3 << std::endl;
    out << "steinerGridStepLength: " << steinerGridStepLength << std::endl;
    out << "steinerGridSideLength: " << steinerGridSideLength << std::endl;

    out << "manifold_update_every: " << manifold_update_every << std::endl;
    out << "initial_manifold_update_skip: " << initial_manifold_update_skip << std::endl;
    out << "save_manifold_every: " << save_manifold_every << std::endl;
    out << "primary_points_visibility_threshold: " << primary_points_visibility_threshold << std::endl;

    out << "fake_points_multiplier: " << fake_points_multiplier << std::endl;

    out << "all_sort_of_output: " << all_sort_of_output << std::endl;
    out << "update_points_position: " << update_points_position << std::endl;

    return out.str();
  }

} ManifoldReconstructionConfig;



struct CameraRect {
    Eigen::Matrix4f P;
    //camera center
    Eigen::Vector3f center;
  std::vector<std::string> names_;
    std::vector<int> viewingPointsIndices;
};


struct SensorParser {
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    float f;
    float k1;
    float k2;
    //camera center
    Eigen::Vector3f center;

    std::vector<int> viewingPointsIndices;
};

#endif /* TYPES_TEST_HPP_ */
