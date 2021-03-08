function [initializationParam, continuousParam] = MalagaParameters(K)

% INITIALIZATION PARAMETERS
initializationParam = struct();

% detectHarrisFeatures() parameters
initializationParam.MinQuality = 5e-5;
initializationParam.FilterSize = 21;

% vision.PointTracker() parameters
initializationParam.NumPyramidLevels = 4;
initializationParam.MaxBidirectionalError = 1;  
initializationParam.BlockSize = [15 15];            
initializationParam.MaxIterations = 30;

% estimateFundamentalMatrix() parameters
initializationParam.Method = 'RANSAC';
initializationParam.NumTrials = 3000;              
initializationParam.DistanceThreshold = 0.2;

% Camera Intrinsic Parameters
initializationParam.Intrinsics = cameraParameters('IntrinsicMatrix',K');

% Threshold for considering 2 points as the same 
initializationParam.Threshold_same_point = 4^2;

% range depth
initializationParam.RangeDepth = 9;

% -----------------------------
% CONTINUOUS PARAMETERS
continuousParam = struct();

% detectHarrisFeatures() parameters
continuousParam.MinQuality = 1e-4;
continuousParam.FilterSize = 21;

% vision.PointTracker() parameters
continuousParam.NumPyramidLevels = 3;
continuousParam.MaxBidirectionalError = 1;  
continuousParam.BlockSize = [21 21];            
continuousParam.MaxIterations = 30;

% Camera Intrinsic Parameters
continuousParam.Intrinsics = cameraParameters('IntrinsicMatrix',K');
continuousParam.K = K;

% estimateWorldCameraPose() parameters
continuousParam.MaxNumTrials = 3000;
continuousParam.Confidence = 90;
continuousParam.MaxReprojectionError = 2;

% Min angle for a landmark to be added
continuousParam.Threshold_angle = 3;

% Threshold for considering 2 points as the same 
continuousParam.Threshold_same_point = 5^2;

% range depth
continuousParam.RangeDepth = 9;

% Pose refinement
continuousParam.poseRefinementFlag = true;

end