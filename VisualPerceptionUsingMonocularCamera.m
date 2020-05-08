%Define Camera Configuration

focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
height = 2.1798;    % mounting height in meters from the ground
pitch  = 14;        % pitch of the camera in degrees
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

%Load a Frame of Video

videoName = 'caltech_cordova1.avi';
videoReader = VideoReader(videoName);
timeStamp = 0.06667;                   % time from the beginning of the video
videoReader.CurrentTime = timeStamp;   % point to the chosen frame

frame = readFrame(videoReader); % read frame at timeStamp seconds
imshow(frame) % display frame

%Create Birds-Eye-View Image
% Using vehicle coordinates, define area to transform

distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 6;  % all other distance quantities are also in meters
bottomOffset      = 3;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);

birdsEyeImage = transformImage(birdsEyeConfig, frame);
figure
imshow(birdsEyeImage)

%Find Lane Markers in Vehicle Coordinates

% Convert to grayscale
birdsEyeImage = rgb2gray(birdsEyeImage);

% Lane marker segmentation ROI in world units
vehicleROI = outView - [-1, 2, -3, 3]; % look 3 meters to left and right, and 4 meters ahead of the sensor
approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters

% Detect lane features
laneSensitivity = 0.25;
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, approxLaneMarkerWidthVehicle,...
    'ROI', vehicleROI, 'Sensitivity', laneSensitivity);

figure
imshow(birdsEyeViewBW)
% Obtain lane candidate points in vehicle coordinates
[imageX, imageY] = find(birdsEyeViewBW);
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
maxLanes      = 2; % look for maximum of two lane markers
boundaryWidth = 3*approxLaneMarkerWidthVehicle; % expand boundary width

[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
    'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);

%Determine Boundaries of the Ego Lane

% Establish criteria for rejecting boundaries based on their length
maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; % establish a threshold

% Reject short boundaries
isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
boundaries    = boundaries(isOfMinLength);
% To compute the maximum strength, assume all image pixels within the ROI
% are lane candidate points
birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));

% Convert the image points to vehicle points
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

% Find the maximum number of unique x-axis locations possible for any lane
% boundary
maxPointsInOneLane = numel(unique(vehiclePoints(:,1)));

% Set the maximum length of a lane boundary to the ROI length
maxLaneLength = diff(vehicleROI(1:2));

% Compute the maximum possible lane strength for this image size/ROI size
% specification
maxStrength   = maxPointsInOneLane/maxLaneLength;

% Reject weak boundaries
isStrong      = [boundaries.Strength] > 0.4*maxStrength;
boundaries    = boundaries(isStrong);
boundaries = classifyLaneTypes(boundaries, boundaryPoints);

% Locate two ego lanes if they are present
xOffset    = 0;   %  0 meters from the sensor
distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);

% Find candidate ego boundaries
leftEgoBoundaryIndex  = [];
rightEgoBoundaryIndex = [];
minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
if ~isempty(minLDistance)
    leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
end
if ~isempty(minRDistance)
    rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
end
leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);
xVehiclePoints = bottomOffset:distAheadOfSensor;
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeWithEgoLane, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');

frameWithEgoLane = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
frameWithEgoLane = insertLaneBoundary(frameWithEgoLane, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

figure
subplot('Position', [0, 0, 0.5, 1.0]) % [left, bottom, width, height] in normalized units
imshow(birdsEyeWithEgoLane)
subplot('Position', [0.5, 0, 0.5, 1.0])
imshow(frameWithEgoLane)


%Locate Vehicles in Vehicle Coordinates
detector = vehicleDetectorACF();

% Width of a common vehicle is between 1.5 to 2.5 meters
vehicleWidth = [1.5, 2.5];
monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);

[bboxes, scores] = detect(monoDetector, frame);
locations = computeVehicleLocations(bboxes, sensor);

% Overlay the detections on the video frame
imgOut = insertVehicleDetections(frame, locations, bboxes);
figure;
imshow(imgOut);


%Simulate a Complete Sensor with Video Input


videoReader.CurrentTime = 0;

isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
   
    % Grab a frame of video
    frame = readFrame(videoReader);
    
    % Compute birdsEyeView image
    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = rgb2gray(birdsEyeImage);
    
    % Detect lane boundary features
    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, ...
        approxLaneMarkerWidthVehicle, 'ROI', vehicleROI, ...
        'Sensitivity', laneSensitivity);

    % Obtain lane candidate points in vehicle coordinates
    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);

    % Find lane boundary candidates
    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);
    
    % Reject boundaries based on their length and strength    
    isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
    boundaries    = boundaries(isOfMinLength);
    isStrong      = [boundaries.Strength] > 0.2*maxStrength;
    boundaries    = boundaries(isStrong);
                
    % Classify lane marker type
    boundaries = classifyLaneTypes(boundaries, boundaryPoints);        
        
    % Find ego lanes
    xOffset    = 0;   %  0 meters from the sensor
    distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
    % Find candidate ego boundaries
    leftEgoBoundaryIndex  = [];
    rightEgoBoundaryIndex = [];
    minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
    minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
    if ~isempty(minLDistance)
        leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
    end
    if ~isempty(minRDistance)
        rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
    end
    leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
    rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);
    
    % Detect vehicles
    [bboxes, scores] = detect(monoDetector, frame);
    locations = computeVehicleLocations(bboxes, sensor);
    
    % Visualize sensor outputs and intermediate results. Pack the core
    % sensor outputs into a struct.
    sensorOut.leftEgoBoundary  = leftEgoBoundary;
    sensorOut.rightEgoBoundary = rightEgoBoundary;
    sensorOut.vehicleLocations = locations;
    
    sensorOut.xVehiclePoints   = bottomOffset:distAheadOfSensor;
    sensorOut.vehicleBoxes     = bboxes;
    
    % Pack additional visualization data, including intermediate results
    intOut.birdsEyeImage   = birdsEyeImage;    
    intOut.birdsEyeConfig  = birdsEyeConfig;
    intOut.vehicleScores   = scores;
    intOut.vehicleROI      = vehicleROI;
    intOut.birdsEyeBW      = birdsEyeViewBW;
    
    closePlayers = ~hasFrame(videoReader);
    isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut, ...
        intOut, closePlayers);
    
    timeStamp = 7.5333; % take snapshot for publishing at timeStamp seconds
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot(frame, sensor, sensorOut);
    end    
end

if ~isempty(snapshot)
    figure
    imshow(snapshot)
end


%Try the Sensor Design on a Different Video

% Sensor configuration
focalLength    = [309.4362, 344.2161];
principalPoint = [318.9034, 257.5352];
imageSize      = [480, 640];
height         = 2.1798;    % mounting height in meters from the ground
pitch          = 14;        % pitch of the camera in degrees

camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
sensor        = monoCamera(camIntrinsics, height, 'Pitch', pitch);

videoReader = VideoReader('caltech_washington1.avi');
monoSensor   = helperMonoSensor(sensor);
monoSensor.LaneXExtentThreshold = 0.5;
% To remove false detections from shadows in this video, we only return
% vehicle detections with higher scores.
monoSensor.VehicleDetectionThreshold = 20;    

isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
    
    frame = readFrame(videoReader); % get a frame
    
    sensorOut = processFrame(monoSensor, frame);

    closePlayers = ~hasFrame(videoReader);
            
    isPlayerOpen = displaySensorOutputs(monoSensor, frame, sensorOut, closePlayers);
    
    timeStamp = 11.1333; % take snapshot for publishing at timeStamp seconds
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot(frame, sensor, sensorOut);
    end
   
end
if ~isempty(snapshot)
    figure
    imshow(snapshot)
end


%Supporting Functions

function isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut,...
    intOut, closePlayers)

    % Unpack the main inputs
    leftEgoBoundary  = sensorOut.leftEgoBoundary;
    rightEgoBoundary = sensorOut.rightEgoBoundary;
    locations        = sensorOut.vehicleLocations;

    xVehiclePoints   = sensorOut.xVehiclePoints;    
    bboxes           = sensorOut.vehicleBoxes;
    
    % Unpack additional intermediate data
    birdsEyeViewImage = intOut.birdsEyeImage;
    birdsEyeConfig          = intOut.birdsEyeConfig;
    vehicleROI        = intOut.vehicleROI;
    birdsEyeViewBW    = intOut.birdsEyeBW;
    
    % Visualize left and right ego-lane boundaries in birds-eye view
    birdsEyeWithOverlays = insertLaneBoundary(birdsEyeViewImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
    birdsEyeWithOverlays = insertLaneBoundary(birdsEyeWithOverlays, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
    
    % Visualize ego-lane boundaries in camera view
    frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
    frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

    frameWithOverlays = insertVehicleDetections(frameWithOverlays, locations, bboxes);

    imageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
    ROI = [imageROI(1) imageROI(3) imageROI(2)-imageROI(1) imageROI(4)-imageROI(3)];

    % Highlight candidate lane points that include outliers
    birdsEyeViewImage = insertShape(birdsEyeViewImage, 'rectangle', ROI); % show detection ROI
    birdsEyeViewImage = imoverlay(birdsEyeViewImage, birdsEyeViewBW, 'blue');
        
    % Display the results
    frames = {frameWithOverlays, birdsEyeViewImage, birdsEyeWithOverlays};
    
    persistent players;
    if isempty(players)
        frameNames = {'Lane marker and vehicle detections', 'Raw segmentation', 'Lane marker detections'};
        players = helperVideoPlayerSet(frames, frameNames);
    end    
    update(players, frames);
    
    % Terminate the loop when the first player is closed
    isPlayerOpen = isOpen(players, 1);
    
    if (~isPlayerOpen || closePlayers) % close down the other players
        clear players;
    end
end

function locations = computeVehicleLocations(bboxes, sensor)

locations = zeros(size(bboxes,1),2);
for i = 1:size(bboxes, 1)
    bbox  = bboxes(i, :);
    
    % Get [x,y] location of the center of the lower portion of the
    % detection bounding box in meters. bbox is [x, y, width, height] in
    % image coordinates, where [x,y] represents upper-left corner.
    yBottom = bbox(2) + bbox(4) - 1;
    xCenter = bbox(1) + (bbox(3)-1)/2; % approximate center
    
    locations(i,:) = imageToVehicle(sensor, [xCenter, yBottom]);
end
end

function imgOut = insertVehicleDetections(imgIn, locations, bboxes)

imgOut = imgIn;

for i = 1:size(locations, 1)
    location = locations(i, :);
    bbox     = bboxes(i, :);
        
    label = sprintf('X=%0.2f, Y=%0.2f', location(1), location(2));

    imgOut = insertObjectAnnotation(imgOut, ...
        'rectangle', bbox, label, 'Color','g');
end
end

function imageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI)

vehicleROI = double(vehicleROI);

loc2 = abs(vehicleToImage(birdsEyeConfig, [vehicleROI(2) vehicleROI(4)]));
loc1 = abs(vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(4)]));
loc4 =     vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(4)]);
loc3 =     vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(3)]);

[minRoiX, maxRoiX, minRoiY, maxRoiY] = deal(loc4(1), loc3(1), loc2(2), loc1(2));

imageROI = round([minRoiX, maxRoiX, minRoiY, maxRoiY]);

end
function isGood = validateBoundaryFcn(params)

if ~isempty(params)
    a = params(1);
    
    % Reject any curve with a small 'a' coefficient, which makes it highly
    % curved.
    isGood = abs(a) < 0.003; % a from ax^2+bx+c
else
    isGood = false;
end
end
function boundaries = classifyLaneTypes(boundaries, boundaryPoints)

for bInd = 1 : numel(boundaries)
    vehiclePoints = boundaryPoints{bInd};
    % Sort by x
    vehiclePoints = sortrows(vehiclePoints, 1);
    
    xVehicle = vehiclePoints(:,1);
    xVehicleUnique = unique(xVehicle);
    
    % Dashed vs solid
    xdiff  = diff(xVehicleUnique);
    % Sufficiently large threshold to remove spaces between points of a
    % solid line, but not large enough to remove spaces between dashes
    xdifft = mean(xdiff) + 3*std(xdiff);
    largeGaps = xdiff(xdiff > xdifft);
    
    % Safe default
    boundaries(bInd).BoundaryType= LaneBoundaryType.Solid;
    if largeGaps>2
        % Ideally, these gaps should be consistent, but you cannot rely
        % on that unless you know that the ROI extent includes at least 3 dashes.
        boundaries(bInd).BoundaryType = LaneBoundaryType.Dashed;
    end
end

end

function I = takeSnapshot(frame, sensor, sensorOut)

    % Unpack the inputs
    leftEgoBoundary  = sensorOut.leftEgoBoundary;
    rightEgoBoundary = sensorOut.rightEgoBoundary;
    locations        = sensorOut.vehicleLocations;
    xVehiclePoints   = sensorOut.xVehiclePoints;
    bboxes           = sensorOut.vehicleBoxes;
      
    frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
    frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');
    frameWithOverlays = insertVehicleDetections(frameWithOverlays, locations, bboxes);
   
    I = frameWithOverlays;

end

