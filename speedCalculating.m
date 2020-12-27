function Copy_3_of_Demo()
clear;
close all;

thresh=15;
show = 1;

results=[];
    %imgAdress='D:\...\person_033.png';
vid=VideoReader('D:\DSC_6557.MOV');
%resultFolder = 'E:\...\';
%mkdir(resultFolder);
%vout = VideoWriter(strcat(resultFolder,num2str(video),'_out'));
%open(vout);

frn=0; %frame number
tracks = initializeTracks();
nextId = 1; % ID of the next track
mybboxs=[];
numberOfFrames = vid.Duration * vid.FrameRate;
time=[];
% try
% ROI= dlmread(strcat('ROI/',num2str(video),'.txt'));
% xROI=ROI(:,1);
% yROI=ROI(:,2);
% catch
%     xROI=[1;960;960;1];
%     yROI=[1;1;540;540];    
% end
        
    
% showROI = true;

detector = vision.ForegroundDetector(...
       'NumTrainingFrames', 100, ... % 5 because of short video
       'InitialVariance', 30*30, ...
       'LearningRate', .005); % initial standard deviation of 30
blob = vision.BlobAnalysis(...
       'CentroidOutputPort', false, 'AreaOutputPort', false, ...
       'BoundingBoxOutputPort', true, ...
       'MinimumBlobAreaSource', 'Property', 'MinimumBlobArea', 250);
    % Load ROI and getting Mask
getROI = false;
while(frn < numberOfFrames ) 
      % if frn > 30 * vid.FrameRate , break; end
   
    im=readFrame(vid);
    im = imrotate(im,90);
    if(frn == 0)
        %delete detailResults.txt
        if(getROI == true)
            ROI = roipoly(im);
            save ROI ROI
        else
            load ROI
        end
    end
    %im=imresize(im,[size(im,1)*.4, size(im,2)*.4]);
    im(~repmat(ROI, [1, 1, 3])) = 0;
    fgMask = step(detector, im );
    bboxes   = step(blob, fgMask);
    bboxes = double(bboxes);
    frn=frn+1;


    tic;
    
    predictNewLocationsOfTracks();
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();

    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();

    time(frn)=toc;
    if (show)
        positions=[];
        labels_str={};
        results=[];
        for j=1:size(tracks,2)
            if(tracks(j).totalVisibleCount > thresh)
                positions = [positions;tracks(j).bbox];
                labels_str{end+1}=num2str(tracks(j).id);

                % for saving in txt file, if it was inside the  ROI, It
                % would be written in txt file

                results = [results;[ frn, tracks(j).id, tracks(j).bbox(1:4)]];
            %    end

            end
        end
        
        dlmwrite('detailResultsFull.txt',results,'-append');
        %im=insertObjectAnnotation(im,'rectangle',bboxes,ones(size(bboxes,1),1),'TextBoxOpacity',0.1,'FontSize',8,'Color','r');

        %im=insertObjectAnnotation(im,'rectangle',positions,labels_str,'TextBoxOpacity',0.1,'FontSize',8);

        %imshow(fgMask);
        %out_image=getframe(h);
        %writeVideo(vout,im);
        %pause(.0000001);
        %drawnow
        %hold on;
         %fill(xROI,yROI,'r','FaceAlpha',0.5);
        %drawnow


    end
    clc;
%    fprintf('Video Number = %d\n',video);
    fprintf('Progress = %.2f %%',frn*100/numberOfFrames);

end
%txtName=sprintf('%s%d.txt',resultFolder,video);
%dlmwrite(txtName,results);
%allTimes{video}=time;
%time=[];
%save(strcat(resultFolder,'allTimes'),'allTimes');

    
function tracks = initializeTracks()
% create an empty array of tracks
tracks = struct(...
    'id', {}, ...
    'bbox', {}, ...
    'kalmanFilter', {}, ...
    'age', {}, ...
    'totalVisibleCount', {}, ...
    'consecutiveInvisibleCount', {});
end


function predictNewLocationsOfTracks()
for i = 1:length(tracks)
    %bbox = tracks(i).bbox;

    % Predict the current location of the track.
    predictedBbbox = predict(tracks(i).kalmanFilter);

    % Shift the bounding box so that its center is at
    % the predicted location.
    %predictedCentroid = predictedCentroid - bbox(3:4) / 2;
    tracks(i).bbox = predictedBbbox;
end
end


function [assignments, unassignedTracks, unassignedDetections] = ...
    detectionToTrackAssignment()

nTracks = length(tracks);
nDetections = size(bboxes, 1);

% Compute the cost of assigning each detection to each track.
cost = zeros(nTracks, nDetections);
for i = 1:nTracks
    for j=1:nDetections
        cost(i,j)=-rectint(tracks(i).bbox,bboxes(j,:))/sqrt(tracks(i).bbox(3)*tracks(i).bbox(4)*bboxes(j,4)*bboxes(j,3));
    end
    %cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
end

% Solve the assignment problem.
costOfNonAssignment = -0.01;
[assignments, unassignedTracks, unassignedDetections] = ...
    assignDetectionsToTracks(cost, costOfNonAssignment);
end



%% Update Assigned Tracks
% The |updateAssignedTracks| function updates each assigned track with the
% corresponding detection. It calls the |correct| method of
% |vision.KalmanFilter| to correct the location estimate. Next, it stores
% the new bounding box, and increases the age of the track and the total
% visible count by 1. Finally, the function sets the invisible count to 0.

function updateAssignedTracks()
numAssignedTracks = size(assignments, 1);
for i = 1:numAssignedTracks
    trackIdx = assignments(i, 1);
    detectionIdx = assignments(i, 2);
%        centroid = centroids(detectionIdx, :);
    bbox = bboxes(detectionIdx, :);

    % Correct the estimate of the object's location
    % using the new detection.
    correct(tracks(trackIdx).kalmanFilter, bbox);

    % Replace predicted bounding box with detected
    % bounding box.
    %tracks(trackIdx).bbox = bbox;

    % Update track's age.
    tracks(trackIdx).age = tracks(trackIdx).age + 1;

    % Update visibility.
    tracks(trackIdx).totalVisibleCount = ...
        tracks(trackIdx).totalVisibleCount + 1;
    tracks(trackIdx).consecutiveInvisibleCount = 0;
end
end

%% Update Unassigned Tracks
% Mark each unassigned track as invisible, and increase its age by 1.

function updateUnassignedTracks()
for i = 1:length(unassignedTracks)
    ind = unassignedTracks(i);
    tracks(ind).age = tracks(ind).age + 1;
    tracks(ind).consecutiveInvisibleCount = ...
        tracks(ind).consecutiveInvisibleCount + 1;
end
end

%% Delete Lost Tracks
% The |deleteLostTracks| function deletes tracks that have been invisible
% for too many consecutive frames. It also deletes recently created tracks
% that have been invisible for too many frames overall.

function deleteLostTracks()
if isempty(tracks)
    return;
end

invisibleForTooLong = 50;
ageThreshold = 8;

% Compute the fraction of the track's age for which it was visible.
ages = [tracks(:).age];
totalVisibleCounts = [tracks(:).totalVisibleCount];
visibility = totalVisibleCounts ./ ages;

% Find the indices of 'lost' tracks.
lostInds = (ages < ageThreshold & visibility < 0.6) | ...
    [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

% Delete lost tracks.
tracks = tracks(~lostInds);
end

%% Create New Tracks
% Create new tracks from unassigned detections. Assume that any unassigned
% detection is a start of a new track. In practice, you can use other cues
% to eliminate noisy detections, such as size, location, or appearance.

function createNewTracks()
%    centroids = centroids(unassignedDetections, :);
bboxes = bboxes(unassignedDetections, :);

for i = 1:size(bboxes, 1)

    %centroid = centroids(i,:);
    bbox = bboxes(i, :);

    % Create a Kalman filter object.
    %kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
    %    bbox, [100, 100], [100 , 10], 1e5);

    StateTransitionModel=[1 0 0 0 1 0 0 0; ...
                                       0 1 0 0 0 1 0 0; ...
                                       0 0 1 0 0 0 1 0; ...
                                       0 0 0 1 0 0 0 1; ...
                                       0 0 0 0 1 0 0 0; ...
                                       0 0 0 0 0 1 0 0; ...
                                       0 0 0 0 0 0 1 0; ...
                                       0 0 0 0 0 0 0 1];
    MeasurementModel= [1 0 0 0 0 0 0 0; ...
                                   0 1 0 0 0 0 0 0; ...
                                   0 0 1 0 0 0 0 0; ...
                                   0 0 0 1 0 0 0 0]; 
    kalmanFilter=vision.KalmanFilter(StateTransitionModel,MeasurementModel);

    kalmanFilter.MeasurementNoise = [1e3 0 0 0; ...
                                      0 1e3 0 0; ...
                                      0 0 1e4 0; ...
                                      0 0 0 1e4];
    kalmanFilter.State = [bbox 0 0 0 0];



    %  kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
    %     centroid, [5, 5], [5, 5], 100);

    % Create a new track.
    newTrack = struct(...
        'id', nextId, ...
        'bbox', bbox, ...
        'kalmanFilter', kalmanFilter, ...
        'age', 1, ...
        'totalVisibleCount', 1, ...
        'consecutiveInvisibleCount', 0);

    % Add it to the array of tracks.
    tracks(end + 1) = newTrack;

    % Increment the next id.
    nextId = nextId + 1;
end
end

end

% filter dtections with size od bounding box
function out=filterBboxesWithSize(d,minSize,maxSize)
    d=d(d(:,3)>minSize,:);
    d=d(d(:,3)<maxSize,:);
    out=d;
end