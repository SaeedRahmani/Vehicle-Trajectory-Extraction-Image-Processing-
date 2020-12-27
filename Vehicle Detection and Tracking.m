function Demo()
% profile on;
clear;
close all;
addpath(genpath('D:\...\toolbox'));
model ='new-models/...';

sizeFilter = 10;

results=[];
load(model);
%imgAdress='D:\....png';
fileName = '00127';
vid=VideoReader(strcat('.../',fileName,'.MTS'));
resultFolder = '';
vout = VideoWriter(strcat(fileName,'_out','.avi'));
open(vout);
h=figure;
set(h,'visible','on');
hold on;
detector.opts.cascThr=0;
detector.opts.pNms.overlap = .2;
frn=0;
tracks = initializeTracks();
nextId = 1; % ID of the next track
totalFrames = vid.FrameRate*vid.duration;
for frn=1:totalFrames
    try
        im=readFrame(vid);
    catch
        break;
    end
    im = imresize(im , .25);
    %     im = imread('H:/.../0_17.jpg');
    % i=i+5;
    %tic;
    d=acfDetect(im,detector);
    d=filterBboxesWithSize(d,sizeFilter,1000);
    centroids = d(:,1:2)+d(:,3:4)/2;
    bboxes =d(:,1:4);
    predictNewLocationsOfTracks();
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();
    
    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();
    
    %toc;
    %imshow(im);
    for j=1:size(tracks,2)
        if(tracks(j).totalVisibleCount > 5 || frn<7)
            im=insertObjectAnnotation(im,'rectangle',tracks(j).bbox ,tracks(j).id);
            %rectangle('position',tracks(j).bbox,'linewidth',3,'edgecolor','r');
            %text(tracks(j).bbox(1),tracks(j).bbox(2),num2str(tracks(j).id),'FontSize',30,'Color','g');
            results = [results;[ frn, tracks(j).id, tracks(j).bbox(1:4)]];
        end
    end
    imshow(im);
    %out_image=getframe(h);
    writeVideo(vout,im);
    %pause(.0000001);
    drawnow
    %toc;
    clc;
    fprintf('%.2f',frn/totalFrames*100);
    if (mod(frn,100)==0)
        txtName=sprintf('%s.txt',fileName);
        dlmwrite(txtName,results);
    end
    
end
%txtName=sprintf('%s.txt',fileName);
%dlmwrite(txtName,results);

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
            bbox = tracks(i).bbox;
            
            % Predict the current location of the track.
            predictedCentroid = predict(tracks(i).kalmanFilter);
            
            % Shift the bounding box so that its center is at
            % the predicted location.
            predictedCentroid = predictedCentroid - bbox(3:4) / 2;
            tracks(i).bbox = [predictedCentroid, bbox(3:4)];
        end
    end


    function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()
        
        nTracks = length(tracks);
        nDetections = size(centroids, 1);
        
        % Compute the cost of assigning each detection to each track.
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            for j=1:nDetections
                cost(i,j)=-rectint(tracks(i).bbox,bboxes(j,:))/sqrt(tracks(i).bbox(3)*tracks(i).bbox(4)*bboxes(j,4)*bboxes(j,3));
            end
            %cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end
        
        % Solve the assignment problem.
        costOfNonAssignment = -.1;
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
            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);
            
            % Correct the estimate of the object's location
            % using the new detection.
            correct(tracks(trackIdx).kalmanFilter, centroid);
            
            % Replace predicted bounding box with detected
            % bounding box.
            tracks(trackIdx).bbox = bbox;
            
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
        
        invisibleForTooLong = 10;
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
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);
        
        for i = 1:size(centroids, 1)
            
            centroid = centroids(i,:);
            bbox = bboxes(i, :);
            
            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [5, 5], [10, 10], 100);
            
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