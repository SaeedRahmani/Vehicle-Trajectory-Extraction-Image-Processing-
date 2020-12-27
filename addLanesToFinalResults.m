load POS_REALPOS_SPEED_FILTERS;
load pespective
l =cell(4,1);
load('Lane1.mat');
l{1}= lane3;
load('Lane2.mat');
l{2}= lane3;
load('Lane3.mat');
l{3}= lane3;
load('Lane4.mat');
l{4}= lane3;
for i=1:4
    T = P*[l{i}.poly ones(size(l{i}.poly,1),1)]';
    T= T./ repmat(T(3,:),3,1);
    l{i}.poly = T(1:2,:)';
end
%changedIDs =[];
Threshold =.4;

Pos = [Pos zeros(size(Pos,1),1)];
%allChangedLane = [];
for id=1:max(Pos(:,2))
    changeFrames = [];
    x=Pos(Pos(:,2)==id,3)+Pos(Pos(:,2)==id,5)/2;
    y=Pos(Pos(:,2)==id,4)+Pos(Pos(:,2)==id,6)/2;
    realPosition = P * [x y ones(length(x),1)]';
    realPosition= realPosition./ repmat(realPosition(3,:),3,1);
    x= realPosition(1,:)';
    y= realPosition(2,:)';
    frNumber = Pos(Pos(:,2)==id,1);
    lane1 = inpolygon(x,y,l{1}.poly(:,1),l{1}.poly(:,2));
    lane2 = inpolygon(x,y,l{2}.poly(:,1),l{2}.poly(:,2));
    lane3 = inpolygon(x,y,l{3}.poly(:,1),l{3}.poly(:,2));
    lane4 = inpolygon(x,y,l{4}.poly(:,1),l{4}.poly(:,2));
    
    [rowID,laneNumber] = find([lane1 lane2 lane3 lane4]==1);
    [rowID,in] = sort(rowID);
    laneNumber = laneNumber(in);
    laneNumber = smooth(laneNumber,51);
    laneNumber(laneNumber>3.5) = 4;
    laneNumber(laneNumber<=3.5 & laneNumber >2.5) = 3;
    laneNumber(laneNumber<=2.5 & laneNumber >=1.5) = 2;
    laneNumber(laneNumber<1.5) = 1;
    laneNumber = medfilt1(laneNumber,11);
    allIndexes = find(Pos(:,2)==id);
    Pos(allIndexes(rowID),end) = laneNumber;
    
    
    
end


save POS_REALPOS_SPEED_FILTERS_LANE Pos;



