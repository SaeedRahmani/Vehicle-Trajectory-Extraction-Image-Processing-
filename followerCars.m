load POS_REALPOS_SPEED_FILTERS_LANE
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
load('Lane5.mat');
l{5}= lane3;

Stat = [];
for i=1:5
    T = P*[l{i}.poly ones(size(l{i}.poly,1),1)]';
    T= T./ repmat(T(3,:),3,1);
    l{i}.poly = T(1:2,:)';
end


for i=1:size(Pos,1)
    car = Pos(i,:);
    fr = car(1);
    id = car(2);
    laneNumber = car(10);
    laneCars = Pos(Pos(:,10) == laneNumber & Pos(:,1) == fr,:);
    laneCarsYs = laneCars(:,8);
    rel = laneCarsYs - car(8);
    rel(rel<=0) = 1000;
    [minDistance,ind] = min(rel);
    if(minDistance~=1000)
        forward_car = laneCars(ind ,:);
        distance = norm(forward_car(7:8) - car(7:8));
        previous_car =  Pos(Pos(:,1)==fr-1 & Pos(:,2) ==id,:);
        previous_forward_car = Pos(Pos(:,1)==fr-1 & Pos(:,2) ==forward_car(2),:);
        if(isempty(previous_car) == false)
            a1 = (car(9) - previous_car(9))*29/3.6;
        else
            a1= -1;
        end
        
        if(isempty(previous_forward_car) == false)
            a2 = (forward_car(9) - previous_forward_car(9))*29/3.6;
        else
            a2= -1;
        end
        
        marginDistance1 = -cv.pointPolygonTest(l{laneNumber+1}.poly,car(7:8),'MeasureDist',1);
        marginDistance2 = -cv.pointPolygonTest(l{laneNumber+1}.poly,forward_car(7:8),'MeasureDist',1);
        Stat = [Stat;fr id forward_car(2) distance forward_car(9) laneNumber a1 a2 marginDistance1 marginDistance2 car(9)];
    end    
end
