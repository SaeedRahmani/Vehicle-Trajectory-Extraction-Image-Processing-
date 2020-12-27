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
for i=1:5
    T = P*[l{i}.poly ones(size(l{i}.poly,1),1)]';
    T= T./ repmat(T(3,:),3,1);
    l{i}.poly = T(1:2,:)';
end
Stat_Saeed = [];
Stat_Saeed1 = [];
Stat_Saeed2 = [];

Threshold = .6;
allChangedLane=[];
for id = unique(Pos(:,2))'
    try
    laneNumber = Pos(Pos(:,2) == id, 10);
    
    rowID = find(laneNumber ~=0);
    laneNumber(laneNumber==0)=[];
    x=Pos(Pos(:,2)==id,3)+Pos(Pos(:,2)==id,5)/2;
    y=Pos(Pos(:,2)==id,4)+Pos(Pos(:,2)==id,6)/2;
    realPosition = P * [x y ones(length(x),1)]';
    realPosition= realPosition./ repmat(realPosition(3,:),3,1);
    x= realPosition(1,:)';
    y= realPosition(2,:)';
    frNumber = Pos(Pos(:,2)==id,1);
    if(sum(abs(diff(laneNumber))) ~= 0)
        continue;
        changeFrame = find(diff(laneNumber)~=0);
        currentLane = laneNumber(changeFrame);
        nextLane = laneNumber(changeFrame+1);
        endLaneDistances =zeros(size(laneNumber,1),length(nextLane));
        startLaneDistances = endLaneDistances;
        for cnt=1:length(currentLane)
            for i=1:size(laneNumber,1)
                startLaneDistances(i,cnt) = cv.pointPolygonTest(l{currentLane(cnt)}.poly,[x(rowID(i)),y(rowID(i))],'MeasureDist',1);
                endLaneDistances(i,cnt) = cv.pointPolygonTest(l{nextLane(cnt)}.poly,[x(rowID(i)),y(rowID(i))],'MeasureDist',1);
            end
            
            
            startLaneDistances(startLaneDistances<0) =0;
            %endLaneDistances(endLaneDistances<0)=-endLaneDistances(endLaneDistances<0);
            plot([endLaneDistances]);
            PrimeEndLaneDistances = diff(smooth(endLaneDistances(:,cnt),30))*32;
            endingFrameNumber =0;
            startingFrameNumber=0;
            for i = changeFrame(cnt)+1 : size(laneNumber,1)
                if(endLaneDistances(i,cnt) > Threshold)
                    endingFrameNumber = frNumber(rowID(i));
                    break;
                end
            end
            
            for i = changeFrame(cnt):-1: 1
                if(abs(PrimeEndLaneDistances(i)) <= 0.1)
                    startingFrameNumber = frNumber(rowID(i));
                    break;
                end
            end
            
            
%             if( startingFrameNumber > endingFrameNumber)
%                 temp = startingFrameNumber;
%                 startingFrameNumber = endingFrameNumber;
%                 endingFrameNumber = temp;
%             end
%             
            if(startingFrameNumber ~=0 )
                allCars=Pos(Pos(:,1)== startingFrameNumber, :);
                meanSpeed = zeros(1,4);
                numOfCars = zeros(1,4);
                for z = 1:4
                    numOfCars(z) = size(allCars(allCars(:,10) == z,:), 1);
                    meanSpeed(z) = mean(allCars(allCars(:,10) == z,9));
                end
                
                nextLaneCars = allCars(allCars(:,10) == nextLane(cnt),:);
                currentLaneCars = allCars(allCars(:,10) == currentLane(cnt),:);
                car = currentLaneCars(currentLaneCars(:,2)== id,:);
                currentLaneCars(currentLaneCars(:,2)== id,:)= [];
                nextYs = nextLaneCars(:,8);
                currentYs = currentLaneCars(:,8);
                 
                
                relYN = nextYs-car(8);
                relYC =currentYs-car(8);
                forward_next = relYN(relYN>0);
                back_next= relYN(relYN<0);
                forward_current = relYC(relYC>0);
                back_current= relYC(relYC<0);
                
                car_near_forward_next =[];
                car_near_back_next =[];
                car_near_forward_current=[];
                car_near_back_current=[];
                v1 = -1;
                v2 = -1;
                v3 =-1;
                v4 = -1;
                dist1 = -1;
                dist2 =-1;
                dist3=-1;
                dist4=-1;
                id1 = -1;
                id2 = -1;
                id3 = -1;
                id4 =-1;
                near_forward_next = min(forward_next)+car(8);
                if(~isempty(near_forward_next)) 
                    car_near_forward_next = nextLaneCars(abs(nextLaneCars(:,8)-near_forward_next)<.01,:);
                    dist4 = norm(car_near_forward_next(7:8)-car(7:8));
                    v4 = car_near_forward_next(9);
                    id4 = car_near_forward_next(2);
                end;
                
                
                near_back_next = max(back_next)+car(8);
                if(isempty(near_back_next) == false) 
                    car_near_back_next = nextLaneCars(abs(nextLaneCars(:,8)-near_back_next)<0.01,:);
                    dist3 = norm(car_near_back_next(7:8)-car(7:8));
                    v3 = car_near_back_next(9);
                    id3 = car_near_back_next(2);
                end
                
                near_forward_current = min(forward_current)+car(8);
                if(~isempty(near_forward_current))
                    car_near_forward_current = currentLaneCars(abs(currentLaneCars(:,8)-near_forward_current)<0.01,:);
                    dist2 = norm(car_near_forward_current(7:8)-car(7:8));
                    v2 = car_near_forward_current(9);
                    id2 = car_near_forward_current(2);
                end
                
                near_back_current = max(back_current)+car(8);
                if(~isempty(near_back_current))
                    car_near_back_current = currentLaneCars(abs(currentLaneCars(:,8)-near_back_current)<0.01,:);
                    dist1 = norm(car_near_back_current(7:8)-car(7:8));
                    v1 = car_near_back_current(9);
                    id1 =  car_near_back_current(2);
                end
                
                
                
                
                
                v = car(9);
                
                car_speeds = Pos(Pos(:,1) >= startingFrameNumber - 3*29 & Pos(:,1) < endingFrameNumber  & Pos(:,2)==id , 9);
              
                
                lefToRight = nextLane(cnt) > currentLane(cnt);
                if(endingFrameNumber > 0)
                    duration = (endingFrameNumber - startingFrameNumber)/29;
                else
                    duration=-1;
                end
                Stat_Saeed = [Stat_Saeed ;id v duration id2 dist2 v2 id1 dist1 v1 id4 dist4 v4 id3 dist3 v3 numOfCars lefToRight currentLane(cnt) nextLane(cnt) meanSpeed];
                dlmwrite('speeds_ali.txt', [id car_speeds'],'-append');
                
            end
            allChangedLane = [allChangedLane ; id startingFrameNumber endingFrameNumber];
            
        end
    else
        laneNumber = unique(laneNumber);
        if(isempty(laneNumber) == false)
        for fr=frNumber'
            allCars = Pos(Pos(:,1)==fr,:);
            laneCars = allCars(allCars(:,10) == laneNumber,:);
            car = allCars(allCars(:,2)==id,:);
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
                    a1 = (car(9) - previous_car(9))*29;
                else
                    a1= -1;
                end
                
                if(isempty(previous_forward_car) == false)
                    a2 = (forward_car(9) - previous_forward_car(9))*29;
                else
                    a2= -1;
                end
                
                marginDistance1 = -cv.pointPolygonTest(l{laneNumber+1}.poly,car(7:8),'MeasureDist',1);
                marginDistance2 = -cv.pointPolygonTest(l{laneNumber+1}.poly,forward_car(7:8),'MeasureDist',1);
                Stat_Saeed1 = [Stat_Saeed1;fr id forward_car(2) distance forward_car(9) laneNumber a1 a2 marginDistance1 marginDistance2 car(9)];
            else
                forward_car=[];
            end
            
        end
        
        end
        
    end
    catch
        continue;
   end
    clc
    fprintf('%.2f',id*100/max(Pos(:,2)));
end
for fr=1 : max(Pos(:,1))
    allCars = Pos(Pos(:,1) == fr,:);
    Stat_Saeed2 = [Stat_Saeed2; fr size(allCars,1) mean(allCars(:,9))];
    
end

dlmwrite('ehsan1.txt',Stat_Saeed1);
dlmwrite('ehsan2.txt',Stat_Saeed2);
%dlmwrite('ali1.txt',Stat_Saeed);
%save allChangedLane allChangedLane
