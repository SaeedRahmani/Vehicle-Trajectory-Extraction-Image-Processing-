Pos = dlmread('detailResultsFull.txt');
Pos(Pos(:,3) > 1080,:) = [];
Pos(Pos(:,4) > 1920,:) = [];
%vid=VideoReader('D:\DSC_6557.MOV');
frn = 0;
% p1 = [195 ; 883 ; 1];
% p2 = [750 ; 868 ; 1];
% p3 = [997 ; 1697 ; 1]; 
% p4 = [83 ; 1757 ; 1];
% 
% q1 = [0;0;1];
% q2 = [15;0;1];
% q3 = [15; 32;1];
% q4 = [0 ; 32;1];

p1 = [347 ; 331 ; 1];
p2 = [703 ; 320 ; 1];
p3 = [997 ; 1697 ; 1]; 
p4 = [83 ; 1757 ; 1];

q1 = [0;0;1];
q2 = [15;0;1];
q3 = [15; 101;1];
q4 = [0 ; 101;1];


B = [q1 q2 q3 q4]; 
A = [p1 p2 p3 p4];

N = [A(1,1) A(2,1) A(3,1)      0      0      0      0      0      0 -B(1,1)       0       0       0; ...
         0      0      0 A(1,1) A(2,1) A(3,1)      0      0      0 -B(2,1)       0       0       0; ...
         0      0      0      0      0      0 A(1,1) A(2,1) A(3,1) -B(3,1)       0       0       0; ...
    A(1,2) A(2,2) A(3,2)      0      0      0      0      0      0       0 -B(1,2)       0       0; ...
         0      0      0 A(1,2) A(2,2) A(3,2)      0      0      0       0 -B(2,2)       0       0; ...
         0      0      0      0      0      0 A(1,2) A(2,2) A(3,2)       0 -B(3,2)       0       0; ...
    A(1,3) A(2,3) A(3,3)      0      0      0      0      0      0       0       0 -B(1,3)       0; ...
         0      0      0 A(1,3) A(2,3) A(3,3)      0      0      0       0       0 -B(2,3)       0; ...
         0      0      0      0      0      0 A(1,3) A(2,3) A(3,3)       0       0 -B(3,3)       0;...
    A(1,4) A(2,4) A(3,4)      0      0      0      0      0      0       0       0       0 -B(1,4);...
         0      0      0 A(1,4) A(2,4) A(3,4)      0      0      0       0       0       0 -B(2,4);...
         0      0      0      0      0      0 A(1,4) A(2,4) A(3,4)       0       0       0 -B(3,4);...
         0      0      0      0      0      0      0      0      1       0       0       0       0 ...
];

y = [ 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 1 ];

x = N\y;

P = reshape(x(1:9),3,3);
P = P';
for id= unique(Pos(:,2))'
Pos(Pos(:,2) == id,3) = smooth(Pos(Pos(:,2) == id,3),30);
Pos(Pos(:,2) == id,4) = smooth(Pos(Pos(:,2) == id,4),30);
Pos(Pos(:,2) == id,5) = smooth(Pos(Pos(:,2) == id,5),30);
Pos(Pos(:,2) == id,6) = smooth(Pos(Pos(:,2) == id,6),30);
end
Pos = [Pos zeros(size(Pos,1),2)]; 

while(1) 
      % if frn > 30 * vid.FrameRate , break; end   
    %im=readFrame(vid);
    %im = imrotate(im,90);
    %imwrite(im,'test.jpg');
    frn = frn+1;
    if(frn > max(Pos(:,1)))
        break;
    end
    boxes = Pos(Pos(:,1) == frn,:);
    centers = [boxes(:,3)+boxes(:,5)/2 boxes(:,4)+boxes(:,6) ones(size(boxes,1),1)];
    realP = P*centers';
    realP = realP./ repmat(realP(3,:),3,1);
    realP = realP';
    Pos(Pos(:,1) == frn,7:8) = realP(:,1:2);
   % im=insertObjectAnnotation(im,'rectangle',boxes(:,3:end),ones(size(boxes,1),1),'TextBoxOpacity',0.1,'FontSize',8,'Color','r');
   % imshow(im)
   % drawnow
end



Pos = [Pos zeros(size(Pos,1),1)]; 
id = 1;
while(1)
    if(id > max(Pos(:,2)))
        break;
    end
    boxes = Pos(Pos(:,2) == id,:);
    if(~isempty(boxes))
        realP = boxes(:,7:8);
        Phat= [ 0 0; realP(1:end-1,:)];
        diff = realP - Phat;
        diff(1,:) = diff(2,:);
        speed = sqrt(diff(:,1).^2 + diff(:,2).^2);
        speed = speed*29*3.6;
        Pos(Pos(:,2) == id , 9) = speed;
    end
    id = id+1;
end

for id= unique(Pos(:,2))'
    Pos(Pos(:,2) == id,9) = smooth(Pos(Pos(:,2) == id,9),30);
end
Pos(Pos(:,3)>1080,:)=[];
Pos(Pos(:,4)>1920,:)=[];
save POS_REALPOS_SPEED Pos