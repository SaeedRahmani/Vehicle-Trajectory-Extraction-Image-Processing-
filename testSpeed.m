clear;
load POS_REALPOS_SPEED_FILTERS
load allChangedLane
vid=VideoReader('D:\....MOV');
vout= VideoWriter('D:\out.avi');
frn = 0;
open(vout);
while(1) 
    
    im=readFrame(vid);
    im = imrotate(im,90);
    
    frn = frn+1;
    if(frn > max(Pos(:,1)) || frn >1000)
        vout.close();
        break;
    end
    boxes = Pos(Pos(:,1) == frn,:);
    for i=1:size(boxes,1)
        changes = allChangedLane(allChangedLane(:,1) == boxes(i,2),:);
        if(isempty(changes) == false && sum(frn <changes(:,3) & frn > changes(:,2))>=1)
            im=insertObjectAnnotation(im,'rectangle',boxes(i,3:6),'Changing Lane','TextBoxOpacity',0.9,'FontSize',30,'Color','r');
        else
            im=insertObjectAnnotation(im,'rectangle',boxes(i,3:6),boxes(i,2),'TextBoxOpacity',0.9,'FontSize',30,'Color','r');
        end
    end
    im= imresize(im,[size(im,1) size(im,2)]*.5);
        
    writeVideo(vout,im);
    imshow(im)
    drawnow
end