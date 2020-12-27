load POS_REALPOS_SPEED
id=1;
while(1)
    if(id > max(Pos(:,2)))
        break;
    end
    boxes = Pos(Pos(:,2) == id,:);
    if(~isempty(boxes))
        if( (mean(boxes(:,9)) <30) || ((max(boxes(:,8))-min(boxes(:,8)))<35))
            Pos(Pos(:,2) == id,:)=[];
        end
    end
    id = id+1;
end

save POS_REALPOS_SPEED_FILTERS Pos;