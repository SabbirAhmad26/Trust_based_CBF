function  [car,CAV_e] = update_positions_twolane(i,car,frame1,CAV_e,trajs)
withid = 1;
axis(frame1);
len = length(car.que1);

for k = 1 : len


    origin = car.que1{k}.lane;
    action = car.que1{k}.id(1);
    distance = CAV_e.pos(i,car.que1{k}.id(2));
    distancecurrent = car.que1{k}.state(1);
    [xinfo,yinfo,angleinfo,j] = getXY(trajs,origin,action,distancecurrent,distance,car.que1{k}.jindex,car.que1{k}.realpose,car.que1{k}.prerealpose);
    car.que1{k}.jindex = j;
    CAV_e.posx(i,car.que1{k}.id(2)) = xinfo;
    CAV_e.posy(i,car.que1{k}.id(2)) = yinfo;
    car.que1{k}.prerealpose=car.que1{k}.realpose;
    car.que1{k}.realpose(1) = xinfo;
    car.que1{k}.realpose(2) = yinfo;
    CAV_e.angle(i,car.que1{k}.id(2)) = angleinfo;

    set(car.que1{k}.p,'XData',car.que1{k}.realpose(1),'YData', car.que1{k}.realpose(2));

    if withid == 1          
        set(car.que1{k}.txt, 'position', car.que1{k}.realpose);
    end

%     [car.que1{k}.state(1) - CAV_e.pos(i,car.que1{k}.id(2)),car.que1{k}.realpose(1) - car.que1{k}.prerealpose(1)]
    

end



drawnow
sti = 1;