function [car,CAV_t] = main_OCBF_time(i, car, mod ,trust_thereshold, MultipleConstraints, CAV_t)

global computationTime1 dt;

order = zeros(length(car.table), 1);
for j = 1 : length(car.table)
    order(j) = car.table{j}(31);
end

len = length(order);
for k = 1 : 1 : len
    
    CAV_t.vel(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(2);
    CAV_t.acc(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(3);
    CAV_t.pos(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(1);

    [ip, index, position] = search_for_conflictCAVS(car.table, car.que1{order(k)});
    % [ip, index, position] = search_for_conflictCAVS_trustversion(car.que1,car.table, car.que1{order(k)},MultipleConstraints,trust_thereshold);
    car.que1{order(k)}.see = Vision(car.que1,car.que1{order(k)},order(k));
    % for lane change
    % ilc = search_i_lanechange(car.que1, car.que1{k});
    
    %if lane change is disallowed
    ilc = [];
    
    tic
    if (mod == 1)
        [rt,c1_org,c1_CBF,c2_org,c2_CBF] = OCBF_time(i, car.que1{order(k)}, car.que1, ip, index, position, ilc);
    elseif (mod == 2)
        rt = OC(i, car.que1{order(k)}, car.que1, ip);
    end
    record = toc;
    computationTime1 = [computationTime1, record];

%     if(numel(i1))
%         CAV_t.lateral(i,car.que1{k}.id(1))=c2_org;
%         CAV_t.lateral_CBF(i,car.que1{k}.id(1))=c2_CBF;
%     end
    if(numel(ip))
        CAV_t.rear_end(i,car.que1{k}.id(1))=c1_org;
        CAV_t.rear_end_CBF(i,car.que1{k}.id(1))=c1_CBF;
    end

    CAV_t.buffer(car.que1{order(k)}.id(2),:)=rt;
%     car.que1{order(k)}.state = rt;

    if(rt(1) <= car.que1{order(k)}.metric(end))

       [car.que1{order(k)},Score,Trust] = TrustCal(i,car.que1,car.table,car.que1{order(k)},order(k),CAV_t,trust_thereshold);      
       car.que1{order(k)}.trust(2) = car.que1{order(k)}.trust(1);
       car.que1{order(k)}.trust(1) = Trust;
       
       Data.Trust(i,car.que1{order(k)}.id(2)) = car.que1{order(k)}.trust(1);
       car.que1{order(k)}.scores = Score;
       car.que1{order(k)}.p.MarkerEdgeColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];
       car.que1{order(k)}.p.MarkerFaceColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];

    end


    if(rt(1) <= car.que1{order(k)}.metric(end))
        car.que1{order(k)}.metric(1) = car.que1{order(k)}.metric(1) + dt;
        if(car.que1{k}.state(3)>= 0)
            car.que1{order(k)}.metric(2) = fuel_consumption(car.que1{order(k)}.metric(2),rt(3),rt(2));
        else
            car.que1{order(k)}.metric(2) = car.que1{order(k)}.metric(2);
        end
        car.que1{order(k)}.metric(3) = car.que1{order(k)}.metric(3) + dt*0.5*rt(3)^2;
    else
        CAV_t.time(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.metric(1);
        CAV_t.fuel(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.metric(2);
        CAV_t.energy(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.metric(3);
    end
            

end
for k=1:1:len
    car.que1{order(k)}.prestate = car.que1{order(k)}.state;
    car.que1{order(k)}.state=CAV_t.buffer(car.que1{order(k)}.id(2),:);
end
for k = 1:1:len
        
     if(car.que1{order(k)}.state(1) <= car.que1{order(k)}.metric(end))

      [car.que1{order(k)},Score,Trust] = TrustCal(i,car.que1,car.table,car.que1{order(k)},order(k),CAV_t,trust_thereshold);
      car.que1{order(k)}.trust(2) = car.que1{order(k)}.trust(1);
      car.que1{order(k)}.trust(1) = Trust;
      CAV_e.Trust(i,car.que1{order(k)}.id(2)) = car.que1{order(k)}.trust(1);
      car.que1{order(k)}.scores = Score;
      car.que1{order(k)}.p.MarkerEdgeColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];
      car.que1{order(k)}.p.MarkerFaceColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];

    end



end

end