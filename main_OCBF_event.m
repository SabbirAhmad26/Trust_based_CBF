function [car,CAV_e] = main_OCBF_event(i, car, mod ,trust_thereshold, MultipleConstraints, CAV_e)

global computationTime1 dt s1 s2 s3;

order = zeros(length(car.table), 1);
for j = 1 : length(car.table)
    order(j) = car.table{j}(31);
end

len = length(order);
for k = 1 : 1 : len
    flag_i=0;
    flag_i1=0;
    flag_ip=0;
    flag_i_trust = 0;

    CAV_e.vel(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(2);
    CAV_e.acc(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(3);
    CAV_e.pos(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(1);

    [ip, index, position] = search_for_conflictCAVS(car.table, car.que1{order(k)});
    % [ip, index, position] = search_for_conflictCAVS_trustversion(car.que1,car.table, car.que1{order(k)},MultipleConstraints,trust_thereshold);
    car.que1{order(k)}.see = Vision(car.que1,car.que1{order(k)},order(k));

    
    % for lane change
    % ilc = search_i_lanechange(car.que1, car.que1{k});
    
    %if lane change is disallowed
    ilc = [];
    
    tic

    if(car.que1{order(k)}.state(2) >= CAV_e.v_tk(car.que1{order(k)}.id(2),1) + s1-1 || car.que1{order(k)}.state(2) <= CAV_e.v_tk(car.que1{order(k)}.id(2),1) - s1+1 || ...
        car.que1{order(k)}.state(1) >= CAV_e.x_tk(car.que1{order(k)}.id(2),1) + s2-1 || car.que1{order(k)}.state(1) <=CAV_e.x_tk(car.que1{order(k)}.id(2),1) - s2+1)
        flag_i=1;
    end
    if(ip ~= -1)
       if( car.que1{ip}.state(2) >= CAV_e.v_tk(car.que1{order(k)}.id(2),2)+s1-1 || car.que1{ip}.state(2) <= CAV_e.v_tk(car.que1{order(k)}.id(2),2)-s1+1 || ...
           car.que1{ip}.state(1) >= CAV_e.x_tk(car.que1{order(k)}.id(2),2)+s2-1 || car.que1{ip}.state(1) <= CAV_e.x_tk(car.que1{order(k)}.id(2),2)-s2+1)
           flag_ip=1;
       end

       if (car.que1{ip}.trust(1) >= car.que1{ip}.trust(2) + s3 || car.que1{ip}.trust(1) <= car.que1{ip}.trust(2) - s3 )
            flag_i_trust=1;
       end
    for kk = 1 : length(index)
        if (index(kk) == -1)
            continue;
        else
       if( car.que1{index(kk)}.state(2) >= CAV_e.v_tk(car.que1{order(k)}.id(2),3 + kk - 1)+s1-1 || car.que1{index(kk)}.state(2) <= CAV_e.v_tk(car.que1{order(k)}.id(2),3 + kk - 1)-s1+1 || ...
           car.que1{index(kk)}.state(1) >= CAV_e.x_tk(car.que1{order(k)}.id(2),3 + kk - 1)+s2-1 || car.que1{index(kk)}.state(1) <= CAV_e.x_tk(car.que1{order(k)}.id(2),3 + kk - 1)-s2+1)
           flag_i1=1;
       end
       if (car.que1{index(kk)}.trust(1) >= car.que1{index(kk)}.trust(2)  + s3 || car.que1{index(kk)}.trust(1) <= car.que1{index(kk)}.trust(2) - s3)
           flag_i_trust=1;
        end

       end
        end
    end

    if(flag_i==1) || (flag_i1==1) || (flag_ip==1) || (car.que1{order(k)}.state(1)==0)
        if i == 80
            stop = 1
        end
        i
            CAV_e.x_tk(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.state(1);
            CAV_e.v_tk(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.state(2);
            if(ip ~= -1)
                CAV_e.x_tk(car.que1{order(k)}.id(2),2)=car.que1{ip}.state(1);
                CAV_e.v_tk(car.que1{order(k)}.id(2),2)=car.que1{ip}.state(2);
            end
            for kk = 1 : length(index)
                if (index(kk) == -1)
                    continue;
                else
                    CAV_e.x_tk(car.que1{order(k)}.id(2),3 + kk - 1)=car.que1{index(kk)}.state(1);
                    CAV_e.v_tk(car.que1{order(k)}.id(2),3 + kk - 1)=car.que1{index(kk)}.state(2);
                end
            end
   


        [rt,c1_org,c1_CBF,c2_org,c2_CBF] = OCBF_event(i, car.que1{order(k)}, car.que1, ip, index, position, ilc);

        record = toc;
        computationTime1 = [computationTime1, record];

        if(ip ~= -1)
            CAV_e.rear_end(i,car.que1{k}.id(2))=c1_org;
            CAV_e.rear_end_CBF(i,car.que1{k}.id(2))=c1_CBF;
        end

        CAV_e.buffer(car.que1{order(k)}.id(2),:)=rt;
    else
          u=car.que1{order(k)}.state(3);
          t=[0 dt];
          x0(1)=car.que1{order(k)}.state(1);
          x0(2)=car.que1{order(k)}.state(2);
          x0(3)=car.que1{order(k)}.state(3);
            if(ip ~= -1)
              CAV_e.rear_end(i,car.que1{order(k)}.id(2))=car.que1{ip}.state(1)-x0(1)-1.8*x0(2);
              CAV_e.rear_end_CBF(i,car.que1{order(k)}.id(2))= car.que1{ip}.state(2) - x0(2) -1.8*x0(3) + car.que1{ip}.state(1) - x0(1) - 1.8 * x0(2);
            end

            % if(numel(i1))
            %     bigPhi = 1.8 * x0(1) / L;
            %     CAV_e.lateral_CBF(i,car.que1{k}.id(1))=car.que1{i1}.state(2) - x0(2) - 1.8*x0(2)*x0(2)/L - bigPhi*x0(3) + car.que1{i1}.state(1) - x0(1) - bigPhi*x0(2);
            %     CAV_e.lateral(i,car.que1{k}.id(1))=car.que1{i1}.state(1) - x0(1) - bigPhi*x0(2);
            % end            
            [~,xx]=ode45('second_order_model',t,x0(1:2));
            rt = [xx(end, 1), xx(end, 2),car.que1{k}.state(3)];
            CAV_e.buffer(car.que1{order(k)}.id(2),:)=rt;
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
        CAV_e.time(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.metric(1);
        CAV_e.fuel(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.metric(2);
        CAV_e.energy(car.que1{order(k)}.id(2),1)=car.que1{order(k)}.metric(3);
    end
            

end
for k=1:1:len
    car.que1{order(k)}.prestate = car.que1{order(k)}.state;
    car.que1{order(k)}.state=CAV_e.buffer(car.que1{order(k)}.id(2),:);
end

for k = 1:1:len
        
     if(car.que1{order(k)}.state(1) <= car.que1{order(k)}.metric(end))

      [car.que1{order(k)},Score,Trust] = TrustCal(i,car.que1,car.table,car.que1{order(k)},order(k),CAV_e,trust_thereshold);
      car.que1{order(k)}.trust(2) = car.que1{order(k)}.trust(1);
      car.que1{order(k)}.trust(1) = Trust;
      CAV_e.Trust(i,car.que1{order(k)}.id(2)) = car.que1{order(k)}.trust(1);
      car.que1{order(k)}.scores = Score;
      car.que1{order(k)}.p.MarkerEdgeColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];
      car.que1{order(k)}.p.MarkerFaceColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];

    end



end
end