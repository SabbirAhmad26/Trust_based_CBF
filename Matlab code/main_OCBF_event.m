function [car,CAV_e] = main_OCBF_event(i, car, mod ,trust_thereshold, MultipleConstraints, CAV_e,geometry_road)

global  dt s3 u;
L = geometry_road.L + 2*geometry_road.r + 4*geometry_road.w;
update_class_k_function = 0;
mitigation = 0;
trust =1;

order = car.order;
len = length(order);
for k = 1 : 1 : len
    car.que1{order(k)}.txt.String = num2str(k);

    CAV_e.vel(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(2);
    CAV_e.acc(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(3);
    CAV_e.pos(i,car.que1{order(k)}.id(2))=car.que1{order(k)}.state(1);

% 
%     if i == 25 
%         car.que1{1}.agent = 4;
% 
%     end
%     if i == 26
%         car.que1{2}.agent = 4;
%     end
%     if i == 27
%         car.que1{3}.agent = 4;
%     end
%     if i == 28
%         car.que1{4}.agent = 4;
%     end
%     if i == 29
%         car.que1{5}.agent = 4;
%     end
%     if i == 30
%         car.que1{6}.agent = 4;
%     end
% % % 
%     if i == 60
%         car.que1{1}.MUSTleave = 1;
%         car.que1{1}.p.MarkerFaceColor = [1 0 0.5];
%     end
%     if i == 68
%         car.que1{2}.MUSTleave = 1;
%         car.que1{2}.p.MarkerFaceColor = [1 0 0.5];
%     end
%     if i == 59
%     car.que1{3}.MUSTleave = 1;
%     car.que1{3}.p.MarkerFaceColor = [1 0 0.5];
%     end
%     if i == 70
%         car.que1{4}.MUSTleave = 1;
%         car.que1{4}.p.MarkerFaceColor = [1 0 0.5];
%     end
%     if i == 63
%         car.que1{5}.MUSTleave = 1;
%         car.que1{5}.p.MarkerFaceColor = [1 0 0.5];
%     end
%     if i == 75
%         car.que1{6}.MUSTleave = 1;
%         car.que1{6}.p.MarkerFaceColor = [1 0 0.5];
%     end



    if car.que1{order(k)}.agent == 1
        CAV_e.buffer(car.que1{order(k)}.id(2),:) = NoModelAttacker(i, car.que1{order(k)});
    elseif car.que1{order(k)}.agent == 2
        CAV_e.buffer(car.que1{order(k)}.id(2),:) = RandomInitAttacker(i, car.que1{order(k)});
    elseif car.que1{order(k)}.agent == 3
        [ip, index, position] = search_for_conflictCAVS(car.table, car.que1{order(k)});
        [CAV_e.buffer(car.que1{order(k)}.id(2),:),~,~,~] = NoRuleAttacker(i, car.que1{order(k)}, car.que1, ip, index, position); 
    elseif car.que1{order(k)}.agent == 4
        [ip, index, position] = search_for_conflictCAVS(car.table, car.que1{order(k)});
        [CAV_e.buffer(car.que1{order(k)}.id(2),:),~,~,~] = StrategicAttacker(i, car.que1{order(k)}, car.que1, ip, index, position,geometry_road);
    else

        [ip, index, position] = search_for_conflictCAVS_trustversion(car.que1,car.table, car.order, k, car.que1{order(k)},MultipleConstraints,trust_thereshold);
        
        car.que1{order(k)}.see = Vision(car.que1,car.que1{order(k)},order(k));
        ip_seen = [];
        for kkk = 1 : length(car.que1{order(k)}.see) 
            ind = car.que1{order(k)}.see(kkk);
            if car.que1{ind}.id(3) == car.que1{order(k)}.id(3)
                if car.que1{ind}.state(1) >= car.que1{order(k)}.state(1)
                    ip_seen = ind;
                end
            end
        end
        % [extended_ip, ~, ~] = Extended_search_for_conflictCAVS(car.table, car.que1{order(k)});
        % ip_seen = max(intersect(extended_ip,car.que1{order(k)}.see));
    
        ilc = [];
        
        [flag_i,flag_i1,flag_ip,flag_ip_seen,flag_i_trust] = Event_detector(car.que1{order(k)}, car.que1, ip, ip_seen, index,CAV_e);
        if flag_i || flag_i1 || flag_ip || flag_ip_seen || flag_i_trust || (car.que1{order(k)}.state(1)==0) ||  (car.que1{order(k)}.state(1) >= L) 
            
            CAV_e.x_tk{car.que1{order(k)}.id(2),1}(1)=car.que1{order(k)}.state(1);
            CAV_e.v_tk{car.que1{order(k)}.id(2),1}(1)=car.que1{order(k)}.state(2);

            for kk = 1 : length(ip) 
                CAV_e.x_tk{car.que1{order(k)}.id(2),2}(kk)=car.que1{ip(kk)}.state(1);
                CAV_e.v_tk{car.que1{order(k)}.id(2),2}(kk)=car.que1{ip(kk)}.state(2);
            end

            
            for kk = 1 : length(index)
                for j = 1:1:length(index{kk})
                    if (index{kk}(j) == -1)
                        continue;
                    else
                        CAV_e.x_tk{car.que1{order(k)}.id(2),3 + kk -1}(j)=car.que1{index{kk}(j)}.state(1);
                        CAV_e.v_tk{car.que1{order(k)}.id(2),3 + kk -1}(j)=car.que1{index{kk}(j)}.state(2);
                end
                end
            end

            if(numel(ip_seen))
                CAV_e.x_tk{car.que1{order(k)}.id(2),8}(1)=car.que1{ip_seen}.state(1);
                CAV_e.v_tk{car.que1{order(k)}.id(2),8}(1)=car.que1{ip_seen}.state(2);
            end

            [rt, CAV_e,infeasiblity] = OCBF_event(i, car.que1{order(k)}, car.que1, ip, index, position, ilc,CAV_e,geometry_road);
            CAV_e.counter = CAV_e.counter + 1;
            car.que1{order(k)}.infeasiblity = infeasiblity;
            CAV_e.buffer(car.que1{order(k)}.id(2),:)=rt;


        else
            u=car.que1{order(k)}.state(3);
            t=[0 dt];
            x0(1)=car.que1{order(k)}.state(1);
            x0(2)=car.que1{order(k)}.state(2);
            x0(3)=car.que1{order(k)}.state(3);
            [~,xx]=ode45('second_order_model',t,x0(1:2));
            rt = [xx(end, 1), xx(end, 2),car.que1{order(k)}.state(3)];
            if (rt(2) < 0)
                rt(2) = 0;
                rt(1) = car.que1{order(k)}.state(1);
            end

            if (rt(1) - car.que1{order(k)}.state(1) < 0)
                pause(1);
            end
            CAV_e.buffer(car.que1{order(k)}.id(2),:)=rt;
        end
    
        if(rt(1) <= car.que1{order(k)}.metric(4))
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
                   
end


for k=1:1:len
    car.que1{order(k)}.prestate = car.que1{order(k)}.state;
    car.que1{order(k)}.state=CAV_e.buffer(car.que1{order(k)}.id(2),:);
end


if trust == 1
    for k = 1:1:len        
        if(car.que1{order(k)}.state(1) <= car.que1{order(k)}.metric(4)) && ~car.que1{order(k)}.MUSTleave
            [car.que1{order(k)},Score,Trust] = TrustCal(i,car.que1,car.table,car.order,car.que1{order(k)},order(k),k,CAV_e,trust_thereshold);
            car.que1{order(k)}.trust(2) = car.que1{order(k)}.trust(1);
            car.que1{order(k)}.trust(1) = Trust;
            CAV_e.Trust(i,car.que1{order(k)}.id(2)) = car.que1{order(k)}.trust(1);
            car.que1{order(k)}.scores = Score;
            car.que1{order(k)}.p.MarkerEdgeColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];
            car.que1{order(k)}.p.MarkerFaceColor = [1-car.que1{order(k)}.trust(1) car.que1{order(k)}.trust(1) 0.5];
        end
        if update_class_k_function == 1
            [ip, index, position] = search_for_conflictCAVS(car.table, car.que1{order(k)});
            if(ip ~= -1)
                car.que1{order(k)}.k_rear = max(car.que1{ip}.trust(1)- s3, 0.01);
            end
            
            for kk = 1 : length(index)            
                if (index(kk) == -1)
                    continue;
                else
                car.que1{order(k)}.k_lateral(kk) = max(car.que1{index(kk)}.trust(1) - s3,0.01);
                end
                 
             end        
        end
    
        if  (i >= CAV_e.arrivalexit(car.que1{order(k)}.id(2),1)/dt+20) && (car.que1{order(k)}.trust(1) <= trust_thereshold.low) && (car.que1{order(k)}.trust(1) - car.que1{order(k)}.trust(2)<= 0.01)
            car.que1{order(k)}.Warning = car.que1{order(k)}.Warning +1;
        else
            car.que1{order(k)}.Warning = 0;
        end
    
        if car.que1{order(k)}.trust(1) <= trust_thereshold.low
            car.que1{order(k)}.NewWarning = car.que1{order(k)}.NewWarning +1;
        end
    
        
        if (car.que1{order(k)}.Warning >= 40 || car.que1{order(k)}.NewWarning >= 100) && mitigation       
            car.que1{order(k)}.MUSTleave = 1;
        end
    
    end
end

    if mitigation  
        [SpoofedCarsList,OrderSpoofedCarList,SpoofedCAVsSuccedingcarlist,car.que1] = FindSpoofedCars(car.que1,car.table,car.order);
        ignorethisround = [];
        temporder = [];
        for j =1:1:length(CAV_e.SpoofedCarsList)
            if CAV_e.SpoofedCAVsSuccedingcarlist(j)~=-1
                for jjj = 1: 1: length(car.que1)
                    if car.que1{jjj}.id(2) == CAV_e.SpoofedCAVsSuccedingcarlist(j)
                        suc_ind = jjj;
                        break
                    end
                end
                for jjj = 1: 1: length(car.que1)
                    if car.que1{jjj}.id(2) == CAV_e.SpoofedCarsList(j)
                        s_ind = jjj;
                        break
                    end
                end
                if car.que1{s_ind}.state(1) > car.que1{suc_ind}.state(1)
                    ignorethisround = [ignorethisround,s_ind];
                end
            end
        end

        for j =length(SpoofedCarsList):-1:1
            if SpoofedCAVsSuccedingcarlist(j)~=-1 
                ExplicitConstraintIndices = ExpImpConstraintFinder_Mitigation(car.que1,j,SpoofedCAVsSuccedingcarlist(j),SpoofedCarsList,car.order,car.table);
                ind1 = find(order == SpoofedCarsList(j));
                ind2 = find(order == SpoofedCAVsSuccedingcarlist(j));
                set1 = order(ind1+1:ind2-1);
                Sj = ExplicitConstraintIndices';
                set2 = order(ind2 + 1:OrderSpoofedCarList(j+1)-1);
                set2 = setxor(set2,intersect(set2,Sj));
                set2 = reshape(set2,1,length(set2));
                if (find(CAV_e.SpoofedCarsList == car.que1{SpoofedCarsList(j)}.id(2)))
                    indd = find(CAV_e.SpoofedCarsList == car.que1{SpoofedCarsList(j)}.id(2));
                    CAV_e.SpoofedCAVsSuccedingcarlist(indd) = car.que1{SpoofedCAVsSuccedingcarlist(j)}.id(2);
                else
                    CAV_e.SpoofedCarsList = [CAV_e.SpoofedCarsList(1:j-1),car.que1{SpoofedCarsList(j)}.id(2),CAV_e.SpoofedCarsList(j:end)];
                    CAV_e.SpoofedCAVsSuccedingcarlist = [CAV_e.SpoofedCAVsSuccedingcarlist(1:j-1),car.que1{SpoofedCAVsSuccedingcarlist(j)}.id(2),CAV_e.SpoofedCAVsSuccedingcarlist(j:end)];
                end
            else
                if (find(CAV_e.SpoofedCarsList == car.que1{SpoofedCarsList(j)}.id(2)))
                    indd = find(CAV_e.SpoofedCarsList == car.que1{SpoofedCarsList(j)}.id(2));
                    CAV_e.SpoofedCAVsSuccedingcarlist(indd) = -1;
                else
                    CAV_e.SpoofedCarsList = [CAV_e.SpoofedCarsList(1:j-1),car.que1{SpoofedCarsList(j)}.id(2),CAV_e.SpoofedCarsList(j:end)];
                    CAV_e.SpoofedCAVsSuccedingcarlist = [CAV_e.SpoofedCAVsSuccedingcarlist(1:j-1),-1,CAV_e.SpoofedCAVsSuccedingcarlist(j:end)];
                end
                ExplicitConstraintIndices = [];
                ind1 = find(order == SpoofedCarsList(j));
                set1 = order(ind1+1:OrderSpoofedCarList(j+1)-1);
                ind2 = [];
                set2 = [];
            end
            order = [order(1:ind1-1),set1,set2,order(ind2),order(ind1),ExplicitConstraintIndices,order(OrderSpoofedCarList(j+1):OrderSpoofedCarList(end)-1)];
            car.order = order;
            car = update_table(car);
        end
        car.order = order;
        car = update_table(car);
    end

%% Constraints data

for k=1:1:len
    if car.que1{order(k)}.id(2) == 19 && i == 421
        stop = 1;
    end
    if car.que1{order(k)}.state(1) <= car.que1{order(k)}.metric(4)
        [ip, index, position] = search_for_conflictCAVS_trustversion(car.que1,car.table,car.order,k, car.que1{order(k)},MultipleConstraints,trust_thereshold);
        % [ip, index, position] = search_for_conflictCAVS(car.table, car.que1{order(k)});

        %[ip, index, position] = search_for_conflictCAVS_py(table, egocar);
        one = car.que1{order(k)};
        x0 = one.state;
        deltaSafetyDistance = car.que1{order(k)}.carlength;
        for kk = 1:length(ip)            
            %k_rear = one.k_rear;
            phiRearEnd = one.phiRearEnd;
            car.que1{order(k)}.rearendconstraint(kk) = car.que1{ip(kk)}.state(1)-x0(1)-phiRearEnd*x0(2)- deltaSafetyDistance;
        end
        for kk = 1 : length(index)
            for j = 1:1:length(index{kk})
                if (index{kk}(j) == -1)
                    continue;
                else
                    %k_lateral = one.k_lateral(kk);
                    phiLateral = one.phiLateral;
                    %vl_tk = car.que1{index{kk}(j)}.state(2);
                    xl_tk = car.que1{index{kk}(j)}.state(1);
                    d1 = car.que1{index{kk}(j)}.metric(position{kk}(j)+4) - xl_tk;  % distance to the merging point
                    d2 = one.metric(kk+4) - x0(1); % distance to the merging point
                end
                bigPhi = phiLateral * x0(1) / one.metric(kk+4);
                car.que1{order(k)}.lateralconstraint{kk}(j) = d2 - d1 - bigPhi*(x0(2) + deltaSafetyDistance);
           end
        end
    end
end




end