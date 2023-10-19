function [car, metric] = main_CBF_update(car, metric, lg, index, sign)

    car.Car_leaves = car.Car_leaves + 1;
    
    metric.Ave_time = ((car.Car_leaves - 1)*metric.Ave_time + car.que1{index}.metric(1))/car.Car_leaves;
    metric.Ave_eng =  ((car.Car_leaves - 1)*metric.Ave_eng + car.que1{index}.metric(2))/car.Car_leaves;
    metric.Ave_u2 =  ((car.Car_leaves - 1)*metric.Ave_u2 + car.que1{index}.metric(3))/car.Car_leaves;
    car.cars1 = car.cars1 - 1;
    car.que1(index) = [];
    
% if(sign == 1)
%     if(car.que1{index}.id(5) == 1 || car.que1{index}.id(5) == 2)
%         car.Car_leavesMain = car.Car_leavesMain + 1;
%         metric.Ave_u2Main = ((car.Car_leavesMain - 1)*metric.Ave_u2Main + car.que1{index}.metric(3))/car.Car_leavesMain;
%         metric.Ave_engMain = ((car.Car_leavesMain - 1)*metric.Ave_engMain + car.que1{index}.metric(2))/car.Car_leavesMain;
%         metric.Ave_timeMain = ((car.Car_leavesMain - 1)*metric.Ave_timeMain + car.que1{index}.metric(1))/car.Car_leavesMain;
%     else
%         car.Car_leavesMerg = car.Car_leavesMerg + 1;
%         metric.Ave_u2Merg = ((car.Car_leavesMerg - 1)*metric.Ave_u2Merg + car.que1{index}.metric(3))/car.Car_leavesMerg;
%         metric.Ave_engMerg = ((car.Car_leavesMerg - 1)*metric.Ave_engMerg + car.que1{index}.metric(2))/car.Car_leavesMerg;
%         metric.Ave_timeMerg = ((car.Car_leavesMerg - 1)*metric.Ave_timeMerg + car.que1{index}.metric(1))/car.Car_leavesMerg;
%     end
%     metric.Ave_time = ((car.Car_leaves - 1)*metric.Ave_time + car.que1{index}.metric(1))/car.Car_leaves;
%     metric.Ave_eng =  ((car.Car_leaves - 1)*metric.Ave_eng + car.que1{index}.metric(2))/car.Car_leaves;
%     metric.Ave_u2 =  ((car.Car_leaves - 1)*metric.Ave_u2 + car.que1{index}.metric(3))/car.Car_leaves;
%     car.cars1 = car.cars1 - 1;
%     car.que1(index) = [];
% else
%     if(car.que2{index}.id(5) == 1 || car.que2{index}.id(5) == 2)
%         car.Car_leavesMain = car.Car_leavesMain + 1;
%         metric.Ave_u2Main = ((car.Car_leavesMain - 1)*metric.Ave_u2Main + car.que2{index}.metric(3))/car.Car_leavesMain;
%         metric.Ave_engMain = ((car.Car_leavesMain - 1)*metric.Ave_engMain + car.que2{index}.metric(2))/car.Car_leavesMain;
%         metric.Ave_timeMain = ((car.Car_leavesMain - 1)*metric.Ave_timeMain + car.que2{index}.metric(1))/car.Car_leavesMain;
%     else
%         car.Car_leavesMerg = car.Car_leavesMerg + 1;
%         metric.Ave_u2Merg = ((car.Car_leavesMerg - 1)*metric.Ave_u2Merg + car.que2{index}.metric(3))/car.Car_leavesMerg;
%         metric.Ave_engMerg = ((car.Car_leavesMerg - 1)*metric.Ave_engMerg + car.que2{index}.metric(2))/car.Car_leavesMerg;
%         metric.Ave_timeMerg = ((car.Car_leavesMerg - 1)*metric.Ave_timeMerg + car.que2{index}.metric(1))/car.Car_leavesMerg;
%     end
%     metric.Ave_time = ((car.Car_leaves - 1)*metric.Ave_time + car.que2{index}.metric(1))/car.Car_leaves;
%     metric.Ave_eng =  ((car.Car_leaves - 1)*metric.Ave_eng + car.que2{index}.metric(2))/car.Car_leaves;
%     metric.Ave_u2 =  ((car.Car_leaves - 1)*metric.Ave_u2 + car.que2{index}.metric(3))/car.Car_leaves;
%     car.cars2 = car.cars2 - 1;
%     car.que2(index) = [];
% end
    t_val = num2str(metric.Ave_time);
    e_val = num2str(metric.Ave_eng);
    c_val = num2str(car.Car_leaves);
    u2_val = num2str(metric.Ave_u2);
    u2Main_val = num2str(metric.Ave_u2Main);
    u2Merg_val = num2str(metric.Ave_u2Merg);
    eMain_val = num2str(metric.Ave_engMain);
    eMerg_val = num2str(metric.Ave_engMerg);
    tMain_val = num2str(metric.Ave_timeMain);
    tMerg_val = num2str(metric.Ave_timeMerg);
    lg.time_legend.String = t_val;
    lg.u2_legend.String = u2_val;
    lg.energy_legend.String = e_val;
    lg.through_legend.String = c_val;
    lg.Mae_lengend.String = eMain_val;
    lg.Mee_lengend.String = eMerg_val;
    lg.Mau_lengend.String = u2Main_val;
    lg.Meu_lengend.String = u2Merg_val;
    lg.Mat_lengend.String = tMain_val;
    lg.Met_lengend.String = tMerg_val;
 
    car.cars = car.cars - 1;
    