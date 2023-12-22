function [init_queue,total] = ScenarioBuilder(sc)
%SCENARIO_BUILDER Summary of this function goes here
%   Detailed explanation goes here
%[index, lane index, time ,speed, ? ,% action 1: go straight, 2: turn left, 3: turn right]

switch sc
    case 1 % RearEnd Accident
        % lateral Accident
        init_queue(1,:)=[1 1 1	10	0 1 0 0];
        [total, ~] = size(init_queue); 

    case 2 
        init_queue(1,:)=[1 1 1	10	0 1 0 0];
        init_queue(2,:)=[1 1 3	10	0 2 0 0];
        init_queue(3,:)=[1 1 5	10	0 3 0 0];

        init_queue(4,:)=[1 2 7	10	0 1 0 0];
        init_queue(5,:)=[1 2 9	10	0 2 0 0];
        init_queue(6,:)=[1 2 11	10	0 3 0 0];


        init_queue(7,:)=[1 3 13	10	0 1 0 0];
        init_queue(8,:)=[1 3 15	10	0 2 0 0];
        init_queue(9,:)=[1 3 17	10	0 3 0 0];

        init_queue(10,:)=[1 4 19	10	0 1 0 0];
        init_queue(11,:)=[1 4 21	10	0 2 0 0];
        init_queue(12,:)=[1 4 23	10	0 3 0 0];

        init_queue(13,:)=[1 5 25	10	0 1 0 0];
        init_queue(14,:)=[1 5 27	10	0 2 0 0];
        init_queue(15,:)=[1 5 29	10	0 3 0 0];


        init_queue(16,:)=[1 6 31	10	0 1 0 0];
        init_queue(17,:)=[1 6 33	10	0 2 0 0];
        init_queue(18,:)=[1 6 35	10	0 3 0 0];

        init_queue(19,:)=[1 7 37	10	0 1 0 0];
        init_queue(20,:)=[1 7 39	10	0 2 0 0];
        init_queue(21,:)=[1 7 41	10	0 3 0 0];

        init_queue(22,:)=[1 8 43	10	0 1 0 0];
        init_queue(23,:)=[1 8 45	10	0 2 0 0];
        init_queue(24,:)=[1 8 47	10	0 3 0 0];


        [total, ~] = size(init_queue); 

    case 3
        init_queue(1,:)=[1 1 1	10	0 2 0 0];

        init_queue(2,:)=[1 2 3	10	0 1 0 0];
        init_queue(3,:)=[1 2 5	10	0 3 0 0];


        init_queue(4,:)=[1 3 7	10	0 2 0 0];


        init_queue(5,:)=[1 4 11	10	0 1 0 0];
        init_queue(6,:)=[1 4 13	10	0 3 0 0];

        init_queue(7,:)=[1 5 15	10	0 2 0 0];

        init_queue(8,:)=[1 6 17	10	0 1 0 0];
        init_queue(9,:)=[1 6 19	10	0 3 0 0];

        init_queue(10,:)=[1 7 21	10	0 2 0 0];

        init_queue(11,:)=[1 8 23	10	0 1 0 0];
        init_queue(12,:)=[1 8 25	10	0 3 0 0];


        [total, ~] = size(init_queue); 


    case 4
        init_queue(1,:)=[1 1 1	10	0 2 0 0];

        init_queue(2,:)=[1 2 1.5	10	0 1 0 0];

        init_queue(3,:)=[1 3 2.5	10	0 2 0 0];

        init_queue(4,:)=[1 2 4	10	0 3 0 0];
        init_queue(5,:)=[1 6 4.5	10	0 1 0 0];
        init_queue(6,:)=[1 4 5	10	0 3 0 0];
        init_queue(7,:)=[1 8 5.5	10	0 1 0 0];
        init_queue(8,:)=[1 5 6	10	0 2 0 0];
        init_queue(9,:)=[1 3 7	10	0 2 0 0];
        init_queue(10,:)=[1 7 7.75	10	0 2 0 0];
         init_queue(11,:)=[1 5 8.5	10	0 2 0 0];

        init_queue(12,:)=[1 6 9.5 	10	0 1 0 0];

       

        init_queue(13,:)=[1 4 10	10	0 1 0 0];

        init_queue(14,:)=[1 4 11.5	10	0 3 0 0];

        init_queue(15,:)=[1 1 15	10	0 2 0 0];

        init_queue(16,:)=[1 2 16	10	0 1 0 0];      


        init_queue(17,:)=[1 7 18	10	0 2 0 0];

        init_queue(18,:)=[1 4 19	10	0 3 0 0];

        
        init_queue(19,:)=[1 5 19.5	10	0 2 0 0];

        init_queue(20,:)=[1 6 20	10	0 1 0 0];

        init_queue(21,:)=[1 1 21	10	0 2 0 0];

        init_queue(22,:)=[1 2 21.5	10	0 3 0 0];


        init_queue(23,:)=[1 3 35	10	0 2 0 0];
        init_queue(24,:)=[1 8 39	10	0 3 0 0];
        init_queue(25,:)=[1 4 42	10	0 3 0 0];

        [total, ~] = size(init_queue);      

    case 5
        init_queue(1,:)=[1 2 1	 5	0 3 0 0];
        init_queue(2,:)=[1 6 1.1 5	0 3 0 0];
        init_queue(3,:)=[1 4 1.2 5	0 3 0 0];
        init_queue(4,:)=[1 8 1.3 5	0 3 0 0];
        init_queue(5,:)=[1 1 1.4 5	0 2 0 0];
        init_queue(6,:)=[1 5 1.5 5	0 2 0 0];

        init_queue(7,:)=[1 2 4	 5	0 3 0 0];
        init_queue(8,:)=[1 6 4.1 5	0 3 0 0];
        init_queue(9,:)=[1 4 4.2 5	0 3 0 0];
        init_queue(10,:)=[1 8 4.3 5	0 3 0 0];
        init_queue(11,:)=[1 1 4.4 5	0 2 0 0];
        init_queue(12,:)=[1 5 4.5 5	0 2 0 0];

        init_queue(13,:)=[1 2 7.5  5	0 3 0 0];
        init_queue(14,:)=[1 6 7.6  5	0 3 0 0];
        init_queue(15,:)=[1 4 7.7 5	0 3 0 0];
        init_queue(16,:)=[1 8 7.8 5	0 3 0 0];
        init_queue(17,:)=[1 1 7.9 5	0 2 0 0];
        init_queue(18,:)=[1 5 8 5	0 2 0 0];


        init_queue(19,:)=[1 2 11	  5	0 3 0 0];
        init_queue(20,:)=[1 6 11.1 5	0 3 0 0];
        init_queue(21,:)=[1 4 11.2 5	0 3 0 0];
        init_queue(22,:)=[1 8 11.3 5	0 3 0 0];
        init_queue(23,:)=[1 1 11.4 5	0 2 0 0];
        init_queue(24,:)=[1 5 11.5 5	0 2 0 0];

        init_queue(25,:)=[1 2 16   5	0 3 0 0];
        init_queue(26,:)=[1 6 16.3 5	0 3 0 0];
        init_queue(27,:)=[1 4 16.5 5	0 3 0 0];
        init_queue(28,:)=[1 8 16.6 5	0 3 0 0];
        init_queue(29,:)=[1 1 16.7 5	0 2 0 0];
        init_queue(30,:)=[1 5 16.8 5	0 2 0 0];


        init_queue(31,:)=[1 2 22	2	0 3 0 0];
        init_queue(32,:)=[1 6 22.5  2	0 3 0 0];
        init_queue(33,:)=[1 4 22.7  2	0 3 0 0];
        init_queue(34,:)=[1 8 22.9 2	0 3 0 0];
        init_queue(35,:)=[1 1 23.1 2	0 2 0 0];
        init_queue(36,:)=[1 5 23.3 5	0 2 0 0];

        init_queue(37,:)=[1 2 35	2	0 3 0 0];
        init_queue(38,:)=[1 6 35.6  2	0 3 0 0];
        init_queue(39,:)=[1 4 36    2	0 3 0 0];
        init_queue(40,:)=[1 8 36.5 2	0 3 0 0];
        init_queue(41,:)=[1 1 36.8 2	0 2 0 0];
        init_queue(42,:)=[1 5 37 5	0 2 0 0];

        init_queue(43,:)=[1 2 47	2	0 3 0 0];
        init_queue(44,:)=[1 6 47.6  2	0 3 0 0];
        init_queue(45,:)=[1 4 48   2	0 3 0 0];
        init_queue(46,:)=[1 8 48.5 2	0 3 0 0];
        init_queue(47,:)=[1 1 49 2	0 2 0 0];
        init_queue(48,:)=[1 5 49.5 5	0 2 0 0];

        [total, ~] = size(init_queue);  
end






