function [flag_i,flag_i1,flag_ip,flag_ip_seen,flag_i_trust] = Event_detector(ego, que1, ip, ip_seen, index, CAV_e)
global s1 s2 s3 const1 const2 const3 const4

    noise_term_position = max(abs(const1 * const2),abs(const1*(1-const2)));
    noise_term_speed = max(abs(const3 * const4),abs(const3*(1-const4)));


    flag_i=0;
    flag_i1=0;
    flag_ip=0;
    flag_ip_seen = 0;
    flag_i_trust = 0;


    if( ego.state(2) >= CAV_e.v_tk{ego.id(2),1}(1) + s1-noise_term_speed ||  ego.state(2) <= CAV_e.v_tk{ego.id(2),1}(1) - s1+noise_term_speed || ...
         ego.state(1) >= CAV_e.x_tk{ego.id(2),1}(1) + s2-noise_term_position ||  ego.state(1) <=CAV_e.x_tk{ego.id(2),1}(1) - s2+noise_term_position)
        flag_i=1;
    end    
    for  k=1 : length(ip)  sssssss
        if(que1{ip(k)}.state(2) >= CAV_e.v_tk{ego.id(2),2}(k)+s1-noise_term_speed ||  que1{ip(k)}.state(2) <= CAV_e.v_tk{ego.id(2),2}(k)-s1+noise_term_speed || ...
            que1{ip(k)}.state(1) >= CAV_e.x_tk{ego.id(2),2}(k)+s2-noise_term_position ||  que1{ip(k)}.state(1) <= CAV_e.x_tk{ego.id(2),2}(k)-s2+noise_term_position)
           flag_ip=1;
        end
        
        if ( que1{ip(k)}.trust(1) >=  que1{ip(k)}.trust(2) + s3 ||  que1{ip(k)}.trust(1) <=  que1{ip(k)}.trust(2) - s3 )
            flag_i_trust=1;
        end
    end
for k = 1 : length(index)
        for j = 1:1:length(index{k})
        if (index{k}(j) == -1)
            continue;
        else
            if(  que1{index{k}(j)}.state(2) >= CAV_e.v_tk{ego.id(2),3 + k -1}(j)+s1-noise_term_speed ||  que1{index{k}(j)}.state(2) <= CAV_e.v_tk{ego.id(2),3 + k -1}(j)-s1+noise_term_speed || ...
                 que1{index{k}(j)}.state(1) >= CAV_e.x_tk{ego.id(2),3 + k -1}(j)+s2-noise_term_position ||  que1{index{k}(j)}.state(1) <= CAV_e.x_tk{ego.id(2),3 + k -1}(j)-s2+noise_term_position)
                flag_i1=1;
            end
            if ( que1{index{k}(j)}.trust(1) >=  que1{index{k}(j)}.trust(2)  + s3 ||  que1{index{k}(j)}.trust(1) <=  que1{index{k}(j)}.trust(2) - s3)
                flag_i_trust=1;
            end
        
        end
    end

end
    if(numel(ip_seen))
       if( que1{ip_seen}.state(2) >= CAV_e.v_tk{ego.id(2),8}(1)+s1-noise_term_speed || que1{ip_seen}.state(2) <= CAV_e.v_tk{ego.id(2),8}(1)-s1+noise_term_speed || ...
           que1{ip_seen}.state(1) >= CAV_e.x_tk{ego.id(2),8}(1)+s2-noise_term_position || que1{ip_seen}.state(1) <= CAV_e.x_tk{ego.id(2),8}(1)-s2+noise_term_position)
           flag_ip_seen=1;
       end
    end
end