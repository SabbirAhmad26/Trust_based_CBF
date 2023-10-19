function  car = update_positions_twolane(car ,frame1)

axis(frame1);
lanechangingLength = 7;
l = lanechangingLength - 3.5*sqrt(3);
len = length(car.que1);

L1 = 300;
w = 3.5;
r = 4;



for k = 1 : len
    if all(car.que1{k}.prerealpose == car.que1{k}.realpose)
        car.que1{k}.prerealpose = car.que1{k}.prerealpose;
    else
        car.que1{k}.prerealpose = car.que1{k}.realpose;
    end
    if (car.que1{k}.lane == 1)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1),L1+1.5*w+r];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)));
                car.que1{k}.realpose = [L1+0.5*w+r,L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end))];
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)) + L1+4*w+2*r); 
            car.que1{k}.realpose = [L1+2.5*w+r, car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)) + L1+4*w+2*r];
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
            car.que1{k}.realpose = [car.que1{k}.state(1),L1+1.5*w+r];
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1),L1+1.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = (2.5 * w + r);
                alpha = deltaX / R;
                xNew = L1 + R * sin(alpha);
                yNew = L1+4*w+2*r - R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1),L1+1.5*w+r];
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData',car.que1{k}.metric(5)- lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30),...
                            'YData',L1+1.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
                car.que1{k}.realpose = [car.que1{k}.metric(5)- lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30),...
                    L1+1.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30)];
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1)-l,'YData', L1+0.5*w+r);
                car.que1{k}.id(3) = 2;
                car.que1{k}.realpose = [car.que1{k}.state(1)-l,L1+0.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = 0.5*w + r;
                alpha = deltaX / R;
                xNew = L1 + R * sin(alpha);
                yNew = L1 + R * cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        end
    elseif (car.que1{k}.lane == 2)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 2)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1),L1+0.5*w+r];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)));
                car.que1{k}.realpose = [L1+0.5*w+r ,L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end))];
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l) + L1+4*w+2*r);
            car.que1{k}.realpose = [L1+2.5*w+r ,car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l) + L1+4*w+2*r];
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
            car.que1{k}.realpose = [car.que1{k}.state(1) ,L1+0.5*w+r];            
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1) ,L1+0.5*w+r];   
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData',car.que1{k}.metric(5)- lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30),...
                            'YData',L1+0.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
                        car.que1{k}.realpose = [car.que1{k}.metric(5)- lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30) ,...
                            L1+0.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30)];   
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1)-l,'YData', L1+1.5*w+r);
                car.que1{k}.id(3) = 1;
                car.que1{k}.realpose = [car.que1{k}.state(1)-l, L1+1.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = (2.5 * w + r);
                alpha = deltaX / R;
                xNew = L1 + R * sin(alpha);
                yNew = L1+4*w+2*r - R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew, yNew];
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1), L1+0.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = 0.5 * w + r;
                alpha = deltaX / R;
                xNew = L1 + R * sin(alpha);
                yNew = L1 + R * cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew, yNew];
            end
        end
    elseif (car.que1{k}.lane == 3)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100, -100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100, -100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+2.5*w+r, car.que1{k}.state(1)];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1) - car.que1{k}.metric(end) + L1+4*w+2*r,'YData', L1+0.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1) - car.que1{k}.metric(end) + L1+4*w+2*r,L1+0.5*w+r];
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))),'YData', L1+2.5*w+r);
            car.que1{k}.realpose = [L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))),L1+2.5*w+r];
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
            car.que1{k}.realpose = [L1+2.5*w+r,car.que1{k}.state(1)];
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+2.5*w+r,car.que1{k}.state(1)];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = 2.5*w + r;
                alpha = deltaX / R;
                yNew = L1 + R * sin(alpha);
                xNew = L1 + R * cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+2.5*w+r,car.que1{k}.state(1)];
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData', L1+2.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
                            'YData',car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30));
                        car.que1{k}.realpose = [L1+2.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
                            car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)];
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1)-l);
                car.que1{k}.realpose = [L1+3.5*w+r,car.que1{k}.state(1)-l];
                car.que1{k}.id(3) = 4;
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = 0.5*w + r;
                alpha = deltaX / R;
                yNew = L1 + R * sin(alpha);
                xNew = L1 + 4*w + 2*r - R * cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        end
    elseif (car.que1{k}.lane == 4)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 2)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+3.5*w+r,car.que1{k}.state(1)];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',car.que1{k}.state(1) - car.que1{k}.metric(end) + L1+4*w+2*r,'YData', L1+0.5*w+r);
                car.que1{k}.realpose = [car.que1{k}.state(1) - car.que1{k}.metric(end) + L1+4*w+2*r,L1+0.5*w+r];
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)),'YData', L1+2.5*w+r);
            car.que1{k}.realpose = [L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)),L1+2.5*w+r];
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
            car.que1{k}.realpose = [L1+3.5*w+r, car.que1{k}.state(1)];
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+3.5*w+r, car.que1{k}.state(1)];
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData', L1+3.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
                    'YData', car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30));
                car.que1{k}.realpose = [L1+3.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30)...
                    ,  car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)];
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData',car.que1{k}.state(1)-l);
                car.que1{k}.id(3) = 3;
                car.que1{k}.realpose = [L1+2.5*w+r, car.que1{k}.state(1)-l];
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = 2.5*w + r;
                alpha = deltaX / R;
                yNew = L1 + R * sin(alpha);
                xNew = L1 + R * cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew, yNew];
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+3.5*w+r, car.que1{k}.state(1)];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = 0.5*w + r;
                alpha = deltaX / R;
                yNew = L1 + R * sin(alpha);
                xNew = L1 + 4*w + 2*r - R * cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        end
    elseif (car.que1{k}.lane == 5)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1),L1+2.5*w+r];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', L1 + 4*w + 2*r + (car.que1{k}.state(1) - car.que1{k}.metric(end)));
                car.que1{k}.realpose = [L1+3.5*w+r,L1 + 4*w + 2*r + (car.que1{k}.state(1) - car.que1{k}.metric(end))];               
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))));
            car.que1{k}.realpose = [L1+1.5*w+r,L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)))];    
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
            car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1),L1+2.5*w+r];
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
                 car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1),L1+2.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = 2.5*w + r;
                alpha = deltaX / R;
                xNew = 2*L1+4*w+2*r - (L1 + R * sin(alpha));
                yNew = L1 + R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];                
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1),L1+2.5*w+r]; 
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r - (car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)),...
                            'YData',L1+2.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
                car.que1{k}.realpose = [2*L1+4*w+2*r - (car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30))...
                    ,L1+2.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30)];        
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1)+l,'YData', L1+3.5*w+r);
                car.que1{k}.id(3) = 6;
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1)+l, L1+3.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = 0.5*w + r;
                alpha = deltaX / R;
                xNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
                yNew = L1 + 4*w + 2*r - R * cos(alpha);
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];   
            end
        end
    elseif (car.que1{k}.lane == 6)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 2)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1),L1+3.5*w+r];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', L1 + 4 * w + 2*r + (car.que1{k}.state(1) - car.que1{k}.metric(end)));
                car.que1{k}.realpose = [L1+3.5*w+r, L1 + 4 * w + 2*r + (car.que1{k}.state(1) - car.que1{k}.metric(end))];
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)));   
                car.que1{k}.realpose = [L1+1.5*w+r,L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l))];
            
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1), L1+3.5*w+r];            
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1), L1+3.5*w+r]; 
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)),...
                            'YData',L1+3.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
                car.que1{k}.realpose = [2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30))...
                    , L1+3.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30)];         
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1)+l,'YData', L1+2.5*w+r);
                car.que1{k}.id(3) = 5;
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1)+l,L1+2.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = 2.5*w + r;
                alpha = deltaX / R;
                xNew = 2*L1+4*w+2*r - (L1 + R * sin(alpha));
                yNew = L1 + R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
                car.que1{k}.realpose = [2*L1+4*w+2*r-car.que1{k}.state(1),L1+3.5*w+r];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = 0.5*w + r;
                alpha = deltaX / R;
                xNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
                yNew = L1 + 4*w + 2*r - R * cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        end
    elseif (car.que1{k}.lane == 7)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+1.5*w+r,2*L1+4*w+2*r-car.que1{k}.state(1)];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)), 'YData', L1+3.5*w+r);
                car.que1{k}.realpose = [L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)),L1+3.5*w+r];
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1 + 4*w + 2*r + (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))),'YData', L1+1.5*w+r);
            car.que1{k}.realpose = [L1 + 4*w + 2*r + (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))),L1+1.5*w+r];
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
            car.que1{k}.realpose = [L1+1.5*w+r, 2*L1+4*w+2*r-car.que1{k}.state(1)];
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+1.5*w+r, 2*L1+4*w+2*r-car.que1{k}.state(1)];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = 2.5*w + r;
                alpha = deltaX / R;
                yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
                xNew = L1+4*w+2*r - R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData', L1+1.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
                            'YData',2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)));
                 car.que1{k}.realpose = [L1+1.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
                     2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30))];        
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1)+l);
                car.que1{k}.id(3) = 8;
                car.que1{k}.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-car.que1{k}.state(1)+l];
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = 0.5*w + r;
                alpha = deltaX / R;
                yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
                xNew = L1 + R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        end
    elseif (car.que1{k}.lane == 8)
        if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
            if (car.que1{k}.id(1) == 2)
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            else
                set(car.que1{k}.p,'XData',-1,'YData', -1);
                car.que1{k}.realpose = [-100,-100];
            end
        elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
            if (car.que1{k}.id(1) == 1)
                set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-car.que1{k}.state(1)];
            elseif (car.que1{k}.id(1) == 3)
                set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)), 'YData', L1+3.5*w+r);
                car.que1{k}.realpose = [L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)),L1+3.5*w+r];
            end
        elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
            set(car.que1{k}.p,'XData',L1 + 4*w + 2*r + (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)),'YData', L1+1.5*w+r);
            car.que1{k}.realpose = [L1 + 4*w + 2*r + (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)),L1+1.5*w+r];
        elseif (car.que1{k}.id(1) == 1)
            set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
            car.que1{k}.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-car.que1{k}.state(1)];
        elseif (car.que1{k}.id(1) == 2)
            if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
                set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-car.que1{k}.state(1)];
            elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
                set(car.que1{k}.p,'XData', L1+0.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
                    'YData', 2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)));
                car.que1{k}.realpose = [L1+0.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30)...
                    ,2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30))];
            elseif (car.que1{k}.state(1) < L1+l)
                set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1)+l);
                car.que1{k}.id(3) = 7;
                car.que1{k}.realpose = [L1+1.5*w+r,2*L1+4*w+2*r-car.que1{k}.state(1)+l];
            else
                deltaX = car.que1{k}.state(1) - (L1+l);
                R = 2.5*w + r;
                alpha = deltaX / R;
                yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
                xNew = L1+4*w+2*r - R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        elseif (car.que1{k}.id(1) == 3)
            if (car.que1{k}.state(1) < L1)
                set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
                car.que1{k}.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-car.que1{k}.state(1)];
            else
                deltaX = car.que1{k}.state(1) - L1;
                R = 0.5*w + r;
                alpha = deltaX / R;
                yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
                xNew = L1 + R *cos(alpha); 
                set(car.que1{k}.p,'XData',xNew,'YData', yNew);
                car.que1{k}.realpose = [xNew,yNew];
            end
        end
    end
end

drawnow

% axis(frame2);
% 
% for k = 1 : len
%     if (car.que1{k}.lane == 1)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)));
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)) + L1+4*w+2*r);            
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = (2.5 * w + r);
%                 alpha = deltaX / R;
%                 xNew = L1 + R * sin(alpha);
%                 yNew = L1+4*w+2*r - R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+1.5*w+r);
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData',car.que1{k}.metric(5)- lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30),...
%                             'YData',L1+1.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1)-l,'YData', L1+0.5*w+r);
%                 car.que1{k}.id(3) = 2;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = 0.5*w + r;
%                 alpha = deltaX / R;
%                 xNew = L1 + R * sin(alpha);
%                 yNew = L1 + R * cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     elseif (car.que1{k}.lane == 2)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 2)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)));
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l) + L1+4*w+2*r);            
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData',car.que1{k}.metric(5)- lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30),...
%                             'YData',L1+0.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1)-l,'YData', L1+1.5*w+r);
%                 car.que1{k}.id(3) = 1;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = (2.5 * w + r);
%                 alpha = deltaX / R;
%                 xNew = L1 + R * sin(alpha);
%                 yNew = L1+4*w+2*r - R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData', L1+0.5*w+r);
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = 0.5 * w + r;
%                 alpha = deltaX / R;
%                 xNew = L1 + R * sin(alpha);
%                 yNew = L1 + R * cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     elseif (car.que1{k}.lane == 3)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1) - car.que1{k}.metric(end) + L1+4*w+2*r,'YData', L1+0.5*w+r);
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))),'YData', L1+2.5*w+r);
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = 2.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = L1 + R * sin(alpha);
%                 xNew = L1 + R * cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData', car.que1{k}.state(1));
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData', L1+2.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
%                             'YData',car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1)-l);
%                 car.que1{k}.id(3) = 4;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = 0.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = L1 + R * sin(alpha);
%                 xNew = L1 + 4*w + 2*r - R * cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     elseif (car.que1{k}.lane == 4)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 2)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1) - car.que1{k}.metric(end) + L1+4*w+2*r,'YData', L1+0.5*w+r);
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)),'YData', L1+2.5*w+r);
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData', L1+3.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
%                     'YData', car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',L1+2.5*w+r,'YData',car.que1{k}.state(1)-l);
%                 car.que1{k}.id(3) = 3;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = 2.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = L1 + R * sin(alpha);
%                 xNew = L1 + R * cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', car.que1{k}.state(1));
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = 0.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = L1 + R * sin(alpha);
%                 xNew = L1 + 4*w + 2*r - R * cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     elseif (car.que1{k}.lane == 5)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', L1 + 4*w + 2*r + (car.que1{k}.state(1) - car.que1{k}.metric(end)));
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))));                
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = 2.5*w + r;
%                 alpha = deltaX / R;
%                 xNew = 2*L1+4*w+2*r - (L1 + R * sin(alpha));
%                 yNew = L1 + R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+2.5*w+r);
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r - (car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)),...
%                             'YData',L1+2.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1)+l,'YData', L1+3.5*w+r);
%                 car.que1{k}.id(3) = 6;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = 0.5*w + r;
%                 alpha = deltaX / R;
%                 xNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
%                 yNew = L1 + 4*w + 2*r - R * cos(alpha);
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     elseif (car.que1{k}.lane == 6)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 2)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',L1+3.5*w+r,'YData', L1 + 4 * w + 2*r + (car.que1{k}.state(1) - car.que1{k}.metric(end)));
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', L1 - (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)));                
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)),...
%                             'YData',L1+3.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1)+l,'YData', L1+2.5*w+r);
%                 car.que1{k}.id(3) = 5;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = 2.5*w + r;
%                 alpha = deltaX / R;
%                 xNew = 2*L1+4*w+2*r - (L1 + R * sin(alpha));
%                 yNew = L1 + R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',2*L1+4*w+2*r-car.que1{k}.state(1),'YData', L1+3.5*w+r);
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = 0.5*w + r;
%                 alpha = deltaX / R;
%                 xNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
%                 yNew = L1 + 4*w + 2*r - R * cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     elseif (car.que1{k}.lane == 7)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)), 'YData', L1+3.5*w+r);
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r) && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1 + 4*w + 2*r + (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r))),'YData', L1+1.5*w+r);
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = 2.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
%                 xNew = L1+4*w+2*r - R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData', L1+1.5*w+r - (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
%                             'YData',2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1)+l);
%                 car.que1{k}.id(3) = 8;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = 0.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
%                 xNew = L1 + R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     elseif (car.que1{k}.lane == 8)
%         if (car.que1{k}.state(1) > car.que1{k}.metric(4) - 10)
%             if (car.que1{k}.id(1) == 2)
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             else
%                 set(car.que1{k}.p,'XData',-1,'YData', -1);
%             end
%         elseif (car.que1{k}.state(1) > car.que1{k}.metric(end) && (car.que1{k}.id(1) == 1 || car.que1{k}.id(1) == 3))
%             if (car.que1{k}.id(1) == 1)
%                 set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%             elseif (car.que1{k}.id(1) == 3)
%                 set(car.que1{k}.p,'XData',L1 - (car.que1{k}.state(1) - car.que1{k}.metric(end)), 'YData', L1+3.5*w+r);
%             end
%         elseif (car.que1{k}.state(1) > L1+0.5*pi*(2.5*w+r)+l && car.que1{k}.id(1) == 2)
%             set(car.que1{k}.p,'XData',L1 + 4*w + 2*r + (car.que1{k}.state(1) - (L1+0.5*pi*(2.5*w+r)+l)),'YData', L1+1.5*w+r);
%         elseif (car.que1{k}.id(1) == 1)
%             set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%         elseif (car.que1{k}.id(1) == 2)
%             if (car.que1{k}.state(1) < car.que1{k}.metric(5)-lanechangingLength)
%                 set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%             elseif (car.que1{k}.state(1) < car.que1{k}.metric(5))
%                 set(car.que1{k}.p,'XData', L1+0.5*w+r + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*sind(30),...
%                     'YData', 2*L1+4*w+2*r-(car.que1{k}.metric(5)-lanechangingLength + (car.que1{k}.state(1)-car.que1{k}.metric(5)+lanechangingLength)*cosd(30)));
%             elseif (car.que1{k}.state(1) < L1+l)
%                 set(car.que1{k}.p,'XData',L1+1.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1)+l);
%                 car.que1{k}.id(3) = 7;
%             else
%                 deltaX = car.que1{k}.state(1) - (L1+l);
%                 R = 2.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
%                 xNew = L1+4*w+2*r - R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         elseif (car.que1{k}.id(1) == 3)
%             if (car.que1{k}.state(1) < L1)
%                 set(car.que1{k}.p,'XData',L1+0.5*w+r,'YData', 2*L1+4*w+2*r-car.que1{k}.state(1));
%             else
%                 deltaX = car.que1{k}.state(1) - L1;
%                 R = 0.5*w + r;
%                 alpha = deltaX / R;
%                 yNew = 2*L1+4*w+2*r-(L1 + R * sin(alpha));
%                 xNew = L1 + R *cos(alpha); 
%                 set(car.que1{k}.p,'XData',xNew,'YData', yNew);
%             end
%         end
%     end
% end
% 
% drawnow

% for k = 1:1:len
%     if(car.que1{k}.id(1) == 1)
%         if(car.que1{k}.id(5) == 1 || car.que1{k}.id(5) == 2)
%             if (car.que1{k}.id(5) == 1)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData',3.5);
%             else
%                 if(car.que1{k}.state(1) <= car.que1{k}.metric(5)-7)
%                     set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData',0);
%                 else
%                     if(car.que1{k}.state(1) > car.que1{k}.metric(5)-7 && car.que1{k}.state(1) <= car.que1{k}.metric(5))
%                         set(car.que1{k}.p,'XData',car.que1{k}.metric(5)-7 + (car.que1{k}.state(1)-car.que1{k}.metric(5)+7)*cosd(30),...
%                             'YData',(car.que1{k}.state(1)-car.que1{k}.metric(5)+7)*sind(30));
%                     else
%                         set(car.que1{k}.p,'XData',car.que1{k}.state(1)-l,'YData', 3.5);
%                         car.que1{k}.id(6) = 1;
%                         index = search_i_in_queue(car.que2, car.que1{k}.id(2));
%                         if(numel(index))
%                             car.que2(index) = [];
%                         end
%                     end
%                 end
%             end
%         else
%             if(car.que1{k}.state(1) <= car.que1{k}.metric(6))
%                 set(car.que1{k}.p,'XData',400 - (400 - car.que1{k}.state(1))*cosd(30),...
%                     'YData',-sind(30)*(400 - car.que1{k}.state(1)));
%                 if(car.que1{k}.state(1) >= car.que1{k}.metric(5))
%                     index = search_i_in_queue(car.que2, car.que1{k}.id(2));
%                     if(numel(index))
%                         car.que2(index) = [];
%                     end
%                 end
%             else
%                 set(car.que1{k}.p,'XData',400+3.5*sqrt(3) + car.que1{k}.state(1)-car.que1{k}.metric(6),...
%                     'YData',3.5);
%                 car.que1{k}.id(6) = 1;
%             end
%         end
%     end
% end
% len = length(car.que2);
% for k = 1:1:len
%     if(car.que2{k}.id(1) == 2) 
%         if(car.que2{k}.id(5) == 2)
%             set(p(car.que2{k}.id(2)),'XData',car.que2{k}.state(1),'YData',0);
%             if(car.que2{k}.state(1) > car.que2{k}.metric(5))
%                 index = search_i_in_queue(car.que1, car.que2{k}.id(2));
%                 if(numel(index))
%                     car.que1(index) = [];
%                 end
%             end
%         else
%             if(car.que2{k}.id(5) == 3)
%                 if(car.que2{k}.state(1) <= car.que2{k}.metric(5))
%                     set(p_s(car.que2{k}.id(2)),'XData',car.que2{k}.metric(5) - (car.que2{k}.metric(5) - car.que2{k}.state(1))*cosd(30),...
%                     'YData',-sind(30)*(car.que2{k}.metric(5) - car.que2{k}.state(1)));
%                 else
%                     set(p_s(car.que2{k}.id(2)),'XData', car.que2{k}.state(1),...
%                     'YData',0);
%                     car.que2{k}.id(6) = 2;
%                     index = search_i_in_queue(car.que1, car.que2{k}.id(2));
%                     if(numel(index))
%                         car.que1(index) = [];
%                     end    
%                 end
%             else
%                 if(car.que2{k}.state(1) <= car.que2{k}.metric(6))
%                     set(p_s(car.que2{k}.id(2)),'XData',car.que2{k}.metric(6) - (car.que2{k}.metric(6) - car.que2{k}.state(1))*cosd(30),...
%                     'YData',-sind(30)*(car.que2{k}.metric(6) - car.que2{k}.state(1)));
%                 else
%                     set(p_s(car.que2{k}.id(2)),'XData', car.que2{k}.state(1),...
%                     'YData',0);
%                     car.que2{k}.id(6) = 2;   
%                 end
%             end
%         end
%     end
% end
% 
% drawnow
% % axis([-20 450 -255 50]); 
% 
% 
% axis(frame2);
% len = length(car.que1);
% for k = 1:1:len
%     if(car.que1{k}.id(1) == 1)
%         if(car.que1{k}.id(5) == 1 || car.que1{k}.id(5) == 2)
%             if (car.que1{k}.id(5) == 1)
%                 set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData',3.5);
%             else
%                 if(car.que1{k}.state(1) <= car.que1{k}.metric(5)-7)
%                     set(car.que1{k}.p,'XData',car.que1{k}.state(1),'YData',0);
%                 else
%                     if(car.que1{k}.state(1) > car.que1{k}.metric(5)-7 && car.que1{k}.state(1) <= car.que1{k}.metric(5))
%                         set(car.que1{k}.p,'XData',car.que1{k}.metric(5)-7 + (car.que1{k}.state(1)-car.que1{k}.metric(5)+7)*cosd(30),...
%                             'YData',(car.que1{k}.state(1)-car.que1{k}.metric(5)+7)*sind(30));
%                     else
%                         set(car.que1{k}.p,'XData',car.que1{k}.state(1)-l,'YData', 3.5); 
%                     end
%                 end
%             end
%         else
%             if(car.que1{k}.state(1) <= car.que1{k}.metric(6))
%                 set(car.que1{k}.p,'XData',400 - (400 - car.que1{k}.state(1))*cosd(30),...
%                     'YData',-sind(30)*(400 - car.que1{k}.state(1)));
%             else
%                 set(car.que1{k}.p,'XData',400+3.5*sqrt(3) + car.que1{k}.state(1)-car.que1{k}.metric(6),...
%                     'YData',3.5);
%             end
%         end
%     end
% end
% len = length(car.que2);
% for k = 1:1:len
%     if(car.que2{k}.id(1) == 2) 
%         if(car.que2{k}.id(5) == 2)
%             set(p2(car.que2{k}.id(2)),'XData',car.que2{k}.state(1),'YData',0);
%         else
%             if(car.que2{k}.id(5) == 3)
%                 if(car.que2{k}.state(1) <= car.que2{k}.metric(5))
%                     set(p_s2(car.que2{k}.id(2)),'XData',car.que2{k}.metric(5) - (car.que2{k}.metric(5) - car.que2{k}.state(1))*cosd(30),...
%                     'YData',-sind(30)*(car.que2{k}.metric(5) - car.que2{k}.state(1)));
%                 else
%                     set(p_s2(car.que2{k}.id(2)),'XData', car.que2{k}.state(1),...
%                     'YData',0);
%                 end
%             else
%                 if(car.que2{k}.state(1) <= car.que2{k}.metric(6))
%                     set(p_s2(car.que2{k}.id(2)),'XData',car.que2{k}.metric(6) - (car.que2{k}.metric(6) - car.que2{k}.state(1))*cosd(30),...
%                     'YData',-sind(30)*(car.que2{k}.metric(6) - car.que2{k}.state(1)));
%                 else
%                     set(p_s2(car.que2{k}.id(2)),'XData', car.que2{k}.state(1),...
%                     'YData',0);
%                 end
%             end
%         end
%     end
% end
% 
% drawnow