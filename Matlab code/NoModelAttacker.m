function rt = NoModelAttacker(i, one) % for vehicles in the different lane


noise1 = 0.15*(rand());
% noise2 = 5*(rand() - 0.5);

x0 = one.state; 


rt = [x0(1)+noise1 , rand(), 0];


end
