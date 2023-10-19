function index = check_leave(que)
len = length(que);
index = []; L = 300;
for i = 1:1:len
    if (que{i}.state(1) > que{i}.metric(4))
        index = i;
        break;
    end
end