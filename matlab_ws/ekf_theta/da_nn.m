function Li = da_nn(z)
%z is a m*4 array, 4 means x, y, z and index in hashtable
%go through the index to find whether index contained in landmark
    global State;
    num = size(z,1);
    Li = zeros(1,num);
    for i = 1:num
        index = find(State.sL == z(i,4));
        if isempty(index)
            Li(i) = 0;
        else
            Li(i) = index;
        end
    end
end
