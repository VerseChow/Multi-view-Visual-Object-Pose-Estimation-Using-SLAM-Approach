function [C, D] = matchFrames(A, B)
%A and B are the hash_frames
A_feat = A.rgb_feat;
B_feat = B.rgb_feat;

C = [];
C.rgb_pix=[];
C.rgb_feat=[];
C.depth_loc=[];

D = [];
D.rgb_pix =[];
D.rgb_feat =[];
D.depth_loc =[];

for i=1:size(A_feat, 2)
   min_1 = 999; min_2 = 999;
   for j=1:size(B_feat, 2)
       d = dist(A_feat(:, i), B_feat(:, j));
       if(min_1 > d)
           min_2 = min_1;
           min_1 = d;
       end
   end
   disp(min_1/min_2);
   if(min_1/min_2 < 0.65)
       C.rgb_pix = [C.rgb_pix, A.rgb_pix(:, i)];
       C.rgb_feat = [C.rgb_feat, A.rgb_feat(:, i)];
       C.depth_loc = [C.depth_loc, A.depth_loc(:, i)];
       
       D.rgb_pix = [D.rgb_pix, B.rgb_pix(:, j)];
       D.rgb_feat = [D.rgb_feat, B.rgb_feat(:, j)];
       D.depth_loc = [D.depth_loc, B.depth_loc(:, j)];
   end
end
end

function d = dist(A, B)
    d=sqrt(double(A)'*double(B));
end