function [ feats ] = extract_features(images,stride,feat_type,folder)
%Takes in a list of images and extract, either SIFT or SURF features

switch nargin
    case 1
        stride = 3;
        feat_type = 'SIFT';
    case 2
        stride = stride;
        feat_type = 'SIFT';
    case 3
        stride = stride;
        feat_type = feat_type;
end

start = 1;
feats = {};
% folder = 'data';

while start < length(images)
    curr_img = imread(fullfile(folder,images{start}));
    I_curr = single(rgb2gray(curr_img));
    [f_curr, d_curr] = vl_sift(I_curr);
    stop = min(start+stride,length(images)-1);
    for i=start:stop
        % Plot images
        figure;imshow(curr_img);
        perm = randperm(size(f_curr,2)) ;
        h = vl_plotframe(f_curr(:,perm(1:50)));

        hold on;    
        next_img = imread(fullfile('data',images{i+1}));
        I_next = single(rgb2gray(next_img));
        % match the two feature arrays
        [f_next, d_next] = vl_sift(I_next);
        [matches,scores] = vl_ubcmatch(d_curr,d_next);

        feats{end+1} = matches;
        f_curr = f_next;
        d_curr = d_next;
    end
    start = start + stride;
end
end

function plot_features(img,num_feats)
I = single(rgb2gray(img));

% these two lines plot the image
image(I);
colormap(gray(256));
[f,d]=vl_sift(I);
perm = randperm(size(f,2));
selected_feats = perm(1:num_feats);
h1 = vl_plotframe(f(:,selected_feats));
h2 = vl_plotframe(f(:,selected_feats));
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);
end

