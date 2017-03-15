function [ feats ] = extract_features(images,stride,feat_type)
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

for i=1:length(images)
    I = imread(images{i});
    I = single(rgb2gray(I));
    image(I); %look at the image
    colormap(gray(256));
%     [f,d] = vl_sift(I); %features and descriptors
%    plot_features(I,50);
end
end

function plot_features(img,num_feats)
I = single(rgb2gray(img));
image(I);
[f,d]=vl_sift(I);
perm = randperm(size(f,2));
selected_feats = perm(1:num_feats);
%h1 = vl_plotframe(f(:,selected_feats));
%h2 = vl_plotframe(f(:,selected_feats));
%set(h1,'color','k','linewidth',3);
%set(h2,'color','y','linewidth',2);
end

