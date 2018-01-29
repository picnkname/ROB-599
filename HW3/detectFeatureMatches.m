function [matchedPoints1, matchedPoints2, matchedf1, matchedf2] = detectFeatureMatches(I1,I2)
%
% detect SURF features
points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);
%
% extract SURF features from the image
[f1,vpts1] = extractFeatures(I1,points1);
[f2,vpts2] = extractFeatures(I2,points2);
%
% match the features between the two images
indexPairs = matchFeatures(f1,f2) ;
% return the list of matched/valid points organized by correspondence
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));
matchedf1 = f1(indexPairs(:,1),:);
matchedf2 = f2(indexPairs(:,2),:);
%
end