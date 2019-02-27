function [mosaic]= Mosaic(I1, I2)
% [num, matchidx, loc1, loc2] = match ("copy1.JPG","copy2.JPG");
% img1_pts = loc1(find(matchidx > 0), 1:2);
% img2_pts = loc2(matchidx(find(matchidx > 0)), 1:2);


if size(I1,3) == 3
    I1 = rgb2gray(I1);
end

if size(I2,3) == 3
    I2 = rgb2gray(I2);
end

points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);

[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);

indexPairs = matchFeatures(features1,features2);
numMatches = size(indexPairs,1);
numMatches

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

%figure; showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);

XY = matchedPoints1.Location;
XYprime = matchedPoints2.Location;

for t = 1:100
    
    for z = 1:4
        subset(z,1) = randi(numMatches);
    end
    
    counter = 1;
   
    for ii = 1:4
        
        i = subset(ii,1);
        xi = XY(i,1);
        yi = XY(i,2);
        xiprime = XYprime(i,1);
        yiprime = XYprime(i,2);

        firstEquation = [-xi -yi -1 0 0 0 xiprime.*xi xiprime.*yi xiprime];
        secondEquation = [0 0 0 -xi -yi -1 yiprime.*xi yiprime.*yi yiprime];

        A(counter,:) = firstEquation;
        A(counter+1,:) = secondEquation;

        counter = counter + 2;
    end

    [U,D,V] = svd(A,0);
    H = V(:,end);
    
    Hs(t,:) = H;
    
    H33 = reshape(H,3,3)';
    inliner = 0;
    
    for j = 1:numMatches
        
        xj = XY(j,1);
        yj = XY(j,2);
        xjprime = XYprime(j,1);
        yjprime = XYprime(j,2);
        
        p = [xj; yj; 1];
        Hp = H33 * p;
        w = Hp(3,1);
        du = Hp(1,1)/w - xjprime ;
        dv = Hp(2,1)/w - yjprime ;
        
        if du*du + dv*dv < 6*6
            inliner = inliner + 1;
        end
       
    end
    
    vote(t,:) = inliner;
    
end

maxHIndex = 0;
maxVote = 0;
for k = 1:100
    if vote(k,:) > maxVote
        maxVote = vote(k,:);
        maxHIndex = k;
    end
end

maxH = Hs(k,:);
maxH = reshape(maxH,3,3)';

% --------------------------------------------------------------------
%                                                               Mosaic
% --------------------------------------------------------------------

box2 = [1  size(I2,2) size(I2,2)  1 ;
        1  1           size(I2,1)  size(I2,1) ;
        1  1           1            1 ] ;
box2_ = inv(maxH) * box2 ;
box2_(1,:) = box2_(1,:) ./ box2_(3,:) ;
box2_(2,:) = box2_(2,:) ./ box2_(3,:) ;
ur = min([1 box2_(1,:)]):max([size(I1,2) box2_(1,:)]) ;
vr = min([1 box2_(2,:)]):max([size(I1,1) box2_(2,:)]) ;

[u,v] = meshgrid(ur,vr) ;
im1_ = interp2(im2double(I1),u,v) ;

z_ = maxH(3,1) * u + maxH(3,2) * v + maxH(3,3) ;
u_ = (maxH(1,1) * u + maxH(1,2) * v + maxH(1,3)) ./ z_ ;
v_ = (maxH(2,1) * u + maxH(2,2) * v + maxH(2,3)) ./ z_ ;
im2_ = interp2(im2double(I2),u_,v_) ;

mass = ~isnan(im1_) + ~isnan(im2_) ;
im1_(isnan(im1_)) = 0 ;
im2_(isnan(im2_)) = 0 ;
mosaic = (im1_ + im2_) ./ mass ;

figure
imshow(mosaic);

end


        
    


   






