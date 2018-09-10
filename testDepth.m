function testDepth()
% Detect the persons located at both sides of depth images by suppressing
% the floor

% R: rotation matrix to make the floor horizontal
% depth_floor: depth of the floor from the camera
% these parameters are found using the floorDetection script
R = [0.9999 0 0.0104; 0.0029 0.9608 -0.2772; -0.0100 0.2773 0.9607];
depth_floor = 2548;
depth_max = depth_floor - 100;
f = 362;  %focal length
rows = 424;
cols = 512;
cc = 0.5*[cols, rows];  %principal point assumed to be at the image center

path = 'imagesPeople/';
dirIm = dir(strcat(path,'*.png'));

[um,vm] = meshgrid(1:cols,1:rows);
for nimg=1:length(dirIm)
    %read a depth image
    fn = strcat(path,dirIm(nimg).name);
    im = double(imread(fn));
    %3D point coordinates
    Z = im(:);
    X = Z.*(um(:)-cc(1))/f;
    Y = Z.*(vm(:)-cc(2))/f;
    %segment image regions above the floor
    Zr = R(3,1)*X + R(3,2)*Y + R(3,3)*Z;
    ind = (Zr<=depth_max) & (Zr>0);
    imb = zeros(rows,cols);
    imb(ind) = 1;
    imb = imerode(imb,ones(3,3));
    %median of the blob coordinates at the left
    [v,u] = find(imb(:,1:round(0.5*cols)));
    loc1 = median([u,v]);
    %median of the blob coordinates at the right
    [v,u] = find(imb(:,round(0.5*cols):cols));
    loc2 = median([u,v]) + [0.5*cols, 0];
    %show the blobs and the medians of the coordinates
    figure(1); hold off;
    imshow(imb); hold on;
    plot(loc1(1),loc1(2),'ro',loc2(1),loc2(2),'bo','linewidth',2);
    title(sprintf('image nr. %d',nimg));
    drawnow;
end