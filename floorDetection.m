function [R,depth_floor] = floorDetection(nimg)
% detect the floor in a depth image using RANSAC algorithm
% nimg: number of the depth image on the folder
% R: rotation matrix to make the floor horizontal
% depth_floor: depth of the floor from the camera

%read a depth image
path = 'imagesPeople/';
dirIm = dir(strcat(path,'*.png'));
fn = strcat(path,dirIm(nimg).name);
im = double(imread(fn));
[rows,cols] = size(im);

fov = 70.6;  %depth camera field of view
f = (cols/2)/tand(fov/2);  %focal length
cc = 0.5*[cols, rows];  %principal point assumed to be at the image center
%find 3D coordinates in millimeters
[um,vm] = meshgrid(1:cols,1:rows);
Z = im(:);
X = Z.*(um(:)-cc(1))/f;
Y = Z.*(vm(:)-cc(2))/f;

%find the floor plane using RANSAC
n = length(Z);
tol = 10;  %tolerance for fitting points to the plane
s = [];  %variable for determining the plane parameters
np = 0; %initial number of points at the floor
num_iters = 1000;
rng(1);
for iter=1:num_iters
    %find a plane from 3 random points
    ind = randperm(n,3);
    M = [X(ind), Y(ind), Z(ind)];
    if( det(M)~=0 )
        sp = M\ones(3,1);
        d = 1/norm(sp);
        a = sp(1)*d;
        b = sp(2)*d;
        c = sp(3)*d;
        %number of points lying at the plane
        nfit = sum(abs(a*X + b*Y + c*Z - d)<=tol);
        if( nfit>np )
            np = nfit;
            s = sp;
        end
    end
end
%floor plane parameters
d = 1/norm(s);
a = s(1)*d;
b = s(2)*d;
c = s(3)*d;
%plot the points before rotation
ind = abs(a*X + b*Y + c*Z - d)<=tol;
figure(1); plot_depth(X,Y,Z,ind);

%find the rotation matrix to make the floor horizontal
thx = asind(-b);
thy = asind(-a/cosd(thx));
R = [cosd(thy) 0 sind(thy); -sind(thx)*sind(thy) cosd(thx) sind(thx)*cosd(thy); ...
    -cosd(thx)*sind(thy) -sind(thx) cosd(thx)*cosd(thy)];

%plot the points after rotation
Xr = R(1,1)*X + R(1,2)*Y + R(1,3)*Z;
Yr = R(2,1)*X + R(2,2)*Y + R(2,3)*Z;
Zr = R(3,1)*X + R(3,2)*Y + R(3,3)*Z;
figure(2); plot_depth(Xr,Yr,Zr,ind);
im2 = Zr<2200 & Zr>0;
im2 = reshape(im2,rows,cols);
im2 = imerode(im2,ones(3,3)); 
figure(3); imshow(im2);
depth_floor = mean(Zr(ind));

%================================================
function plot_depth(X,Y,Z,ind)
subplot(2,1,1);
plot(X(ind==0),Z(ind==0),'b.',X(ind),Z(ind),'r.');
xlabel('X (mm)');
ylabel('Z (mm)');
axis([-3000 3000 0 4000]);
grid on;
subplot(2,1,2);
plot(Y(ind==0),Z(ind==0),'b.',Y(ind),Z(ind),'r.');
xlabel('Y (mm)');
ylabel('Z (mm)');
axis([-3000 3000 0 4000]);
grid on;