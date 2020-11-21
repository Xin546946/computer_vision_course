%% define a sdf
sdf = zeros(100,100);
center = [50,50];
r = 30;
for i=1:100
    for j = 1:100
        sdf(i,j) = sqrt((i-center(1))*(i-center(1)) + (j-center(2))*(j-center(2))) -r;
    end
end
figure(1)
mesh(double(sdf))
hold on 
[x,y] = meshgrid(0:100:100)
z = zeros(size(x,1))
CO(:,:,1) = zeros(25); % red
CO(:,:,2) = ones(25).*linspace(0.5,0.6,25); % green
CO(:,:,3) = ones(25).*linspace(0,1,25); % blue
surf(x,y,z,CO)