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