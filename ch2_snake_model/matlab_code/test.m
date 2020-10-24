Ix=ImageDerivatives2D(I,Sigma,'x');
Iy=ImageDerivatives2D(I,Sigma,'y');
Ixx=ImageDerivatives2D(I,Sigma,'xx');
Ixy=ImageDerivatives2D(I,Sigma,'xy');
Iyy=ImageDerivatives2D(I,Sigma,'yy');


Eline = imgaussian(I,10);
Eterm = (Iyy.*Ix.^2 -2*Ixy.*Ix.*Iy + Ixx.*Iy.^2)./((1+Ix.^2 + Iy.^2).^(3/2));
Eedge = sqrt(Ix.^2 + Iy.^2); 

Eextern= (0.04*Eline - 2*Eedge -0.01 * Eterm); 

figure(1), imshow(Eline,[]);
figure(2), imshow(Eterm,[]);
figure(3), imshow(Eedge,[]);
figure(4), imshow(Eextern,[]);

