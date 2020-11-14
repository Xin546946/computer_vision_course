I = imread("testimage.png");
I = double(I);
Sigma = 0;
Ix=ImageDerivatives2D(I,Sigma,'x');
Iy=ImageDerivatives2D(I,Sigma,'y');
Ixx=ImageDerivatives2D(I,Sigma,'xx');
Ixy=ImageDerivatives2D(I,Sigma,'xy');
Iyy=ImageDerivatives2D(I,Sigma,'yy');
Eterm = (Iyy.*Ix.^2 -2*Ixy.*Ix.*Iy + Ixx.*Iy.^2)./((1+Ix.^2 + Iy.^2).^(3/2));
Eterm = uint8(Eterm);
imshow(Eterm,[])