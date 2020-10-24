 % Show an image
  figure, imshow(imread('testimage.png'));
 % Select some points with the mouse
  [y,x] = getpts;
 % Make an array with the clicked coordinates
  P=[x(:) y(:)];
 % Interpolate inbetween the points
  Pnew=InterpolateContourPoints2D(P,100)
 % Show the result
  hold on; plot(P(:,2),P(:,1),'b*');
  plot(Pnew(:,2),Pnew(:,1),'r.');