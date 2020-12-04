
clear all;
close all;
clc;

%%%%    可以使用for循环完成，这里为了方便理解，直接一帧一帧读
im1 = double( imread('video\ori_081.bmp') );
im2 = double( imread('video\ori_091.bmp') );
im3 = double( imread('video\ori_101.bmp') );
im4 = double( imread('video\ori_111.bmp') );
im5 = double( imread('video\ori_121.bmp') );
im6 = double( imread('video\ori_131.bmp') );

[height width] = size( im1 );

imBackground = zeros( height , width );
for dh = 1 : height
    for dw = 1 : width
        imPixelList = [im1(dh,dw) im2(dh,dw) im3(dh,dw) im4(dh,dw) im5(dh,dw) im6(dh,dw)];
        imBackground(dh,dw) = median( imPixelList );
    end
end

imCurrent = double( imread('video\ori_280.bmp') );

imDiff = abs( imCurrent - imBackground );

figure(1001);
imshow( imBackground , [] );
imwrite( uint8(imBackground) , 'background.bmp' );

figure(1002);
subplot(121); imshow( imDiff , [] );
subplot(122); imshow( imDiff > 50 , [] );
imwrite( uint8(mat2gray(imDiff)*255) , 'imDiff.bmp' );
imwrite( uint8(double(imDiff>50)*255) , 'imDiffT.bmp' );