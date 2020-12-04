
clear all;
close all;
clc;

%%%%    可以使用for循环完成，这里为了方便理解，直接一帧一帧读
im1 = double( imread('video/ori_081.bmp') );
im2 = double( imread('video/ori_091.bmp') );
im3 = double( imread('video/ori_101.bmp') );
im4 = double( imread('video/ori_111.bmp') );
im5 = double( imread('video/ori_121.bmp') );
im6 = double( imread('video/ori_131.bmp') );

imBackground = (im1+im2+im3+im4+im5+im6) / 6;

imCurrent = double( imread('video/ori_280.bmp') );

imDiff = abs( imCurrent - imBackground );

figure(1001);
imshow( imBackground , [] );
imwrite( uint8(imBackground) , 'background.bmp' );

figure(1002);
subplot(121); imshow( imDiff , [] );
subplot(122); imshow( imDiff > 50 , [] );
imwrite( uint8(mat2gray(imDiff)*255) , 'imDiff.bmp' );
imwrite( uint8(double(imDiff>50)*255) , 'imDiffT.bmp' );