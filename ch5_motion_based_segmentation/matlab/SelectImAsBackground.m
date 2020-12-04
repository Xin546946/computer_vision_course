
clear all;
close all;
clc;

imBackground = double( imread('video/ori_096.bmp') );

imCurrent = double( imread('video/ori_280.bmp') );

imDiff = abs( imCurrent - imBackground );

subplot(121); imshow( imDiff , [] );
subplot(122); imshow( imDiff > 50 , [] );
imwrite( uint8(mat2gray(imDiff)*255) , 'imDiff.bmp' );
imwrite( uint8(double(imDiff>50)*255) , 'imDiffT.bmp' );