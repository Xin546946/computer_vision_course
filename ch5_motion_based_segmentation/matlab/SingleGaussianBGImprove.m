

clear all;
close all;
clc;

%%%%    得到文件列表
fileList = dir( 'video\*.bmp' );

%%%%    参数设置
sigmaRatio  = 2.5;          % 判断是否为前景的阈值
alpha       = 0.01;         % 学习率，决定更新速度
sigmaInit   = 25;           % 初始方差

%%%%    读取数据处理
for frameIndex = 1 : length( fileList );
    
    %%%%    读取当前图像
    currentIm           = double( imread( ['video\' fileList(frameIndex).name] ) );
    [ height width ]    = size( currentIm );
    
    if 1 == frameIndex
        %%%%    如果是第一帧， 初始化高斯的均值图像(均值为初始图像，方差较大）
        backgroundImMean    = currentIm;
        backgroundImVar     = ones( size( currentIm ) ) * sigmaInit;
    else
        %%%%    如果是后续帧，则判断是否为前景
        backgroundMask      = double(abs(currentIm-backgroundImMean) < backgroundImVar * sigmaRatio);
        backgroundImMean    = ((1-alpha)*backgroundImMean + alpha*currentIm).*backgroundMask+backgroundImMean.*(1-backgroundMask);
        backgroundImVar     = sqrt((1-alpha)*backgroundImVar.^2 + alpha*(currentIm-backgroundImMean).^2);
        
        %%%%    显示结果
        ShowResult( currentIm , backgroundMask , backgroundImMean );
    end
    
end