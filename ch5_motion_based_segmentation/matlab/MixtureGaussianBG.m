

clear all;
close all;
clc;

%%    得到文件列表
fileList = dir( 'video/*.bmp' );

%%    参数设置
numOfGau    = 3;            % 混合高斯数目 
numOfGauUsed= 2;            % 混合高斯背景使用数目 
sigmaRatio  = 2.5;          % 判断是否为前景的阈值
alpha       = 0.01;         % 学习率，决定更新速度
sigmaInit   = 30;           % 初始方差
fgThreshold = 0.25;         % 前景阈值

%%    混合高斯相关参数设置
% get the first image as current (reference) image
currentIm               = double( imread( ['video/' fileList(1).name] ) );
% get the height and width of the image
[ height, width ]       = size( currentIm );
% 
w                       = zeros( height , width , numOfGau );                                   % 高斯的权重系数
backgroundImMean        = zeros( height , width , numOfGau );                                   % 高斯均值
backgroundImVar         = sigmaInit*ones( height , width , numOfGau );                          % 高斯方差
matchCount              = ones( height , width , numOfGau );                                    % 匹配次数
diffCurToMean           = zeros( height , width , numOfGau );                                   % 当前图像与高斯均值的差异

backgroundMask          = zeros( height , width );
backgroundIm            = zeros( height , width );


%%    读取数据处理
for frameIndex = 1 : length( fileList );
    
    %%%%    读取当前图像
    currentIm           = double( imread( ['video/' fileList(frameIndex).name] ) );
    
    if 1 == frameIndex
        
        %%%%    如果是第一帧， 初始化混合高斯的均值图像(均值为初始图像，方差较大）
        w(:,:,1)                = 1;                                                                    % 第一个高斯权重系数为1
        backgroundImMean(:,:,1) = currentIm;                                                            % 第一个高斯分布的均值设为初始图像
        
    else
        %%%%    如果是后续帧，则判断是否为前景
        diffCurToMean = abs( repmat(currentIm,[1 1 numOfGau]) - backgroundImMean );
        
        %%%%    为了便于理解，此处开始，用最简洁的语法写
        for dh = 1 : height
        for dw = 1 : width 
            
            for dc = 1 : numOfGau

                match = 0;                                                                                                                              % 初始化为没有匹配的
                if diffCurToMean(dh,dw,dc) < sigmaRatio*backgroundImVar(dh,dw,dc)
                    match       = 1;                                                                                                                    % 有匹配
                    w(dh,dw,dc) = (1-alpha)*w(dh,dw,dc) + alpha;
                    p           = alpha/w(dh,dw,dc);                                                                                                    % 这里和论文略有差异，直接用权重代替概率了
                    backgroundImMean(dh,dw,dc)  = (1-p)*backgroundImMean(dh,dw,dc) + p*currentIm(dh,dw);                                                % 更新均值
                    backgroundImVar(dh,dw,dc)   = sqrt((1-p)*(backgroundImVar(dh,dw,dc)^2) + p*(currentIm(dh,dw) - backgroundImMean(dh,dw,dc))^2);      % 更新方差

                    if matchCount(dh,dw,dc) <= 255                                                                                                         % 255为最大匹配次数
                        matchCount(dh,dw,dc) = matchCount(dh,dw,dc) + 1;  
                    end 
                end

                % 没有匹配的高斯，建立新的高斯取代：排序后排在最后面的那个
                if 0 == match 
                    [ minWValue, minWIndex ]            = min( w( dh , dw , : ) );                                      % 查找权值最小的哪一个  
                    matchCount(dh,dw,minWIndex)         = 1;                                                            % 匹配次数初始化为1  
                    w(dh,dw,minWIndex)                  = 1 / ( sum(matchCount(dh,dw,:)) - 1 );                         % 权值为其它高斯分布匹配次数之和的倒数  
                    backgroundImMean(dh,dw,minWIndex)   = currentIm(dh,dw);                                             % 初始化均值
                    backgroundImVar(dh,dw,minWIndex)    = sigmaInit;                                                    % 初始化方差
                end

            end
            
            %%%%    归一化权重系数
            wSum        = sum( w(dh,dw,:) );  
            w(dh,dw,:)  = w(dh,dw,:) / wSum;
            
            %%%%    计算背景像素值以及前景判断
            for dc = 1 : numOfGau
                backgroundIm(dh,dw) = backgroundIm(dh,dw) + backgroundImMean(dh,dw,dc)*w(dh,dw,dc);
            end
            
            rankValue                           = reshape( w(dh,dw,:)./backgroundImVar(dh,dw,:) , [3 1]);               
            [randValueSorted, rankValueIndex]   = sort(rankValue, 'descend'); 
            
            match = 0;
            for dc = 1 : numOfGauUsed
                if( ( w(dh,dw,rankValueIndex(dc)) >= fgThreshold ) & ( diffCurToMean(dh,dw,rankValueIndex(dc)) < sigmaRatio*backgroundImVar(dh,dw,rankValueIndex(dc)) ) )
                    match = 1;
                    break;
                end
            end
            if match == 1
                backgroundMask(dh,dw) = 1;
            else
                backgroundMask(dh,dw) = 0;
            end
            
        end
        end
        
        %%%%    显示结果
        ShowResult( currentIm , backgroundMask , backgroundIm );
    end
    
end