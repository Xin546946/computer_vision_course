

clear all;
close all;
clc;

%%    �õ��ļ��б�
fileList = dir( 'video/*.bmp' );

%%    ��������
numOfGau    = 3;            % ��ϸ�˹��Ŀ 
numOfGauUsed= 2;            % ��ϸ�˹����ʹ����Ŀ 
sigmaRatio  = 2.5;          % �ж��Ƿ�Ϊǰ������ֵ
alpha       = 0.01;         % ѧϰ�ʣ����������ٶ�
sigmaInit   = 30;           % ��ʼ����
fgThreshold = 0.25;         % ǰ����ֵ

%%    ��ϸ�˹��ز�������
% get the first image as current (reference) image
currentIm               = double( imread( ['video/' fileList(1).name] ) );
% get the height and width of the image
[ height, width ]       = size( currentIm );
% 
w                       = zeros( height , width , numOfGau );                                   % ��˹��Ȩ��ϵ��
backgroundImMean        = zeros( height , width , numOfGau );                                   % ��˹��ֵ
backgroundImVar         = sigmaInit*ones( height , width , numOfGau );                          % ��˹����
matchCount              = ones( height , width , numOfGau );                                    % ƥ�����
diffCurToMean           = zeros( height , width , numOfGau );                                   % ��ǰͼ�����˹��ֵ�Ĳ���

backgroundMask          = zeros( height , width );
backgroundIm            = zeros( height , width );


%%    ��ȡ���ݴ���
for frameIndex = 1 : length( fileList );
    
    %%%%    ��ȡ��ǰͼ��
    currentIm           = double( imread( ['video/' fileList(frameIndex).name] ) );
    
    if 1 == frameIndex
        
        %%%%    ����ǵ�һ֡�� ��ʼ����ϸ�˹�ľ�ֵͼ��(��ֵΪ��ʼͼ�񣬷���ϴ�
        w(:,:,1)                = 1;                                                                    % ��һ����˹Ȩ��ϵ��Ϊ1
        backgroundImMean(:,:,1) = currentIm;                                                            % ��һ����˹�ֲ��ľ�ֵ��Ϊ��ʼͼ��
        
    else
        %%%%    ����Ǻ���֡�����ж��Ƿ�Ϊǰ��
        diffCurToMean = abs( repmat(currentIm,[1 1 numOfGau]) - backgroundImMean );
        
        %%%%    Ϊ�˱������⣬�˴���ʼ����������﷨д
        for dh = 1 : height
        for dw = 1 : width 
            
            for dc = 1 : numOfGau

                match = 0;                                                                                                                              % ��ʼ��Ϊû��ƥ���
                if diffCurToMean(dh,dw,dc) < sigmaRatio*backgroundImVar(dh,dw,dc)
                    match       = 1;                                                                                                                    % ��ƥ��
                    w(dh,dw,dc) = (1-alpha)*w(dh,dw,dc) + alpha;
                    p           = alpha/w(dh,dw,dc);                                                                                                    % ������������в��죬ֱ����Ȩ�ش��������
                    backgroundImMean(dh,dw,dc)  = (1-p)*backgroundImMean(dh,dw,dc) + p*currentIm(dh,dw);                                                % ���¾�ֵ
                    backgroundImVar(dh,dw,dc)   = sqrt((1-p)*(backgroundImVar(dh,dw,dc)^2) + p*(currentIm(dh,dw) - backgroundImMean(dh,dw,dc))^2);      % ���·���

                    if matchCount(dh,dw,dc) <= 255                                                                                                         % 255Ϊ���ƥ�����
                        matchCount(dh,dw,dc) = matchCount(dh,dw,dc) + 1;  
                    end 
                end

                % û��ƥ��ĸ�˹�������µĸ�˹ȡ��������������������Ǹ�
                if 0 == match 
                    [ minWValue, minWIndex ]            = min( w( dh , dw , : ) );                                      % ����Ȩֵ��С����һ��  
                    matchCount(dh,dw,minWIndex)         = 1;                                                            % ƥ�������ʼ��Ϊ1  
                    w(dh,dw,minWIndex)                  = 1 / ( sum(matchCount(dh,dw,:)) - 1 );                         % ȨֵΪ������˹�ֲ�ƥ�����֮�͵ĵ���  
                    backgroundImMean(dh,dw,minWIndex)   = currentIm(dh,dw);                                             % ��ʼ����ֵ
                    backgroundImVar(dh,dw,minWIndex)    = sigmaInit;                                                    % ��ʼ������
                end

            end
            
            %%%%    ��һ��Ȩ��ϵ��
            wSum        = sum( w(dh,dw,:) );  
            w(dh,dw,:)  = w(dh,dw,:) / wSum;
            
            %%%%    ���㱳������ֵ�Լ�ǰ���ж�
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
        
        %%%%    ��ʾ���
        ShowResult( currentIm , backgroundMask , backgroundIm );
    end
    
end