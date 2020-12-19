
clear all;
close all;
clc;

%%%%    ��Ƶ����
filePath                = './data25/';                    	% ��Ƶ�ļ���λ��
fileType                = '*.jpg';                         	% ��Ƶ�ļ�������
fileList                = dir( [filePath fileType] );     	% �����ļ���
movLength               = length( fileList(:,1) );        	% ��Ƶ�ĳ���
startFrm                = 1;                              	% ��ʼ֡��
endFrm                  = movLength;                       	% ���֡��

%%%%    Mean-shift ����
minDist     = 0.1;                                       	% Mean-shift ������ֵ
maxIterNum  = 15;                                           % Mean-shift ����������
incre       = 7;                                            % Mean-shift ��ѡ������������ ( ���ԣ�7 , 3 , 0 )

%%%%    ֱ��ͼBins
redBins     = 16;
greenBins   = 16;
blueBins    = 16;

%%%%    ��ȡһ֡����
initFrame = imread( [filePath fileList(1,1).name] );

%%%%    ��ʼ����Ϣ
cMin        = 300;
cMax        = 321;
rMin        = 112;
rMax        = 164;

center(1,1) = floor((rMin+rMax+1)/2);                       % ���ĵ�
center(1,2) = floor((cMin+cMax+1)/2);                       %                   
halfSize(1) = round(abs(rMax-rMin)/2);                      % ����һ���С 
halfSize(2) = round(abs(cMax-cMin)/2);                      % 

%%%%    ����Ŀ��ֱ��ͼ
q_u         = GetTargetHistogram( double(initFrame), center, halfSize);   

%%%%    ����
figure(10001)
set(gcf,'DoubleBuffer','on');                               % ����˫����
[height width ch] = size(initFrame);                     


for frameIndex = startFrm : 1 : endFrm                     
    if frameIndex == startFrm                          
        currentFrame = initFrame;
    elseif frameIndex > 1
        currentFrame            = imread( [filePath fileList(frameIndex,1).name] );
        [center backprojection] = MeanShiftTrackingCore( double(currentFrame) , center , halfSize , q_u , minDist , maxIterNum , incre );  
        
        %%%%    ����ֱ��ͼ
        q_u2 = GetTargetHistogram( double(currentFrame), center, halfSize); 
        if sum(sqrt(q_u2).*sqrt(q_u)) > 0.85
            q_u = (1-0.001)*q_u + 0.001*q_u2;
        end
    end  
    
    %%%%    �߽紦��
    rMin=center(1)-halfSize(1);
    rMax=center(1)+halfSize(1);
    cMin=center(2)-halfSize(2);
    cMax=center(2)+halfSize(2);
    
    if rMin < 2
        rMin = 2;               
    end
    if rMax > height-1     
        rMax = height-1;        
    end    
    if cMin < 2            
        cMin = 2;               
    end    
    if cMax > width-1      
        cMax = width-1          
    end
    
    trace(frameIndex,1) = rMin;
    trace(frameIndex,2) = rMax;
    trace(frameIndex,3) = cMin;
    trace(frameIndex,4) = cMax;
    
    %%%%    �����������ʾ
    trackFrame = currentFrame;                 
    for r= rMin:rMax
        trackFrame(r, cMin-1:cMin,:) = 0;      
        trackFrame(r, cMin-1:cMin,3) = 255;    
        trackFrame(r, cMax:cMax+1,:) = 0;        
        trackFrame(r, cMax:cMax+1,3) = 255;
    end
    for c= cMin:cMax
        trackFrame(rMin-1:rMin, c,:) = 0;
        trackFrame(rMin-1:rMin, c,3) = 255;
        trackFrame(rMax:rMax+1, c,:) = 0;        
        trackFrame(rMax:rMax+1, c,3) = 255;
    end   
    
    figure(10001)
    imshow( trackFrame );
    title( [num2str(frameIndex) , '/' , num2str(movLength)] );
    pause( 0.1 );
    
end