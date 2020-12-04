

clear all;
close all;
clc;

%%%%    �õ��ļ��б�
fileList = dir( 'video\*.bmp' );

%%%%    ��������
sigmaRatio  = 2.5;          % �ж��Ƿ�Ϊǰ������ֵ
alpha       = 0.01;         % ѧϰ�ʣ����������ٶ�
sigmaInit   = 25;           % ��ʼ����

%%%%    ��ȡ���ݴ���
for frameIndex = 1 : length( fileList );
    
    %%%%    ��ȡ��ǰͼ��
    currentIm           = double( imread( ['video\' fileList(frameIndex).name] ) );
    [ height width ]    = size( currentIm );
    
    if 1 == frameIndex
        %%%%    ����ǵ�һ֡�� ��ʼ����˹�ľ�ֵͼ��(��ֵΪ��ʼͼ�񣬷���ϴ�
        backgroundImMean    = currentIm;
        backgroundImVar     = ones( size( currentIm ) ) * sigmaInit;
    else
        %%%%    ����Ǻ���֡�����ж��Ƿ�Ϊǰ��
        backgroundMask      = double(abs(currentIm-backgroundImMean) < backgroundImVar * sigmaRatio);
        backgroundImMean    = ((1-alpha)*backgroundImMean + alpha*currentIm).*backgroundMask+backgroundImMean.*(1-backgroundMask);
        backgroundImVar     = sqrt((1-alpha)*backgroundImVar.^2 + alpha*(currentIm-backgroundImMean).^2);
        
        %%%%    ��ʾ���
        ShowResult( currentIm , backgroundMask , backgroundImMean );
    end
    
end