

%%%%    Mean-shift �����㷨�ĺ���

function [center backprojection] = MeanshiftTrackingCore( currentFrame , center , halfSize , q_u , minDist , maxIterNum , incre )

sum_p           = 0; 
histo           = zeros(16,16,16);      % ��ʼ����ѡĿ��

iterations      = 0;               % 
centerOld       = center;

% Ŀ���λ��
rMin = center(1)-halfSize(1)-incre;
rMax = center(1)+halfSize(1)+incre;
cMin = center(2)-halfSize(2)-incre;
cMax = center(2)+halfSize(2)+incre;

[height width ch]  = size( currentFrame );
    
while 1
    
    %%%%    �����ѡĿ��ֱ��ͼ
    wmax=(rMin-center(1)).^2+(cMin-center(2)).^2+1;   
    for i = rMin:rMax
        for j = cMin:cMax
            if (i>=1 & i<=height & j>=1 & j<=width)
                d = (i-center(1)).^2+(j-center(2)).^2;
                w = wmax-d;
                R = floor(currentFrame(i,j,1)/16)+1;
                G = floor(currentFrame(i,j,2)/16)+1;
                B = floor(currentFrame(i,j,3)/16)+1;
                histo(R,G,B) = histo(R,G,B) + w;
            end
        end
    end

    for i=1:16
        for j=1:16
            for k=1:16
                index = (i-1)*256+(j-1)*16+k;
                p_u(index) = histo(i,j,k);
                sum_p = sum_p+p_u(index);
            end
        end
    end
  
    p_u=p_u/sum_p; 

    %%%%    ����Ȩ��ϵ�� w
    backprojection  = zeros( height , width );      % ����ͶӰֵ(Ȩ����ɵ�ͼ��) 
    n = 1;
    for i = rMin:rMax
        for j = cMin:cMax
            if (i>=1 & i<=height & j>=1 & j<=width)  
                R = floor(currentFrame(i,j,1)/16)+1;
                G = floor(currentFrame(i,j,2)/16)+1;
                B = floor(currentFrame(i,j,3)/16)+1;
                u = (R-1)*256+(G-1)*16+B;
                x(1,n) = i;
                x(2,n) = j;                
                w_i(n) = sqrt(q_u(u)/p_u(u));
                backprojection(i,j)= w_i(n);                                                                
                n = n+1;
            end
        end
    end

    centerR     = (x*w_i'/sum(w_i))';                   % �µ�λ��   
    MS          = sqrt(sum((centerR-centerOld).^2));    % Ư�������ĳ���
    iterations  = iterations+1;                         % ��������
    
    % �ж��Ƿ����
    if (MS<minDist | iterations>=maxIterNum)
        break;
    end
    
    centerOld   = centerR;                          
    center      = floor(centerR);                     

    rMin = center(1)-halfSize(1)-incre;         
    rMax = center(1)+halfSize(1)+incre;
    cMin = center(2)-halfSize(2)-incre;
    cMax = center(2)+halfSize(2)+incre;
    
    histo   = zeros(16,16,16);                     % reinitilize candiate model
    sum_p   = 0;         
end

end

