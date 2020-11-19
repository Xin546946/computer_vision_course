

function [inputIm initialLSF] = GetImgAndInitialLSF( imgID , initContourSelectInd );

    inputIm = imread(['data/' num2str(imgID) '.bmp']);
    %inputIm = imread('bears.jpg');
    inputIm = double(inputIm(:,:,1));
    
    rho     = 2;
    switch imgID
        case 1
            initialLSF = ones(size(inputIm)).*rho;
            initialLSF(20:70,20:100) = -rho;
            [u,v] = size(inputIm);
            % initialLSF(0.1*u:0.9*u,0.1*v:0.9*v) = -rho;
    end

end