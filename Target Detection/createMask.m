function [BW,maskedRGBImage] = createMask(RGB) 

% Convert RGB image to HSV image
I = rgb2hsv(RGB);
% Define thresholds for 'Hue'. Modify these values to filter out different range of colors.
%channel1Min = 0.965;
%channel1Max = 0.188;
channel1Min = 0.920;
channel1Max = 0.070;
% Define thresholds for 'Saturation'
channel2Min = 0.500;
channel2Max = 1.000;
% Define thresholds for 'Value'
channel3Min = 0.250;
channel3Max = 0.875;
% Create mask based on chosen histogram thresholds
BW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;