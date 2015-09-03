cd('C:\Users\shayo\Dropbox (MIT)\Code\Xray Multiple View\RadEye4Matlab');
h=RadEyeWrapper('Init')
if h ~= 1
    fprintf('Error initializing camera\n');
    return;
end

% Single image grabbing
exposureTimeMS = 990;
overheadMS = 300; % might be faster on fast computers...
RadEyeWrapper('GrabSingleFrame',exposureTimeMS); % non-blocking
tic, while toc < (exposureTimeMS+overheadMS)/1000, end
RadEyeWrapper('GetBufferSize') % should return 1
I=RadEyeWrapper('GetImageBuffer');

% Continuous acqusition
% Note that exposure time is actually fixed
fps = 2.5;
RadEyeWrapper('StartContinuous', fps);

for k=1:10
    fprintf('%s Last Image Timestamp: %d, Num Images In Buffer: %d\n',datestr(now),RadEyeWrapper('getNumTrigs'),RadEyeWrapper('GetBufferSize'));
    tic, while toc < 0.5, end
end
RadEyeWrapper('StopContinuous');
I=RadEyeWrapper('GetImageBuffer');

figure(11);
clf;
imagesc(mean(I,3))


%% For triggering the x-ray

% Option 2)
% Ramp X-ray voltage and current
% Call RadEyeWrapper('StartContinuous'); 
