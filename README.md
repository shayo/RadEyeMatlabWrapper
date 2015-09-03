A matlab wrapper for GigE x-ray camera (RadEye)
====================

This repository contains a matlab wrapper (mex file) to interface with RadEye 4 (1024x2048).

Driver installation
===================
Download and install ebus-vision_package_4.0.3.2015 (http://www.elvitec.fr/modules/ftp/indexbis.php?dossier=/Pleora/SDK-LOGICIEL)


Compilation 
====
Compiled and tested under Visual Studio 2012 (64 bit)
Make sure you have the following enviornment variables:
MATLAB64 (should point to where you installed matlab)
IPORT (should be automatically setup when you install eBUS SDK)

API
=====
Pretty straight forward. Call RadEyeWrapper with the appropriate parameters.

1) First, initialize the wrapper by calling:

RadEyeWrapper('Init')

2) Single image grabbing
exposureTimeMS = 990;
overheadMS = 300; % might be faster on fast computers...
RadEyeWrapper('GrabSingleFrame',exposureTimeMS); % non-blocking
tic, while toc < (exposureTimeMS+overheadMS)/1000, end
RadEyeWrapper('GetBufferSize') % should return 1
I=RadEyeWrapper('GetImageBuffer');

3) Continuous acqusition
fps = 2.5; % Close to theoretical maximum, which is 2.7
RadEyeWrapper('StartContinuous', fps);

for k=1:10
    fprintf('%s Last Image Timestamp: %d, Num Images In Buffer: %d\n',datestr(now),RadEyeWrapper('getNumTrigs'),RadEyeWrapper('GetBufferSize'));
    tic, while toc < 0.5, end
end
RadEyeWrapper('StopContinuous');
I=RadEyeWrapper('GetImageBuffer'); % note, I is 3D (y,x,z)

LIMITATION
===========

Current implementation is limited to a single camera (first one identified is used).
Should be straightforward to extend this to multiple cameras...
