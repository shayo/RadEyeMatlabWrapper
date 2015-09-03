/*
Teledynedalsa X-Ray Detector Matlab Wrapper
Programmed by Shay Ohayon
DiCarlo Lab @ MIT

Revision History
Version 0.1 9/1/2015, Initial code from Teledynedalsa imported
                      Deinterlaced function re-implemented to remove the dependency on 32-bit
					  Currently only supports 2048 x 1024 cameras (hard coded).
*/
#include <stdio.h>
#include "mex.h"

#include <CyXMLDocument.h>
#include <CyConfig.h>
#include <CyGrabber.h>
#include <CyImageBuffer.h>
#include <CyDeviceFinder.h>

#include <Windows.h>
#include <queue>
#include <deque>

#define MIN(a,b) (a)<(b)?(a):(b)
#define MAX(a,b) (a)>(b)?(a):(b)
bool calledOnce = false;

using namespace std;

class myImage 
{
public:
	myImage();
	myImage(int width, int height, short *_imageData, unsigned long _frameCounter);
	~myImage();
	short *imageData;
	unsigned long frameCounter;
};

myImage::myImage()
{
	imageData = NULL;
	frameCounter = -1;
}

	myImage::myImage(int width, int height, short *_imageData, unsigned long _frameCounter) : frameCounter(_frameCounter)
	{
		imageData = new short[width*height];
		memcpy(imageData,_imageData, width*height*sizeof(short));
	}

	myImage::~myImage()
	{
		delete imageData;
	}




template<typename T, typename Container=std::deque<T> >
class iterable_queue : public std::queue<T,Container>
{
public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    iterator begin() { return this->c.begin(); }
    iterator end() { return this->c.end(); }
    const_iterator begin() const { return this->c.begin(); }
    const_iterator end() const { return this->c.end(); }
};


class RadEyeWrapper {
public:
		RadEyeWrapper();
		~RadEyeWrapper();
		bool isInitialized();
		bool init();
		int getWidth();
		int getHeight();
		void frameCallback(short *imageData);
		int getNumImagesInBuffer();

		void setExposure(float value);
		float getExposure();
		int getBytesPerPixel();
		void clearBuffer();
		int getNumTrigs();

		void setTrigger(bool state);
		void resetTriggerCounter();
		bool isThreadRunning();
	bool getImage();
	bool isTriggeredEnabled();

	int copyAndClearBuffer(unsigned char *imageBufferPtr, int N);

void stopContinuousMode();
void startContinuousMode(double framteRateHz);
void grabSingleFrame(double exposureTimeMS);

void generateSinglePulse();
	void threadTerminated();

private:
	void stoptInternalPulseGeneration();

	void myDeinterlace(short *I, int nCols, int nRows);
	void startInternalPulseGeneration(bool periodic);
	bool Init_iPORT();


	void startThread();
    void stopThread();

	unsigned long numTrig;
	bool triggerEnabled;
	void lockMutex();
	void unlockMutex();

	int bytesPerPixel;
	void release();

	bool initialized;
	bool deviceOpened;
	bool streaming;
	int lastResult;
	bool triggered;
	HANDLE ghMutex; 
	int width, height;
	unsigned long maxImagesInBuffer;
	int mutexCount;
	unsigned long trigsSinceBufferRead;
	iterable_queue<myImage*> imageQueue;
	int imageFrameCounter;
	
	CyGrabber* m_cyGrabber;

	short *m_lpFrameBase1;
	bool threadRunning;
	DWORD   threadID;
	HANDLE threadHandle ;
	bool threadExittedNormally;
	int pulseDelayMS;
	bool inContinuousMode;
		int threadTrigCounter,numImagesToCaptureBeforeThreadTerminate ;
};

void RadEyeWrapper::threadTerminated()
{
	threadExittedNormally = true;
}

bool RadEyeWrapper::isThreadRunning()
{
	return threadRunning;
}

void RadEyeWrapper::setTrigger(bool state)
{
	triggerEnabled = state;
}

bool RadEyeWrapper::isTriggeredEnabled()
{
		return triggerEnabled;
}

bool RadEyeWrapper::isInitialized()
{
	return initialized;
}

void RadEyeWrapper::resetTriggerCounter()
{
	numTrig = 0;

}
int RadEyeWrapper::getNumTrigs()
{
	return numTrig;
}

int RadEyeWrapper::getNumImagesInBuffer()
{
	int n;
	lockMutex();
	n = (int) imageQueue.size();
	unlockMutex();
	return n;
}


void RadEyeWrapper::lockMutex()
{
	mutexCount++;
       int dwWaitResult = WaitForSingleObject( 
            ghMutex,    // handle to mutex
            INFINITE);  // no time-out interval
}

void RadEyeWrapper::unlockMutex()
{
	ReleaseMutex(ghMutex);
	mutexCount--;
}


void RadEyeWrapper::frameCallback(short *imageData)
{
	lockMutex();
	triggered = true;
	threadTrigCounter++;
	numTrig++;
	trigsSinceBufferRead++;

	myImage* pImage = new myImage(width, height, imageData,numTrig);
	imageQueue.push(pImage);
	
	if (imageQueue.size() > maxImagesInBuffer)
	{
		myImage *p = imageQueue.front();
		delete p;
		imageQueue.pop();
	}

	if (numImagesToCaptureBeforeThreadTerminate > 0 && 
		threadTrigCounter >= numImagesToCaptureBeforeThreadTerminate)
	{
		threadRunning = false;
		// also turn off the pulse generator
		stoptInternalPulseGeneration();
	}


	unlockMutex();
}


void RadEyeWrapper::clearBuffer()
{
	lockMutex();
	int N = imageQueue.size();	
	for (int k = 0; k < N;k++)
	{
		myImage *p = imageQueue.front();
		delete p;
		imageQueue.pop();
	}
	unlockMutex();
}
int RadEyeWrapper::getWidth()
{
	return width;
}

int RadEyeWrapper::getHeight()
{
	return height;
}


void RadEyeWrapper::release()
{
	if (initialized )
	{
		if (deviceOpened)
		{
			delete m_cyGrabber;
				mexPrintf("OK!\n");
		}

		mexPrintf("Closing mutex...");
		CloseHandle(ghMutex);
		mexPrintf("OK!\n");
		ghMutex = NULL;
		initialized = false;
	}
	mexPrintf("Release sequence finished!\n");
}

/*
void OnImageGrabbed(myImage* pImage, const void* pCallbackData)
{
	RadEyeWrapper *cls = (RadEyeWrapper*)pCallbackData;
	if (cls->triggerEnabled) 
	{
		cls->lockMutex();
		myImage *deepcopy = new myImage;
		deepcopy->DeepCopy(pImage);
		cls->frameCallback(deepcopy);
		cls->unlockMutex();
	}
}
*/



bool RadEyeWrapper::Init_iPORT()
{
	// Step 1: Discover the IP engines your PC can see
	CyDeviceFinder lFinder;
	CyDeviceFinder::DeviceList lIPEngineList;

	// Step 1.1: All IP engines available through High Performance Driver ("Mode 0")
	lFinder.Find(CY_DEVICE_ACCESS_MODE_DRV, lIPEngineList, 500, true);

	// Step 1.2: All IP engines available through GigE/eBus Driver ("Mode 6")
	lFinder.Find(CY_DEVICE_ACCESS_MODE_GEV_EBUS, lIPEngineList, 500, false);

	// Step 1.3: All IP engines available through GigE Driver ("Mode 5")
	lFinder.Find(CY_DEVICE_ACCESS_MODE_GEV, lIPEngineList, 500, false);

	// Step 1.4: All IP engines available through eBus Driver ("Mode 4")
	lFinder.Find(CY_DEVICE_ACCESS_MODE_EBUS, lIPEngineList, 500, false);

	// Step 1.5: All IP engines available through Network Stack ("Mode 1")
	lFinder.Find(CY_DEVICE_ACCESS_MODE_UDP, lIPEngineList, 500, false);

	if (lIPEngineList.size() < 1) 
		return false; // no IP Engine found

	// Step 2: Create a configuration to connect to the first IP engine in the list
	// Step 2.1: Retrieve the first IP engine from the list
	const CyDeviceFinder::DeviceEntry &lIPEngine = lIPEngineList[0];

	// Step 2.2: Add an entry to the CyConfig object
	CyString mAddress;
	CyConfig lConfig;
	lConfig.AddDevice();

	// Step 2.3: Configure the CyConfig object's connection parameters
	lConfig.SetParameter( CY_CONFIG_PARAM_ACCESS_MODE, lIPEngine.mMode );
	lConfig.SetParameter( CY_CONFIG_PARAM_ADDRESS_IP,  lIPEngine.mAddressIP );
	lConfig.SetParameter( CY_CONFIG_PARAM_ADDRESS_MAC, lIPEngine.mAddressMAC );
	lConfig.SetParameter( CY_CONFIG_PARAM_ADAPTER_ID,  lIPEngine.mAdapterID.GetIdentifier() );

	// A packet payload size of 1440 bytes is a safe value for all connection modes
	lConfig.SetParameter( CY_CONFIG_PARAM_PACKET_SIZE, 1440 );

	// Step 2.4: Set desired timeouts (these are the default values)
	lConfig.SetParameter( CY_CONFIG_PARAM_PACKET_TIMEOUT,       500 );
	lConfig.SetParameter( CY_CONFIG_PARAM_ANSWER_TIMEOUT,       1000 );
	lConfig.SetParameter( CY_CONFIG_PARAM_FIRST_PACKET_TIMEOUT, 30000 );
	lConfig.SetParameter( CY_CONFIG_PARAM_REQUEST_TIMEOUT,      30000 );

	// Step 2.5: Set the connection topology to unicast
	lConfig.SetParameter( CY_CONFIG_PARAM_DATA_SENDING_MODE, CY_DEVICE_DSM_UNICAST );
	lConfig.SetParameter( CY_CONFIG_PARAM_DATA_SENDING_MODE_MASTER, true );

	// Step 3: Create and connect a CyGrabber object to the IP Engine.
	m_cyGrabber = new CyGrabber;
	CyResult cyError = m_cyGrabber->Connect( lConfig );

	if (cyError != CY_RESULT_OK)
	{	
		// try again with Pleora DeviceFinder GUI...
		CyDeviceFinder::DeviceEntry lEntry;
		if (lFinder.SelectDevice(lEntry, NULL) == CY_RESULT_OK)
		{
			lConfig.SetParameter( CY_CONFIG_PARAM_ACCESS_MODE, lEntry.mMode );
			lConfig.SetParameter( CY_CONFIG_PARAM_ADDRESS_IP,  lEntry.mAddressIP );
			lConfig.SetParameter( CY_CONFIG_PARAM_ADDRESS_MAC, lEntry.mAddressMAC );
			lConfig.SetParameter( CY_CONFIG_PARAM_ADAPTER_ID,  lEntry.mAdapterID.GetIdentifier() );
			cyError = m_cyGrabber->Connect( lConfig, CY_DEVICE_FLAG_SET_IP_ON_CONNECT );
		}
		if (cyError != CY_RESULT_OK)
		{	
			// do something to process error...
			return false;
		}
	}
	lConfig.GetParameter( CY_CONFIG_PARAM_ADDRESS_IP, mAddress );

	// Step 4: Create a buffer for grabbing images and set image properties in the CyGrabber object
	// Step 4.1: Set the image size (edit these based on camera parameters)
	unsigned long nCols = 2*1024;
	unsigned long nRows = 1024;
	unsigned long nXoffset = 8;
	unsigned long nYoffset = 0;
	unsigned long nImageSize = nCols * nRows * 2; // Grayscale 16

	// Step 4.2: Create two image buffers.
	m_lpFrameBase1 = (short*)malloc(nImageSize);
	if (!m_lpFrameBase1 )
	{
		// do something to process error...
		return false;
	}

	// Step 4.3: Apply settings to the IP engine
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_SIZE_X,      nCols );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_SIZE_Y,      nRows );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_OFFSET_X,    nXoffset );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_OFFSET_Y,    nYoffset );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_IMAGE_SIZE,  nImageSize );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_PIXEL_DEPTH, 16 );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_TAP_QUANTITY, 1 );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_PACKED,      false );
	m_cyGrabber->SetParameter( CY_GRABBER_PARAM_NORMALIZED,  false );
	m_cyGrabber->SaveConfig(); // This saves the cached values to the grabber


	return true;	// all done
}

void RadEyeWrapper::startInternalPulseGeneration(bool periodic)
{
		// Step 5: Program PLC to generate sync pulse for camera and set ctrl inputs high
	CyDevice& lDevice = m_cyGrabber->GetDevice();
	if (!lDevice.HasExtension(CY_DEVICE_EXT_PULSE_GENERATOR))
	{
		// do something to process error...
		return ;
	}
	CyDeviceExtension* lExtension = NULL;
	lExtension = &lDevice.GetExtension(CY_DEVICE_EXT_PULSE_GENERATOR);
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_WIDTH, 10); // width (high) = 10ms
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_DELAY, pulseDelayMS); // delay (low) = 990ms (edit this as needed)
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_GRANULARITY, 33333); // granularity factor (x30ns)
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_PERIODIC, periodic); // emit periodic pulse
	lExtension->SaveToDevice();

	// Reverse enginerring by shay:

	// The following sets up the communication between Pleora FPGA and Shadow-cam FPGA
	// Pin configuration:
	// 0 corresponds to the pulse generator OUTPUT pin (prrobably on Pleora)
	// 4 corresponds to Frame valid pin on Shadow-cam FPGA (goes high whenever a frame acqusition starts).

	// To set TTL_OUT0, use the Q0 register.
	// For example, Q0 is linked to I7 (CY_GPIO_LUT_PARAM_INPUT_CONFIG7), which is set to pin 0 (pulse generator)
	// To set TTL_OUT1, use Q1 register
	// For example, Q1 is linked to I2 (CY_GPIO_LUT_PARAM_INPUT_CONFIG2), which is set to pin 4 (frame valid)

	// "Camera Frame Valid" (1/2.7*1000 = 370.37 ms).
	
	// Q4-Q6 registers are linked to camera controls pins (unknown function)
	// Q7 register is connected to the FRAME_SYNC pin in the shadow-cam FPGA and is linked to pin 0 (i.e., the pulse generator).
	CyString lProgram = "Q0 = I7\r\nQ1 = I2\r\nQ4 = 1\r\nQ5 = 1\r\nQ6 = 1\r\nQ7 = I7\r\n";
	
	lExtension = &lDevice.GetExtension(CY_DEVICE_EXT_GPIO_LUT);
	lExtension->SetParameter(CY_GPIO_LUT_PARAM_INPUT_CONFIG2, 4); // I2: camera frame valid
	lExtension->SetParameter(CY_GPIO_LUT_PARAM_INPUT_CONFIG7, 0); // I7: pulse generator 0 output

	lExtension->SetParameter(CY_GPIO_LUT_PARAM_GPIO_LUT_PROGRAM, lProgram);
	lExtension->SaveToDevice();

}

void RadEyeWrapper::generateSinglePulse()
{
		CyDevice& lDevice = m_cyGrabber->GetDevice();
	if (!lDevice.HasExtension(CY_DEVICE_EXT_PULSE_GENERATOR))
	{
		// do something to process error...
		return ;
	}
	CyDeviceExtension* lExtension = NULL;
	lExtension = &lDevice.GetExtension(CY_DEVICE_EXT_PULSE_GENERATOR);
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_WIDTH, 10); // width (high) = 10ms
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_DELAY, pulseDelayMS); // delay (low) = 990ms (edit this as needed)
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_GRANULARITY, 33333); // granularity factor (x30ns)
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_PERIODIC, false); // emit periodic pulse
	lExtension->SaveToDevice();


}

void RadEyeWrapper::stoptInternalPulseGeneration()
{
	CyDevice& lDevice = m_cyGrabber->GetDevice();
	if (!lDevice.HasExtension(CY_DEVICE_EXT_PULSE_GENERATOR))
	{
		// do something to process error...
		return ;
	}
	CyDeviceExtension* lExtension = NULL;
	lExtension = &lDevice.GetExtension(CY_DEVICE_EXT_PULSE_GENERATOR);
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_WIDTH, 10); // width (high) = 10ms
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_DELAY, pulseDelayMS); // delay (low) = 990ms (edit this as needed)
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_GRANULARITY, 33333); // granularity factor (x30ns)
	lExtension->SetParameter(CY_PULSE_GEN_PARAM_PERIODIC, false); // emit periodic pulse
	lExtension->SaveToDevice();
}


bool RadEyeWrapper::init() 
{
	if (initialized)
		return true;

	if (!Init_iPORT())
		return false;

	bytesPerPixel = 2; // 12 bit ADC
	
	width = 2048;
	height = 1024;

	deviceOpened = true;
	streaming = true;
	initialized = true;
	mexPrintf("Camera initialized and in capture mode\n");
   return true;
}

RadEyeWrapper::RadEyeWrapper() : initialized(false), deviceOpened(false), streaming(false),triggered(false),numTrig(0)
{
	m_cyGrabber = NULL;
    ghMutex = CreateMutex( 
        NULL,              // default security attributes
        FALSE,             // initially not owned
        NULL);             // unnamed mutex
	maxImagesInBuffer = 250; // ~ 1GB
	triggerEnabled = true;
	trigsSinceBufferRead = 0;
	mutexCount = 0;
	threadRunning = false;
	pulseDelayMS = 990; // default: 1 second acqusition
	inContinuousMode = false;
}


RadEyeWrapper::~RadEyeWrapper()
{
	release();
}


void RadEyeWrapper::myDeinterlace(short *I, int nCols, int nRows)
{
	// The Shadow cameras are always read in parallel, this means
	// that image data coming off the device looks like this:
	// 1, 513,  1025, 1537,  2,  514, 1026, 1538,...
	// This needs to be reorganized like this:
	// 1,2,3,....
	

	// first, make a lookup table, then map values accordingly...
	int *map = new int[nCols];
	for (int k=0;k<512;k++)
	{
		map[512*0+k] = 4*k+0;
		map[512*1+k] = 4*k+1;
		map[512*2+k] = 4*k+2;
		map[512*3+k] = 4*k+3;

	}
	short *deinterlaced = new short[nCols*nRows];
	for (int y=0; y<nRows;y++)
	{
		for (int x=0; x< nCols;x++)
		{
			int out_ind = nCols*y+x;
			int in_ind = nCols*y + map[x];
			deinterlaced[out_ind] = I[in_ind];
		}
	}
	
	memcpy(I,deinterlaced, nCols *nRows *sizeof(short));
	delete deinterlaced;
	delete map;
}



bool RadEyeWrapper::getImage()
{
	// edit these based on camera parameters
	unsigned long nCols = 1024*2;
	unsigned long nRows = 1024;
	unsigned long nSize = nCols * nRows * sizeof(short);
	unsigned long nBytesWritten;

	// grab image and wait
	CyResult cyError = m_cyGrabber->Grab(CyChannel(0), (unsigned char *)m_lpFrameBase1, nSize);
	if ( cyError != CY_RESULT_OK )
	{
		// do something to process error...
		return false;
	}

	// if acquiring 12-bit data, we need to shift image data to align on LSB
	// (skip this if using 14-bit data)
	unsigned short* pBuf = (unsigned short*)m_lpFrameBase1;
	unsigned short* endBuf = pBuf + nCols*nRows;
	while (pBuf < endBuf) *pBuf++ = *pBuf>>2;

	// refer to SIL User's Manual for more information on ScDeinterlace function, or skip this step
	//ScDeinterlace(m_lpFrameBase1, nCols, nCols, nRows, SCCAMTYPE_2048, FALSE);
	myDeinterlace(m_lpFrameBase1, nCols, nRows);

	frameCallback(m_lpFrameBase1); // Deep copy


}

int RadEyeWrapper::copyAndClearBuffer(unsigned char *imageBufferPtr, int N){

	trigsSinceBufferRead = 0;

	unsigned short *imageBufferIntPtr = (unsigned short *)imageBufferPtr;
	lockMutex();
	int numCopied = 0;
	
	int numToCopy = MIN(imageQueue.size(),N);
	int FirstImageTrig = (numToCopy > 0) ? (imageQueue.front())->frameCounter : -1;
	unsigned short Pixel = 0;
	for (long k=0;k<numToCopy;k++)
	{
		myImage *I = imageQueue.front();

		unsigned short *data16bits = (unsigned short*) I->imageData;
		long long offset = ((long long )width*(long long )height*k);

		int counter = 0;
		for (long y=0;y<height;y++)
		{
			for (long x=0;x<width;x++)
			{
	  			    imageBufferIntPtr[offset + (long long)((y)+x*(long)height)]=data16bits[counter];
					counter++;
			}
		}
		
		
		imageQueue.pop();
		delete I;
	}
	unlockMutex();

	return FirstImageTrig;

}

DWORD WINAPI thread_blocking_get_image(LPVOID lpParam)
{
	RadEyeWrapper *cls = (RadEyeWrapper*)lpParam;
	
	while (cls->isThreadRunning())
	{

		if (cls->isTriggeredEnabled()) 
		{
			cls->getImage();
		} else
		{
			Sleep(100);
		}
	}
	cls->threadTerminated();
	ExitThread(0);
	return 0;
}

void RadEyeWrapper::stopThread()
{
	threadRunning = false;
	Sleep(1000); // Allow thread to safely terminate
	// Ideally, the thread has stopped by now...

}


void RadEyeWrapper::stopContinuousMode()
{
	if (inContinuousMode) {

		stoptInternalPulseGeneration();
		stopThread();
		inContinuousMode = false;
	}
}


void RadEyeWrapper::grabSingleFrame(double exposureTimeMS)
{
	if (inContinuousMode)
	{
		mexPrintf("Already in continuous acuisition\n");
		return;
	}
	pulseDelayMS = exposureTimeMS;
	inContinuousMode = true;
	numImagesToCaptureBeforeThreadTerminate = 1;
	threadTrigCounter = 0;
	startInternalPulseGeneration(true);
	startThread();
}

void RadEyeWrapper::startContinuousMode(double framteRateHz)
{
	inContinuousMode = true;
	pulseDelayMS = 1.0/framteRateHz * 1000 - 10;
	numImagesToCaptureBeforeThreadTerminate = -1; // Inf
	startInternalPulseGeneration(true);
	startThread();
}

void RadEyeWrapper::startThread()
{
	
threadRunning = true;
threadExittedNormally = false;
 threadHandle = CreateThread(
			NULL,                   // default security attributes
			0,                      // use default stack size  
			thread_blocking_get_image,       // thread function name
			this,          // argument to thread function 
			0,                      // use default creation flags 
			&threadID);   // returns the thread identifier 
			

}
RadEyeWrapper *camera=nullptr;


void exitFunction()
{
	if (camera != nullptr)
		delete camera;
}

void mexFunction( int nlhs, mxArray *plhs[], 
				 int nrhs, const mxArray *prhs[] ) {


if (nrhs == 0)
	return;
 int StringLength = int(mxGetNumberOfElements(prhs[0])) + 1;
 char* Command = new char[StringLength];
 if (mxGetString(prhs[0], Command, StringLength) != 0){
		mexErrMsgTxt("\nError extracting the command.\n");
		return;
} else if (strcmp(Command, "Init") == 0) {
	 mexAtExit(exitFunction);
	 if (camera != nullptr)
		 delete camera;

	camera = new RadEyeWrapper();
	bool Success = camera->init();
	 plhs[0] = mxCreateDoubleScalar(Success);
	 delete Command;

	 return;
 } 
 if (strcmp(Command, "IsInitialized") == 0) {
	 if (camera==nullptr)
		plhs[0] = mxCreateDoubleScalar(false);
	 else
		 plhs[0] = mxCreateDoubleScalar(camera->isInitialized());
	 return;
 }

 if (camera == nullptr)
 {
	 mexErrMsgTxt("You need to call Initialize first!.\n");
	 delete Command;
     plhs[0] = mxCreateDoubleScalar(0);
	 return;
 }

 if (strcmp(Command, "Release") == 0) {
	 if (camera != nullptr)
	 {
		 delete camera;
		 camera = NULL;
		 mexPrintf("Camera handles released.\n");
	 }
	  plhs[0] = mxCreateDoubleScalar(1);
 } 	else if (strcmp(Command,"GrabSingleFrame") == 0)
 {
	double exposureTimeMS;
	  if (nrhs > 1)
		 exposureTimeMS = *(double*)mxGetPr(prhs[1]);
	  else
	  {
		  mexPrintf("Please specify exposure time  (MS)\n");
		  return;
	  }
	  camera->grabSingleFrame( exposureTimeMS);
 }
 
 else if (strcmp(Command,"StartContinuous") == 0)
 {
	 double framteRateHz;
	  if (nrhs > 1) {
		 framteRateHz = *(double*)mxGetPr(prhs[1]);
		
	  }
	  else
	  {
		  mexPrintf("Please specify frame rate (Hz)\n");
		  return;
	  }
	 camera->startContinuousMode( framteRateHz);
 }  else if (strcmp(Command,"GetBufferSize") == 0)
 {
	 plhs[0] = mxCreateDoubleScalar(camera->getNumImagesInBuffer());
 }	else if (strcmp(Command,"StopContinuous") == 0)
 {
	 camera->stopContinuousMode();
 }
  else if (strcmp(Command,"GetImageBuffer") == 0) {
	 int N = camera->getNumImagesInBuffer();

	 if (nrhs > 1)
	 {
		 // grab a subset of images
		int requestedNumberOfImages = (int)*(double*)mxGetPr(prhs[1]);
		N = MIN(N, requestedNumberOfImages);
	 }

	 int w = camera->getWidth();
	 int h = camera->getHeight();
	 mwSize dim[3] = {h,w,N};
	 mxArray* imageBuffer =  mxCreateNumericArray(3, dim, mxUINT16_CLASS, mxREAL);
	
	 if (imageBuffer == nullptr)
	 {
		 mexPrintf("Error allocating memory for buffer.\n");
	 }

	 unsigned char*imageBufferPtr = (unsigned char*)mxGetData(imageBuffer);
	 
	 plhs[0] = imageBuffer;
     plhs[1] = mxCreateDoubleScalar(camera->copyAndClearBuffer(imageBufferPtr, N));
  }
 else  if (strcmp(Command, "ClearBuffer") == 0)
 {
	 camera->clearBuffer();
	 plhs[0] = mxCreateDoubleScalar(1);
 }
 else  if (strcmp(Command, "ResetTriggerCounter") == 0)
 {
	 camera->resetTriggerCounter();
	 plhs[0] = mxCreateDoubleScalar(1);
 }
 else  if (strcmp(Command, "TriggerOFF") == 0)
 {
	 camera->setTrigger(false);
	 plhs[0] = mxCreateDoubleScalar(1);
 }
 else  if (strcmp(Command, "TriggerON") == 0)
 {
	 camera->setTrigger(true);
	 plhs[0] = mxCreateDoubleScalar(1);
 }
 else  if (strcmp(Command, "getNumTrigs") == 0)
 {
	 plhs[0] = mxCreateDoubleScalar(camera->getNumTrigs());
 }
 
 else {
	 mexPrintf("Error. Unknown command\n");
 }

 delete Command;

}
