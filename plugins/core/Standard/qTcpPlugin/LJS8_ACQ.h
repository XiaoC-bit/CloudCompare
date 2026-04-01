//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.


#ifndef _LJS8_ACQ_H
#define _LJS8_ACQ_H

typedef struct {
	int		timeout_ms;
	int		useExternalTrigger;
	int		usePcImageFilter;
} LJS8_ACQ_SETPARAM;

typedef struct {
	int		luminance_enabled;
	int		x_pointnum;
	int		y_linenum_acquired;
	float	x_pitch_um;
	float	y_pitch_um;
	float	z_pitch_um;
	int		isProcTimeoutOccurred;
} LJS8_ACQ_GETPARAM;

const int MAX_LJS8_DEVICENUM = 6;
const int MAX_LJS8_XDATANUM = 3200;
const unsigned int BUFFER_FULL_COUNT = 30000;

#ifdef LJS8_ACQ_API_EXPORT
#define LJS8_ACQ_API  
#else
#define LJS8_ACQ_API  
#endif

extern "C"
{
	LJS8_ACQ_API int LJS8_ACQ_OpenDevice(int lDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig, int HighSpeedPortNo);
	LJS8_ACQ_API void LJS8_ACQ_CloseDevice(int lDeviceId);

	//Blocking I/F
	LJS8_ACQ_API int LJS8_ACQ_Acquire(int lDeviceId, unsigned short* pwHeightImage, unsigned char* pbyLuminanceImage, LJS8_ACQ_SETPARAM* pSetParam, LJS8_ACQ_GETPARAM* pGetParam);

	//Non-blocking I/F
	LJS8_ACQ_API int LJS8_ACQ_StartAsync(int lDeviceId, LJS8_ACQ_SETPARAM* pSetParam);
	LJS8_ACQ_API int LJS8_ACQ_AcquireAsync(int lDeviceId, unsigned short* pwHeightImage, unsigned char* pbyLuminanceImage, LJS8_ACQ_SETPARAM* pSetParam, LJS8_ACQ_GETPARAM* pGetParam);
}

#endif /* _LJS8_ACQ_H */
