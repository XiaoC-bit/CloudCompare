//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.

#include <stdio.h>
#include <string.h>
#include <windows.h>
#include <stdlib.h>
#include <mutex>

#include "LJS8_IF.h"
#include "LJS8_ErrorCode.h"

#include "LJS8_ACQ.h"

#pragma comment(lib, "winmm.lib")

using namespace std;

// Static variable
static LJS8IF_ETHERNET_CONFIG _ethernetConfig[MAX_LJS8_DEVICENUM];
static int _highSpeedPortNo[MAX_LJS8_DEVICENUM];
static int _imageAvailable[MAX_LJS8_DEVICENUM];
static int _lastImageSizeHeight[MAX_LJS8_DEVICENUM];
static LJS8_ACQ_GETPARAM _getParam[MAX_LJS8_DEVICENUM];
static LJS8IF_PROFILE_HEADER _aProfileHeader[MAX_LJS8_DEVICENUM];
static unsigned short *_apwHeightBuf[MAX_LJS8_DEVICENUM];
static unsigned char *_apbyLuminanceBuf[MAX_LJS8_DEVICENUM];

// Function prototype
void myCallbackFunc(LJS8IF_PROFILE_HEADER* pProfileHeaderArray, WORD* pwHeightProfileArray, BYTE* pbyLuminanceProfileArray, DWORD dwLuminanceEnable, DWORD dwProfileDataCount, DWORD dwCount, DWORD dwNotify, DWORD dwUser);

extern "C"
{
	LJS8_ACQ_API int LJS8_ACQ_OpenDevice(int lDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig, int HighSpeedPortNo) {
		int errCode = LJS8IF_EthernetOpen(lDeviceId, pEthernetConfig);

		_ethernetConfig[lDeviceId] = *pEthernetConfig;
		_highSpeedPortNo[lDeviceId] = HighSpeedPortNo;
		printf("[@(LJS8_ACQ_OpenDevice) Open device](0x%x)\n", errCode);

		return errCode;
	}

	LJS8_ACQ_API void LJS8_ACQ_CloseDevice(int lDeviceId) {
		LJS8IF_FinalizeHighSpeedDataCommunication(lDeviceId);
		LJS8IF_CommunicationClose(lDeviceId);
		printf("[@(LJS8_ACQ_CloseDevice) Close device]\n");
	}

	LJS8_ACQ_API int LJS8_ACQ_Acquire(int lDeviceId, unsigned short* pwHeightImage, unsigned char* pbyLuminanceImage, LJS8_ACQ_SETPARAM* pSetParam, LJS8_ACQ_GETPARAM* pGetParam) {
		int errCode;

		int timeout_ms = pSetParam->timeout_ms;
		int useExternalTrigger = pSetParam->useExternalTrigger;

		//Initialize
		errCode = LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(lDeviceId, &_ethernetConfig[lDeviceId], _highSpeedPortNo[lDeviceId], &myCallbackFunc, lDeviceId);
		printf("[@(LJS8_ACQ_Acquire) Initialize HighSpeed](0x%x)\n", errCode);

		//PreStart
		LJS8IF_HIGH_SPEED_PRE_START_REQ startReq;
		startReq.bySendPosition = 2;
		LJS8IF_HEIGHT_IMAGE_INFO heightImageInfo;

		errCode = LJS8IF_PreStartHighSpeedDataCommunication(lDeviceId, &startReq, (byte)pSetParam->usePcImageFilter, &heightImageInfo);
		printf("[@(LJS8_ACQ_Acquire) PreStart](0x%x)\n", errCode);

		//Allocate memory
		_apwHeightBuf[lDeviceId] = (unsigned short*)malloc(heightImageInfo.wYLineNum * MAX_LJS8_XDATANUM * sizeof(unsigned short));
		if (_apwHeightBuf[lDeviceId] == NULL) {
			return LJS8IF_RC_ERR_NOMEMORY;
		}

		_apbyLuminanceBuf[lDeviceId] = (unsigned char*)malloc(heightImageInfo.wYLineNum * MAX_LJS8_XDATANUM * sizeof(unsigned char));
		if (_apbyLuminanceBuf[lDeviceId] == NULL) {
			if (_apwHeightBuf[lDeviceId] != NULL)
			{
				free(_apwHeightBuf[lDeviceId]);
				_apwHeightBuf[lDeviceId] = NULL;
			}
			return LJS8IF_RC_ERR_NOMEMORY;
		}

		//Start HighSpeed
		_imageAvailable[lDeviceId] = 0;
		_lastImageSizeHeight[lDeviceId] = 0;

		errCode = LJS8IF_StartHighSpeedDataCommunication(lDeviceId);
		printf("[@(LJS8_ACQ_Acquire) Start HighSpeed](0x%x)\n", errCode);

		//StartMeasure
		if (useExternalTrigger > 0) {
		}
		else {
			errCode = LJS8IF_Trigger(lDeviceId);
			printf("[@(LJS8_ACQ_Acquire) Measure Start](0x%x)\n", errCode);
		}

		// Acquire. Polling to confirm complete.
		// Or wait until a timeout occurs.
		printf(" [@(LJS8_ACQ_Acquire) acquring image...]\n");
		DWORD start = timeGetTime();
		while (true)
		{
			DWORD ts = timeGetTime() - start;
			if ((DWORD)timeout_ms < ts) {
				break;
			}
			if (_imageAvailable[lDeviceId]) break;
		}

		if (_imageAvailable[lDeviceId] != 1) {
			printf(" [@(LJS8_ACQ_Acquire) timeout]\n");

			//Stop HighSpeed
			errCode = LJS8IF_StopHighSpeedDataCommunication(lDeviceId);
			printf("[@(LJS8_ACQ_Acquire) Stop HighSpeed](0x%x)\n", errCode);

			//Free memory
			if (_apwHeightBuf[lDeviceId] != NULL) {
				free(_apwHeightBuf[lDeviceId]);
				_apwHeightBuf[lDeviceId] = NULL;
			}

			if (_apbyLuminanceBuf[lDeviceId] != NULL) {
				free(_apbyLuminanceBuf[lDeviceId]);
				_apbyLuminanceBuf[lDeviceId] = NULL;
			}
			return LJS8IF_RC_ERR_TIMEOUT;
		}
		printf(" [@(LJS8_ACQ_Acquire) done]\n");

		//Stop HighSpeed
		errCode = LJS8IF_StopHighSpeedDataCommunication(lDeviceId);
		printf("[@(LJS8_ACQ_Acquire) Stop HighSpeed](0x%x)\n", errCode);

		//---------------------------------------------------------------------
		//  Organaize parameters related to acquired image 
		//---------------------------------------------------------------------

		_getParam[lDeviceId].luminance_enabled = heightImageInfo.byLuminanceOutput;
		_getParam[lDeviceId].x_pointnum = heightImageInfo.wXPointNum;
		_getParam[lDeviceId].y_linenum_acquired = _lastImageSizeHeight[lDeviceId];
		_getParam[lDeviceId].x_pitch_um = heightImageInfo.dwPitchX / 100.0f;
		_getParam[lDeviceId].y_pitch_um = heightImageInfo.dwPitchY / 100.0f;
		_getParam[lDeviceId].z_pitch_um = heightImageInfo.dwPitchZ / 100.0f;
		_getParam[lDeviceId].isProcTimeoutOccurred = _aProfileHeader[lDeviceId].byProcTimeout;

		*pGetParam = _getParam[lDeviceId];

		//---------------------------------------------------------------------
		//  Copy internal buffer to user buffer
		//---------------------------------------------------------------------
		int xDataNum = _getParam[lDeviceId].x_pointnum;

		unsigned short* pwHeightBuf = (unsigned short*)&_apwHeightBuf[lDeviceId][0];
		memcpy(pwHeightImage, pwHeightBuf, xDataNum * heightImageInfo.wYLineNum * sizeof(unsigned short));

		if (_getParam[lDeviceId].luminance_enabled > 0) {
			unsigned char* pbyLuminanceBuf = (unsigned char*)&_apbyLuminanceBuf[lDeviceId][0];
			memcpy(pbyLuminanceImage, pbyLuminanceBuf, xDataNum * heightImageInfo.wYLineNum * sizeof(unsigned char));
		}

		//Free memory
		if (_apwHeightBuf[lDeviceId] != NULL) {
			free(_apwHeightBuf[lDeviceId]);
			_apwHeightBuf[lDeviceId] = NULL;
		}

		if (_apbyLuminanceBuf[lDeviceId] != NULL) {
			free(_apbyLuminanceBuf[lDeviceId]);
			_apbyLuminanceBuf[lDeviceId] = NULL;
		}

		return LJS8IF_RC_OK;
	}

	LJS8_ACQ_API int LJS8_ACQ_StartAsync(int lDeviceId, LJS8_ACQ_SETPARAM* pSetParam) {
		int errCode;

		int useExternalTrigger = pSetParam->useExternalTrigger;

		//Allocate memory
		if (_apwHeightBuf[lDeviceId] != NULL) {
			free(_apwHeightBuf[lDeviceId]);
			_apwHeightBuf[lDeviceId] = NULL;
		}

		if (_apbyLuminanceBuf[lDeviceId] != NULL) {
			free(_apbyLuminanceBuf[lDeviceId]);
			_apbyLuminanceBuf[lDeviceId] = NULL;
		}

		//Initialize
		errCode = LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(lDeviceId, &_ethernetConfig[lDeviceId], _highSpeedPortNo[lDeviceId], &myCallbackFunc, lDeviceId);
		printf("[@(LJS8_ACQ_StartAsync) Initialize HighSpeed](0x%x)\n", errCode);

		//PreStart
		LJS8IF_HIGH_SPEED_PRE_START_REQ startReq;
		startReq.bySendPosition = 2;
		LJS8IF_HEIGHT_IMAGE_INFO heightImageInfo;

		errCode = LJS8IF_PreStartHighSpeedDataCommunication(lDeviceId, &startReq, (byte)pSetParam->usePcImageFilter, &heightImageInfo);
		printf("[@(LJS8_ACQ_StartAsync) PreStart](0x%x)\n", errCode);

		_apwHeightBuf[lDeviceId] = (unsigned short*)malloc(heightImageInfo.wYLineNum * MAX_LJS8_XDATANUM * sizeof(unsigned short));
		_apbyLuminanceBuf[lDeviceId] = (unsigned char*)malloc(heightImageInfo.wYLineNum * MAX_LJS8_XDATANUM * sizeof(unsigned char));

		if (_apwHeightBuf[lDeviceId] == NULL) {
			if (_apbyLuminanceBuf[lDeviceId] != NULL)
			{
				free(_apbyLuminanceBuf[lDeviceId]);
				_apbyLuminanceBuf[lDeviceId] = NULL;
			}
			return LJS8IF_RC_ERR_NOMEMORY;
		}

		if (_apbyLuminanceBuf[lDeviceId] == NULL) {
			if (_apwHeightBuf[lDeviceId] != NULL)
			{
				free(_apwHeightBuf[lDeviceId]);
				_apwHeightBuf[lDeviceId] = NULL;
			}
			return LJS8IF_RC_ERR_NOMEMORY;
		}

		//Start HighSpeed
		_imageAvailable[lDeviceId] = 0;
		_lastImageSizeHeight[lDeviceId] = 0;

		errCode = LJS8IF_StartHighSpeedDataCommunication(lDeviceId);
		printf("[@(LJS8_ACQ_StartAsync) Start HighSpeed](0x%x)\n", errCode);

		//StartMeasure
		if (useExternalTrigger > 0) {
		}
		else {
			errCode = LJS8IF_Trigger(lDeviceId);
			printf("[@(LJS8_ACQ_StartAsync) Measure Start](0x%x)\n", errCode);
		}

		//---------------------------------------------------------------------
		//  Organaize parameters related to acquired image 
		//---------------------------------------------------------------------

		_getParam[lDeviceId].luminance_enabled = heightImageInfo.byLuminanceOutput;
		_getParam[lDeviceId].x_pointnum = heightImageInfo.wXPointNum;
		_getParam[lDeviceId].y_linenum_acquired = heightImageInfo.wYLineNum;
		_getParam[lDeviceId].x_pitch_um = heightImageInfo.dwPitchX / 100.0f;
		_getParam[lDeviceId].y_pitch_um = heightImageInfo.dwPitchY / 100.0f;
		_getParam[lDeviceId].z_pitch_um = heightImageInfo.dwPitchZ / 100.0f;

		return LJS8IF_RC_OK;
	}

	LJS8_ACQ_API int LJS8_ACQ_AcquireAsync(int lDeviceId, unsigned short* pwHeightImage, unsigned char* pbyLuminanceImage, LJS8_ACQ_SETPARAM* pSetParam, LJS8_ACQ_GETPARAM* pGetParam) {
		int errCode;

		//Allocated memory?
		if (_apwHeightBuf[lDeviceId] == NULL || _apbyLuminanceBuf[lDeviceId] == NULL) {
			return LJS8IF_RC_ERR_NOMEMORY;
		}

		if (_imageAvailable[lDeviceId] != 1) {
			return LJS8IF_RC_ERR_TIMEOUT;
		}
		printf(" [@(LJS8_ACQ_AcquireAsync) done]\n");

		//Stop HighSpeed
		errCode = LJS8IF_StopHighSpeedDataCommunication(lDeviceId);
		printf("[@(LJS8_ACQ_AcquireAsync) Stop HighSpeed](0x%x)\n", errCode);

		_getParam[lDeviceId].isProcTimeoutOccurred = _aProfileHeader[lDeviceId].byProcTimeout;
		*pGetParam = _getParam[lDeviceId];

		//---------------------------------------------------------------------
		//  Copy internal buffer to user buffer
		//---------------------------------------------------------------------
		int xDataNum = _getParam[lDeviceId].x_pointnum;

		unsigned short* pwHeightBuf = (unsigned short*)&_apwHeightBuf[lDeviceId][0];
		memcpy(pwHeightImage, pwHeightBuf, xDataNum* _getParam[lDeviceId].y_linenum_acquired * sizeof(unsigned short));

		if (_getParam[lDeviceId].luminance_enabled > 0) {
			unsigned char* pbyLuminanceBuf = (unsigned char*)&_apbyLuminanceBuf[lDeviceId][0];
			memcpy(pbyLuminanceImage, pbyLuminanceBuf, xDataNum * _getParam[lDeviceId].y_linenum_acquired * sizeof(unsigned char));
		}

		//Free memory
		if (_apwHeightBuf[lDeviceId] != NULL) {
			free(_apwHeightBuf[lDeviceId]);
			_apwHeightBuf[lDeviceId] = NULL;
		}

		if (_apbyLuminanceBuf[lDeviceId] != NULL) {
			free(_apbyLuminanceBuf[lDeviceId]);
			_apbyLuminanceBuf[lDeviceId] = NULL;
		}
		return LJS8IF_RC_OK;
	}
}

mutex mtx;

void myCallbackFunc(LJS8IF_PROFILE_HEADER * pProfileHeaderArray, WORD* pwHeightProfileArray, BYTE* pbyLuminanceProfileArray, DWORD dwLuminanceEnable, DWORD dwProfileDataCount, DWORD dwCount, DWORD dwNotify, DWORD dwUser)
{
	if (dwCount == 0) return;
	if (_imageAvailable[dwUser] == 1) return;

	lock_guard<mutex> lock(mtx);

	_aProfileHeader[dwUser] = *pProfileHeaderArray;

	memcpy(&_apwHeightBuf[dwUser][0], pwHeightProfileArray, dwProfileDataCount * dwCount * sizeof(WORD));

	if (dwLuminanceEnable == 1)
	{
		memcpy(&_apbyLuminanceBuf[dwUser][0], pbyLuminanceProfileArray, dwProfileDataCount * dwCount * sizeof(BYTE));
	}

	_imageAvailable[dwUser] = 1;
	_lastImageSizeHeight[dwUser] = dwCount;
}
