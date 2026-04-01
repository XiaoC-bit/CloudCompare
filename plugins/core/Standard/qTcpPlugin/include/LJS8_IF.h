//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.
/** @file
@brief	LJS8_IF Header
*/

#pragma once
#pragma managed(push, off)

#ifdef LJS8_IF_EXPORT
#define LJS8_IF_API __declspec(dllexport)
#else
#define LJS8_IF_API __declspec(dllimport)
#endif

/// Setting value storage level designation
typedef enum {
	LJS8IF_SETTING_DEPTH_WRITE		= 0x00,		// Write settings area
	LJS8IF_SETTING_DEPTH_RUNNING	= 0x01,		// Running settings area
	LJS8IF_SETTING_DEPTH_SAVE		= 0x02,		// Save area
} LJS8IF_SETTING_DEPTH;

/// Initialization target setting item designation
typedef enum {
	LJS8IF_INIT_SETTING_TARGET_PRG0		= 0x00,		// Program 0
	LJS8IF_INIT_SETTING_TARGET_PRG1		= 0x01,		// Program 1
	LJS8IF_INIT_SETTING_TARGET_PRG2		= 0x02,		// Program 2
	LJS8IF_INIT_SETTING_TARGET_PRG3		= 0x03,		// Program 3
	LJS8IF_INIT_SETTING_TARGET_PRG4		= 0x04,		// Program 4
	LJS8IF_INIT_SETTING_TARGET_PRG5		= 0x05,		// Program 5
	LJS8IF_INIT_SETTING_TARGET_PRG6		= 0x06,		// Program 6
	LJS8IF_INIT_SETTING_TARGET_PRG7		= 0x07,		// Program 7
	LJS8IF_INIT_SETTING_TARGET_PRG8		= 0x08,		// Program 8
	LJS8IF_INIT_SETTING_TARGET_PRG9		= 0x09,		// Program 9
	LJS8IF_INIT_SETTING_TARGET_PRG10	= 0x0A,		// Program 10
	LJS8IF_INIT_SETTING_TARGET_PRG11	= 0x0B,		// Program 11
	LJS8IF_INIT_SETTING_TARGET_PRG12	= 0x0C,		// Program 12
	LJS8IF_INIT_SETTING_TARGET_PRG13	= 0x0D,		// Program 13
	LJS8IF_INIT_SETTING_TARGET_PRG14	= 0x0E,		// Program 14
	LJS8IF_INIT_SETTING_TARGET_PRG15	= 0x0F,		// Program 15
} LJS8IF_INIT_SETTING_TARGET;

/// Get height image data position specification method designation
typedef enum {
	LJS8IF_HEIGHT_IMAGE_POSITION_CURRENT		= 0x00,		// From current
	LJS8IF_HEIGHT_IMAGE_POSITION_SPEC			= 0x02,		// Specify position
	LJS8IF_HEIGHT_IMAGE_POSITION_COMMITTED		= 0x03,		// From current after height image commitment
} LJS8IF_HEIGHT_IMAGE_POSITION;

/// Version info structure
typedef struct {
	INT	nMajorNumber;		// Major number
	INT	nMinorNumber;		// Minor number
	INT	nRevisionNumber;	// Revision number
	INT	nBuildNumber;		// Build number
} LJS8IF_VERSION_INFO;

/// Ethernet settings structure
typedef struct {
	BYTE	abyIpAddress[4];	// The IP address of the head to connect to.
	WORD	wPortNo;			// The port number of the head to connect to.
	BYTE	reserve[2];			// Reserved
} LJS8IF_ETHERNET_CONFIG;

/// Setting item designation structure
typedef struct {
	BYTE	byType;			// Setting type
	BYTE	byCategory;		// Category
	BYTE	byItem;			// Setting item
	BYTE	reserve;		// Reserved
	BYTE	byTarget1;		// Setting Target 1
	BYTE	byTarget2;		// Setting Target 2
	BYTE	byTarget3;		// Setting Target 3
	BYTE	byTarget4;		// Setting Target 4
} LJS8IF_TARGET_SETTING;

/// HeightImage information structure
typedef struct {
	WORD	wXPointNum;			// Number of data in X direction.
	WORD	wYLineNum;			// Number of lines in Y direction.
	BYTE	byLuminanceOutput;	// Whether luminance output is on.
	BYTE	reserve[3];			// Reserved
	LONG	lXStart;			// X coordinate of 1st data.
	DWORD	dwPitchX;			// Data pitch in X direction.
	LONG	lYStart;			// Y coordinate of 1st data.
	DWORD	dwPitchY;			// Data pitch in Y direction.
	BYTE	reserve2[4];		// Reserved
	DWORD	dwPitchZ;			// Data pitch in Z direction.
} LJS8IF_HEIGHT_IMAGE_INFO;

/// Profile header information structure
typedef struct {
	DWORD	reserve;		// Reserved
	DWORD	dwHeightImageNo;// The height image number
	DWORD	dwProfileNo;	// The profile number
	BYTE	byProcTimeout;	// Indicates that a timeout occurred in stray light suppression processing.
	BYTE	reserve2[11];	// Reserved
} LJS8IF_PROFILE_HEADER;

/// Profile footer information structure
typedef struct {
	DWORD	reserve;	// Reserved
} LJS8IF_PROFILE_FOOTER;

/// Get profile request structure
typedef struct {
	BYTE	reserve;			// Reserved
	BYTE	byPositionMode;		// The get profile position specification method
	BYTE	reserve2[2];		// Reserved
	DWORD	dwGetHeightImageNo;	// The height image number for the profile to get
	DWORD	dwGetProfileNo;		// The profile number to start getting profiles from in the specified height image number.
	WORD	wGetProfileCount;	// The number of profiles to read.
	BYTE	byErase;			// Specifies whether to erase the profile data that was read and the profile data older than that.
	BYTE	reserve3;			// Reserved
} LJS8IF_GET_HEIGHT_IMAGE_PROFILE_REQUEST;

/// Get profile response structure
typedef struct {
	DWORD	dwCurrentHeightImageNo;				// The height image number at the current point in time.
	DWORD	dwCurrentHeightImageProfileCount;	// The number of profiles in the newest height image.
	DWORD	dwOldestHeightImageNo;				// The height image number for the oldest height image held by the head.
	DWORD	dwOldestHeightImageProfileCount;	// The number of profiles in the oldest height image held by the head.
	DWORD	dwGetHeightImageNo;					// The height image number that was read this time.
	DWORD	dwGetHeightImageProfileCount;		// The number of profiles in the height image that was read this time.
	DWORD	dwGetHeightImageTopProfileNo;		// The oldest profile number in the height image out of the profiles that were read this time.
	WORD	wGetProfileCount;					// The number of profiles that were read this time.
	BYTE	byCurrentHeightImageCommitted;		// The height image measurements for the newest height image number has finished.
	BYTE	reserve;							// Reserved
} LJS8IF_GET_HEIGHT_IMAGE_PROFILE_RESPONSE;

/// High-speed communication prep start request structure
typedef struct {
	BYTE	bySendPosition;			// Send start position
	BYTE	reserve[3];				// Reserved
} LJS8IF_HIGH_SPEED_PRE_START_REQ;


/**
Callback function interface for high-speed data communication
@param	pProfileHeaderArray		A pointer to the buffer that stores the header data array.
@param	pHeightProfileArray		A pointer to the buffer that stores the profile data array.
@param	pLuminanceProfileArray		A pointer to the buffer that stores the luminance profile data array.
@param	dwLuminanceEnable		The value indicating whether luminance data output is enable or not.
@param	dwProfileDataCount		The data count of one profile.
@param	dwCount		The number of profile or header data stored in buffer.
@param	dwNotify	Notification of an interruption in high-speed communication or a break in height image measurements.
@param	dwUser		User information
*/
typedef void(_cdecl *LJS8IF_CALLBACK_SIMPLE_ARRAY)(LJS8IF_PROFILE_HEADER* pProfileHeaderArray, WORD* pHeightProfileArray, BYTE* pLuminanceProfileArray, DWORD dwLuminanceEnable, DWORD dwProfileDataCount, DWORD dwCount, DWORD dwNotify, DWORD dwUser);


extern "C"
{
	// Functions
	// Operations for the DLL
	/**
	Initializes the DLL
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_Initialize(void);

	/**
	Finalize DLL
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_Finalize(void);

	/**
	Get DLL version
	@return	DLL version
	*/
	LJS8_IF_API LJS8IF_VERSION_INFO WINAPI LJS8IF_GetVersion(void);

	/**
	Ethernet communication connection
	@param	lDeviceId		The communication device to communicate with.
	@param	pEthernetConfig	Ethernet communication settings
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_EthernetOpen(LONG lDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig);
	
	/**
	Disconnect communication path
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_CommunicationClose(LONG lDeviceId);


	// System control
	/**
	Reboot the head
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_Reboot(LONG lDeviceId);

	/**
	Return to factory state
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_ReturnToFactorySetting(LONG lDeviceId);

	/**
	Control Laser
	@param	lDeviceId	The communication device to communicate with.
	@param	byState		Laser state
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_ControlLaser(LONG lDeviceId, BYTE byState);

	/**
	Get system error information
	@param	lDeviceId		The communication device to communicate with.
	@param	byReceivedMax	The maximum amount of system error information to receive
	@param	pbyErrCount		The buffer to receive the amount of system error information.
	@param	pwErrCode		The buffer to receive the system error information.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetError(LONG lDeviceId, BYTE byReceivedMax, BYTE* pbyErrCount, WORD* pwErrCode);

	/**
	Clear system error
	@param	lDeviceId	The communication device to communicate with.
	@param	wErrCode	The error code for the error you wish to clear.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_ClearError(LONG lDeviceId, WORD wErrCode);

	/**
	Get head temperature
	@param	lDeviceId				The communication device to communicate with.
	@param	pnSensorTemperature		The buffer to receive sensor temperature.
	@param	pnProcessor1Temperature	The buffer to receive processor1 temperature.
	@param	pnProcessor2Temperature	The buffer to receive processor2 temperature.
	@param	pnCaseTemperature		The buffer to receive case temperature.
	@param	pnDriveUnitTemperature	The buffer to receive drive unit temperature.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetHeadTemperature(LONG lDeviceId, SHORT* pnSensorTemperature, SHORT* pnProcessor1Temperature, SHORT* pnProcessor2Temperature, SHORT* pnCaseTemperature, SHORT* pnDriveUnitTemperature);

	/**
	Get head model
	@param	lDeviceId			The communication device to communicate with.
	@param	pHeadModel			The buffer to receive head model.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetHeadModel(LONG lDeviceId, CHAR* pHeadModel);

	/**
	Get serial Number
	@param	lDeviceId			The communication device to communicate with.
	@param	pSerialNo			The buffer to receive serial number
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetSerialNumber(LONG lDeviceId, CHAR* pSerialNo);

	/**
	Get current attention status value
	@param	lDeviceId			The communication device to communicate with.
	@param	pwAttentionStatus	The buffer to receive attention status
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetAttentionStatus(LONG lDeviceId, WORD* pwAttentionStatus);

	// Measurement control
	/**
	Trigger
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_Trigger(LONG lDeviceId);

	/**
	Clear memory
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_ClearMemory(LONG lDeviceId);


	// Functions related to modifying or reading settings
	/**
	Send setting
	@param	lDeviceId		The communication device to communicate with.
	@param	byDepth			The level to reflect the setting value
	@param	TargetSetting	The item that is the target
	@param	pData			The buffer that stores the setting data
	@param	dwDataSize		The size in BYTEs of the setting data
	@param	pdwError		Detailed setting error
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_SetSetting(LONG lDeviceId, BYTE byDepth, LJS8IF_TARGET_SETTING TargetSetting, void* pData, DWORD dwDataSize, DWORD* pdwError);

	/**
	Get setting
	@param	lDeviceId		The communication device to communicate with.
	@param	byDepth			The level of the setting value to get.
	@param	TargetSetting	The item that is the target
	@param	pData			The buffer to receive the setting data
	@param	dwDataSize		The size of the buffer to receive the acquired data in BYTEs.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetSetting(LONG lDeviceId, BYTE byDepth, LJS8IF_TARGET_SETTING TargetSetting, void* pData, DWORD dwDataSize);

	/**
	Initialize setting
	@param	lDeviceId	The communication device to communicate with.
	@param	byDepth		The level to reflect the initialized setting.
	@param	byTarget	The setting that is the target for initialization.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_InitializeSetting(LONG lDeviceId, BYTE byDepth, BYTE byTarget);

	/**
	Request to reflect settings in the write settings area
	@param	lDeviceId	The communication device to communicate with.
	@param	byDepth		The level to reflect the setting value
	@param	pdwError	Detailed setting error
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_ReflectSetting(LONG lDeviceId, BYTE byDepth, DWORD* pdwError);

	/**
	Update write settings area
	@param	lDeviceId	The communication device to communicate with.
	@param	byDepth		The level of the settings to update the write settings area with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_RewriteTemporarySetting(LONG lDeviceId, BYTE byDepth);

	/**
	Check the status of saving to the save area
	@param	lDeviceId	The communication device to communicate with.
	@param	pbyBusy		Other than 0: Accessing the save area, 0: no access.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_CheckMemoryAccess(LONG lDeviceId, BYTE* pbyBusy);

	/**
	Change program
	@param	lDeviceId	The communication device to communicate with.
	@param	byProgramNo	Program number after the change.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_ChangeActiveProgram(LONG lDeviceId, BYTE byProgramNo);

	/**
	Get the active program number
	@param	lDeviceId		The communication device to communicate with.
	@param	pbyProgramNo	The buffer to receive the active program number.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetActiveProgram(LONG lDeviceId, BYTE* pbyProgramNo);

	/**
	Get height image profiles by simple array format
	@param	lDeviceId				The communication device to communicate with.
	@param	pReq					The position, etc., of the profiles to get.
	@param	byUsePCImageFilter		Specifies whether to use PC image filter.
	@param	pRsp					The position, etc., of the profiles that were actually acquired.
	@param	pHeightImageInfo		The information for the acquired height images.
	@param	pProfileHeaderArray		The buffer to get array of header.
	@param	pHeightProfileArray		The buffer to get array of profile data.
	@param	pLuminanceProfileArray	The buffer to get array of luminance profile data.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_GetHeightImageSimpleArray(LONG lDeviceId, LJS8IF_GET_HEIGHT_IMAGE_PROFILE_REQUEST* pReq, BYTE byUsePCImageFilter, LJS8IF_GET_HEIGHT_IMAGE_PROFILE_RESPONSE* pRsp, LJS8IF_HEIGHT_IMAGE_INFO* pHeightImageInfo, LJS8IF_PROFILE_HEADER* pProfileHeaderArray, WORD* pHeightProfileArray, BYTE* pLuminanceProfileArray);

	/**
	Initialize Ethernet high-speed data communication for simple array
	@param	lDeviceId				The communication device to communicate with.
	@param	pEthernetConfig			The Ethernet settings used in high-speed communication.
	@param	wHighSpeedPortNo		The port number used in high-speed communication.
	@param	pCallBackSimpleArray	The callback function to call when data is received by high-speed communication.
	@param	dwThreadId				Thread ID.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(LONG lDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig, WORD wHighSpeedPortNo,
		LJS8IF_CALLBACK_SIMPLE_ARRAY pCallBackSimpleArray, DWORD dwThreadId);

	/**
	Request preparation before starting high-speed data communication
	@param	lDeviceId			The communication device to communicate with.
	@param	pReq				What data to send high-speed communication from.
	@param	byUsePCImageFilter	Specifies whether to use PC image filters.
	@param	pHeightImageInfo	Stores the height-image information.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_PreStartHighSpeedDataCommunication(LONG lDeviceId, LJS8IF_HIGH_SPEED_PRE_START_REQ* pReq, BYTE byUsePCImageFilter, LJS8IF_HEIGHT_IMAGE_INFO* pHeightImageInfo);

	/**
	Start high-speed data communication
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_StartHighSpeedDataCommunication(LONG lDeviceId);

	/**
	Stop high-speed data communication
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_StopHighSpeedDataCommunication(LONG lDeviceId);

	/**
	Finalize high-speed data communication
	@param	lDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	LJS8_IF_API LONG WINAPI LJS8IF_FinalizeHighSpeedDataCommunication(LONG lDeviceId);
};
#pragma managed(pop)