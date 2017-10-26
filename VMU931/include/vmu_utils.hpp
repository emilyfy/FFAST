#ifndef FFAST_VMU931_VMU_UTILS_HPP_
#define FFAST_VMU931_VMU_UTILS_HPP_

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include <string>

#if defined(_WIN32)
#define VAR_OS_WINDOWS


#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
#define VAR_OS_UNIX


#if (defined(__APPLE__) && defined(__MACH__))
#define VAR_OS_MAC
#include <CoreFoundation/CFByteOrder.h>

#endif

#endif

/*
 ******************************************************************************
 * CONSTANTS and MACROS
 ******************************************************************************
 */

/// \cond
#ifdef __cplusplus
#define BEGIN_DECL extern "C" {
#define END_DECL }
#else
#define BEGIN_DECL
#define END_DECL
#endif

// Endian swap (OS independant)
#define is_bigendian() ( (*(char*)&i) == 0)
#define B0_MASK 0x000000FF
#define B1_MASK 0x0000FF00
#define B2_MASK 0x00FF0000
#define B3_MASK 0xFF000000
#define B0_BIT_SHIFT << 24u
#define B1_BIT_SHIFT << 8u
#define B2_BIT_SHIFT >> 8u
#define B3_BIT_SHIFT >> 24u
/// \endcond

/**
 *  \name Error codes
 *  @{
 */
#define VMU_ERR_SUCCESS 0   ///< Success
#define VMU_ERR_INV_ARG -1  ///< Invalid argument
#define VMU_ERR_INV_MESS -2 ///< Invalid message
#define VMU_ERR_FAILED -3   ///< Function failed
#define VMU_ERR_INV_CMD -4  ///< Invalid command
/// @}

/**
 *  \name Size of messages
 *  @{
 */
#define VMU_RAW_MESS_SIZE 255   ///< We don't expect the VMU to send a message longer than 255 bytes
#define VMU_STRING_MESS_SIZE VMU_RAW_MESS_SIZE  ///< We don't expect the VMU to send a string longer than 255 bytes
#define VMU_DATA_MESS_SIZE 32   ///< Length of a message containing data
#define VMU_DATA_XYZ_SIZE 16    ///< Length of the data part of a message containing either accelerometers, gyroscopes, magnetometers or Euler angles
#define VMU_DATA_WXYZ_SIZE 20   ///< Length of the data part of a message containing quaternions
#define VMU_DATA_H_SIZE 8       ///< Length of the data part of a message containing the heading
#define VMU_DATA_STATUS_SIZE 7  ///< Length of the data part of a message containing the status of the VMU
/// @}

/**
 *  \name Data message first/last bytes
 *  @{
 */
#define VMU_DATA_START 0x01 ///< First byte of every message containing data
#define VMU_DATA_END 0x04   ///< Last byte of every message containing data
/// @}

/**
 *  \name String message first/last bytes
 *  @{
 */
#define VMU_STRING_START 0x02   ///< First byte of every message containing text
#define VMU_STRING_END 0x03     ///< Last byte of every message containing text
/// @}

/**
 *  \name Offsets of the header bytes
 *  @{
 */
#define VMU_START_BYTE_OFFS 0   ///< First byte of a message
#define VMU_SIZE_OFFS 1     ///< Second byte a message (contains the size of the message)
#define VMU_TYPE_OFFS 2     ///< Third byte a message (contains the type of data in the message)
/// @}

/**
 *  \name Number of header/footer bytes
 *  @{
 */
#define VMU_NUM_HEADER_BYTES 3  ///< (First byte) + (Size of message) + (Type of data)
#define VMU_NUM_FOOTER_BYTES 1  ///< (Last byte)
#define VMU_NUM_HEAD_FOOT_BYTES (VMU_NUM_HEADER_BYTES + VMU_NUM_FOOTER_BYTES)   ///< Sum of the number of header and footer bytes
/// @}

/** \name Number of bytes to be sent when sending a command
 *  @{
 */
#define VMU_CMD_NUM_BYTES 4     ///< 'V' + 'A' + 'R' + (cmd)
/// @}

/**
 *  \name Sensors status masks
 *  @{
 */
#define VMU_STATUS_MASK_ACCEL_ON 0x01   ///< Accelerometers are turned ON if the bit is equal to 1
#define VMU_STATUS_MASK_GYRO_ON 0x02    ///< Gyroscopes are turned ON if the bit is equal to 1
#define VMU_STATUS_MASK_MAG_ON 0x04     ///< Magnetometers are turned ON if the bit is equal to 1
/// @}

/**
 *  \name Sensors resolution masks
 *  Only one of the *RES_ACCEL* bit can be equal to 1.\n
 *  Same goes for the *RES_GYRO* bits.
 *  @{
 */
#define VMU_STATUS_MASK_RES_ACCEL_2G 0x01   ///< Accelerometers resolution is set to 2g if the bit is equal to 1
#define VMU_STATUS_MASK_RES_ACCEL_4G 0x02   ///< Accelerometers resolution is set to 4g if the bit is equal to 1
#define VMU_STATUS_MASK_RES_ACCEL_8G 0x04   ///< Accelerometers resolution is set to 8g if the bit is equal to 1
#define VMU_STATUS_MASK_RES_ACCEL_16G 0x08  ///< Accelerometers resolution is set to 16g if the bit is equal to 1
#define VMU_STATUS_MASK_RES_GYRO_250DPS 0x10    ///< Gyroscopes resolution is set to 250dps if the bit is equal to 1
#define VMU_STATUS_MASK_RES_GYRO_500DPS 0x20    ///< Gyroscopes resolution is set to 500dps if the bit is equal to 1
#define VMU_STATUS_MASK_RES_GYRO_1000DPS 0x40   ///< Gyroscopes resolution is set to 1000dps if the bit is equal to 1
#define VMU_STATUS_MASK_RES_GYRO_2000DPS 0x80   ///< Gyroscopes resolution is set to 2000dps if the bit is equal to 1
/// @}

/**
 *  \name Low output rate status masks
 *  Low output = 200Hz\n
 *  High output = 1000Hz
 *  @{
 */
#define VMU_STATUS_MASK_LOW_OUTPUT_RATE 0x01    ///< Output rate is low if the bit is equal to 1
/// @}

/**
 *  \name Data currently streaming masks
 *  @{
 */
#define VMU_STATUS_MASK_STREAM_ACCEL 0x01   ///< Accelerometers are currently streaming if this bit is equal to 1
#define VMU_STATUS_MASK_STREAM_GYRO 0x02    ///< Gyroscopes are currently streaming if this bit is equal to 1
#define VMU_STATUS_MASK_STREAM_QUAT 0x04    ///< Quaternions are currently streaming if this bit is equal to 1
#define VMU_STATUS_MASK_STREAM_MAG 0x08     ///< Magnetometers are currently streaming if this bit is equal to 1
#define VMU_STATUS_MASK_STREAM_EULER 0x10   ///< Euler angles are currently streaming if this bit is equal to 1
#define VMU_STATUS_MASK_STREAM_HEADING 0x40 ///< Heading is currently streaming if this bit is equal to 1
/// @}

/**
 *  \name Accelerometer limit values
 *  @{
 */
#define VMU_ACCEL_2G_MIN (-2.0f)    ///< Minimum value of the accelerometers is the resolution is set to 2g
#define VMU_ACCEL_2G_MAX (2.0f)     ///< Maximum value of the accelerometers is the resolution is set to 2g
#define VMU_ACCEL_4G_MIN (-4.0f)    ///< Minimum value of the accelerometers is the resolution is set to 4g
#define VMU_ACCEL_4G_MAX (4.0f)     ///< Maximum value of the accelerometers is the resolution is set to 4g
#define VMU_ACCEL_8G_MIN (-8.0f)    ///< Minimum value of the accelerometers is the resolution is set to 8g
#define VMU_ACCEL_8G_MAX (8.0f)     ///< Maximum value of the accelerometers is the resolution is set to 8g
#define VMU_ACCEL_16G_MIN (-16.0f)  ///< Minimum value of the accelerometers is the resolution is set to 16g
#define VMU_ACCEL_16G_MAX (16.0f)   ///< Maximum value of the accelerometers is the resolution is set to 16g
/// @}

/**
 *  \name Gyroscope limit values
 *  @{
 */
#define VMU_GYRO_250DPS_MIN (-250.0f)   ///< Minimum value of the gyroscopes is the resolution is set to 250dps
#define VMU_GYRO_250DPS_MAX (250.0f)    ///< Maximum value of the gyroscopes is the resolution is set to 250dps
#define VMU_GYRO_500DPS_MIN (-500.0f)   ///< Minimum value of the gyroscopes is the resolution is set to 500dps
#define VMU_GYRO_500DPS_MAX (500.0f)    ///< Maximum value of the gyroscopes is the resolution is set to 500dps
#define VMU_GYRO_1000DPS_MIN (-1000.0f) ///< Minimum value of the gyroscopes is the resolution is set to 1000dps
#define VMU_GYRO_1000DPS_MAX (1000.0f)  ///< Maximum value of the gyroscopes is the resolution is set to 1000dps
#define VMU_GYRO_2000DPS_MIN (-2000.0f) ///< Minimum value of the gyroscopes is the resolution is set to 2000dps
#define VMU_GYRO_2000DPS_MAX (2000.0f)  ///< Maximum value of the gyroscopes is the resolution is set to 2000dps
/// @}

/**
 *  \name Magnetometer limit values
 *  @{
 */
#define VMU_MAG_MIN (-4800.0f)  ///< Minimum value of the magnetometers
#define VMU_MAG_MAX (4800.0f)   ///< Maximum value of the magnetometers
/// @}

/**
 *  \name Quaternion limit values
 *  @{
 */
#define VMU_QUAT_MIN (-1.0f)    ///< Minimum value of the quaternion
#define VMU_QUAT_MAX (1.0f)     ///< Maximum value of the quaternion
/// @}

/**
 *  \name Euler angles limit values
 *  @{
 */
#define VMU_EULER_X_MIN (-180.0f)   ///< Minimum value of the X element of the Euler angles
#define VMU_EULER_X_MAX (180.0f)    ///< Maximum value of the X element of the Euler angles
#define VMU_EULER_Y_MIN (-90.0f)    ///< Minimum value of the Y element of the Euler angles
#define VMU_EULER_Y_MAX (90.0f)     ///< Maximum value of the Y element of the Euler angles
#define VMU_EULER_Z_MIN (-180.0f)   ///< Minimum value of the Z element of the Euler angles
#define VMU_EULER_Z_MAX (180.0f)    ///< Maximum value of the Z element of the Euler angles
/// @}

/**
 *  \name Heading limit values
 *  @{
 */
#define VMU_HEADING_MIN (0.0f)      ///< Minimum value of the heading
#define VMU_HEADING_MAX (360.0f)    ///< Maximum value of the heading
/// @}


/*
 ******************************************************************************
 * STRUCT and ENUMS
 ******************************************************************************
 */

/// Types of message the VMU can send
typedef enum {
    mt_string,  ///< Message contains text
    mt_data     ///< Message contains data
} MessType;

/// Data types you can receive from the VMU
typedef enum {
    dt_accel = (uint8_t)'a',    //!< Accelerometers
    dt_gyro = (uint8_t)'g',     //!< Gyroscopes
    dt_mag = (uint8_t)'c',      //!< Magnetometers
    dt_quat = (uint8_t)'q',     //!< Quaternions
    dt_euler = (uint8_t)'e',    //!< Euler angles
    dt_heading = (uint8_t)'h',  //!< Heading
    dt_status = (uint8_t)'s'    //!< Status
} DataType;

/// Commands you can send to the VMU
typedef enum {
    cmd_toggle_accel = (uint8_t)'a',    //!< Toggle the streaming of the accelerometers
    cmd_toggle_gyro = (uint8_t)'g',     //!< Toggle the streaming of the gyroscopes
    cmd_toggle_mag = (uint8_t)'c',      //!< Toggle the streaming of the magnetometers
    cmd_toggle_quat = (uint8_t)'q',     //!< Toggle the streaming of the quaternions
    cmd_toggle_euler = (uint8_t)'e',    //!< Toggle the streaming of the Euler angles
    cmd_toggle_heading = (uint8_t)'h',  //!< Toggle the streaming of the heading

    cmd_req_status = (uint8_t)'s',      //!< Request for the VMU status to be sent

    cmd_accel_2g = (uint8_t)'4',        //!< Request for the accelerometers resolution to be changed to 2G
    cmd_accel_4g = (uint8_t)'5',        //!< Request for the accelerometers resolution to be changed to 4G
    cmd_accel_8g = (uint8_t)'6',        //!< Request for the accelerometers resolution to be changed to 8G
    cmd_accel_16g = (uint8_t)'7',       //!< Request for the accelerometers resolution to be changed to 16G

    cmd_gyro_250dps = (uint8_t)'0',     //!< Request for the gyroscopes resolution to be changed to 250dps
    cmd_gyro_500dps = (uint8_t)'1',     //!< Request for the gyroscopes resolution to be changed to 500dps
    cmd_gyro_1000dps = (uint8_t)'2',    //!< Request for the gyroscopes resolution to be changed to 1000dps
    cmd_gyro_2000dps = (uint8_t)'3',    //!< Request for the gyroscopes resolution to be changed to 2000dps

    cmd_self_test = (uint8_t)'t',       //!< Request for a self-test of the VMU
    cmd_calibration = (uint8_t)'l'      //!< Request for a calibration of the VMU
} Cmd;


/// \cond Variable type used to hold a single message when the function vmutils_loadMessage is used
typedef struct {
    uint8_t rawMess[VMU_RAW_MESS_SIZE];

    uint8_t startByte;
    uint8_t size;
    uint8_t type;
    uint8_t message[VMU_RAW_MESS_SIZE];
    uint8_t endByte;

    MessType messType;
    uint8_t messLength;
} SingleMessage;
/// \endcond

/// Variable type that can hold the accel, gyro, mag or Euler angles
typedef union {
    uint8_t rawData[VMU_DATA_XYZ_SIZE]; ///< Raw bytes contained in the message
    struct {
        uint32_t timestamp; ///< Timestamp (ms)
        float_t x;  ///< X
        float_t y;  ///< Y
        float_t z;  ///< Z
    };
} Data_xyz;

/// Variable type that can hold the quaternions
typedef union {
    uint8_t rawData[VMU_DATA_WXYZ_SIZE];    ///< Raw bytes contained in the message
    struct {
        uint32_t timestamp; ///< Timestamp (ms)
        float_t w;  ///< W
        float_t x;  ///< X
        float_t y;  ///< Y
        float_t z;  ///< Z
    };
} Data_wxyz;

/// Variable type that can hold the heading
typedef union {
    uint8_t rawData[VMU_DATA_H_SIZE];   ///< Raw bytes contained in the message
    struct {
        uint32_t timestamp; ///< Timestamp (ms)
        float_t h;  ///< Heading
    };
} Data_h;

/// Variable type that can hold the status
typedef struct {
    union {
        uint8_t rawData[VMU_DATA_STATUS_SIZE];  ///< Raw bytes contained in the message
        struct {
            uint8_t sensors_status;     ///< \link #VMU_STATUS_MASK_ACCEL_ON Sensors status\endlink
            uint8_t sensors_res;        ///< \link #VMU_STATUS_MASK_RES_ACCEL_2G Sensors resolution\endlink
            uint8_t low_output_rate;    ///< \link #VMU_STATUS_MASK_LOW_OUTPUT_RATE Output rate\endlink
            uint8_t data_streaming_array[sizeof(uint32_t)];     ///< Raw bytes forming the data_streaming member
        };
    };
    uint32_t data_streaming;    ///< \link #VMU_STATUS_MASK_STREAM_ACCEL Data currently streaming\endlink
} Data_status;


/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/// \cond
const int i = 1; // Endianness verification
extern SingleMessage singleMessage;
/// \endcond

BEGIN_DECL
int vmutils_loadMessage(uint8_t *mess, size_t size, MessType *messType);
int vmutils_retrieveText(char *text);
DataType vmutils_retrieveDataType();
int vmutils_retrieveXYZData(Data_xyz *data);
int vmutils_retrieveWXYZData(Data_wxyz *data);
int vmutils_retrieveHData(Data_h *data);
int vmutils_retrieveStatus(Data_status *data);
int vmutils_buildCmd(Cmd cmd, char *string);
END_DECL

#endif
