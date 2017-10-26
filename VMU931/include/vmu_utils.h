/**
 ******************************************************************************
 *
 *  \copyright  Copyright (C) 2017 Variense, Inc.
 *
 *  \file       vmu_utils.h
 *
 *  \date       March 22<sup>th</sup> 2017
 *
 *  \version    1.0.0
 *
 *  \brief      Utility functions for the Variense VMU products
 *
 ******************************************************************************
 *  \mainpage VMU utilities API
 *  \section License
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the
 *  Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 *  Boston, MA  02110-1301, USA.
 *
 ******************************************************************************
 *  \section Contact
 *  Variense, Inc., hereby disclaims all copyright interest in the header
 *  file 'vmu_utils' (utility functions for VMU products) written
 *  by Alexandre Bernier.
 *
 *  To report bugs, please contact me at alexandre.bernier@variense.com
 *
 ******************************************************************************
 *  \section Files
 *  \ref vmu_utils.h
 *
 ******************************************************************************
 *  \section Usage
 *  \attention Please read the User Guide before attempting to use this API.
 *
 *  This header file contains tools to help you interact with the VMU products.
 *
 *  \warning This header file doesn't interact with ANY hardware. It's your
 *      responsability to connect, read and write to the VMU.
 *
 *  \subsection read Read from the VMU
 *  If you want to extract readable data from the stream of bytes the VMU
 *  sends you, do the following:
 *  -# Extract one single message from the stream of bytes. A message can
 *      either start with #VMU_DATA_START and end with #VMU_DATA_END or
 *      start with #VMU_STRING_START and end with #VMU_STRING_END.
 *      You can find the last byte of a message by using the length of the
 *      message which is always the second byte (#VMU_SIZE_OFFS).
 *  -# Call the function ::vmutils_loadMessage to load the message you found.
 *  -# If the message type returned by the argument
 *      \link ::vmutils_loadMessage messType\endlink is \link #MessType
 *      mt_string\endlink, you can directly call ::vmutils_retrieveText to
 *      retrieve the text contained in the message.
 *  -# Otherwise, if the message type returned by the argument
 *      \link ::vmutils_loadMessage messType\endlink is \link #MessType
 *      mt_data\endlink, you need to call ::vmutils_retrieveDataType to
 *      retrieve the exact nature of the data contained in the message.
 *  -# At this point, depending on the result of ::vmutils_retrieveDataType,
 *      you'll need to call the correct function to retrieve the actual data.
 *      Variable types are at your disposal to store the extracted
 *      data (#Data_xyz, #Data_wxyz, #Data_h, #Data_status):
 *      - If the data type returned is \link #DataType dt_accel\endlink, \link
 *          #DataType dt_gyro\endlink, \link #DataType dt_mag\endlink or \link
 *          #DataType dt_euler\endlink, you need to call
 *          ::vmutils_retrieveXYZData.
 *      - If the data type returned is \link #DataType dt_quat\endlink, you
 *          need to call ::vmutils_retrieveWXYZData.
 *      - If the data type returned is \link #DataType dt_heading\endlink, you
 *          need to call ::vmutils_retrieveHData.
 *      - If the data type returned is \link #DataType dt_status\endlink, you
 *          need to call ::vmutils_retrieveStatus.
 *          - No functions has yet been implemented in this header file to
 *              extract the information contained in #Data_status, but some
 *              macros are available to help you retrieve the information
 *              yourself (\link #VMU_STATUS_MASK_ACCEL_ON
 *              Sensors status\endlink, \link #VMU_STATUS_MASK_RES_ACCEL_2G
 *              Sensors resolution\endlink, \link
 *              #VMU_STATUS_MASK_LOW_OUTPUT_RATE Low output rate\endlink,
 *              \link #VMU_STATUS_MASK_STREAM_ACCEL Data currently
 *              streaming\endlink).
 *
 *
 *  \subsection write Write to the VMU
 *  As mentionned above, you need to deal with writting to the VMU yourself.
 *  Although, we provide a simple function that builds the string you need to
 *  send to the VMU depending on the command you want (::vmutils_buildCmd).
 *  A list of all the supported commands can be found \link #Cmd here\endlink.
 *
 *  \bug When you write to the VMU, make sure to wait for at least 1ms
 *  (one millisecond) between each character. In example, let say you want to
 *  send the command "vara" (toggle the streaming of the accelerometers),
 *  make sure to do the following:
 *  -# Send the character 'v'
 *  -# Wait for 1ms
 *  -# Send the character 'a'
 *  -# Wait for 1ms
 *  -# Send the character 'r'
 *  -# Wait for 1ms
 *  -# Send the character 'a'
 *  -# Wait for 1ms
 *
 ******************************************************************************
 */


#ifndef VMU_UTILS_H_
#define VMU_UTILS_H_

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>

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

SingleMessage singleMessage;    // Contains loaded message
/// \endcond


/*
 ******************************************************************************
 * FUNCTIONS
 ******************************************************************************
 */

BEGIN_DECL

/**
 *  \brief Loads one message and discovers its type.
 *
 *  \param [in] mess Array that contains a single message received from a VMU.
 *  \param [in] size Size of the message.
 *  \param [out] messType Variable that holds the type of the message loaded (string or data).
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Message loaded successfully
 *  - VMU_ERR_INV_ARG: At least one argument is empty
 *  - VMU_ERR_INV_MESS: Something is wrong with the message you're trying to load.
 *  - VMU_ERR_FAILED: The message didn't load properly
 */
int vmutils_loadMessage(uint8_t *mess, size_t size, MessType *messType)
{
    MessType t;

    // Verify arguments
    if (mess == 0 || size == 0 || messType == 0)
    {
        // Invalid arguments
        return VMU_ERR_INV_ARG;
    }

    // Verify the message
    if (mess[VMU_START_BYTE_OFFS] == VMU_DATA_START)
    {
        if (mess[(mess[VMU_SIZE_OFFS]-1)] == VMU_DATA_END)
        {
            // Valid data message
            t = mt_data;
        }

        else
        {
            // Invalid message
            return VMU_ERR_INV_MESS;
        }
    }

    else if (mess[VMU_START_BYTE_OFFS] == VMU_STRING_START)
    {
        if (mess[(mess[VMU_SIZE_OFFS]-1)] == VMU_STRING_END)
        {
            // Valid string message
            t = mt_string;
        }

        else
        {
            // Invalid message
            return VMU_ERR_INV_MESS;
        }
    }

    else
    {
        // Invalid message
        return VMU_ERR_INV_MESS;
    }


    // Load message
    memcpy(singleMessage.rawMess, mess, size);
    if (memcmp(singleMessage.rawMess, mess, size))
    {
        // Copy failed
        return VMU_ERR_FAILED;
    }

    // Extract information from the message
    singleMessage.startByte = singleMessage.rawMess[VMU_START_BYTE_OFFS];
    singleMessage.size = singleMessage.rawMess[VMU_SIZE_OFFS];
    singleMessage.type = singleMessage.rawMess[VMU_TYPE_OFFS];
    singleMessage.endByte = singleMessage.rawMess[singleMessage.size-1];

    singleMessage.messLength = singleMessage.size - VMU_NUM_HEAD_FOOT_BYTES;
    memcpy(singleMessage.message, &singleMessage.rawMess[VMU_NUM_HEADER_BYTES], singleMessage.messLength);

    singleMessage.messType = t;

    *messType = singleMessage.messType;

    // Success
    return VMU_ERR_SUCCESS;
}


/**
 *  \brief Extracts the string of text from the last message loaded.
 *
 *  \param [out] text String of text contained in the last message loaded.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Text retrieved successfully
 *  - VMU_ERR_INV_ARG: Argument is empty
 *  - VMU_ERR_INV_MESS: The loaded message doens't contain text
 *  - VMU_ERR_FAILED: Failed to retrieved the text in the loaded message
 */
int vmutils_retrieveText(char *text)
{
    // Verify arguments
    if (text == 0)
    {
        // Invalid arguments
        return VMU_ERR_INV_ARG;
    }

    // Verification of the loaded message
    if (singleMessage.messType != mt_string)
    {
        // Loaded message doesn't contain text
        return VMU_ERR_INV_MESS;
    }

    // Extract string
    memcpy(text, singleMessage.message, singleMessage.messLength);
    if (memcmp(text, singleMessage.message, singleMessage.messLength))
    {
        // Copy failed
        return VMU_ERR_FAILED;
    }

    // Success
    return VMU_ERR_SUCCESS;
}


/**
 *  \brief Returns the data type of the last message loaded.
 *
 *  \returns the data type of the last message loaded.
 *  - 0: Either the loaded message doesn't contain data or no message is loaded
 *  - DataType: The data type of the loaded message
 */
DataType vmutils_retrieveDataType()
{
    // Verify the loaded message
    if (singleMessage.messType != mt_data)
    {
        // Loaded message doesn't contain data
        return (DataType) 0;
    }

    // Extract data type
    if (singleMessage.type == 0)
    {
        // Data type empty
        return (DataType) 0;
    }

    return (DataType) singleMessage.type;
}


/*
 *  \brief Swaps the byte order of a Data_xyz to match the endianess of the host.
 *
 *  \param [in,out] data The Data_xyz to be swapped.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Swap was successfull
 *  - VMU_ERR_INV_ARG: Argument is empty
 */
static int _swapEndianXYZ(Data_xyz *data)
{
    if (!is_bigendian())
    {
        // Verify arguments
        if (data == 0)
        {
            // Invalid arguments
            return VMU_ERR_INV_ARG;
        }

#ifdef VAR_OS_MAC

        union {
            float_t v;
            CFSwappedFloat32 sv;
        } x, y, z;
        x.v = data->x;
        y.v = data->y;
        z.v = data->z;

        data->timestamp = CFSwapInt32BigToHost(data->timestamp);
        data->x = CFConvertFloatSwappedToHost(x.sv);
        data->y = CFConvertFloatSwappedToHost(y.sv);
        data->z = CFConvertFloatSwappedToHost(z.sv);


#else

        uint32_t swappedValue_u32;
        float_t swappedValue_f;
        uint8_t *swappedValue_bytes;
        uint8_t *valueToConvert;

        // UINT32
        swappedValue_bytes = (uint8_t*) & swappedValue_u32;
        // Swap timestamp
        valueToConvert = (uint8_t*) & data->timestamp;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->timestamp = swappedValue_u32;

        // FLOATS
        swappedValue_bytes = (uint8_t*) & swappedValue_f;
        // Swap X
        valueToConvert = (uint8_t*) & data->x;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->x = swappedValue_f;

        // Swap Y
        valueToConvert = (uint8_t*) & data->y;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->y = swappedValue_f;

        // Swap Z
        valueToConvert = (uint8_t*) & data->z;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->z = swappedValue_f;

#endif

    }

    // Success
    return VMU_ERR_SUCCESS;
}


/**
 *  \brief Extracts the XYZ data from the last loaded message.
 *
 *  \param [out] data Address of the variable that will hold the values.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Data retrieved successfully
 *  - VMU_ERR_INV_ARG: Argument is empty
 *  - VMU_ERR_INV_MESS: The loaded message doens't contain Data_xyz
 *  - VMU_ERR_FAILED: Failed to retrieved the data in the loaded message
 */
int vmutils_retrieveXYZData(Data_xyz *data)
{
    // Verify arguments
    if (data == 0)
    {
        // Invalid arguments
        return VMU_ERR_INV_ARG;
    }

    // Verify the loaded message
    if (singleMessage.messType != mt_data)
    {
        // Loaded message doesn't contain data
        return VMU_ERR_INV_MESS;
    }

    // Verify the data type of the loaded message
    if (singleMessage.type != dt_accel &&
        singleMessage.type != dt_gyro &&
        singleMessage.type != dt_mag &&
        singleMessage.type != dt_euler)
    {
        // Loaded message doesn't contain a Data_xyz
        return VMU_ERR_INV_MESS;
    }

    // Extract the data
    memcpy(data->rawData, singleMessage.message, VMU_DATA_XYZ_SIZE);
    if (memcmp(data->rawData, singleMessage.message, VMU_DATA_XYZ_SIZE))
    {
        // Copy failed
        return VMU_ERR_FAILED;
    }

    // Correct the byte order if necessary
    return _swapEndianXYZ(data);
}


/*
 *  \brief Swaps the byte order of a Data_wxyz to match the endianess of the host.
 *
 *  \param [in,out] data The Data_wxyz to be swapped.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Swap was successfull
 *  - VMU_ERR_INV_ARG: Argument is empty
 */
static int _swapEndianWXYZ(Data_wxyz *data)
{
    if (!is_bigendian())
    {

        // Verify arguments
        if (data == 0)
        {
            // Invalid arguments
            return VMU_ERR_INV_ARG;
        }

#ifdef VAR_OS_MAC

        union {
            float_t v;
            CFSwappedFloat32 sv;
        } w, x, y, z;
        w.v = data->w;
        x.v = data->x;
        y.v = data->y;
        z.v = data->z;

        data->timestamp = CFSwapInt32BigToHost(data->timestamp);
        data->w = CFConvertFloatSwappedToHost(w.sv);
        data->x = CFConvertFloatSwappedToHost(x.sv);
        data->y = CFConvertFloatSwappedToHost(y.sv);
        data->z = CFConvertFloatSwappedToHost(z.sv);


#else

        uint32_t swappedValue_u32;
        float_t swappedValue_f;
        uint8_t *swappedValue_bytes;
        uint8_t *valueToConvert;

        // UINT32
        swappedValue_bytes = (uint8_t*) & swappedValue_u32;
        // Swap timestamp
        valueToConvert = (uint8_t*) & data->timestamp;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->timestamp = swappedValue_u32;

        // FLOATS
        swappedValue_bytes = (uint8_t*) & swappedValue_f;
        // Swap W
        valueToConvert = (uint8_t*) & data->w;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->w = swappedValue_f;

        // Swap X
        valueToConvert = (uint8_t*) & data->x;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->x = swappedValue_f;

        // Swap Y
        valueToConvert = (uint8_t*) & data->y;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->y = swappedValue_f;

        // Swap Z
        valueToConvert = (uint8_t*) & data->z;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->z = swappedValue_f;

#endif

    }

    // Success
    return VMU_ERR_SUCCESS;
}


/**
 *  \brief Extracts the WXYZ data from the last loaded message.
 *
 *  \param [out] data Address of the variable that will hold the values.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Data retrieved successfully
 *  - VMU_ERR_INV_ARG: Argument is empty
 *  - VMU_ERR_INV_MESS: The loaded message doens't contain Data_wxyz
 *  - VMU_ERR_FAILED: Failed to retrieved the data in the loaded message
 */
int vmutils_retrieveWXYZData(Data_wxyz *data)
{
    // Verify arguments
    if (data == 0)
    {
        // Invalid arguments
        return VMU_ERR_INV_ARG;
    }

    // Verify the loaded message
    if (singleMessage.messType != mt_data)
    {
        // Loaded message doesn't contain data
        return VMU_ERR_INV_MESS;
    }

    // Verify the data type of the loaded message
    if (singleMessage.type != dt_quat)
    {
        // Loaded message doesn't contain a Data_wxyz
        return VMU_ERR_INV_MESS;
    }

    // Extract the data
    memcpy(data->rawData, singleMessage.message, VMU_DATA_WXYZ_SIZE);
    if (memcmp(data->rawData, singleMessage.message, VMU_DATA_WXYZ_SIZE))
    {
        // Copy failed
        return VMU_ERR_FAILED;
    }

    // Correct the byte order if necessary
    return _swapEndianWXYZ(data);
}


/*
 *  \brief Swaps the byte order of a Data_h to match the endianess of the host.
 *
 *  \param [in,out] data The Data_h to be swapped.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Swap was successfull
 *  - VMU_ERR_INV_ARG: Argument is empty
 */
static int _swapEndianH(Data_h *data)
{
    if (!is_bigendian())
    {

        // Verify arguments
        if (data == 0)
        {
            // Invalid arguments
            return VMU_ERR_INV_ARG;
        }

#ifdef VAR_OS_MAC

        union {
            float_t v;
            CFSwappedFloat32 sv;
        } h;
        h.v = data->h;

        data->timestamp = CFSwapInt32BigToHost(data->timestamp);
        data->h = CFConvertFloatSwappedToHost(h.sv);


#else

        uint32_t swappedValue_u32;
        float_t swappedValue_f;
        uint8_t *swappedValue_bytes;
        uint8_t *valueToConvert;

        // UINT32
        swappedValue_bytes = (uint8_t*) & swappedValue_u32;
        // Swap timestamp
        valueToConvert = (uint8_t*) & data->timestamp;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->timestamp = swappedValue_u32;

        // FLOATS
        swappedValue_bytes = (uint8_t*) & swappedValue_f;
        // Swap W
        valueToConvert = (uint8_t*) & data->h;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->h = swappedValue_f;

#endif

    }

    // Success
    return VMU_ERR_SUCCESS;
}


/**
 *  \brief Extracts the H data from the last loaded message.
 *
 *  \param [out] data Address of the variable that will hold the values.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Data retrieved successfully
 *  - VMU_ERR_INV_ARG: Argument is empty
 *  - VMU_ERR_INV_MESS: The loaded message doens't contain Data_h
 *  - VMU_ERR_FAILED: Failed to retrieved the data in the loaded message
 */
int vmutils_retrieveHData(Data_h *data)
{
    // Verify arguments
    if (data == 0)
    {
        // Invalid arguments
        return VMU_ERR_INV_ARG;
    }

    // Verify the loaded message
    if (singleMessage.messType != mt_data)
    {
        // Loaded message doesn't contain data
        return VMU_ERR_INV_MESS;
    }

    // Verify the data type of the loaded message
    if (singleMessage.type != dt_heading)
    {
        // Loaded message doesn't contain a Data_h
        return VMU_ERR_INV_MESS;
    }

    // Extract the data
    memcpy(data->rawData, singleMessage.message, VMU_DATA_H_SIZE);
    if (memcmp(data->rawData, singleMessage.message, VMU_DATA_H_SIZE))
    {
        // Copy failed
        return VMU_ERR_FAILED;
    }

    // Correct the byte order if necessary
    return _swapEndianH(data);
}


/*
 *  \brief Swaps the byte order of a Data_status to match the endianess of the host.
 *
 *  \param [in,out] data The Data_status to be swapped.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Swap was successfull
 *  - VMU_ERR_INV_ARG: Argument is empty
 */
static int _swapEndianStatus(Data_status *data)
{
    if (!is_bigendian())
    {

        // Verify arguments
        if (data == 0)
        {
            // Invalid arguments
            return VMU_ERR_INV_ARG;
        }

#ifdef VAR_OS_MAC

        data->data_streaming = CFSwapInt32BigToHost(data->data_streaming);


#else

        uint32_t swappedValue_u32;
        uint8_t *swappedValue_bytes;
        uint8_t *valueToConvert;

        // UINT32
        swappedValue_bytes = (uint8_t*) & swappedValue_u32;
        // Swap timestamp
        valueToConvert = (uint8_t*) & data->data_streaming;
        swappedValue_bytes[0] = valueToConvert[3];
        swappedValue_bytes[1] = valueToConvert[2];
        swappedValue_bytes[2] = valueToConvert[1];
        swappedValue_bytes[3] = valueToConvert[0];
        data->data_streaming = swappedValue_u32;

#endif

    }

    // Success
    return VMU_ERR_SUCCESS;
}


/**
 *  \brief Extracts the Status data from the last loaded message.
 *
 *  \param [out] data Address of the variable that will hold the status.
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Data retrieved successfully
 *  - VMU_ERR_INV_ARG: Argument is empty
 *  - VMU_ERR_INV_MESS: The loaded message doens't contain Data_h
 *  - VMU_ERR_FAILED: Failed to retrieved the data in the loaded message
 */
int vmutils_retrieveStatus(Data_status *data)
{
    // Verify arguments
    if (data == 0)
    {
        // Invalid arguments
        return VMU_ERR_INV_ARG;
    }

    // Verify the loaded message
    if (singleMessage.messType != mt_data)
    {
        // Loaded message doesn't contain data
        return VMU_ERR_INV_MESS;
    }

    // Verify the data type of the loaded message
    if (singleMessage.type != dt_status)
    {
        // Loaded message doesn't contain a Data_status
        return VMU_ERR_INV_MESS;
    }

    // Extract the data
    memcpy(data->rawData, singleMessage.message, VMU_DATA_STATUS_SIZE);
    if (memcmp(data->rawData, singleMessage.message, VMU_DATA_STATUS_SIZE))
    {
        // Copy failed
        return VMU_ERR_FAILED;
    }
    memcpy(&data->data_streaming, data->data_streaming_array, sizeof(uint32_t));

    // Success
    return _swapEndianStatus(data);
}


/**
 *  \brief Build the string to be sent to the VMU according to the command asked.
 *
 *  \param [in] cmd Command you wish to send to the VMU
 *  \param [out] string String to be sent to the VMU (needs to point to an array of at least 4 characters long)
 *
 *  \returns an error code:
 *  - VMU_ERR_SUCCESS: Data retrieved successfully
 *  - VMU_ERR_INV_ARG: At least one argument is empty
 *  - VMU_ERR_INV_CMD: The command requested isn't part of #Cmd
 */
int vmutils_buildCmd(Cmd cmd, char *string)
{
    // Verify arguments
    if (cmd == 0 || string == 0)
    {
        // Invalid argument
        return VMU_ERR_INV_ARG;
    }

    // Verify the length of the array
    if (sizeof(string) < VMU_CMD_NUM_BYTES)
    {
        // Array too small
        return VMU_ERR_INV_ARG;
    }

    // Verify the command asked
    switch (cmd)
    {
        case cmd_toggle_accel: break;
        case cmd_toggle_gyro: break;
        case cmd_toggle_mag: break;
        case cmd_toggle_quat: break;
        case cmd_toggle_euler: break;
        case cmd_toggle_heading: break;

        case cmd_req_status: break;

        case cmd_accel_2g: break;
        case cmd_accel_4g: break;
        case cmd_accel_8g: break;
        case cmd_accel_16g: break;

        case cmd_gyro_250dps: break;
        case cmd_gyro_500dps: break;
        case cmd_gyro_1000dps: break;
        case cmd_gyro_2000dps: break;

        case cmd_self_test: break;
        case cmd_calibration: break;

        default:
            // Invalid command
            return VMU_ERR_INV_CMD;
    }

    string[0] = 'v';
    string[1] = 'a';
    string[2] = 'r';
    string[3] = cmd;

    // Success
    return VMU_ERR_SUCCESS;
}

END_DECL

#endif // !VMU_UTILS_H_
