#include "vmu_utils.hpp"

SingleMessage singleMessage;

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
