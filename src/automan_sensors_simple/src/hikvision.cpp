//
// created by mustang on 07/01/2022
//
#define FMT_HEADER_ONLY
#include "hikvision.h"

#include <cstring>
#include <iostream>
#include <fmt/format.h>
#include <fmt/color.h>
#include <chrono>

#define HIK_ERROR_MASK 0x0FFFFFFF

#define HIK_ASSERT_WARNING(expr, stat, info, ...) do{         \
    if((stat = (expr)) == false){                                \
        fmt::print(fg(fmt::color::orange),              \
                   "[WARNING] " #expr info "\n", ##__VA_ARGS__); \
    }                                                   \
}while(0)

#define HIK_ASSERT_ERROR(expr, stat, info, ...) do{           \
    if((stat = (expr)) == false){                                \
        fmt::print(fg(fmt::color::red),                 \
                   "[ERROR] " #expr info "\n", ##__VA_ARGS__);   \
        return false;                                   \
    }                                                   \
}while(0)

#define HIK_ASSERT_THROW(expr, stat, info, ...) do{                               \
    if((stat = (expr)) == false){                                                    \
        throw HikVision_FrameError(                                         \
            fmt::format("'"#expr"' = ({}) " info, status, ##__VA_ARGS__));  \
    }                                                                       \
}while(0)

#define HIK_CHECK_API_WARNING(expr, stat, info, ...) do{                              \
    auto status = (expr);                                                       \
    if((stat = (status)) != MV_OK){                                        \
        fmt::print(fg(fmt::color::orange),                                      \
                   "[WARNING] '"#expr"' = ({}) " info "\n", (status & HIK_ERROR_MASK), ##__VA_ARGS__);  \
    }                                                                           \
}while(0)

#define HIK_CHECK_API_ERROR(expr, stat, info, ...)   do{                              \
    auto status = (expr);                                                       \
    if((stat = status) != MV_OK){                                        \
        fmt::print(fg(fmt::color::red),                                         \
                   "[ERROR] '"#expr"' = ({}) " info "\n", (status & HIK_ERROR_MASK), ##__VA_ARGS__);   \
        return false;                                                           \
    }                                                                           \
}while(0)

#define HIK_CHECK_API_THROW(expr, status, info, ...)   do{                              \
    auto status = (expr);                                                       \
    if((stat = (status)) != MV_OK){                                        \
        throw HikVision_FrameError(                                             \
            fmt::format("'"#expr"' = ({}) " info, (status & HIK_ERROR_MASK), ##__VA_ARGS__));      \
    }                                                                           \
}while(0)

HikVision::HikVision(const char *camera_name, const char *camera_cfg)
        : camera_name(camera_name), camera_cfg(camera_cfg), m_hDevHandle(MV_NULL) {
    HikVision::GetSDKVersion();
    std::cerr << "save log: " << MV_CC_SetSDKLogPath("./") << std::endl;
}

HikVision::~HikVision() {
    close();
}

int HikVision::GetSDKVersion()
{
    fmt::print(fmt::fg(fmt::color::blue), "HikVision SDK version: {}\n", MV_CC_GetSDKVersion());
    return 1;
}

int HikVision::EnumDevices()
{
    int nret = MV_OK;
    MV_CC_DEVICE_INFO_LIST m_stDevList;
    memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    HIK_CHECK_API_WARNING(MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList), nret, "failed to enum devices");
    
    if (nret != MV_OK)  // failed
        return false;
    
    // ch:将值加入到信息列表框中并显示出来 | en:Add value to the information list box and display
    for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO* pDeviceInfo = m_stDevList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
            continue;

        if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
            if (strcmp("", (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName) != 0)
                std::cout << "[" << i << "]UsbV3:  " << pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName << std::endl;
            else
                std::cout << "[" << i << "]UsbV3:  " << pDeviceInfo->SpecialInfo.stUsb3VInfo.chManufacturerName << " " 
                            << pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName << " (" << pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber << ")" << std::endl;
        } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
            continue;
        else
            std::cout << "Unknown device enumerated" << std::endl;
    }

    if (0 == m_stDevList.nDeviceNum)
        std::cout << "No device" << std::endl;
    
    return (nret == MV_OK);
}

bool HikVision::findDevice(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* m_stDevList, const std::string &camera_name, size_t &device_idx)
{
    int nret = MV_OK;
    memset(m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    HIK_CHECK_API_WARNING(MV_CC_EnumDevices(nTLayerType, m_stDevList), nret, "failed to enum devices");
    
    if (nret != MV_OK)  // failed
        return false;
    
    // ch:将值加入到信息列表框中并显示出来 | en:Add value to the information list box and display
    for (unsigned int i = 0; i < m_stDevList->nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO* pDeviceInfo = m_stDevList->pDeviceInfo[i];
        if (NULL == pDeviceInfo)
        {
            continue;
        }
        if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
            if (strcmp("", (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName) != 0)
                if (strcmp(camera_name.c_str(), (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName) == 0)
                {
                    device_idx = i;
                    break;
                }
        } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            continue;
        }
    }

    if (0 == m_stDevList->nDeviceNum)
        return false;

    return (nret == MV_OK);
}


bool HikVision::open() {
    int nret = MV_OK;
    // return if a handle has been created
    if (isOpen()) {
        if (!close()) return false;
    }

    // find device first
    MV_CC_DEVICE_INFO_LIST m_stDevList;
    size_t idx;
    HIK_ASSERT_ERROR(findDevice(MV_USB_DEVICE, &m_stDevList, camera_name, idx), nret, "target camera {} not found", camera_name);

    // create camera handle and open camera
    HIK_ASSERT_ERROR((idx >= 0) && (idx < MV_MAX_DEVICE_NUM) && (NULL != m_stDevList.pDeviceInfo[idx]), nret, "");
    HIK_CHECK_API_ERROR(MV_CC_CreateHandle(&m_hDevHandle, m_stDevList.pDeviceInfo[idx]), nret, "failed to create camera handle");
    nret = MV_CC_OpenDevice(m_hDevHandle);
    if (MV_OK != nret)
    {
        MV_CC_DestroyHandle(m_hDevHandle);
        m_hDevHandle = MV_NULL;
        fmt::print(fmt::fg(fmt::color::red), "[ERROR] failed to open camera {}\n", nret);
    }

    // read camera feature
    HIK_CHECK_API_WARNING(MV_CC_FeatureLoad(m_hDevHandle, camera_cfg.c_str()), nret, "config file '{}' read error!", camera_cfg);

    // use bilinearity bayer interpolation
    HIK_CHECK_API_WARNING(MV_CC_SetBayerCvtQuality(m_hDevHandle, 1), nret, "failed to use bilinearity bayer interpolation");

    // get frame info
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    HIK_CHECK_API_ERROR(MV_CC_GetIntValue(m_hDevHandle, "PayloadSize", &stParam), nret, "failed to get PayloadSize");
    HIK_CHECK_API_ERROR(MV_CC_GetIntValue(m_hDevHandle, "Height", &nHeightValue), nret, "failed to get nheight");
    HIK_CHECK_API_ERROR(MV_CC_GetIntValue(m_hDevHandle, "Width", &nWidthValue), nret, "failed to get nwidth");

    // start grabbing
    HIK_CHECK_API_ERROR(MV_CC_StartGrabbing(m_hDevHandle), nret, "camera failed to start grabbing");

    return true;
}

bool HikVision::close() {
    int nret = MV_OK;
    HIK_ASSERT_WARNING(m_hDevHandle != MV_NULL, nret, "camera already closed.");
    if (m_hDevHandle == MV_NULL)
        return true;

    HIK_CHECK_API_WARNING(MV_CC_StopGrabbing(m_hDevHandle), nret, "failed to stop grabbing");
    HIK_CHECK_API_WARNING(MV_CC_CloseDevice(m_hDevHandle), nret, "failed to close camera");
    HIK_CHECK_API_WARNING(MV_CC_DestroyHandle(m_hDevHandle), nret, "failed to destroy handle");
    m_hDevHandle = MV_NULL;

    return (nret == MV_OK);
}

bool HikVision::isOpen() const {
    return (m_hDevHandle != nullptr);
}

bool HikVision::read(cv::Mat &img, double &timestamp_ms) const {
    int nret = MV_OK;
    HIK_ASSERT_ERROR(this->isOpen(), nret, "camera not open.");

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    img = cv::Mat(nHeightValue.nCurValue, nWidthValue.nCurValue, CV_8UC3);
    unsigned int nDataSize = nHeightValue.nCurValue * nWidthValue.nCurValue *  4 + 2048;
    HIK_CHECK_API_ERROR(MV_CC_GetImageForBGR(m_hDevHandle, img.data, nDataSize, &stImageInfo, 1000), nret, "failed to get image buffer!");

    // auto t1 = std::chrono::system_clock::now();
    // MV_FRAME_OUT stOutFrame = {0};
    // memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    // HIK_CHECK_API_ERROR(MV_CC_GetImageBuffer(m_hDevHandle, &stOutFrame, 1000), nret, "failed to get image buffer!");
    // img = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3);
    // // 像素格式转换
    // // convert pixel format 
    // MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
    // // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
    // // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
    // // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format, 
    // // destination pixel format, output data buffer, provided output buffer size
    // stConvertParam.nWidth = stOutFrame.stFrameInfo.nWidth;
    // stConvertParam.nHeight = stOutFrame.stFrameInfo.nHeight;
    // stConvertParam.pSrcData = stOutFrame.pBufAddr;
    // stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen;
    // stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
    // stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    // stConvertParam.pDstBuffer = img.data;
    // stConvertParam.nDstBufferSize = stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight *  4 + 2048;
    // HIK_CHECK_API_ERROR(MV_CC_ConvertPixelType(m_hDevHandle, &stConvertParam), nret, "failed to convert to RGB");
    // HIK_CHECK_API_ERROR(MV_CC_FreeImageBuffer(m_hDevHandle, &stOutFrame), nret, "failed to release image buffer");
    // auto t2 = std::chrono::system_clock::now();
    // std::cout << "read cost time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;


    // get frame timestamp
    timestamp_ms = stImageInfo.nHostTimeStamp / 10.;
    // timestamp_ms = stOutFrame.stFrameInfo.nHostTimeStamp / 10.;

    return (nret == MV_OK);
}


bool HikVision::get_exposure_us(double &us) const {
    int nret;
    HIK_ASSERT_ERROR(isOpen(), nret, "camera not open.");
    MVCC_FLOATVALUE eus;
    HIK_CHECK_API_ERROR(MV_CC_GetExposureTime(m_hDevHandle, &eus), nret, "");
    us = eus.fCurValue;
    return (nret == MV_OK);
}

bool HikVision::set_exposure_us(double us) const {
    int nret;
    HIK_ASSERT_ERROR(isOpen(), nret, "camera not open.");
    HIK_CHECK_API_ERROR(MV_CC_SetExposureTime(m_hDevHandle, us), nret, "");
    return (nret == MV_OK);
}