//
// created by mustang on 07/01/2022
//
#ifndef HIKVISION_H
#define HIKVISION_H

#include "HIKSDK/MvCameraControl.h"
#include <opencv2/core/core.hpp>

#define MV_NULL 0

class HikVision_FrameError : std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class HikVision {
public:
    /**
     * @brief Construct a new Hik Vision object
     * 
     * @param camera_name   camera name
     * @param camera_cfg    camera config path
     */
    explicit HikVision(const char *camera_name = "", const char *camera_cfg = "");

    ~HikVision();

    // ch:获取SDK版本号 | en:Get SDK Version
    static int GetSDKVersion();

    // ch:枚举设备 | en:Enumerate Device
    static int EnumDevices();

    /**
     * @brief open a camera
     * @details
     *      1. enum devices and find target
     *      2. created camera handle
     *      3. open camera
     *      4. read camera feature
     *      5. get frame info
     *      6. start grabbing
     * @warning 
     *      1. if current camera is opened, this function will try to reopen the camera
     * 
     * @return true     success
     * @return false    failed
     */
    bool open();

    /**
     * @brief close a camera
     * @details 
     *      1. close camera
     *      2. destroy handle
     * @warning
     *      1. if current camera is closed, this function will give out a warning and do nothing
     * 
     * @return true     success
     * @return false    failed
     */
    bool close();

    bool isOpen() const;

    bool read(cv::Mat &img, double &timestamp_ms) const;

    bool get_exposure_us(double &us) const;

    bool set_exposure_us(double us) const;
private:
    const std::string   camera_name;
    const std::string   camera_cfg;
    void*               m_hDevHandle;
    MVCC_INTVALUE       stParam;
    MVCC_INTVALUE       nHeightValue;
    MVCC_INTVALUE       nWidthValue;

    // ch:开启抓图 | en:Start Grabbing
    int StartGrabbing();

    // ch:停止抓图 | en:Stop Grabbing
    int StopGrabbing();

    // ch:主动获取一帧图像数据 | en:Get one frame initiatively
    int GetImageBuffer(MV_FRAME_OUT* pFrame, int nMsec);

    // ch:释放图像缓存 | en:Free image buffer
    int FreeImageBuffer(MV_FRAME_OUT* pFrame);

    /**
     * @brief find device with name
     * 
     * @param camera_name   device name
     * @return bool         success
     */
    bool findDevice(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* pstDevList, const std::string &camera_name, size_t &device_idx);
};

#endif // HIKVISION_H