#include "camera_mipi.h"

bool CameraMIPI::capture(int& i, int& capture_buf, IEGLOutputStream *iEglOutputStream, IFrameConsumer *iFrameConsumer, unsigned char* output_buffer)
{
    i += 1;

    UniqueObj<Frame> frame(iFrameConsumer->acquireFrame());

    IFrame *iFrame = interface_cast<IFrame>(frame);
    if (!iFrame)
        return false;

    NV::IImageNativeBuffer *iNativeBuffer =
        interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
    if (!iNativeBuffer)
        ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");

    if (capture_buf == -1)
    {
        std::cout << "\n start createNvBuffer\n";

        capture_buf = iNativeBuffer->createNvBuffer(iEglOutputStream->getResolution(),
                                                    NvBufferColorFormat_ABGR32,
                                                    NvBufferLayout_Pitch);

        std::cout << "\n end createNvBuffer\n";
        
        if (capture_buf == -1)
            CONSUMER_PRINT("\tFailed to create NvBuffer\n");
        else if (iNativeBuffer->copyToNvBuffer(capture_buf) != STATUS_OK)
        {
            ORIGINATE_ERROR("Failed to copy frame to NvBuffer.");
        }
    }
    else if (iNativeBuffer->copyToNvBuffer(capture_buf) != STATUS_OK)
    {
        ORIGINATE_ERROR("Failed to copy frame to NvBuffer.");
    }

    void* data_ptr = NULL;
    NvBufferMemMap(capture_buf, 0, NvBufferMem_Read, &data_ptr);
    NvBufferMemSyncForCpu(capture_buf, 0, &data_ptr);
    cv::Mat image_temp(iEglOutputStream->getResolution().height(), iEglOutputStream->getResolution().width(), CV_8UC4, data_ptr);

    cv::Mat image_show;
    cv::cvtColor(image_temp, image_show, cv::COLOR_RGBA2BGR);

    cv::Mat outputImg(image_show.size(), CV_8UC3, output_buffer);

    image_show.copyTo(outputImg);

    NvBufferMemUnMap(capture_buf, 0, &data_ptr);
}

bool FactoryThread::threadExecute()
{
    UniqueObj<CameraProvider> cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());

    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
    if (!iCameraProvider)
        ORIGINATE_ERROR("Failed to create CameraProvider");

    std::vector<CameraDevice*> cameraDevices;
    iCameraProvider->getCameraDevices(&cameraDevices);
    if (cameraDevices.size() == 0)
        ORIGINATE_ERROR("No cameras available");

    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(cameraDevices[0]);
    if (!iCameraProperties)
        ORIGINATE_ERROR("Failed to get ICameraProperties interface");

    captureSession_m = UniqueObj<CaptureSession>(iCameraProvider->createCaptureSession(cameraDevices[0]));

    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession_m);

    iCaptureSession_m = iCaptureSession;

    if (!iCaptureSession)
        ORIGINATE_ERROR("Failed to get ICaptureSession interface");

    PRODUCER_PRINT("Creating output stream\n");
    UniqueObj<OutputStreamSettings> streamSettings(
        iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));

    IEGLOutputStreamSettings *iEglStreamSettings =
        interface_cast<IEGLOutputStreamSettings>(streamSettings);
    if (!iEglStreamSettings)
        ORIGINATE_ERROR("Failed to get IEGLOutputStreamSettings interface");

    iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);

    /********获取sensor分辨率**********/
    ISensorMode *iSensorMode;
    std::vector<SensorMode*> sensorModes;

    iCameraProperties->getBasicSensorModes(&sensorModes);
    if (sensorModes.size() == 0)
        ORIGINATE_ERROR("Failed to get sensor modes");

    PRODUCER_PRINT("Available Sensor modes :\n");

    int max_resolution_area = 0;
    int SENSOR_MODE = 0;

    for (uint32_t i = 0; i < sensorModes.size(); i++) {
        iSensorMode = interface_cast<ISensorMode>(sensorModes[i]);
        Size2D<uint32_t> resolution = iSensorMode->getResolution();
        PRODUCER_PRINT("[%u] W=%u H=%u\n", i, resolution.width(), resolution.height());
        if (resolution.width() * resolution.height() > max_resolution_area)
        {
            max_resolution_area = resolution.width() * resolution.height();
            SENSOR_MODE = i;
            image_width_m = resolution.width();
            image_height_m = resolution.height();
        }
    }
    /********获取sensor分辨率**********/

    iEglStreamSettings->setResolution(Size2D<uint32_t>(image_width_m, image_height_m));

    UniqueObj<OutputStream> previewStream(iCaptureSession->createOutputStream(streamSettings.get()));
    outstream_ptr_m = previewStream.get();
    consumer_m = UniqueObj<FrameConsumer>(FrameConsumer::create(outstream_ptr_m));
    if (!consumer_m)
        ORIGINATE_ERROR("Failed to create FrameConsumer");

    PRODUCER_PRINT("Launching consumer thread\n");

    UniqueObj<Request> request(iCaptureSession->createRequest());
    request_ptr_m = request.get();

    IRequest *iRequest = interface_cast<IRequest>(request);

    iRequest_ptr_m = iRequest;

    if (!iRequest)
        ORIGINATE_ERROR("Failed to create Request");

    iRequest->enableOutputStream(previewStream.get());

    ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    if (!iSourceSettings)
        ORIGINATE_ERROR("Failed to get ISourceSettings interface");

    if (SENSOR_MODE >= sensorModes.size())
        ORIGINATE_ERROR("Sensor mode index is out of range");

    std::cout << "SENSOR_MODE: " << SENSOR_MODE << std::endl;
    SensorMode *sensorMode = sensorModes[SENSOR_MODE];

    iSensorMode = interface_cast<ISensorMode>(sensorMode);

    iSourceSettings->setSensorMode(sensorMode);

    Range<uint64_t> sensorDuration(iSensorMode->getFrameDurationRange());
    Range<uint64_t> desireDuration(1e9/CAPTURE_FPS+0.9);
    if (desireDuration.min() < sensorDuration.min() ||
            desireDuration.max() > sensorDuration.max()) {
        PRODUCER_PRINT("Requested FPS out of range. Fall back to 30\n");
        CAPTURE_FPS = 30;
    }

    std::cout << "sensorDuration.min(): " << sensorDuration.min() << std::endl;
    std::cout << "sensorDuration.max(): " << sensorDuration.max() << std::endl;

    iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9/CAPTURE_FPS));

    PRODUCER_PRINT("Starting repeat capture requests.\n");

    if (iCaptureSession->repeat(request.get()) != STATUS_OK)
        ORIGINATE_ERROR("Failed to start repeat capture request");

    while(1)
    {
        sleep(1);
    }

    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();

    previewStream.reset();

    PRODUCER_PRINT("Done -- exiting.\n");

    return true;
}

CameraMIPI::CameraMIPI()
{
    factoryThread_m = NULL;

    capture_buf_m = -1;
    i_m = 0;
    stream_m = NULL;

    iEglOutputStream_m = NULL;
    iFrameConsumer_m = NULL;
}

CameraMIPI::~CameraMIPI()
{

}

bool initProviderThread()
{
    
}

bool CameraMIPI::openCamera()
{
    // 创建一个生产者线程
    factoryThread_m = new FactoryThread();
    if (!factoryThread_m->initialize())
    {
        std::cout << "factoryThread initialize failed! " << std::endl;
        return false;
    }
    else
    {
        std::cout << "factoryThread initialize success! " << std::endl;
    }
    if (!factoryThread_m->waitRunning())
    {
        std::cout << "wait running failed! " << std::endl;
        return false;
    }
    else
    {
        std::cout << "wait running success! " << std::endl;
    }
    sleep(3);

    capture_buf_m = -1;
    i_m = 0;
    stream_m = factoryThread_m->outstream_ptr_m;

    iEglOutputStream_m = interface_cast<IEGLOutputStream>(stream_m);
    iFrameConsumer_m = interface_cast<IFrameConsumer>(factoryThread_m->consumer_m);
    if (iEglOutputStream_m->waitUntilConnected() != 0)
    {
        std::cout << "connect error! " << std::endl;
        return false;
    }
    image_height_ = factoryThread_m->image_height_m;
    image_width_ = factoryThread_m->image_width_m;
    return true;

    // 然后将stream以及一些别的接口保存到对应的成员变量之中。

    // 创建一个采集的线程
}

bool CameraMIPI::streamOn()
{
    // factoryThread_m->captureSession_m.get()
    if (factoryThread_m->iCaptureSession_m->repeat(factoryThread_m->request_ptr_m) == 0)
    {
        std::cout << "stream on success" << std::endl;
        return true;
    }
    else
    {
        std::cout << "stream on failed" << std::endl;
        return false;
    }
}
bool CameraMIPI::streamOff()
{
    factoryThread_m->iCaptureSession_m->stopRepeat();

    std::cout << "stream off success" << std::endl;
    return true;
}

bool CameraMIPI::grap(unsigned char* buf)
{
    // 调用接口
    // LOG(INFO) << "capture_times: " << i_m;
    capture(i_m, capture_buf_m, iEglOutputStream_m, iFrameConsumer_m, buf);
}

