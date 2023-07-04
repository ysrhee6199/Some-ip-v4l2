#include <iostream>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#define CAMERA_DEVICE "/dev/video0"  // 웹캠 장치 경로
#define IMAGE_WIDTH 640             // 이미지 너비
#define IMAGE_HEIGHT 480            // 이미지 높이
#define BUFFER_COUNT 1              // 버퍼 개수

struct buffer {
    void* start;
    size_t length;
};

class WebcamStreaming {
public:
    WebcamStreaming();
    ~WebcamStreaming();
    bool initialize();
    void startStreaming();
    void stopStreaming();

private:
    int cameraFd;
    struct v4l2_format format;
    struct v4l2_requestbuffers reqbuf;
    struct v4l2_buffer bufferinfo;
    struct buffer* buffers;
};

WebcamStreaming::WebcamStreaming() : cameraFd(-1), buffers(nullptr) {}

WebcamStreaming::~WebcamStreaming() {
    if (cameraFd != -1)
        close(cameraFd);
    if (buffers != nullptr) {
        for (int i = 0; i < reqbuf.count; ++i) {
            munmap(buffers[i].start, buffers[i].length);
        }
        delete[] buffers;
    }
}

bool WebcamStreaming::initialize() {
    // 웹캠 장치 열기
    cameraFd = open(CAMERA_DEVICE, O_RDWR);
    if (cameraFd == -1) {
        std::cerr << "Failed to open camera" << std::endl;
        return false;
    }

    // 입력 형식 설정
    std::memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = IMAGE_WIDTH;
    format.fmt.pix.height = IMAGE_HEIGHT;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    if (ioctl(cameraFd, VIDIOC_S_FMT, &format) == -1) {
        std::cerr << "Failed to set format" << std::endl;
        close(cameraFd);
        return false;
    }

    // 버퍼 요청
    std::memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.count = BUFFER_COUNT;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(cameraFd, VIDIOC_REQBUFS, &reqbuf) == -1) {
        std::cerr << "Failed to request buffers" << std::endl;
        close(cameraFd);
        return false;
    }

    // 버퍼 메모리 할당
    buffers = new struct buffer[reqbuf.count];
    for (int i = 0; i < reqbuf.count; ++i) {
        std::memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;
        if (ioctl(cameraFd, VIDIOC_QUERYBUF, &bufferinfo) == -1) {
            std::cerr << "Failed to query buffer" << std::endl;
            close(cameraFd);
            delete[] buffers;
            return false;
        }
        buffers[i].length = bufferinfo.length;
        buffers[i].start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE, MAP_SHARED, cameraFd, bufferinfo.m.offset);
        if (buffers[i].start == MAP_FAILED) {
            std::cerr << "Failed to mmap buffer" << std::endl;
            close(cameraFd);
            delete[] buffers;
            return false;
        }
    }

    // 버퍼 큐에 넣기
    for (int i = 0; i < reqbuf.count; ++i) {
        std::memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;
        if (ioctl(cameraFd, VIDIOC_QBUF, &bufferinfo) == -1) {
            std::cerr << "Failed to enqueue buffer" << std::endl;
            close(cameraFd);
            delete[] buffers;
            return false;
        }
    }

    // 스트리밍 시작
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cameraFd, VIDIOC_STREAMON, &type) == -1) {
        std::cerr << "Failed to start streaming" << std::endl;
        close(cameraFd);
        delete[] buffers;
        return false;
    }

    return true;
}
std::chrono::steady_clock::time_point cap_time_;
void WebcamStreaming::startStreaming() {
    cv::namedWindow("Webcam Streaming", cv::WINDOW_NORMAL);

    while (true) {
        // 이미지 데이터 가져오기
        std::memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        if (ioctl(cameraFd, VIDIOC_DQBUF, &bufferinfo) == -1) {
            std::cerr << "Failed to dequeue buffer" << std::endl;
            break;
        }
        std::chrono::steady_clock::time_point update_time_ = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSeconds = update_time_- cap_time_ ;
        std::cout <<elapsedSeconds.count() << '\n';
        cap_time_ = update_time_;
        // YUYV 형식을 BGR로 변환하여 이미지 생성
        cv::Mat yuyvImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2, buffers[bufferinfo.index].start);
        cv::Mat bgrImage;
        cv::cvtColor(yuyvImage, bgrImage, cv::COLOR_YUV2BGR_YUYV);

        // 이미지를 윈도우에 표시
        cv::imshow("Webcam Streaming", bgrImage);

        // 사용자가 'ESC' 키를 누르면 종료
        if (cv::waitKey(1) == 27)
            break;

        // 처리가 끝난 버퍼를 큐에 다시 넣기
        if (ioctl(cameraFd, VIDIOC_QBUF, &bufferinfo) == -1) {
            std::cerr << "Failed to requeue buffer" << std::endl;
            break;
        }
    }

    cv::destroyWindow("Webcam Streaming");
}

void WebcamStreaming::stopStreaming() {
    // 스트리밍 중지
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cameraFd, VIDIOC_STREAMOFF, &type);
}

int main() {
    WebcamStreaming streaming;

    if (!streaming.initialize()) {
        std::cerr << "Failed to initialize WebcamStreaming" << std::endl;
        return EXIT_FAILURE;
    }

    streaming.startStreaming();
    streaming.stopStreaming();

    return EXIT_SUCCESS;
}
