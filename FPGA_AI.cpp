#include <iostream>
#include <string>
#include <experimental/filesystem>
#include <vector>
#include <ctime>
#include <random>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "BRAM-uio-driver/src/bram_uio.h"
#include <chrono>
#include <thread>

#define BRAM_UIO_NUMBER 0
#define BRAM_SIZE 256

namespace fs = std::experimental::filesystem;

cv::Mat orig_frame;
cv::Mat resized_frame;
cv::Mat gray_frame;
cv::Mat invert_frame;
cv::Mat frame;
int frame_count = 0;



int main(int argc, char* argv[])
{
    BRAM bram(BRAM_UIO_NUMBER, BRAM_SIZE);

    // get frame from camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cerr << "Error opening camera" << std::endl;
        return -1;
    }

    while (1)
    {

        cap.read(orig_frame);
        if (orig_frame.empty())
        {
            std::cerr << "Error reading frame" << std::endl;
            return -1;
        }

        
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
        cv::resize(orig_frame, resized_frame, cv::Size(10, 10), cv::INTER_AREA); // resize img to fit dims
    

        // convert the RGB image to grayscale
        cv::cvtColor(resized_frame, gray_frame, cv::COLOR_BGR2GRAY);


        // treshold frame
        cv::threshold(gray_frame, invert_frame, 100, 255, cv::THRESH_BINARY);


        // invert frame
        cv::bitwise_not(invert_frame, frame);

        // save frame
        cv::imwrite("invertedframe.jpg", frame);
        frame.convertTo(frame, CV_32F, 1.0/255.0); // convert to float32


        uint8_t BufferPtr_rx[400] = {0x00};
        // copy image to buffer
        for (int i = 0; i < frame.rows; i++)
        {
            for (int j = 0; j < frame.cols; j++)
            {
                float pixel_value = frame.at<float>(i, j);
                uint8_t* byte_array = reinterpret_cast<uint8_t*>(&pixel_value);
                for (int k = 0; k < 4; k++)
                {
                    BufferPtr_rx[(i * frame.cols + j) * 4 + k] = byte_array[k];
                }
            }
        }

        for (int i = 0; i < 100; i++)
        {
            // concatenate 8-bit input messages into 32-bit values
            uint32_t tempInt = ((BufferPtr_rx[i * 4 + 3] << 24) | (BufferPtr_rx[i * 4 + 2] << 16) |
                                (BufferPtr_rx[i * 4 + 1] << 8) | BufferPtr_rx[i * 4]);

            // prints current values in BRAM
            float tempFloat = *((float*)&tempInt); // int bits to float
            char buffer2[10];
            sprintf(buffer2, "%f", tempFloat); // tempInt
            // std::cout << "BRAM[" << i << "]: " << buffer2 << std::endl;

            bram[i] = tempInt; // write to BRAM
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS

        // only read every 5th picture to eliminate delay
        if (frame_count == 0) {
            // print image on terminal
            system("ascii-image-converter -C invertedframe.jpg");
            std::cout << std::endl;


            // read neural network output
            uint32_t recognized_number = bram[128];
            std::cout << "Recognized number: " << recognized_number << std::endl;#

            // create and execute command for ros
            std::string command = "ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition \"{position: " + std::to_string(recognized_number) + "}\"";
            std::cout << "Executing command: " << command << std::endl;
            std::system(command.c_str());
            frame_count = frame_count + 1;
        }
        else if (frame_count == 4) {
            frame_count = 0;
        }
        else {
            frame_count = frame_count + 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        
        
    }

    return 0;
}
