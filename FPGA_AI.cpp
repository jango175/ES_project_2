#include <iostream>
#include <string>
#include <experimental/filesystem>
#include <vector>
#include <ctime>
#include <random>
#include <opencv2/opencv.hpp>
#include "BRAM-uio-driver/src/bram_uio.h"

#define BRAM_UIO_NUMBER 0
#define BRAM_SIZE 256

namespace fs = std::experimental::filesystem;


// Function to get a list of subdirectories in a given path
std::vector<std::string> get_subdirectories(const std::string& path)
{
    std::vector<std::string> subdirs;

    try
    {
        for (const auto& entry : fs::directory_iterator(path))
        {
            // Use status() method for checking directory
            if (fs::is_directory(entry.status()))
            {
                // Extract just the folder name, not the full path
                subdirs.push_back(entry.path().filename().string());
            }
        }
    }
    catch (const fs::filesystem_error& e)
    {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
        return {};
    }

    return subdirs;
}


// Function to get a list of files in a directory (excluding subdirectories)
std::vector<std::string> get_files_in_directory(const std::string& path)
{
    std::vector<std::string> files;

    try
    {
        for (const auto& entry : fs::directory_iterator(path))
        {
            // Use status() method for checking regular file
            if (fs::is_regular_file(entry.status()))
            {
                // Extract just the filename, not the full path
                files.push_back(entry.path().filename().string());
            }
        }
    }
    catch (const fs::filesystem_error& e)
    {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
        return {};
    }

    return files;
}


// Function to randomly select an item from a vector
template<typename T> T select_random_item(const std::vector<T>& items)
{
    if (items.empty())
    {
        throw std::runtime_error("Vector is empty");
    }

    // Use current time as seed for random generator
    static std::mt19937 rng(std::time(nullptr));

    // Create uniform distribution across item indices
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, items.size() - 1);

    // Select random index
    size_t random_index = dist(rng);

    return items[random_index];
}


std::string get_random_dir(std::string base_path)
{
    // Get list of subdirectories
    std::vector<std::string> subdirs = get_subdirectories(base_path);

    // If no directories, exit
    if (subdirs.empty())
    {
        std::cerr << "No directories found in the specified path." << std::endl;
        return NULL;
    }

    // Randomly select a directory
    std::string selected_directory = select_random_item(subdirs);

    // Construct full path to selected directory
    fs::path full_directory_path = fs::path(base_path) / selected_directory;

    // Get files in the selected directory
    std::vector<std::string> files_in_directory = get_files_in_directory(full_directory_path.string());

    // If no files, exit
    if (files_in_directory.empty())
    {
        std::cerr << "No files found in the selected directory." << std::endl;
        return NULL;
    }

    // Randomly select a file
    std::string selected_file = select_random_item(files_in_directory);

    // Full path of selected file
    fs::path full_file_path = full_directory_path / selected_file;

    return full_file_path.string();
}


int main(int argc, char* argv[])
{
    BRAM bram(BRAM_UIO_NUMBER, BRAM_SIZE);

    cv::Mat orig_frame;
    cv::Mat frame;

    while (1)
    {
        std::string file_path = get_random_dir("/home/mp4d/Downloads/MNIST_Dataset_JPG/MNIST_JPG_testing");
        if (file_path.empty())
        {
            std::cerr << "Error getting file path" << std::endl;
            return -1;
        }
        std::cout << "File path: " << file_path << std::endl;

        // read image from file
        orig_frame = cv::imread(file_path, cv::IMREAD_GRAYSCALE); // read img as grayscale
        if (orig_frame.empty())
        {
            std::cerr << "Error reading image from file" << std::endl;
            return -1;
        }
        cv::resize(orig_frame, frame, cv::Size(10, 10), cv::INTER_AREA); // resize img to fit dims
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

        cv::waitKey(100);

        // read neural network output
        uint32_t recognized_number = bram[128];
        std::cout << "Recognized number: " << recognized_number << std::endl << std::endl;

        cv::waitKey(1000);
    }

    return 0;
}

