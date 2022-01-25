#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>
#include <memory_manager.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>

#include <controller.h>

struct OnnxInference
{
    OnnxInference(StateStore &store)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        size_t input_size;
        size_t output_size;
        store.create_critical_buffer("observations", 0, input_buffer_, input_size);
        store.create_critical_buffer("actions", 1, output_buffer_, output_size);

        input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_buffer_, input_size, input_shape_.data(), input_shape_.size());
        output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, output_buffer_, output_size, output_shape_.data(), output_shape_.size());
    }

    int Run()
    {
        const char *input_names[] = {"observations"};
        const char *output_names[] = {"actions"};

        session_.Run(opt_, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);

        return 0;
    }

private:
    float *input_buffer_ = nullptr;
    float *output_buffer_ = nullptr;

    Ort::Env env_;
    Ort::Session session_{env_, "Quadcopter.pth.onnx", Ort::SessionOptions{nullptr}};
    Ort::RunOptions opt_{nullptr};

    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 2> input_shape_{1,21};

    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 2> output_shape_{1,12};
};

int main(int argc, char **argv)
{
    StateStore m;
    OnnxInference inf(m);

    int i = 0;

    size_t input_size;
    float *input_buffer;
    if (-1 == m.create_critical_buffer("observations", 1, input_buffer, input_size))
    {
        errExit("observations");
    };

    size_t output_size;
    float *output_buffer;
    if (-1 == m.create_critical_buffer("actions", 0, output_buffer, output_size))
    {
        errExit("actions");
    };

    timespec time1, time2;
    long long diff;
    
    int ret;
    pthread_t this_thread = pthread_self();
    struct sched_param params;
    params.sched_priority = 80;
    std::cout << "Set realtime priority = " << params.sched_priority << std::endl;
    ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret != 0) {
        std::cout << "Failed" << std::endl;
        return 0;     
    }
    // Print thread scheduling priority
    std::cout << "Thread priority is " << params.sched_priority << std::endl; 
    while (1)
    {
        i++;
        
        std::generate(input_buffer, input_buffer + input_size, [&]
                      { return rand() % 255; }); // generate random numbers in the range [0, 255]
        clock_gettime(CLOCK_MONOTONIC, &time1);
        inf.Run();
        //std::cout << inf.Run() << std::endl;

        // std::cout << "output: ";
        // for (int j = 0; j < output_size; j++)
        // {
        //     std::cout << output_buffer[j] << ",";
        // }
        // std::cout << std::endl;
        clock_gettime(CLOCK_MONOTONIC, &time2);

        diff = 1e9*(time2.tv_sec-time1.tv_sec) + (time2.tv_nsec-time1.tv_nsec);
        if (diff>100000) std::cout << "Inference Time (us): " << diff/1000<<std::endl;
        // /* the size (in bytes) of shared memory object */
        // const int SIZE = 2048;

        // /* name of the shared memory object */
        // const char *name = "TEST";

        // /* shared memory file descriptor */
        // int shm_fd;

        // /* pointer to shared memory object */
        // void *ptr;

        // /* open the shared memory object */
        // shm_fd = shm_open(name, O_RDONLY, 0666);
        // if (shm_fd == -1)
        // {
        //     puts("FAILS");
        //     errExit("shm_open");
        // }

        // /* memory map the shared memory object */
        // ptr = mmap(0, SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
        // if (ptr != MAP_FAILED)
        // {
        //     /* read from the shared memory object */
        //     printf("%s", (char *)ptr);
        //     printf("%d", i);
        // }
        // else
        // {
        //     errExit("mmap");
        // }
        // if (close(shm_fd) == -1)
        // {
        //     errExit("close");
        // }
        // if (munmap(ptr, SIZE))
        // {
        //     errExit("munmap");
        // };
    }

    return 0;
}
