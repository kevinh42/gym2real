#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>
#include <memory_manager.h>
#include <stdio.h>
#include <assert.h>

template<int NUM_OBS,int NUM_ACT>
struct OnnxController
{
    OnnxController(StateStore &store, std::string onnx_file, int controller_id)
    {
        session_{env_, onnx_file, Ort::SessionOptions{nullptr}}
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
    Ort::Session session_;
    Ort::RunOptions opt_{nullptr};

    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 2> input_shape_{1,NUM_OBS};

    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 2> output_shape_{1,NUM_ACT};
};