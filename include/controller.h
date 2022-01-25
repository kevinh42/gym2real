#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>
#include <memory_manager.h>
#include <stdio.h>
#include <assert.h>
#include <remap.h>

/** Controllers should be defined with relevant transforms/remaps
 *  Should be used whenever data is being copied 
 *  
 *  Example:
 * 
 *  `
 *  - BalanceController:
 *      observations: 4 # Number of observations
 *      actions: 2 # Number of actions
 *      transforms:
 *          - transform_1
 *          - transform_2
 *  `
 */
class OnnxController
{
    std::vector<TransformRule<float, float>> pre_transforms;
    std::vector<TransformRule<float, float>> post_transforms;
    int observations_;
    int actions_;
    std::string model_path_;

    OnnxController(int observations, int actions, std::string model_path) : observations_(observations),
                                                                            actions_(actions),
                                                                            model_path_(model_path)
    {

        init();
    };

    int init()
    {
        session_ = Ort::Session{env_, model_path_.c_str(), Ort::SessionOptions{nullptr}};
        input_shape_ = {1, observations_};
        output_shape_ = {1, actions_};

        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_buffer_, observations_ * sizeof(float), input_shape_.data(), input_shape_.size());
        output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, output_buffer_, actions_ * sizeof(float), output_shape_.data(), output_shape_.size());
        return 0;
    };

    int run()
    {
        //Pre-transforms (load data into buffers)
        for (auto &trans : pre_transforms)
        {
            transform(trans);
        }

        //Main control step (inference)
        control_step();

        //Post-transforms (write data to buffers)
        for (auto &trans : post_transforms)
        {
            transform(trans);
        }
        return 0;
    };

    int control_step()
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
    Ort::Session session_{nullptr};
    Ort::RunOptions opt_{nullptr};

    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 2> input_shape_{1, 21};

    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 2> output_shape_{1, 12};
};