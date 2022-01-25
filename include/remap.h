#pragma once
/** Remaps copy data from one buffer to another and remaps/inverts/clips based on configuration.
 *  Data in the memory stores should always be in the same space for easier troubleshooting.
 *  
 *  Examples:
 *
 *   Interfacing with actuators (assumes actuator commands and feedback are in same space):
 *   `
 *   - actuator_1: # Actuator space (depends on actuator firmware) to joint space (URDF)
 *       actuator_range: [-1, 1] # Values that should be given to motor interface
 *       joint: joint_1 # Gets limits from URDF
 *       invert: True
 *       type: effort # position/velocity/effort
 * 
 *   - actuator_2: # Actuator space (depends on actuator firmware) to joint space (URDF)
 *       actuator_range: [-1, 1] # Values that should be given to motor interface
 *       joint_range: [0, 3.14159] # Overrides limits from URDF
 *       invert: False
 *       type: effort # position/velocity/effort
 *   `
 *
 *   Interfacing with sensors:
 *   `
 *   - sensor_1: # Sensor space
 *       sensor_range: [0, 255] 
 *       controller_range: [0, 1] 
 *       invert: False
 *   `
**/
#include <string>
#include <vector>
#include <urdf_parser/urdf_parser.h>

enum ControlType
{
    CONTROL_POSITION,
    CONTROL_VELOCITY,
    CONTROL_EFFORT
};

enum TransformDirection
{
    TRANSFORM_FORWARD,
    TRANSFORM_BACK,
};

//Levels of spaces Actuator->Joint (assumes controller uses Joint space as input),Sensor->Controller
template <class T1, class T2>
struct RemapRule
{
    bool invert = false;
    std::pair<T1, T1> from_range;
    std::pair<T2, T2> to_range;

    RemapRule(
        std::pair<T1, T1> &from_range,
        std::pair<T2, T2> &to_range,
        bool invert) : from_range(from_range), to_range(to_range), invert(invert){};
};

template <class T1, class T2>
struct JointRemapRule : RemapRule<T1, T2>
{
    std::string joint = "";
    ControlType type = CONTROL_EFFORT;

    JointRemapRule(
        std::string &joint,
        std::pair<T1, T1> &from_range,
        std::pair<T2, T2> &to_range,
        ControlType type,
        bool invert) : joint(joint), type(type), RemapRule<T1, T2>(from_range, to_range, invert){};
};

template <class T1, class T2>
struct ControllerRemapRule : RemapRule<T1, T2>
{
    ControllerRemapRule(
        std::pair<T1, T1> &from_range,
        std::pair<T2, T2> &to_range,
        bool invert) : RemapRule<T1, T2>(from_range, to_range, invert){};
};

template <class T1, class T2> //TODO: Support integers
void remap_forward(T1 *in_ptr, T2 *out_ptr, RemapRule<T1, T2> &rule)
{
    if (rule.invert)
        out_ptr[0] = rule.to_range.second + (*in_ptr - rule.from_range.first) * (rule.to_range.first - rule.to_range.second) / (rule.from_range.second - rule.from_range.first);
    else
        out_ptr[0] = rule.to_range.first + (*in_ptr - rule.from_range.first) * (rule.to_range.second - rule.to_range.first) / (rule.from_range.second - rule.from_range.first);
};

template <class T1, class T2> //TODO: Support integers
void remap_back(T1 *in_ptr, T2 *out_ptr, RemapRule<T2, T1> &rule)
{
    if (rule.invert)
        out_ptr[0] = rule.from_range.second + (*in_ptr - rule.to_range.first) * (rule.from_range.first - rule.from_range.second) / (rule.to_range.second - rule.to_range.first);
    else
        out_ptr[0] = rule.from_range.first + (*in_ptr - rule.to_range.first) * (rule.from_range.second - rule.from_range.first) / (rule.to_range.second - rule.to_range.first);
};

/** Transformations copy data from any number of buffers to a buffer in specified order.
 *  Should be used whenever data is being copied 
 *  Remaps can be 
 *  
 *  Example:
 * 
 *  `
 *  - transform_1:
 *      direction: ToController # It should always be clear which direction data is being transformed
 *      to: to_buffer # Buffer we're copying data to
 *      from:
 *          - buffer_1: # Buffer we're copying data from
 *              from_idx: [0,1,2]
 *              to_idx: [0,1,2]
 *          - buffer_2:
 *              from_idx: [4,2,1,1] # Allow for copying elements in different order as well as repeats
 *              to_idx: [6,5,4,3]
 *              remap: actuator_1
 *  `
 * 
 */
template <class T1,class T2>
struct TransformFromRule
{
    T1 *from_buffer;
    std::vector<size_t> from_idx;
    std::vector<size_t> to_idx;
    RemapRule<T1,T2> remap_rule;
};

template <class T1, class T2>
struct TransformRule
{
    TransformDirection direction;
    std::vector<TransformFromRule<T1,T2>> from_rules;
    T2 *to_buffer;
};

template <class T1, class T2>
void transform(TransformRule<T1, T2> &rule)
{
    for (auto &from_rule : rule.from_rules)
    {
        for (size_t i = 0; i < from_rule.from_idx.size(); i++)
        {
            if (rule.direction == TRANSFORM_FORWARD)
                remap_forward(from_rule.from_buffer + from_rule.from_idx[i], rule.to_buffer + from_rule.to_idx[i], from_rule.remap_rule);
            else if (rule.direction == TRANSFORM_BACK)
                remap_back(from_rule.from_buffer + from_rule.from_idx[i], rule.to_buffer + from_rule.to_idx[i], from_rule.remap_rule);
        }
    }
};