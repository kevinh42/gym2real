/** Remaps copy data from one buffer to another and remaps/inverts/clips based on configuration.
 *  Data in the memory stores should always be in the same space for easier troubleshooting.
 *  
 *  Example:
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

template <class T1, class T2>
void remap(T1 *in_buf, int in_size, T2 *out_buf, int out_size)
{
}

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

template <class T1, class T2>
void transform(T1 *in_buf, int in_size, T2 *out_buf, int out_size)
{
}


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