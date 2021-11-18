#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

rcl_timer_t Ros_StateServer_Timer;
rclc_executor_t Ros_StateServer_Executor;

rcl_publisher_t publisherTransform;
tf2_msgs__msg__TFMessage *msgTransform;

typedef struct _timespec
{
    time_t      tv_sec;         /* seconds */
    long        tv_nsec;        /* nanoseconds (0 -1,000,000,000) */
} timespec;

typedef int BOOL;

#define MAX_JOINT_NAME_LENGTH   32

void Ros_StateServer_TimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time; //unused

    if (timer != NULL)
    {
        BOOL bRet;
        timespec timestamp;
        geometry_msgs__msg__Transform *transform;

        //timestamp
        clock_gettime(0, &timestamp);

        for (int i = 0; i < 6; i += 1)
        {
            msgTransform->transforms.data[i].header.stamp.sec = timestamp.tv_sec;
            msgTransform->transforms.data[i].header.stamp.nanosec = timestamp.tv_nsec;

            transform = &msgTransform->transforms.data[i].transform;

            transform->translation.x = 1;
            transform->translation.y = .5;
            transform->translation.z = .25;

            transform->rotation.x = 3.1415927;
            transform->rotation.y = -0.7853981;
            transform->rotation.z = 0;
        }

        rcl_publish(&publisherTransform, msgTransform, NULL);
    }
}

int main()
{
    rcl_allocator_t rclAllocator;
    rclc_support_t rclSupport;
    rcl_node_t rclNode;
    rcl_ret_t ret;

    //Wait for agent to become available
    do
    {
        usleep(1000);
        ret = rmw_uros_ping_agent(1000, 2);
    } while (ret != RCL_RET_OK);

    //=============================================
    rclAllocator = rcl_get_default_allocator();    
    
    rclc_support_init(&rclSupport, 0, NULL, &rclAllocator);
    rclc_node_init_default(&rclNode, "testnode", "", &rclSupport);

    //=============================================
    rclc_publisher_init_default(
        &publisherTransform,
        &rclNode,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "/tf");

    msgTransform = tf2_msgs__msg__TFMessage__create();
    geometry_msgs__msg__TransformStamped__Sequence__init(&msgTransform->transforms, 6);

    char formatBuffer[MAX_JOINT_NAME_LENGTH];
    int robotIterator = 0;
    for (int i = 0; i < 6; i += 3)
    {
        rosidl_runtime_c__String__assign(&msgTransform->transforms.data[i].header.frame_id, "world");

        snprintf(formatBuffer, MAX_JOINT_NAME_LENGTH, "base_r%d", robotIterator + 1);
        rosidl_runtime_c__String__assign(&msgTransform->transforms.data[i].child_frame_id, formatBuffer);
        rosidl_runtime_c__String__assign(&msgTransform->transforms.data[i + 1].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_JOINT_NAME_LENGTH, "flange_r%d", robotIterator + 1);
        rosidl_runtime_c__String__assign(&msgTransform->transforms.data[i + 1].child_frame_id, formatBuffer);
        rosidl_runtime_c__String__assign(&msgTransform->transforms.data[i + 2].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_JOINT_NAME_LENGTH, "tcp_%d", robotIterator);
        rosidl_runtime_c__String__assign(&msgTransform->transforms.data[i + 2].child_frame_id, formatBuffer);

        robotIterator += 1;
    }

    //=============================================
    rclc_timer_init_default(
        &Ros_StateServer_Timer,
        &rclSupport,
        RCL_MS_TO_NS(20),
        Ros_StateServer_TimerCallback);

    Ros_StateServer_Executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&Ros_StateServer_Executor, &rclSupport.context, 1, &rclAllocator);
    rclc_executor_add_timer(&Ros_StateServer_Executor, &Ros_StateServer_Timer);

    rclc_executor_spin(&Ros_StateServer_Executor);
}