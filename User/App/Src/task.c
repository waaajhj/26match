#include "task.h"

#include "motion_control.h"
#include "bsp_can.h"
#include "DM_Motor.h"
#include "jy61p.h"
#include "maixcam.h"
#include "pid.h"

// 正5 cm阶段提高位置P并降低速度阻尼，兼顾到达速度和目标精度。
#define TASK_2_POSITIVE_KP_SCALE 1.65f // 降低+5 cm转向前惯性，同时保持快速到达
#define TASK_2_POSITIVE_KV 0.00020f

// 任务6参数独立于任务2；特殊环境调参只修改这一组定义和专用加载函数。
#define TASK_2_SPECIAL_POSITIVE_KP_SCALE 1.4f
#define TASK_2_SPECIAL_POSITIVE_KV 0.00020f
// 任务6负5目标独立增加5 pixel；普通任务2仍使用全局标定值325 pixel。
#define TASK_2_SPECIAL_NEGATIVE_5CM_TARGET_POSITION \
    (BALL_BALANCE_NEGATIVE_5CM_TARGET_POSITION + 10.0f)

// 任务5启动前最多等待1 s的新视觉有效包；超时不会启动球杆和底盘。
#define TASK_5_FIRST_VISUAL_TIMEOUT_MS 1000U
// 任务5只接受100 ms内的Pitch CAN反馈，避免把断线前的旧角度作为新水平基准。
#define TASK_5_MOTOR_FEEDBACK_TIMEOUT_MS 100U
// 任务5使能握手最多等待100 ms，并以3 ms周期重发使能/清错请求。
#define TASK_5_MOTOR_ENABLE_TIMEOUT_MS 100U
#define TASK_5_MOTOR_ENABLE_RETRY_MS 3U
// 任务7按用户要求每3 ms直接重发一次Pitch电机失能帧。
#define TASK_7_DISABLE_PERIOD_MS 3U

// 任务2阶段只在主循环修改，可在Keil Watch中观察当前目标阶段。
volatile Task2Stage_e task_2_stage = TASK_2_STAGE_IDLE;
static float task_2_original_kp = 0.0f;
static float task_2_original_kv = 0.0f;
static uint8_t task_2_positive_kp_enabled = 0U;
// 当前任务2流程的负5目标，单位pixel；由主循环启动任务2或任务6时选择。
static float task_2_negative_target_position =
    BALL_BALANCE_NEGATIVE_5CM_TARGET_POSITION;

/**
 * @brief 恢复任务2启动前的位置P和速度反馈系数。
 * @note 仅正5 cm阶段临时修改参数，任务切换或进入第二阶段时调用。
 */
static void Task2_RestoreOriginalParams(void)
{
    if (task_2_positive_kp_enabled != 0U)
    {
        PID_DM_Pitch_Position.Kp = task_2_original_kp;
        ball_balance_velocity_kv = task_2_original_kv;
        task_2_positive_kp_enabled = 0U;
    }
}

/**
 * @brief 将普通任务2从+5 cm转向-5 cm阶段的分段PID参数加载到运行配置。
 * @retval 无。
 * @note 调用前必须关闭任务2分段控制，避免TIM4在写入期间读取不完整配置。
 *       参数单位分别为pixel、s、pixel/s及rad/pixel；函数无循环、延时和通信，
 *       不会启动电机。所有数值与当前已调好的任务2初始化值保持一致。
 */
static void Task2_BallControlParamsApply(void)
{
    task2_segmented_control.near_error_limit_pixel = 40.0f;
    task2_segmented_control.middle_error_limit_pixel = 100.0f;
    task2_segmented_control.velocity_filter_time_constant_s = 0.025f;
    task2_segmented_control.near_velocity_deadband_pixel_s = 15.0f;
    task2_segmented_control.low_pixel_near.Kp = 0.00032f;
    task2_segmented_control.low_pixel_near.Ki = 0.000002f;
    task2_segmented_control.low_pixel_near.Kv = 0.00028f;
    task2_segmented_control.high_pixel_near.Kp = 0.00020f;
    task2_segmented_control.high_pixel_near.Ki = 0.0f;
    task2_segmented_control.high_pixel_near.Kv = 0.00015f;
    task2_segmented_control.middle.Kp = 0.00030f;
    task2_segmented_control.middle.Ki = 0.0f;
    task2_segmented_control.middle.Kv = 0.00018f;
    task2_segmented_control.far.Kp = 0.00024f;
    task2_segmented_control.far.Ki = 0.0f;
    task2_segmented_control.far.Kv = 0.00024f;
}

/**
 * @brief 加载任务6（特殊环境任务2）的独立分段PID参数。
 * @retval 无。
 * @note 调用前必须关闭任务2分段控制；本函数保存任务6的独立参数。以后只需在
 *       本函数修改对应字段，即可保持任务2参数不变。函数不含循环、
 *       延时或通信，不会启动电机，也不会绕过球杆角度和电机限位。
 */
static void Task2Special_BallControlParamsApply(void)
{
    /*
     * 保留完整独立副本，避免调整普通任务2时连带改变特殊环境任务。
     * 任务6所有分段参数都在本函数维护，可直接修改对应字段。
     */
    task2_segmented_control.near_error_limit_pixel = 40.0f;
    task2_segmented_control.middle_error_limit_pixel = 100.0f;
    task2_segmented_control.velocity_filter_time_constant_s = 0.025f;
    task2_segmented_control.near_velocity_deadband_pixel_s = 15.0f;
    task2_segmented_control.low_pixel_near.Kp = 0.00028f;
    task2_segmented_control.low_pixel_near.Ki = 0.000001f;
    // 0.00022实测会在297 pixel附近停顿并大幅回摆，恢复合格率更高的阻尼。
    task2_segmented_control.low_pixel_near.Kv = 0.0002f;
    task2_segmented_control.high_pixel_near.Kp = 0.00020f;
    task2_segmented_control.high_pixel_near.Ki = 0.0f;
    task2_segmented_control.high_pixel_near.Kv = 0.00015f;
    task2_segmented_control.middle.Kp = 0.00030f;
    task2_segmented_control.middle.Ki = 0.0f;
    task2_segmented_control.middle.Kv = 0.00018f;
    task2_segmented_control.far.Kp = 0.00026f;
    task2_segmented_control.far.Ki = 0.0f;
    task2_segmented_control.far.Kv = 0.00024f;
}

/**
 * @brief 按普通或特殊环境配置启动任务2的公共两阶段控制流程。
 * @param special_mode 0表示普通任务2，非0表示串口屏任务6的特殊环境配置。
 * @retval 无。
 * @note 调用前要求视觉UART4、TIM4位置环、CAN和俯仰电机已初始化并使能。
 *       本函数只配置状态和参数，无循环、延时及阻塞通信；后续由TIM4闭环，
 *       由主循环task_2_update()在到达+5 cm后切换至-5 cm。视觉或CAN失效时的
 *       安全行为沿用现有球杆控制保护，不在本函数内持续驱动或等待反馈。
 */
static void Task2_StartCommon(uint8_t special_mode)
{
    float positive_kp_scale;
    float positive_velocity_kv;

    Task2SegmentedControl_Disable();
    Task3SegmentedControl_Disable();
    Task2_RestoreOriginalParams();

    if (special_mode != 0U)
    {
        Task2Special_BallControlParamsApply();
        positive_kp_scale = TASK_2_SPECIAL_POSITIVE_KP_SCALE;
        positive_velocity_kv = TASK_2_SPECIAL_POSITIVE_KV;
        task_2_negative_target_position =
            TASK_2_SPECIAL_NEGATIVE_5CM_TARGET_POSITION;
    }
    else
    {
        Task2_BallControlParamsApply();
        positive_kp_scale = TASK_2_POSITIVE_KP_SCALE;
        positive_velocity_kv = TASK_2_POSITIVE_KV;
        task_2_negative_target_position =
            BALL_BALANCE_NEGATIVE_5CM_TARGET_POSITION;
    }

    task_2_original_kp = PID_DM_Pitch_Position.Kp;
    task_2_original_kv = ball_balance_velocity_kv;
    PID_DM_Pitch_Position.Kp =
        task_2_original_kp * positive_kp_scale;
    ball_balance_velocity_kv = positive_velocity_kv;
    task_2_positive_kp_enabled = 1U;

    // 两个任务分时复用同一RAM采样区，通过task_id 2或6区分数据来源。
    if (special_mode != 0U)
    {
        Task2SpecialDebugRecorder_Start();
    }
    else
    {
        Task2DebugRecorder_Start();
    }

    BallBalanceControl_Start(
        BALL_BALANCE_POSITIVE_5CM_TARGET_POSITION,
        BALL_BALANCE_INTERMEDIATE_TOLERANCE_PIXEL);
    task_2_stage = TASK_2_STAGE_TO_POSITIVE_5CM;
    // 防止主循环重复启动同一任务。
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}

/**
 * @brief 任务1：只运行底盘循迹，不启用球杆闭环。
 */
void task_1(void)
{
    Task2SegmentedControl_Disable();
    Task3SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    task_2_stage = TASK_2_STAGE_IDLE;
    BallBalanceControl_Stop();
    ChassisTrack_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}

/**
 * @brief 任务2：底盘保持静止，只运行TIM4球杆闭环。
 */
void task_2(void)
{
    Task2_StartCommon(0U);
}

/**
 * @brief 任务6：使用独立PID配置执行与任务2相同的+5 cm到-5 cm控制流程。
 * @retval 无。
 * @note 串口屏发送0xAA 0x06 0x55后由主循环调用；要求视觉UART4、TIM4、CAN
 *       和俯仰电机均已初始化。函数本身无循环和延时，不阻塞主循环；当前特殊
 *       参数和负5目标均与任务2独立，且继续受现有球杆角度及电机电控限位保护。
 */
void task_2_special(void)
{
    Task2_StartCommon(1U);
}

/**
 * @brief 在主循环中推进任务2的目标点序列。
 * @note TIM4负责更新连续到达计数；本函数无阻塞行为。
 *       到达+5 cm后直接切换到-5 cm，避免在中心停顿影响总时间。
 *       切换后启用任务2专用分段PID，并持续闭环保持最终位置。
 */
void task_2_update(void)
{
    if (ball_balance_target_in_range_count == 0U)
    {
        return;
    }

    switch (task_2_stage)
    {
        case TASK_2_STAGE_TO_POSITIVE_5CM:
            // 直接进入-5 cm阶段，省略中心目标以缩短任务时间。
            Task2_RestoreOriginalParams();
            Task2SegmentedControl_Enable();
            BallBalanceControl_SetTarget(
                task_2_negative_target_position,
                BALL_BALANCE_FINAL_TOLERANCE_PIXEL);
            task_2_stage = TASK_2_STAGE_TO_NEGATIVE_5CM;
            break;

        case TASK_2_STAGE_TO_NEGATIVE_5CM:
            // 最终目标不再切换，位置环和速度反馈持续稳定小球。
            break;

        default:
            break;
    }
}

/**
 * @brief 加载任务3的球杆分段控制参数。
 * @note 本函数无参数和返回值；调用前必须先关闭任务3分段控制，避免TIM4在
 *       参数写入过程中读取到一半新一半旧的配置。本函数不包含循环、延时或
 *       通信，不会启动电机；角度限幅和刹车限幅均保留任务3的安全设置。
 */
static void Task3_BallControlParamsApply(void)
{
    // 分段阈值、视觉速度滤波和五段位置/速度反馈参数。
    task3_segmented_control.near_error_limit_pixel = 30.0f;
    task3_segmented_control.middle_error_limit_pixel = 100.0f;
    task3_segmented_control.velocity_filter_time_constant_s = 0.040f;
    task3_segmented_control.near_velocity_deadband_pixel_s = 15.0f;
    // 任务3运动全过程采用非对称速度阻尼，减小低像素回摆后的高侧反弹。
    task3_segmented_control.low_direction_velocity_kv_enabled = 1U;

    task3_segmented_control.near.Kp = 0.00028f;
    task3_segmented_control.near.Ki = 0.0f;
    task3_segmented_control.near.Kv = 0.00038f; // 实测0.00034未减小振幅，恢复原近段速度阻尼
    task3_segmented_control.low_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.low_pixel_middle.Ki = 0.0f;
    task3_segmented_control.low_pixel_middle.Kv = 0.00030f;
    task3_segmented_control.low_pixel_far.Kp = 0.00028f;
    task3_segmented_control.low_pixel_far.Ki = 0.0f;
    task3_segmented_control.low_pixel_far.Kv = 0.00024f;
    task3_segmented_control.high_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.high_pixel_middle.Ki = 0.0f;
    task3_segmented_control.high_pixel_middle.Kv = 0.00024f;
    task3_segmented_control.high_pixel_far.Kp = 0.00028f;
    task3_segmented_control.high_pixel_far.Ki = 0.0f;
    task3_segmented_control.high_pixel_far.Kv = 0.00020f;

    /*
     * 两轮片上采样表明-10偏向高像素而-18会过度拉向低像素；
     * 任务3采用折中零偏-14，且不改变任务2目标。
     */
    task3_segmented_control.target_offset_pixel = -14.0f;
    // 加速时向低像素滑动使用较小Kv，减小电机目标大幅反向。
    task3_segmented_control.startup_velocity_kv = 0.00030f;
    // 提前撤除仍向高像素侧推球的正向前馈，给高速回摆保留制动距离。
    task3_segmented_control.startup_feedforward_cutoff_start_pixel = 232.0f;
    task3_segmented_control.startup_feedforward_cutoff_velocity_pixel_s = 10.0f;
    // 高像素超过240 pixel且仍向高侧运动时，提前介入任务3过渡期软边界。
    task3_segmented_control.transition_high_brake_start_pixel = 240.0f;
    // 实测0.0016会激起更大的反向回摆，恢复匀速段更平稳的0.0013。
    task3_segmented_control.transition_high_brake_gain_rad_per_pixel = 0.0013f;
    task3_segmented_control.transition_high_brake_limit_rad = 0.01221730f; // 最大附加制动0.7°
    // 实测0.35会因前馈残留扩大高侧回摆，恢复效果更好的0.45退出系数。
    task3_segmented_control.acceleration_release_filter_alpha = 0.45f;
    // 3 s缓加速后高侧仍偏大，任务3单独降低前馈增益，将启动峰值角约降至1.32°。
    task3_segmented_control.acceleration_feedforward_gain = 0.00330f;
    // 恢复实测低像素侧保护更好的任务3正向启动前馈限幅+4.5°。
    task3_segmented_control.acceleration_feedforward_limit_rad = 0.07853982f;
    task3_segmented_control.acceleration_brake_feedforward_limit_rad =
        0.10471976f;
    task3_segmented_control.acceleration_filter_alpha = 1.0f;
    task3_segmented_control.brake_release_filter_alpha = 0.65f;
    // 任务3始终使用工程默认绝对零点，避免继承任务5临时软件基准。
    task3_segmented_control.motor_zero_angle_rad = 0.0f;
    task3_segmented_control.pitch_motor_kp = 3.5f;
    // 实测Kd=0.2会扩大快速目标变化时的跟随滞后，任务3恢复原MIT速度阻尼0.1。
    task3_segmented_control.pitch_motor_kd = 0.1f;
}

/**
 * @brief 加载任务4专用的球杆分段控制参数，初始数值完整复制自任务3。
 * @note 本函数无参数和返回值；调用前必须先关闭任务3分段控制，避免TIM4在
 *       参数写入过程中读取到不完整配置。本函数不包含循环、延时或通信，
 *       不会启动电机；任务4后续调参只修改本函数，不会影响任务3。
 */
static void Task4_BallControlParamsApply(void)
{
    // 任务4独立保存分段阈值、视觉速度滤波和五段位置/速度反馈参数。
    task3_segmented_control.near_error_limit_pixel = 30.0f;
    task3_segmented_control.middle_error_limit_pixel = 100.0f;
    task3_segmented_control.velocity_filter_time_constant_s = 0.040f;
    task3_segmented_control.near_velocity_deadband_pixel_s = 15.0f;
    // 同步任务3已验证的全过程非对称速度阻尼，仍由任务4函数独立保存参数。
    task3_segmented_control.low_direction_velocity_kv_enabled = 1U;

    task3_segmented_control.near.Kp = 0.00028f;
    task3_segmented_control.near.Ki = 0.0f;
    task3_segmented_control.near.Kv = 0.00038f;
    task3_segmented_control.low_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.low_pixel_middle.Ki = 0.0f;
    task3_segmented_control.low_pixel_middle.Kv = 0.00030f;
    task3_segmented_control.low_pixel_far.Kp = 0.00028f;
    task3_segmented_control.low_pixel_far.Ki = 0.0f;
    task3_segmented_control.low_pixel_far.Kv = 0.00024f;
    task3_segmented_control.high_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.high_pixel_middle.Ki = 0.0f;
    task3_segmented_control.high_pixel_middle.Kv = 0.00024f;
    task3_segmented_control.high_pixel_far.Kp = 0.00028f;
    task3_segmented_control.high_pixel_far.Ki = 0.0f;
    task3_segmented_control.high_pixel_far.Kv = 0.00020f;

    task3_segmented_control.target_offset_pixel = -14.0f;
    task3_segmented_control.startup_velocity_kv = 0.00030f;
    // 同步任务3当前已连续两轮满足±18 pixel的启动前馈撤除阈值。
    task3_segmented_control.startup_feedforward_cutoff_start_pixel = 232.0f;
    task3_segmented_control.startup_feedforward_cutoff_velocity_pixel_s = 10.0f;
    task3_segmented_control.transition_high_brake_start_pixel = 240.0f;
    task3_segmented_control.transition_high_brake_gain_rad_per_pixel = 0.0013f;
    task3_segmented_control.transition_high_brake_limit_rad = 0.01221730f;
    task3_segmented_control.acceleration_release_filter_alpha = 0.45f;
    task3_segmented_control.acceleration_feedforward_gain = 0.00330f;
    task3_segmented_control.acceleration_feedforward_limit_rad = 0.07853982f;
    task3_segmented_control.acceleration_brake_feedforward_limit_rad =
        0.10471976f;
    task3_segmented_control.acceleration_filter_alpha = 1.0f;
    task3_segmented_control.brake_release_filter_alpha = 0.65f;
    // 任务4仍使用工程默认绝对零点；任务5会在复制参数后单独覆盖该字段。
    task3_segmented_control.motor_zero_angle_rad = 0.0f;
    task3_segmented_control.pitch_motor_kp = 3.5f;
    task3_segmented_control.pitch_motor_kd = 0.1f;
}

/**
 * @brief 加载任务5球杆控制参数，当前完整继承任务4的已配置数值。
 * @note 本函数无参数和返回值；调用前必须关闭分段控制。函数不包含循环、延时
 *       或通信，也不会启动电机；后续任务5需要独立调参时可在本函数末尾覆盖
 *       对应字段，不会改变任务4的初始化代码。
 */
static void Task5_BallControlParamsApply(void)
{
    Task4_BallControlParamsApply();
}

/**
 * @brief 捕获Pitch电机当前角度作为任务5水平基准，并完成MIT模式使能和原位保持。
 * @retval 1表示反馈有效、软件基准已设置且电机已经使能；0表示反馈无效、越界或
 *         100 ms内未完成使能，调用者必须保持闭环停止并发送失能指令。
 * @note 调用前必须执行BallBalanceControl_Pause()、关闭任务3分段控制并完成CAN
 *       初始化。角度来自Gimbal_Motor[1]的CAN反馈，单位rad、逆时针为正；函数
 *       最多阻塞100 ms，每3 ms重试一次，使能后只发送捕获角原位保持指令。
 */
static uint8_t Task5_MotorReferenceCaptureAndEnable(void)
{
    const DM_Motor_t *pitch_motor = get_gimbal_motor_measure_point(1U);
    uint32_t now_ms = HAL_GetTick();
    uint32_t enable_start_ms;
    float captured_position_rad;

    if ((pitch_motor->measure.ID != (uint8_t)DM_PITCH_TX_ID) ||
        (pitch_motor->LastFeedbackTime == 0U) ||
        ((uint32_t)(now_ms - pitch_motor->LastFeedbackTime) >
         TASK_5_MOTOR_FEEDBACK_TIMEOUT_MS))
    {
        return 0U;
    }

    captured_position_rad = pitch_motor->Position;
    if (BallBalanceMotorHorizontalReferenceSet(captured_position_rad) == 0U)
    {
        return 0U;
    }

    enable_start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - enable_start_ms) <
           TASK_5_MOTOR_ENABLE_TIMEOUT_MS)
    {
        if (pitch_motor->measure.State == (uint8_t)MOTOR_ENABLE)
        {
            /*
             * 电机已经使能时发送捕获角保持帧，避免等待视觉首帧期间机构偏离
             * 人工设置的水平姿态；目标仍受DM_pos_limit保护。
             */
            DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                          captured_position_rad, 0.0f,
                          task3_segmented_control.pitch_motor_kp,
                          task3_segmented_control.pitch_motor_kd, 0.0f);
            return 1U;
        }

        /*
         * 反馈为失能时该接口发送0xFC使能帧；若电机报告错误状态则先清错，
         * 下一周期继续尝试。超时后由调用者重新失能，避免无反馈持续驱动。
         */
        DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                      captured_position_rad, 0.0f,
                      task3_segmented_control.pitch_motor_kp,
                      task3_segmented_control.pitch_motor_kd, 0.0f);
        HAL_Delay(TASK_5_MOTOR_ENABLE_RETRY_MS);
    }

    return 0U;
}

/**
 * @brief 任务3：主循环执行底盘循迹，TIM4同时执行球杆闭环。
 */
void task_3(void)
{
    Task2SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    Task3SegmentedControl_Disable();
    Task3_BallControlParamsApply();
    Task3SegmentedControl_Enable();
    // 任务3启动共享RAM记录器；其他任务启动时会覆盖本轮样本。
    Task3DebugRecorder_Start();
    task_2_stage = TASK_2_STAGE_IDLE;
    BallBalanceControl_Start(
        BALL_BALANCE_DEFAULT_TARGET_POSITION,
        0.0f); // 任务3无到达阈值，TIM4始终执行五段闭环。
    ChassisTrack2_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}
void task_4(void)
{ 
    Task2SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    Task3SegmentedControl_Disable();
    // 当前数值复制自任务3，但由任务4专用函数独立维护，后续调参互不影响。
    Task4_BallControlParamsApply();
    Task3SegmentedControl_Enable();
    // 任务4与任务2/3/5复用RAM采样区，新任务启动时覆盖上一轮数据。
    Task4DebugRecorder_Start();
    task_2_stage = TASK_2_STAGE_IDLE;
    BallBalanceControl_Start(
        BALL_BALANCE_DEFAULT_TARGET_POSITION,
        0.0f); // 任务4无到达阈值，TIM4始终执行五段闭环。
    ChassisTrack3_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;

}
/**
 * @brief 任务5：以启动后第一帧有效钢球坐标为固定目标，同时执行任务4底盘运动。
 * @note 调用前要求UART4视觉接收、TIM4、CAN和Pitch电机已经初始化。函数先暂停
 *       TIM4且不发送回零帧，读取100 ms内的Pitch反馈角作为本任务软件水平基准，
 *       再以3 ms周期进行最多100 ms的使能握手；随后阻塞等待最多1 s的新视觉包。
 *       反馈、使能或视觉任一失败时会失能电机、保持底盘停止并退出。捕获成功后
 *       调用ChassisTrack3_Run()，其间TIM4持续执行位置闭环。
 */
void task_5(void)
{
    uint32_t last_packet_count;
    uint32_t wait_start_tick_ms;
    float task_5_target_position = BALL_BALANCE_DEFAULT_TARGET_POSITION;
    float task_5_target_offset_pixel = 0.0f;
    uint8_t target_captured = 0U;

    /*
     * 必须先停TIM4且不发送固定零点命令，否则会在读取人工设置角度前改变机构姿态。
     * 该写入与TIM4共享，ball_balance_control_enabled使用volatile声明。
     */
    BallBalanceControl_Pause();
    Task2SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    Task3SegmentedControl_Disable();
    task_2_stage = TASK_2_STAGE_IDLE;
    Task5_BallControlParamsApply();

    if (Task5_MotorReferenceCaptureAndEnable() == 0U)
    {
        // CAN反馈或使能失败时保持安全失能，且不让临时基准遗留给下一次任务。
        DMMotorDisable(DM_PITCH_TX_ID, MIT_MODE);
        task3_segmented_control.motor_zero_angle_rad = 0.0f;
        serial_screen_task = SERIAL_SCREEN_TASK_NONE;
        return;
    }

    last_packet_count = point_packet_rx_count;
    wait_start_tick_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - wait_start_tick_ms) <
           TASK_5_FIRST_VISUAL_TIMEOUT_MS)
    {
        uint32_t current_packet_count = point_packet_rx_count;

        if (current_packet_count != last_packet_count)
        {
            float candidate_position =
                (float)point_packet.centerpoint_x;

            last_packet_count = current_packet_count;
            if ((candidate_position >= BALL_BALANCE_VISUAL_START_PIXEL) &&
                (candidate_position <= BALL_BALANCE_VISUAL_END_PIXEL))
            {
                // 固定保存任务启动后的第一帧有效钢球坐标，后续不再跟随测量值改变。
                task_5_target_position = candidate_position;
                target_captured = 1U;
                break;
            }
        }

        // 仅阻塞主循环1 ms，UART4和TIM中断仍可接收视觉数据并更新时间基准。
        HAL_Delay(1U);
    }

    if (target_captured == 0U)
    {
        // 未获得视觉目标时停止已使能的Pitch电机，等待下一次任务5重新捕获基准。
        DMMotorDisable(DM_PITCH_TX_ID, MIT_MODE);
        task3_segmented_control.motor_zero_angle_rad = 0.0f;
        serial_screen_task = SERIAL_SCREEN_TASK_NONE;
        return;
    }

    task_5_target_offset_pixel =
        task_5_target_position - BALL_BALANCE_DEFAULT_TARGET_POSITION;
    /*
     * 加速度前馈角仍由底盘平移加速度决定；这里只按任务5目标相对中心的像素差
     * 平移前馈撤除点和高侧软边界，使保护距离继续保持任务4的+5/+13 pixel。
     */
    task3_segmented_control.startup_feedforward_cutoff_start_pixel +=
        task_5_target_offset_pixel;
    task3_segmented_control.transition_high_brake_start_pixel +=
        task_5_target_offset_pixel;

    // 目标靠近相机边缘时限制保护阈值，避免产生视觉范围外的无效配置。
    if (task3_segmented_control.startup_feedforward_cutoff_start_pixel <
        BALL_BALANCE_VISUAL_START_PIXEL)
    {
        task3_segmented_control.startup_feedforward_cutoff_start_pixel =
            BALL_BALANCE_VISUAL_START_PIXEL;
    }
    else if (task3_segmented_control.startup_feedforward_cutoff_start_pixel >
             BALL_BALANCE_VISUAL_END_PIXEL)
    {
        task3_segmented_control.startup_feedforward_cutoff_start_pixel =
            BALL_BALANCE_VISUAL_END_PIXEL;
    }

    if (task3_segmented_control.transition_high_brake_start_pixel <
        BALL_BALANCE_VISUAL_START_PIXEL)
    {
        task3_segmented_control.transition_high_brake_start_pixel =
            BALL_BALANCE_VISUAL_START_PIXEL;
    }
    else if (task3_segmented_control.transition_high_brake_start_pixel >
             BALL_BALANCE_VISUAL_END_PIXEL)
    {
        task3_segmented_control.transition_high_brake_start_pixel =
            BALL_BALANCE_VISUAL_END_PIXEL;
    }

    Task3SegmentedControl_Enable();
    Task5DebugRecorder_Start();
    BallBalanceControl_Start(task_5_target_position, 0.0f);
    ChassisTrack3_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}

/**
 * @brief 任务7：在主循环中持续失能Pitch电机，供人工调整任务5水平姿态。
 * @retval 无。
 * @note 串口屏发送0xAA 0x07 0x55后进入；调用前要求USART2中断、HAL时基和
 *       CAN1已经初始化。函数会暂停TIM4球杆输出，然后每3 ms直接发送一帧
 *       MIT失能指令并阻塞主循环；USART2和HAL时基中断仍可运行。收到任意新的
 *       有效任务号时退出，且故意不清除新任务号，使下一轮主循环继续执行它。
 */
void task_7(void)
{
    /*
     * Pause只关闭TIM4输出、不发送回零帧，避免失能前先把人工姿态拉回默认零点。
     * 同时退出原任务分段状态，防止后续TIM4与失能帧交替发送。
     */
    BallBalanceControl_Pause();
    Task2SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    Task3SegmentedControl_Disable();
    task_2_stage = TASK_2_STAGE_IDLE;

    while (serial_screen_task == SERIAL_SCREEN_TASK_7)
    {
        /*
         * 必须直接调用DMMotorDisable：DM_MitControl在反馈已经失能时会转而发送
         * 普通MIT控制帧，不能满足持续重发0xFD失能帧的要求。
         */
        DMMotorDisable(DM_PITCH_TX_ID, MIT_MODE);
        HAL_Delay(TASK_7_DISABLE_PERIOD_MS);
    }
}

/**
 * @brief 根据串口屏发送的任务号执行对应任务。
 * @note 本函数在主循环调用；任务1、3、4和5包含底盘运动阻塞过程，运行期间
 *       TIM4仍可中断执行球杆控制。任务7也会阻塞主循环，但可由USART2中断收到
 *       的下一条有效任务指令退出。
 */
void task_switch(void)
{
    switch (serial_screen_task)
    {
        case SERIAL_SCREEN_TASK_1:
            task_1();
            break;
        case SERIAL_SCREEN_TASK_2:
            task_2();
            break;
        case SERIAL_SCREEN_TASK_3:
            task_3();
            break;
        case SERIAL_SCREEN_TASK_4:
            task_4();
            break;
        case SERIAL_SCREEN_TASK_5:
            task_5();
            break;
        case SERIAL_SCREEN_TASK_6:
            task_2_special();
            break;
        case SERIAL_SCREEN_TASK_7:
            task_7();
            break;
        default:
            break;
    }
}
