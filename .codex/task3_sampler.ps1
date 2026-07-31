[CmdletBinding()]
param(
    [ValidateSet('Start', 'Stop', 'Status')]
    [string]$Action = 'Start',
    [ValidateRange(100, 1000)]
    [int]$SampleCount = 300,
    [ValidateRange(0, 1000)]
    [int]$IntervalMs = 20,
    [ValidateRange(5, 100)]
    [int]$TailSampleCount = 25,
    [ValidateRange(10000, 600000)]
    [int]$TriggerTimeoutMs = 120000
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$axfPath = Join-Path $projectRoot 'MDK-ARM\26matchF4\26matchF4.axf'
$gdbPath = 'E:\link\gdb\gdb.exe'
$openOcdPath = 'E:\link\openocd\bin\openocd.exe'
$openOcdScripts = 'E:\link\openocd\share\openocd\scripts'
$pidPath = Join-Path $env:TEMP 'codex_task3_sampler.pid'
$tclPath = Join-Path $env:TEMP 'codex_task3_sampler.tcl'
$outputLogPath = Join-Path $env:TEMP 'codex_task3_latest_output.log'
$errorLogPath = Join-Path $env:TEMP 'codex_task3_latest_error.log'

function Get-Task3SamplerProcess
{
    if (-not (Test-Path -LiteralPath $pidPath))
    {
        return $null
    }

    $savedPidText = [System.IO.File]::ReadAllText($pidPath).Trim()
    $savedPid = 0
    if (-not [int]::TryParse($savedPidText, [ref]$savedPid))
    {
        return $null
    }

    $savedProcess = Get-Process -Id $savedPid -ErrorAction SilentlyContinue
    if (($null -eq $savedProcess) -or
        ($savedProcess.Path -ne $openOcdPath))
    {
        return $null
    }

    return $savedProcess
}

if ($Action -eq 'Status')
{
    $samplerProcess = Get-Task3SamplerProcess
    if ($null -eq $samplerProcess)
    {
        Write-Host 'Task3 sampler is not running.'
    }
    else
    {
        Write-Host "Task3 sampler is running. PID=$($samplerProcess.Id)."
    }

    Write-Host "Latest data log: $errorLogPath"
    exit 0
}

if ($Action -eq 'Stop')
{
    $samplerProcess = Get-Task3SamplerProcess
    if ($null -eq $samplerProcess)
    {
        Write-Host 'Task3 sampler is already stopped.'
        exit 0
    }

    Stop-Process -Id $samplerProcess.Id -Force
    Write-Host "Task3 sampler stopped. Log: $errorLogPath"
    exit 0
}

$runningSampler = Get-Task3SamplerProcess
if ($null -ne $runningSampler)
{
    throw "Task3 sampler is already running. PID=$($runningSampler.Id)."
}

$otherOpenOcd = Get-Process -Name 'openocd' -ErrorAction SilentlyContinue |
    Where-Object { $_.Path -eq $openOcdPath }
if ($null -ne $otherOpenOcd)
{
    throw 'Another OpenOCD debug or sampling process is using DAP-Link.'
}

foreach ($requiredPath in @($axfPath, $gdbPath, $openOcdPath, $openOcdScripts))
{
    if (-not (Test-Path -LiteralPath $requiredPath))
    {
        throw "Required sampler path is missing: $requiredPath"
    }
}

# Resolve addresses from the latest AXF after every firmware rebuild.
$symbolNames = @(
    'uwTick',
    'point_packet.centerpoint_x',
    'point_packet_rx_count',
    'chassis_motion_timing_active',
    'chassis_motion_elapsed_ms',
    'can_tx_enqueue_error_count',
    'ball_balance_control_enabled',
    'ball_balance_target_position',
    'ball_balance_raw_velocity_pixel_s',
    'ball_balance_filtered_velocity_pixel_s',
    'ball_balance_velocity_kv',
    'ball_balance_velocity_feedback_angle_rad',
    'ball_balance_rod_target_angle_rad',
    'task3_segmented_control.enabled',
    'task3_segmented_control.active_segment',
    'task3_segmented_control.near_error_limit_pixel',
    'task3_segmented_control.middle_error_limit_pixel',
    'task3_segmented_control.velocity_filter_time_constant_s',
    'task3_segmented_control.near_velocity_deadband_pixel_s',
    'task3_segmented_control.near.Kp',
    'task3_segmented_control.near.Ki',
    'task3_segmented_control.near.Kv',
    'task3_segmented_control.low_pixel_middle.Kp',
    'task3_segmented_control.low_pixel_middle.Ki',
    'task3_segmented_control.low_pixel_middle.Kv',
    'task3_segmented_control.low_pixel_far.Kp',
    'task3_segmented_control.low_pixel_far.Ki',
    'task3_segmented_control.low_pixel_far.Kv',
    'task3_segmented_control.high_pixel_middle.Kp',
    'task3_segmented_control.high_pixel_middle.Ki',
    'task3_segmented_control.high_pixel_middle.Kv',
    'task3_segmented_control.high_pixel_far.Kp',
    'task3_segmented_control.high_pixel_far.Ki',
    'task3_segmented_control.high_pixel_far.Kv',
    'task3_segmented_control.chassis_acceleration_raw_rad_s2',
    'task3_segmented_control.chassis_acceleration_rad_s2',
    'task3_segmented_control.acceleration_filter_alpha',
    'task3_segmented_control.acceleration_feedforward_gain',
    'task3_segmented_control.acceleration_feedforward_limit_rad',
    'task3_segmented_control.acceleration_feedforward_angle_rad',
    'PID_DM_Pitch_Position.Kp',
    'PID_DM_Pitch_Position.Ki',
    'PID_DM_Pitch_Position.Kd',
    'PID_DM_Pitch_Position.Error',
    'PID_DM_Pitch_Position.Integral',
    'PID_DM_Pitch_Position.IntegralOutput',
    'PID_DM_Pitch_Position.Differential',
    'PID_DM_Pitch_Position.Output',
    'Gimbal_Motor[1].Position',
    'Chassis_Motor[0].Omega',
    'Chassis_Motor[0].Acceleration',
    'Chassis_Motor[1].Omega',
    'Chassis_Motor[1].Acceleration',
    'bais',
    'W_out'
)

$gdbArguments = @(
    '-batch',
    '-ex', ('file ' + ($axfPath -replace '\\', '/'))
)
foreach ($symbolName in $symbolNames)
{
    $gdbArguments += @('-ex', "p/x &$symbolName")
}

$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$gdbOutput = & $gdbPath @gdbArguments 2>&1
$gdbExitCode = $LASTEXITCODE
$ErrorActionPreference = $previousErrorActionPreference
if ($gdbExitCode -ne 0)
{
    throw "Failed to read AXF symbols:`n$($gdbOutput -join "`n")"
}

$addressMatches = [regex]::Matches(
    ($gdbOutput -join "`n"),
    '\$\d+\s*=\s*(0x[0-9a-fA-F]+)')
if ($addressMatches.Count -ne $symbolNames.Count)
{
    throw "Unexpected AXF address count. Expected $($symbolNames.Count), got $($addressMatches.Count)."
}

$addresses = @{}
for ($index = 0; $index -lt $symbolNames.Count; $index++)
{
    $hexAddress = $addressMatches[$index].Groups[1].Value
    $addresses[$symbolNames[$index]] =
        [Convert]::ToUInt64($hexAddress.Substring(2), 16)
}

function Get-AlignedAddress([uint64]$Address)
{
    return $Address - ($Address % 4)
}

function Get-WordIndex([uint64]$Address, [uint64]$BaseAddress)
{
    return [int]((Get-AlignedAddress $Address) - $BaseAddress) / 4
}

function Get-ByteShift([uint64]$Address)
{
    return [int](($Address % 4) * 8)
}

function Format-HexAddress([uint64]$Address)
{
    return ('0x{0:x8}' -f $Address)
}

function Get-GroupLayout([string[]]$Names)
{
    $alignedAddresses = $Names | ForEach-Object {
        Get-AlignedAddress $addresses[$_]
    }
    $base = [uint64](($alignedAddresses | Measure-Object -Minimum).Minimum)
    $end = [uint64](($alignedAddresses | Measure-Object -Maximum).Maximum)
    return @{
        Base = $base
        Count = [int](($end - $base) / 4) + 1
    }
}

$controlNames = @(
    'ball_balance_control_enabled',
    'ball_balance_target_position',
    'ball_balance_raw_velocity_pixel_s',
    'ball_balance_filtered_velocity_pixel_s',
    'ball_balance_velocity_kv',
    'ball_balance_velocity_feedback_angle_rad',
    'ball_balance_rod_target_angle_rad'
)
$task3Names = $symbolNames | Where-Object {
    $_ -like 'task3_segmented_control.*'
}
$pidNames = $symbolNames | Where-Object {
    $_ -like 'PID_DM_Pitch_Position.*'
}
$chassisMotorNames = @(
    'Chassis_Motor[0].Omega',
    'Chassis_Motor[0].Acceleration',
    'Chassis_Motor[1].Omega',
    'Chassis_Motor[1].Acceleration'
)
$trackNames = @('bais', 'W_out')
$timingNames = @(
    'chassis_motion_timing_active',
    'chassis_motion_elapsed_ms'
)

$controlLayout = Get-GroupLayout $controlNames
$task3Layout = Get-GroupLayout $task3Names
$pidLayout = Get-GroupLayout $pidNames
$chassisMotorLayout = Get-GroupLayout $chassisMotorNames
$trackLayout = Get-GroupLayout $trackNames
$timingLayout = Get-GroupLayout $timingNames

$replacementValues = @{
    '__CONTROL_BASE__' = Format-HexAddress $controlLayout.Base
    '__CONTROL_COUNT__' = [string]$controlLayout.Count
    '__CONTROL_ENABLE_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_control_enabled'] $controlLayout.Base)
    '__CONTROL_ENABLE_SHIFT__' = [string](Get-ByteShift $addresses['ball_balance_control_enabled'])
    '__TARGET_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_target_position'] $controlLayout.Base)
    '__RAW_VELOCITY_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_raw_velocity_pixel_s'] $controlLayout.Base)
    '__FILTERED_VELOCITY_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_filtered_velocity_pixel_s'] $controlLayout.Base)
    '__KV_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_velocity_kv'] $controlLayout.Base)
    '__VELOCITY_FEEDBACK_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_velocity_feedback_angle_rad'] $controlLayout.Base)
    '__ROD_TARGET_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_rod_target_angle_rad'] $controlLayout.Base)

    '__TASK3_BASE__' = Format-HexAddress $task3Layout.Base
    '__TASK3_COUNT__' = [string]$task3Layout.Count
    '__TASK3_ENABLE_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.enabled'] $task3Layout.Base)
    '__TASK3_ENABLE_SHIFT__' = [string](Get-ByteShift $addresses['task3_segmented_control.enabled'])
    '__SEGMENT_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.active_segment'] $task3Layout.Base)
    '__SEGMENT_SHIFT__' = [string](Get-ByteShift $addresses['task3_segmented_control.active_segment'])
    '__NEAR_LIMIT_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.near_error_limit_pixel'] $task3Layout.Base)
    '__MIDDLE_LIMIT_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.middle_error_limit_pixel'] $task3Layout.Base)
    '__VELOCITY_TAU_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.velocity_filter_time_constant_s'] $task3Layout.Base)
    '__VELOCITY_DEADBAND_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.near_velocity_deadband_pixel_s'] $task3Layout.Base)
    '__NEAR_KP_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.near.Kp'] $task3Layout.Base)
    '__NEAR_KI_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.near.Ki'] $task3Layout.Base)
    '__NEAR_KV_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.near.Kv'] $task3Layout.Base)
    '__LOW_MIDDLE_KP_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.low_pixel_middle.Kp'] $task3Layout.Base)
    '__LOW_MIDDLE_KI_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.low_pixel_middle.Ki'] $task3Layout.Base)
    '__LOW_MIDDLE_KV_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.low_pixel_middle.Kv'] $task3Layout.Base)
    '__LOW_FAR_KP_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.low_pixel_far.Kp'] $task3Layout.Base)
    '__LOW_FAR_KI_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.low_pixel_far.Ki'] $task3Layout.Base)
    '__LOW_FAR_KV_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.low_pixel_far.Kv'] $task3Layout.Base)
    '__HIGH_MIDDLE_KP_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.high_pixel_middle.Kp'] $task3Layout.Base)
    '__HIGH_MIDDLE_KI_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.high_pixel_middle.Ki'] $task3Layout.Base)
    '__HIGH_MIDDLE_KV_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.high_pixel_middle.Kv'] $task3Layout.Base)
    '__HIGH_FAR_KP_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.high_pixel_far.Kp'] $task3Layout.Base)
    '__HIGH_FAR_KI_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.high_pixel_far.Ki'] $task3Layout.Base)
    '__HIGH_FAR_KV_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.high_pixel_far.Kv'] $task3Layout.Base)
    '__RAW_ACCEL_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.chassis_acceleration_raw_rad_s2'] $task3Layout.Base)
    '__FILTERED_ACCEL_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.chassis_acceleration_rad_s2'] $task3Layout.Base)
    '__ACCEL_ALPHA_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.acceleration_filter_alpha'] $task3Layout.Base)
    '__FF_GAIN_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.acceleration_feedforward_gain'] $task3Layout.Base)
    '__FF_LIMIT_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.acceleration_feedforward_limit_rad'] $task3Layout.Base)
    '__FF_ANGLE_INDEX__' = [string](Get-WordIndex $addresses['task3_segmented_control.acceleration_feedforward_angle_rad'] $task3Layout.Base)

    '__PID_BASE__' = Format-HexAddress $pidLayout.Base
    '__PID_COUNT__' = [string]$pidLayout.Count
    '__PID_KP_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.Kp'] $pidLayout.Base)
    '__PID_KI_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.Ki'] $pidLayout.Base)
    '__PID_KD_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.Kd'] $pidLayout.Base)
    '__PID_ERROR_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.Error'] $pidLayout.Base)
    '__PID_INTEGRAL_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.Integral'] $pidLayout.Base)
    '__PID_IOUT_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.IntegralOutput'] $pidLayout.Base)
    '__PID_DIFF_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.Differential'] $pidLayout.Base)
    '__PID_OUTPUT_INDEX__' = [string](Get-WordIndex $addresses['PID_DM_Pitch_Position.Output'] $pidLayout.Base)

    '__CHASSIS_MOTOR_BASE__' = Format-HexAddress $chassisMotorLayout.Base
    '__CHASSIS_MOTOR_COUNT__' = [string]$chassisMotorLayout.Count
    '__MOTOR0_OMEGA_INDEX__' = [string](Get-WordIndex $addresses['Chassis_Motor[0].Omega'] $chassisMotorLayout.Base)
    '__MOTOR0_ACCEL_INDEX__' = [string](Get-WordIndex $addresses['Chassis_Motor[0].Acceleration'] $chassisMotorLayout.Base)
    '__MOTOR1_OMEGA_INDEX__' = [string](Get-WordIndex $addresses['Chassis_Motor[1].Omega'] $chassisMotorLayout.Base)
    '__MOTOR1_ACCEL_INDEX__' = [string](Get-WordIndex $addresses['Chassis_Motor[1].Acceleration'] $chassisMotorLayout.Base)

    '__TRACK_BASE__' = Format-HexAddress $trackLayout.Base
    '__TRACK_COUNT__' = [string]$trackLayout.Count
    '__BAIS_INDEX__' = [string](Get-WordIndex $addresses['bais'] $trackLayout.Base)
    '__WOUT_INDEX__' = [string](Get-WordIndex $addresses['W_out'] $trackLayout.Base)

    '__TIMING_BASE__' = Format-HexAddress $timingLayout.Base
    '__TIMING_COUNT__' = [string]$timingLayout.Count
    '__TIMING_ACTIVE_INDEX__' = [string](Get-WordIndex $addresses['chassis_motion_timing_active'] $timingLayout.Base)
    '__TIMING_ACTIVE_SHIFT__' = [string](Get-ByteShift $addresses['chassis_motion_timing_active'])
    '__ELAPSED_INDEX__' = [string](Get-WordIndex $addresses['chassis_motion_elapsed_ms'] $timingLayout.Base)

    '__TICK_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['uwTick'])
    '__TICK_SHIFT__' = [string](Get-ByteShift $addresses['uwTick'])
    '__POINT_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['point_packet.centerpoint_x'])
    '__POINT_SHIFT__' = [string](Get-ByteShift $addresses['point_packet.centerpoint_x'])
    '__PACKET_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['point_packet_rx_count'])
    '__PACKET_SHIFT__' = [string](Get-ByteShift $addresses['point_packet_rx_count'])
    '__GIMBAL_POSITION_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['Gimbal_Motor[1].Position'])
    '__GIMBAL_POSITION_SHIFT__' = [string](Get-ByteShift $addresses['Gimbal_Motor[1].Position'])
    '__CAN_ERROR_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['can_tx_enqueue_error_count'])
    '__CAN_ERROR_SHIFT__' = [string](Get-ByteShift $addresses['can_tx_enqueue_error_count'])

    '__SAMPLE_COUNT__' = [string]$SampleCount
    '__INTERVAL_MS__' = [string]$IntervalMs
    '__TAIL_SAMPLE_COUNT__' = [string]$TailSampleCount
    '__TRIGGER_TIMEOUT_MS__' = [string]$TriggerTimeoutMs
}

$tclTemplate = @'
# Auto-generated from the latest AXF by task3_sampler.ps1.
proc task3_sample {sample_limit interval_ms tail_limit trigger_timeout_ms} {
    init
    echo "TASK3_SAMPLE_WAITING"
    set waited_ms 0

    # Trigger only after Task3 control and chassis motion are both active.
    while {$waited_ms < $trigger_timeout_ms} {
        mem2array trigger_task3 32 __TASK3_BASE__ __TASK3_COUNT__
        mem2array trigger_timing 32 __TIMING_BASE__ __TIMING_COUNT__
        set task3_enabled [expr {($trigger_task3(__TASK3_ENABLE_INDEX__) >> __TASK3_ENABLE_SHIFT__) & 0xff}]
        set motion_active [expr {($trigger_timing(__TIMING_ACTIVE_INDEX__) >> __TIMING_ACTIVE_SHIFT__) & 0xff}]
        if {($task3_enabled != 0) && ($motion_active != 0)} {
            break
        }

        sleep 10
        incr waited_ms 10
    }

    if {$waited_ms >= $trigger_timeout_ms} {
        echo "TASK3_SAMPLE_TRIGGER_TIMEOUT"
        shutdown
        return
    }

    echo "TASK3_SAMPLE_TRIGGERED"
    echo "TASK3_SAMPLE_BEGIN"
    echo "H,index,tick,packet,point_x,elapsed_ms,motion_active,control_enabled,task3_enabled,segment,target,pid_kp,pid_ki,pid_kd,pid_error,pid_integral,pid_iout,pid_diff,pid_output,raw_velocity,filtered_velocity,kv,velocity_feedback,rod_target,gimbal_position,near_limit,middle_limit,velocity_tau,velocity_deadband,near_kp,near_ki,near_kv,low_middle_kp,low_middle_ki,low_middle_kv,low_far_kp,low_far_ki,low_far_kv,high_middle_kp,high_middle_ki,high_middle_kv,high_far_kp,high_far_ki,high_far_kv,raw_accel,filtered_accel,accel_alpha,ff_gain,ff_limit,ff_angle,motor0_omega,motor1_omega,motor0_accel,motor1_accel,bais,w_out,can_error"

    set sample_index 0
    set tail_count 0
    while {$sample_index < $sample_limit} {
        mem2array control_words 32 __CONTROL_BASE__ __CONTROL_COUNT__
        mem2array task3_words 32 __TASK3_BASE__ __TASK3_COUNT__
        mem2array pid_words 32 __PID_BASE__ __PID_COUNT__
        mem2array chassis_motor_words 32 __CHASSIS_MOTOR_BASE__ __CHASSIS_MOTOR_COUNT__
        mem2array track_words 32 __TRACK_BASE__ __TRACK_COUNT__
        mem2array timing_words 32 __TIMING_BASE__ __TIMING_COUNT__
        mem2array tick_word 32 __TICK_ADDRESS__ 1
        mem2array point_word 32 __POINT_ADDRESS__ 1
        mem2array packet_word 32 __PACKET_ADDRESS__ 1
        mem2array gimbal_position_word 32 __GIMBAL_POSITION_ADDRESS__ 1
        mem2array can_error_word 32 __CAN_ERROR_ADDRESS__ 1

        set point_x [expr {($point_word(0) >> __POINT_SHIFT__) & 0xffff}]
        set motion_active [expr {($timing_words(__TIMING_ACTIVE_INDEX__) >> __TIMING_ACTIVE_SHIFT__) & 0xff}]
        set control_enabled [expr {($control_words(__CONTROL_ENABLE_INDEX__) >> __CONTROL_ENABLE_SHIFT__) & 0xff}]
        set task3_enabled [expr {($task3_words(__TASK3_ENABLE_INDEX__) >> __TASK3_ENABLE_SHIFT__) & 0xff}]
        set active_segment [expr {($task3_words(__SEGMENT_INDEX__) >> __SEGMENT_SHIFT__) & 0xff}]

        set line [format "S,%d,%08x,%08x,%d,%08x,%d,%d,%d,%d" \
            $sample_index \
            [expr {$tick_word(0) >> __TICK_SHIFT__}] \
            [expr {$packet_word(0) >> __PACKET_SHIFT__}] \
            $point_x \
            $timing_words(__ELAPSED_INDEX__) \
            $motion_active \
            $control_enabled \
            $task3_enabled \
            $active_segment]

        append line [format ",%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x" \
            $control_words(__TARGET_INDEX__) \
            $pid_words(__PID_KP_INDEX__) \
            $pid_words(__PID_KI_INDEX__) \
            $pid_words(__PID_KD_INDEX__) \
            $pid_words(__PID_ERROR_INDEX__) \
            $pid_words(__PID_INTEGRAL_INDEX__) \
            $pid_words(__PID_IOUT_INDEX__) \
            $pid_words(__PID_DIFF_INDEX__) \
            $pid_words(__PID_OUTPUT_INDEX__)]

        append line [format ",%08x,%08x,%08x,%08x,%08x,%08x" \
            $control_words(__RAW_VELOCITY_INDEX__) \
            $control_words(__FILTERED_VELOCITY_INDEX__) \
            $control_words(__KV_INDEX__) \
            $control_words(__VELOCITY_FEEDBACK_INDEX__) \
            $control_words(__ROD_TARGET_INDEX__) \
            [expr {$gimbal_position_word(0) >> __GIMBAL_POSITION_SHIFT__}]]

        append line [format ",%08x,%08x,%08x,%08x" \
            $task3_words(__NEAR_LIMIT_INDEX__) \
            $task3_words(__MIDDLE_LIMIT_INDEX__) \
            $task3_words(__VELOCITY_TAU_INDEX__) \
            $task3_words(__VELOCITY_DEADBAND_INDEX__)]

        append line [format ",%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x" \
            $task3_words(__NEAR_KP_INDEX__) \
            $task3_words(__NEAR_KI_INDEX__) \
            $task3_words(__NEAR_KV_INDEX__) \
            $task3_words(__LOW_MIDDLE_KP_INDEX__) \
            $task3_words(__LOW_MIDDLE_KI_INDEX__) \
            $task3_words(__LOW_MIDDLE_KV_INDEX__) \
            $task3_words(__LOW_FAR_KP_INDEX__) \
            $task3_words(__LOW_FAR_KI_INDEX__) \
            $task3_words(__LOW_FAR_KV_INDEX__)]

        append line [format ",%08x,%08x,%08x,%08x,%08x,%08x" \
            $task3_words(__HIGH_MIDDLE_KP_INDEX__) \
            $task3_words(__HIGH_MIDDLE_KI_INDEX__) \
            $task3_words(__HIGH_MIDDLE_KV_INDEX__) \
            $task3_words(__HIGH_FAR_KP_INDEX__) \
            $task3_words(__HIGH_FAR_KI_INDEX__) \
            $task3_words(__HIGH_FAR_KV_INDEX__)]

        append line [format ",%08x,%08x,%08x,%08x,%08x,%08x" \
            $task3_words(__RAW_ACCEL_INDEX__) \
            $task3_words(__FILTERED_ACCEL_INDEX__) \
            $task3_words(__ACCEL_ALPHA_INDEX__) \
            $task3_words(__FF_GAIN_INDEX__) \
            $task3_words(__FF_LIMIT_INDEX__) \
            $task3_words(__FF_ANGLE_INDEX__)]

        append line [format ",%08x,%08x,%08x,%08x,%08x,%08x,%08x" \
            $chassis_motor_words(__MOTOR0_OMEGA_INDEX__) \
            $chassis_motor_words(__MOTOR1_OMEGA_INDEX__) \
            $chassis_motor_words(__MOTOR0_ACCEL_INDEX__) \
            $chassis_motor_words(__MOTOR1_ACCEL_INDEX__) \
            $track_words(__BAIS_INDEX__) \
            $track_words(__WOUT_INDEX__) \
            [expr {$can_error_word(0) >> __CAN_ERROR_SHIFT__}]]

        echo $line
        incr sample_index

        if {$motion_active == 0} {
            incr tail_count
            if {$tail_count >= $tail_limit} {
                break
            }
        } else {
            set tail_count 0
        }

        sleep $interval_ms
    }

    echo "TASK3_SAMPLE_END"
    shutdown
}

task3_sample __SAMPLE_COUNT__ __INTERVAL_MS__ __TAIL_SAMPLE_COUNT__ __TRIGGER_TIMEOUT_MS__
'@

foreach ($replacementKey in $replacementValues.Keys)
{
    $tclTemplate =
        $tclTemplate.Replace($replacementKey, $replacementValues[$replacementKey])
}

$utf8WithoutBom = [System.Text.UTF8Encoding]::new($false)
[System.IO.File]::WriteAllText($tclPath, $tclTemplate, $utf8WithoutBom)

$openOcdArgumentLine =
    "-s `"$openOcdScripts`" " +
    '-f "interface/cmsis-dap.cfg" ' +
    '-c "transport select swd" ' +
    '-f "target/stm32f4x.cfg" ' +
    # A lower SWD clock is more tolerant of chassis motor interference.
    '-c "adapter speed 1000" ' +
    "-f `"$tclPath`""

$samplerProcess = Start-Process `
    -FilePath $openOcdPath `
    -ArgumentList $openOcdArgumentLine `
    -RedirectStandardOutput $outputLogPath `
    -RedirectStandardError $errorLogPath `
    -WindowStyle Hidden `
    -PassThru

Start-Sleep -Milliseconds 700
if ($samplerProcess.HasExited)
{
    $startupError = ''
    if (Test-Path -LiteralPath $errorLogPath)
    {
        $startupError =
            (Get-Content -LiteralPath $errorLogPath -Tail 30) -join "`n"
    }
    throw "Task3 sampler failed to start:`n$startupError"
}

[System.IO.File]::WriteAllText(
    $pidPath,
    [string]$samplerProcess.Id,
    $utf8WithoutBom)

Write-Host "Task3 sampler started. PID=$($samplerProcess.Id)."
Write-Host 'Waiting for Task3 and chassis motion to become active.'
Write-Host "Max samples=$SampleCount, interval=${IntervalMs}ms, tail samples=$TailSampleCount."
Write-Host "Latest data log: $errorLogPath"
