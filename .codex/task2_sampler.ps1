[CmdletBinding()]
param(
    [ValidateSet('Start', 'Stop', 'Status')]
    [string]$Action = 'Start',
    [ValidateRange(100, 5000)]
    [int]$SampleCount = 800,
    [ValidateRange(0, 1000)]
    [int]$IntervalMs = 20,
    [ValidateRange(10000, 600000)]
    [int]$TriggerTimeoutMs = 120000
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$axfPath = Join-Path $projectRoot 'MDK-ARM\26matchF4\26matchF4.axf'
$gdbPath = 'E:\link\gdb\gdb.exe'
$openOcdPath = 'E:\link\openocd\bin\openocd.exe'
$openOcdScripts = 'E:\link\openocd\share\openocd\scripts'
$pidPath = Join-Path $env:TEMP 'codex_task2_sampler.pid'
$tclPath = Join-Path $env:TEMP 'codex_task2_sampler.tcl'
$outputLogPath = Join-Path $env:TEMP 'codex_task2_latest_output.log'
$errorLogPath = Join-Path $env:TEMP 'codex_task2_latest_error.log'

function Get-Task2SamplerProcess
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
    if ($null -eq $savedProcess)
    {
        return $null
    }

    if ($savedProcess.Path -ne $openOcdPath)
    {
        return $null
    }

    return $savedProcess
}

if ($Action -eq 'Status')
{
    $samplerProcess = Get-Task2SamplerProcess
    if ($null -eq $samplerProcess)
    {
        Write-Host 'Task2 sampler is not running.'
    }
    else
    {
        Write-Host "Task2 sampler is running. PID=$($samplerProcess.Id)."
    }
    Write-Host "Latest data log: $errorLogPath"
    exit 0
}

if ($Action -eq 'Stop')
{
    $samplerProcess = Get-Task2SamplerProcess
    if ($null -eq $samplerProcess)
    {
        Write-Host 'Task2 sampler is already stopped.'
        exit 0
    }

    Stop-Process -Id $samplerProcess.Id -Force
    Write-Host "Task2 sampler stopped. Log: $errorLogPath"
    exit 0
}

$runningSampler = Get-Task2SamplerProcess
if ($null -ne $runningSampler)
{
    throw "Task2 sampler is already running. PID=$($runningSampler.Id)."
}

foreach ($requiredPath in @($axfPath, $gdbPath, $openOcdPath, $openOcdScripts))
{
    if (-not (Test-Path -LiteralPath $requiredPath))
    {
        throw "Required sampler path is missing: $requiredPath"
    }
}

# Resolve all addresses from the latest AXF after every rebuild.
$symbolNames = @(
    'uwTick',
    'point_packet.centerpoint_x',
    'point_packet_rx_count',
    'ball_balance_target_position',
    'ball_balance_target_tolerance_pixel',
    'ball_balance_velocity_kv',
    'task2_segmented_control',
    'task_2_stage',
    'PID_DM_Pitch_Position',
    'Gimbal_Motor[1].Position',
    'ball_balance_raw_velocity_pixel_s',
    'ball_balance_filtered_velocity_pixel_s',
    'ball_balance_target_in_range_count',
    'ball_balance_velocity_feedback_angle_rad',
    'ball_balance_rod_target_angle_rad'
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

$controlAddresses = @(
    $addresses['ball_balance_target_position'],
    $addresses['ball_balance_target_tolerance_pixel'],
    $addresses['ball_balance_velocity_kv'])
$controlAligned = $controlAddresses | ForEach-Object { Get-AlignedAddress $_ }
$controlBase = [uint64](($controlAligned | Measure-Object -Minimum).Minimum)
$controlEnd = [uint64](($controlAligned | Measure-Object -Maximum).Maximum)
$controlCount = [int](($controlEnd - $controlBase) / 4) + 1

$motionAddresses = @(
    $addresses['ball_balance_raw_velocity_pixel_s'],
    $addresses['ball_balance_filtered_velocity_pixel_s'],
    $addresses['ball_balance_target_in_range_count'],
    $addresses['ball_balance_velocity_feedback_angle_rad'],
    $addresses['ball_balance_rod_target_angle_rad'])
$motionAligned = $motionAddresses | ForEach-Object { Get-AlignedAddress $_ }
$motionBase = [uint64](($motionAligned | Measure-Object -Minimum).Minimum)
$motionEnd = [uint64](($motionAligned | Measure-Object -Maximum).Maximum)
$motionCount = [int](($motionEnd - $motionBase) / 4) + 1

$pointAddresses = @(
    $addresses['uwTick'],
    $addresses['point_packet.centerpoint_x'],
    $addresses['point_packet_rx_count'],
    $addresses['task_2_stage'])
$pointAligned = $pointAddresses | ForEach-Object { Get-AlignedAddress $_ }
$pointBase = [uint64](($pointAligned | Measure-Object -Minimum).Minimum)
$pointEnd = [uint64](($pointAligned | Measure-Object -Maximum).Maximum)
$pointCount = [int](($pointEnd - $pointBase) / 4) + 1

$replacementValues = @{
    '__CONTROL_BASE__' = Format-HexAddress $controlBase
    '__CONTROL_COUNT__' = [string]$controlCount
    '__TARGET_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_target_position'] $controlBase)
    '__TOLERANCE_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_target_tolerance_pixel'] $controlBase)
    '__KV_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_velocity_kv'] $controlBase)
    '__TASK2_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['task2_segmented_control'])
    '__MOTOR_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['Gimbal_Motor[1].Position'])
    '__PID_ADDRESS__' = Format-HexAddress (Get-AlignedAddress $addresses['PID_DM_Pitch_Position'])
    '__MOTION_BASE__' = Format-HexAddress $motionBase
    '__MOTION_COUNT__' = [string]$motionCount
    '__RAW_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_raw_velocity_pixel_s'] $motionBase)
    '__FILTERED_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_filtered_velocity_pixel_s'] $motionBase)
    '__RANGE_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_target_in_range_count'] $motionBase)
    '__RANGE_SHIFT__' = [string](Get-ByteShift $addresses['ball_balance_target_in_range_count'])
    '__VFB_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_velocity_feedback_angle_rad'] $motionBase)
    '__ROD_INDEX__' = [string](Get-WordIndex $addresses['ball_balance_rod_target_angle_rad'] $motionBase)
    '__POINT_BASE__' = Format-HexAddress $pointBase
    '__POINT_COUNT__' = [string]$pointCount
    '__TICK_INDEX__' = [string](Get-WordIndex $addresses['uwTick'] $pointBase)
    '__POINT_INDEX__' = [string](Get-WordIndex $addresses['point_packet.centerpoint_x'] $pointBase)
    '__POINT_SHIFT__' = [string](Get-ByteShift $addresses['point_packet.centerpoint_x'])
    '__PACKET_INDEX__' = [string](Get-WordIndex $addresses['point_packet_rx_count'] $pointBase)
    '__STAGE_INDEX__' = [string](Get-WordIndex $addresses['task_2_stage'] $pointBase)
    '__STAGE_SHIFT__' = [string](Get-ByteShift $addresses['task_2_stage'])
    '__SAMPLE_COUNT__' = [string]$SampleCount
    '__INTERVAL_MS__' = [string]$IntervalMs
    '__TRIGGER_TIMEOUT_MS__' = [string]$TriggerTimeoutMs
}

$tclTemplate = @'
# Auto-generated from the latest AXF by task2_sampler.ps1.
proc task2_sample {sample_limit interval_ms trigger_timeout_ms} {
    init
    echo "TASK2_SAMPLE_WAITING"
    set waited_ms 0

    # TASK_2_STAGE_TO_POSITIVE_5CM=1 marks the start of a new Task2 run.
    # Waiting for this exact stage avoids recording the previous run's final stage.
    while {$waited_ms < $trigger_timeout_ms} {
        mem2array trigger_words 32 __POINT_BASE__ __POINT_COUNT__
        set trigger_stage [expr {($trigger_words(__STAGE_INDEX__) >> __STAGE_SHIFT__) & 0xff}]
        if {$trigger_stage == 1} {
            break
        }

        sleep 10
        incr waited_ms 10
    }

    if {$waited_ms >= $trigger_timeout_ms} {
        echo "TASK2_SAMPLE_TRIGGER_TIMEOUT"
        shutdown
        return
    }

    echo "TASK2_SAMPLE_TRIGGERED"
    echo "TASK2_SAMPLE_BEGIN"
    set sample_index 0

    while {$sample_index < $sample_limit} {
        mem2array control_words 32 __CONTROL_BASE__ __CONTROL_COUNT__
        mem2array task2_word 32 __TASK2_ADDRESS__ 1
        mem2array rod_motor_word 32 __MOTOR_ADDRESS__ 1
        mem2array pid_words 32 __PID_ADDRESS__ 10
        mem2array motion_words 32 __MOTION_BASE__ __MOTION_COUNT__
        mem2array point_words 32 __POINT_BASE__ __POINT_COUNT__

        set point_x [expr {($point_words(__POINT_INDEX__) >> __POINT_SHIFT__) & 0xffff}]
        set task_stage [expr {($point_words(__STAGE_INDEX__) >> __STAGE_SHIFT__) & 0xff}]
        set task2_enabled [expr {$task2_word(0) & 0xff}]
        set active_segment [expr {($task2_word(0) >> 8) & 0xff}]
        set in_range_count [expr {($motion_words(__RANGE_INDEX__) >> __RANGE_SHIFT__) & 0xff}]

        echo [format \
            "S,%d,%08x,%08x,%d,%08x,%08x,%d,%d,%d,%d,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x" \
            $sample_index \
            $point_words(__TICK_INDEX__) \
            $point_words(__PACKET_INDEX__) \
            $point_x \
            $control_words(__TARGET_INDEX__) \
            $control_words(__TOLERANCE_INDEX__) \
            $task_stage \
            $task2_enabled \
            $active_segment \
            $in_range_count \
            $pid_words(0) \
            $pid_words(1) \
            $pid_words(3) \
            $pid_words(5) \
            $pid_words(8) \
            $motion_words(__RAW_INDEX__) \
            $motion_words(__FILTERED_INDEX__) \
            $motion_words(__VFB_INDEX__) \
            $control_words(__KV_INDEX__) \
            $motion_words(__ROD_INDEX__) \
            $rod_motor_word(0)]

        incr sample_index
        sleep $interval_ms
    }

    echo "TASK2_SAMPLE_END"
    shutdown
}

task2_sample __SAMPLE_COUNT__ __INTERVAL_MS__ __TRIGGER_TIMEOUT_MS__
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
    '-c "adapter speed 2000" ' +
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
            (Get-Content -LiteralPath $errorLogPath -Tail 20) -join "`n"
    }
    throw "Task2 sampler failed to start:`n$startupError"
}

[System.IO.File]::WriteAllText(
    $pidPath,
    [string]$samplerProcess.Id,
    $utf8WithoutBom)

Write-Host "Task2 sampler started. PID=$($samplerProcess.Id)."
Write-Host "Waiting for a new Task2 run (stage=1)."
Write-Host "Samples=$SampleCount, interval=${IntervalMs}ms, trigger timeout=${TriggerTimeoutMs}ms."
Write-Host "Latest data log: $errorLogPath"
