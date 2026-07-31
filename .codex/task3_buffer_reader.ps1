[CmdletBinding()]
param(
    [ValidateRange(100, 5000)]
    [int]$AdapterKHz = 1000
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$axfPath = Join-Path $projectRoot 'MDK-ARM\26matchF4\26matchF4.axf'
$gdbPath = 'E:\link\gdb\gdb.exe'
$openOcdPath = 'E:\link\openocd\bin\openocd.exe'
$openOcdScripts = 'E:\link\openocd\share\openocd\scripts'
$bufferPath = Join-Path $env:TEMP 'codex_task3_buffer_latest.bin'
$configPath = Join-Path $env:TEMP 'codex_task3_config_latest.bin'
$csvPath = Join-Path $env:TEMP 'codex_task3_buffer_latest.csv'
$metaPath = Join-Path $env:TEMP 'codex_task3_buffer_latest_meta.json'

foreach ($requiredPath in @($axfPath, $gdbPath, $openOcdPath, $openOcdScripts))
{
    if (-not (Test-Path -LiteralPath $requiredPath))
    {
        throw "Required path is missing: $requiredPath"
    }
}

# 片上记录器已经按视觉新包保存数据，读取时只需短暂停核并一次性导出RAM。
$otherOpenOcd = Get-Process -Name 'openocd' -ErrorAction SilentlyContinue |
    Where-Object { $_.Path -eq $openOcdPath }
if ($null -ne $otherOpenOcd)
{
    throw 'Another OpenOCD or sampler process is using DAP-Link.'
}

$sampleFields = @(
    @{ Name = 'tick_ms'; Type = 'u32' },
    @{ Name = 'elapsed_ms'; Type = 'u32' },
    @{ Name = 'packet_count'; Type = 'u32' },
    @{ Name = 'can_error_count'; Type = 'u32' },
    @{ Name = 'point_x'; Type = 'u16' },
    @{ Name = 'segment'; Type = 'u8' },
    @{ Name = 'motion_active'; Type = 'u8' },
    @{ Name = 'effective_target_pixel'; Type = 'float' },
    @{ Name = 'pid_kp'; Type = 'float' },
    @{ Name = 'pid_ki'; Type = 'float' },
    @{ Name = 'pid_kd'; Type = 'float' },
    @{ Name = 'pid_error'; Type = 'float' },
    @{ Name = 'pid_integral_output'; Type = 'float' },
    @{ Name = 'pid_differential'; Type = 'float' },
    @{ Name = 'pid_output'; Type = 'float' },
    @{ Name = 'raw_velocity_pixel_s'; Type = 'float' },
    @{ Name = 'filtered_velocity_pixel_s'; Type = 'float' },
    @{ Name = 'velocity_kv'; Type = 'float' },
    @{ Name = 'velocity_feedback_angle_rad'; Type = 'float' },
    @{ Name = 'rod_target_angle_rad'; Type = 'float' },
    @{ Name = 'raw_acceleration_rad_s2'; Type = 'float' },
    @{ Name = 'filtered_acceleration_rad_s2'; Type = 'float' },
    @{ Name = 'feedforward_angle_rad'; Type = 'float' },
    @{ Name = 'motor_target_angle_rad'; Type = 'float' },
    @{ Name = 'motor_position_rad'; Type = 'float' },
    @{ Name = 'motor_velocity_rad_s'; Type = 'float' },
    @{ Name = 'motor_torque_nm'; Type = 'float' },
    @{ Name = 'chassis_motor_1_velocity_rad_s'; Type = 'float' },
    @{ Name = 'chassis_motor_1_acceleration_rad_s2'; Type = 'float' },
    @{ Name = 'chassis_motor_2_velocity_rad_s'; Type = 'float' },
    @{ Name = 'chassis_motor_2_acceleration_rad_s2'; Type = 'float' },
    @{ Name = 'chassis_track_bias'; Type = 'float' },
    @{ Name = 'chassis_track_output'; Type = 'float' }
)

# 不随采样变化的任务3参数单独导出，避免在每个样本中重复占用RAM。
$configFields = @(
    'near_error_limit_pixel',
    'middle_error_limit_pixel',
    'target_offset_pixel',
    'velocity_filter_time_constant_s',
    'near_velocity_deadband_pixel_s',
    'startup_velocity_kv',
    'near.Kp', 'near.Ki', 'near.Kv',
    'low_pixel_middle.Kp', 'low_pixel_middle.Ki', 'low_pixel_middle.Kv',
    'low_pixel_far.Kp', 'low_pixel_far.Ki', 'low_pixel_far.Kv',
    'high_pixel_middle.Kp', 'high_pixel_middle.Ki', 'high_pixel_middle.Kv',
    'high_pixel_far.Kp', 'high_pixel_far.Ki', 'high_pixel_far.Kv',
    'pitch_motor_kp',
    'pitch_motor_kd',
    'acceleration_filter_alpha',
    'acceleration_release_filter_alpha',
    'brake_release_filter_alpha',
    'acceleration_feedforward_gain',
    'acceleration_feedforward_limit_rad',
    'acceleration_brake_feedforward_limit_rad'
)

$queries = [ordered]@{
    recorder_base = '&task3_debug_recorder'
    recorder_size = 'sizeof(task3_debug_recorder)'
    sample_count = '&task3_debug_recorder.sample_count'
    recording = '&task3_debug_recorder.recording'
    complete = '&task3_debug_recorder.complete'
    overflow = '&task3_debug_recorder.overflow'
    sample_0 = '&task3_debug_recorder.samples[0]'
    sample_1 = '&task3_debug_recorder.samples[1]'
    config_base = '&task3_segmented_control'
    config_size = 'sizeof(task3_segmented_control)'
}
foreach ($field in $sampleFields)
{
    $queries["sample_$($field.Name)"] =
        "&task3_debug_recorder.samples[0].$($field.Name)"
}
for ($index = 0; $index -lt $configFields.Count; $index++)
{
    $queries["config_$index"] =
        "&task3_segmented_control.$($configFields[$index])"
}

$gdbArguments = @('-batch', '-ex', ('file ' + ($axfPath -replace '\\', '/')))
foreach ($query in $queries.GetEnumerator())
{
    $gdbArguments += @('-ex', "p/x $($query.Value)")
}

$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$gdbOutput = & $gdbPath @gdbArguments 2>&1
$gdbExitCode = $LASTEXITCODE
$ErrorActionPreference = $previousErrorActionPreference
if ($gdbExitCode -ne 0)
{
    throw "Failed to resolve AXF layout:`n$($gdbOutput -join "`n")"
}

$matches = [regex]::Matches(($gdbOutput -join "`n"), '\$\d+\s*=\s*(0x[0-9a-fA-F]+)')
if ($matches.Count -ne $queries.Count)
{
    throw "Unexpected GDB result count. Expected $($queries.Count), got $($matches.Count)."
}

$values = @{}
$queryKeys = @($queries.Keys)
for ($index = 0; $index -lt $queryKeys.Count; $index++)
{
    $hexValue = $matches[$index].Groups[1].Value
    $values[$queryKeys[$index]] = [Convert]::ToUInt64($hexValue.Substring(2), 16)
}

$recorderBase = [uint64]$values.recorder_base
$recorderSize = [uint64]$values.recorder_size
$configBase = [uint64]$values.config_base
$configSize = [uint64]$values.config_size

foreach ($generatedPath in @($bufferPath, $configPath, $csvPath, $metaPath))
{
    if (Test-Path -LiteralPath $generatedPath)
    {
        Remove-Item -LiteralPath $generatedPath -Force
    }
}

$bufferOpenOcdPath = $bufferPath -replace '\\', '/'
$configOpenOcdPath = $configPath -replace '\\', '/'
$openOcdCommand =
    ('adapter speed {0}; init; halt; dump_image {1} 0x{2:x8} 0x{3:x}; ' +
     'dump_image {4} 0x{5:x8} 0x{6:x}; resume; shutdown') -f
    $AdapterKHz, $bufferOpenOcdPath, $recorderBase, $recorderSize,
    $configOpenOcdPath, $configBase, $configSize

$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$openOcdOutput = & $openOcdPath -s $openOcdScripts `
    -f 'interface/cmsis-dap.cfg' -f 'target/stm32f4x.cfg' `
    -c $openOcdCommand 2>&1
$openOcdExitCode = $LASTEXITCODE
$ErrorActionPreference = $previousErrorActionPreference
if ($openOcdExitCode -ne 0)
{
    throw "Failed to read Task3 RAM buffer:`n$($openOcdOutput -join "`n")"
}

$bufferBytes = [System.IO.File]::ReadAllBytes($bufferPath)
$configBytes = [System.IO.File]::ReadAllBytes($configPath)

function Read-Value([byte[]]$Bytes, [int]$Offset, [string]$Type)
{
    switch ($Type)
    {
        'u8' { return [uint32]$Bytes[$Offset] }
        'u16' { return [uint32][BitConverter]::ToUInt16($Bytes, $Offset) }
        'u32' { return [uint64][BitConverter]::ToUInt32($Bytes, $Offset) }
        'float' { return [double][BitConverter]::ToSingle($Bytes, $Offset) }
        default { throw "Unsupported field type: $Type" }
    }
}

$sampleBase = [uint64]$values.sample_0
$sampleStride = [int]([uint64]$values.sample_1 - $sampleBase)
$countOffset = [int]([uint64]$values.sample_count - $recorderBase)
$recordingOffset = [int]([uint64]$values.recording - $recorderBase)
$completeOffset = [int]([uint64]$values.complete - $recorderBase)
$overflowOffset = [int]([uint64]$values.overflow - $recorderBase)
$samplesOffset = [int]($sampleBase - $recorderBase)
$sampleCount = [int](Read-Value $bufferBytes $countOffset 'u16')
$recording = [int](Read-Value $bufferBytes $recordingOffset 'u8')
$complete = [int](Read-Value $bufferBytes $completeOffset 'u8')
$overflow = [int](Read-Value $bufferBytes $overflowOffset 'u8')
$capacity = [int](($bufferBytes.Length - $samplesOffset) / $sampleStride)
if ($sampleCount -gt $capacity)
{
    throw "Invalid sample count $sampleCount, capacity is $capacity."
}

$sampleOffsets = @{}
foreach ($field in $sampleFields)
{
    $sampleOffsets[$field.Name] =
        [int]([uint64]$values["sample_$($field.Name)"] - $sampleBase)
}

$rows = [System.Collections.Generic.List[object]]::new()
for ($sampleIndex = 0; $sampleIndex -lt $sampleCount; $sampleIndex++)
{
    $row = [ordered]@{ sample_index = $sampleIndex }
    $rowBase = $samplesOffset + $sampleIndex * $sampleStride
    foreach ($field in $sampleFields)
    {
        $row[$field.Name] = Read-Value $bufferBytes `
            ($rowBase + $sampleOffsets[$field.Name]) $field.Type
    }
    $rows.Add([pscustomobject]$row)
}
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8

$metadata = [ordered]@{
    sample_count = $sampleCount
    capacity = $capacity
    sample_stride_bytes = $sampleStride
    recording = $recording
    complete = $complete
    overflow = $overflow
}
for ($index = 0; $index -lt $configFields.Count; $index++)
{
    $fieldAddress = [uint64]$values["config_$index"]
    $fieldOffset = [int]($fieldAddress - $configBase)
    $metadata[$configFields[$index]] =
        Read-Value $configBytes $fieldOffset 'float'
}
$metadata | ConvertTo-Json -Depth 4 |
    Set-Content -LiteralPath $metaPath -Encoding UTF8

Write-Host "Task3 RAM capture exported: samples=$sampleCount, complete=$complete, overflow=$overflow"
Write-Host "CSV: $csvPath"
Write-Host "Config: $metaPath"
if ($recording -ne 0)
{
    Write-Warning 'The capture was read before Task3 motion stopped.'
}
