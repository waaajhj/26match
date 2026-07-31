# 通过OpenOCD在CPU全速运行时只读采样任务3控制变量。
proc task3_sample {sample_limit interval_ms} {
    init
    echo "TASK3_SAMPLE_BEGIN"
    set sample_index 0

    while {$sample_index < $sample_limit} {
        # 连续块读取可减少DAP通信次数。
        mem2array control_words 32 0x20000018 34
        mem2array rod_motor_word 32 0x200001f8 1
        mem2array pid_words 32 0x20000410 10
        mem2array motion_words 32 0x200004ec 6
        mem2array point_words 32 0x200008c8 16

        set point_x [expr {($point_words(0) >> 8) & 0xffff}]
        set active_segment [expr {($control_words(5) >> 8) & 0xff}]

        # float以原始位输出，后处理时无损转换。
        echo [format \
            "S,%d,%08x,%08x,%d,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%d" \
            $sample_index \
            $point_words(15) \
            $point_words(3) \
            $point_x \
            $control_words(0) \
            $pid_words(0) \
            $pid_words(1) \
            $pid_words(3) \
            $pid_words(5) \
            $pid_words(8) \
            $motion_words(2) \
            $motion_words(0) \
            $motion_words(5) \
            $control_words(2) \
            $motion_words(3) \
            $rod_motor_word(0) \
            $control_words(33) \
            $active_segment]

        incr sample_index
        sleep $interval_ms
    }

    echo "TASK3_SAMPLE_END"
    shutdown
}
