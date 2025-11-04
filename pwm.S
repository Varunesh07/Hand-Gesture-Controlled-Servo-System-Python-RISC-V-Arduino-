.section .text
.global set_pwm_duty
set_pwm_duty:
    # a0 = channel (0–4), a1 = angle (0–180) #inialising the channels and angles
    li t0, 0x10400000    # PWM base (Address Map)
    slli t1, a0, 2       # Offset = channel * 4
    add t0, t0, t1       # Address = base + offset
    # Duty = 50,000 + (angle * 200,000) / 180 (0.5ms to 2.5ms @ 100 MHz)
    li t1, 50000         # Base duty (0.5ms)
    li t2, 200000
    mul t2, a1, t2       # angle * 200,000
    li t3, 180
    div t2, t2, t3       # (angle * 200,000) / 180
    add t1, t1, t2       # Duty = 50,000 + result
    sw t1, 4(t0)         # Write duty (PWM_0 at 0x10400004)
    ret