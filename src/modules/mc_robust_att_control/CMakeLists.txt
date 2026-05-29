px4_add_module(
    MODULE modules__mc_robust_att_control
    MAIN mc_robust_att_control
    SRCS
        mc_robust_att_control_main.cpp
        mc_robust_att_control.hpp
    DEPENDS
        mathlib
        px4_work_queue
)
