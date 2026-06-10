# manipulation_pkg
放 grasp_pose_estimator_node、motion_planner_node、arm_controller_node、gripper_controller_node。
原则：只负责抓取候选、轨迹规划、执行控制。
      (建议拆成：preprocess，detection，localization，tracking)
        后续扩展方式：
        - 可增加吸盘、二指夹爪、三指夹爪不同策略
        - 可增加不同工件的抓取插件