# perception_pkg
放 image_preprocess、yolo_detector、object_localization、target_tracker。
原则：只负责“看见并理解目标”，不做任务编排。
      （建议拆成：preprocess，detection，localization，tracking)
        后续扩展方式：
        - YOLO 可替换为其他 detector
        - 定位算法可从“框+深度”升级到“关键点/PnP/分割抓取”