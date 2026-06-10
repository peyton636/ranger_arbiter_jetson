# coordination_pkg
放 task_fsm_node。
原则：统一编排导航、感知、抓取、放置、重试。
      (建议把任务编排和工艺规则分开：task_fsm，task_policy)
      后续扩展方式：不同客户工艺只换策略，不动底层执行框架