# safety_pkg
放 safety_monitor_node、system_supervisor_node、diagnostics_node。
原则：安全和健康管理单独隔离，优先级高于业务流程。
      (建议拆成：runtime_safety_monitor，functional_safety_gateway，diagnostics_aggregator)
      后续扩展方式：功能安全升级时不影响普通业务逻辑