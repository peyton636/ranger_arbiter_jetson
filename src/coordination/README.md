# coordination

任务编排层。

| 包 | 说明 |
|----|------|
| `task_fsm_node` | Pick-place FSM：统一编排导航、感知、抓取、放置、重试 |

原则：任务编排（FSM）与工艺规则（Policy）分离；不同客户工艺可只换 Policy。
