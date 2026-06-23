#!/usr/bin/env bash
set -euo pipefail

ACTIVATE_RETRIES="${ACTIVATE_RETRIES:-3}"
ACTIVATE_RETRY_DELAY="${ACTIVATE_RETRY_DELAY:-1}"
WAIT_MISSING_SECONDS="${WAIT_MISSING_SECONDS:-5}"
STRICT_MISSING="${STRICT_MISSING:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_FILE="${SCRIPT_DIR}/nodes.txt"

# 与机器人保持同一 ROS_DOMAIN_ID
if [[ -f "${SCRIPT_DIR}/install/share/cangyi_bringup/scripts/ros_env.sh" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/install/share/cangyi_bringup/scripts/ros_env.sh"
elif [[ -f "${SCRIPT_DIR}/src/bringup/cangyi_bringup/scripts/ros_env.sh" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/src/bringup/cangyi_bringup/scripts/ros_env.sh"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
fi

usage() {
  cat <<'EOF'
Usage:
  ./lifecycle_manager.sh <command> [node1 node2 ...]

Commands:
  up         按优先级升序：unconfigured->configure->activate, inactive->activate(失败兜底cleanup->configure->activate)
  down       按优先级降序：active->deactivate->cleanup, inactive->cleanup
  restart    先 down(降序) 再 up(升序)
  configure  按优先级升序执行 configure
  activate   按优先级升序执行 activate
  deactivate 按优先级降序执行 deactivate
  cleanup    按优先级降序执行 cleanup
  shutdown   按优先级降序执行 shutdown
  status     按优先级升序查看状态

nodes.txt:
  <priority> <node_name>
example:
  10 /preprocess_node
  20 /yolos_detector
  30 /fusion_pose_node
EOF
}

log_info() { echo "[INFO] $*"; }
log_warn() { echo "[WARN] $*" >&2; }
log_err()  { echo "[ERROR] $*" >&2; }

load_nodes() {
  # 命令行传参：按顺序给优先级
  if [[ $# -gt 0 ]]; then
    local i=0
    for n in "$@"; do
      i=$((i + 1))
      printf "%d\t%s\n" "$i" "$n"
    done
    return
  fi

  [[ -f "${NODES_FILE}" ]] || { log_err "nodes file not found: ${NODES_FILE}"; exit 1; }

  local line p n
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line%%#*}"                                   # 去注释
    line="$(echo "${line}" | sed 's/\r$//;s/^[ \t]*//;s/[ \t]*$//')"  # 去CRLF/空白
    [[ -z "${line}" ]] && continue

    if [[ "${line}" =~ ^([0-9]+)[[:space:]]+(.+)$ ]]; then
      p="${BASH_REMATCH[1]}"
      n="${BASH_REMATCH[2]}"
    else
      p=100
      n="${line}"
    fi
    printf "%s\t%s\n" "${p}" "${n}"
  done < "${NODES_FILE}"
}

sort_entries() {
  local direction="$1"
  shift
  local -a arr=("$@")
  if [[ "${direction}" == "asc" ]]; then
    printf "%s\n" "${arr[@]}" | sort -n -k1,1
  else
    printf "%s\n" "${arr[@]}" | sort -nr -k1,1
  fi
}

print_recent_ros_log() {
  local node="$1"
  local latest_log_dir
  latest_log_dir="$(ls -dt ~/.ros/log/* 2>/dev/null | head -n1 || true)"
  [[ -z "${latest_log_dir}" ]] && return 0

  log_warn "recent ROS log (${latest_log_dir}) for ${node}:"
  grep -R --line-number --ignore-case "${node#/}" "${latest_log_dir}" 2>/dev/null | tail -n 40 || true
}

diagnose_node() {
  local node="$1"
  log_warn "diagnose ${node}:"
  ros2 lifecycle get "${node}" 2>&1 || true
  ros2 lifecycle list "${node}" 2>&1 || true
  print_recent_ros_log "${node}"
}

get_state() {
  local node="$1"
  local out lc

  out="$(ros2 lifecycle get "${node}" 2>&1 || true)"
  lc="$(tr '[:upper:]' '[:lower:]' <<< "${out}")"

  if grep -qiE "unknown lifecycle node|node not found|not found|service not available|waiting for service" <<< "${lc}"; then
    echo "missing"
    return
  fi

  if [[ "${lc}" =~ current[[:space:]]+state[[:space:]]+is[[:space:]]+\[([a-z_]+)\] ]]; then
    echo "${BASH_REMATCH[1]}"
    return
  fi

  if [[ "${lc}" =~ ^[[:space:]]*([a-z_]+)[[:space:]]+\[[0-9]+\] ]]; then
    echo "${BASH_REMATCH[1]}"
    return
  fi

  if grep -qE "\b(unconfigured|inactive|active|finalized|configuring|activating|deactivating|cleaningup|shuttingdown)\b" <<< "${lc}"; then
    grep -oE "(unconfigured|inactive|active|finalized|configuring|activating|deactivating|cleaningup|shuttingdown)" <<< "${lc}" | head -n1
    return
  fi

  echo "unknown"
}

wait_stable_state() {
  local node="$1"
  local retries="${2:-20}"
  local s i=0
  while (( i < retries )); do
    s="$(get_state "${node}")"
    case "${s}" in
      configuring|activating|deactivating|cleaningup|shuttingdown)
        sleep 0.2
        i=$((i + 1))
        ;;
      *)
        echo "${s}"
        return 0
        ;;
    esac
  done
  echo "${s}"
}

do_set() {
  local node="$1"
  local transition="$2"
  local out lc

  log_info "${node} -> ${transition}"
  out="$(ros2 lifecycle set "${node}" "${transition}" 2>&1 || true)"
  echo "${out}"

  lc="$(tr '[:upper:]' '[:lower:]' <<< "${out}")"
  if grep -q "transitioning successful" <<< "${lc}"; then
    return 0
  fi
  if grep -qE "transitioning failed|unknown transition requested|error|failed" <<< "${lc}"; then
    return 1
  fi
  return 1
}

wait_node_exist() {
  local node="$1"
  local deadline=$((SECONDS + WAIT_MISSING_SECONDS))
  local s
  while (( SECONDS < deadline )); do
    s="$(get_state "${node}")"
    [[ "${s}" != "missing" ]] && return 0
    sleep 0.2
  done
  return 1
}

activate_with_retry() {
  local node="$1"
  local i s
  for ((i=1; i<=ACTIVATE_RETRIES; i++)); do
    if do_set "${node}" activate; then
      s="$(wait_stable_state "${node}" 15)"
      [[ "${s}" == "active" ]] && return 0
    fi
    s="$(get_state "${node}")"
    log_warn "${node} activate attempt ${i}/${ACTIVATE_RETRIES} failed, current=${s}"
    sleep "${ACTIVATE_RETRY_DELAY}"
  done
  diagnose_node "${node}"
  return 1
}

up_node() {
  local node="$1"
  local s
  s="$(wait_stable_state "${node}")"

  if [[ "${s}" == "missing" ]]; then
    if wait_node_exist "${node}"; then
      s="$(wait_stable_state "${node}")"
    else
      log_warn "${node} not found, skip."
      [[ "${STRICT_MISSING}" == "1" ]] && exit 2
      return
    fi
  fi

  case "${s}" in
    unconfigured)
      do_set "${node}" configure || { log_warn "${node} configure failed"; diagnose_node "${node}"; return; }
      activate_with_retry "${node}" || { log_warn "${node} activate failed after retries"; return; }
      ;;
    inactive)
      if ! activate_with_retry "${node}"; then
        log_warn "${node} direct activate failed, fallback cleanup->configure->activate"
        do_set "${node}" cleanup || true
        do_set "${node}" configure || { diagnose_node "${node}"; return; }
        activate_with_retry "${node}" || { log_warn "${node} activate failed after fallback"; return; }
      fi
      ;;
    active)
      log_info "${node} already active, skip."
      ;;
    finalized)
      log_warn "${node} is finalized, cannot up directly."
      ;;
    *)
      log_warn "${node} state=${s}, try cleanup->configure->activate"
      do_set "${node}" cleanup || true
      do_set "${node}" configure || true
      activate_with_retry "${node}" || true
      ;;
  esac
}

down_node() {
  local node="$1"
  local s
  s="$(wait_stable_state "${node}")"

  case "${s}" in
    missing)
      log_warn "${node} not found, skip."
      [[ "${STRICT_MISSING}" == "1" ]] && exit 2
      ;;
    active)
      do_set "${node}" deactivate || true
      do_set "${node}" cleanup || { log_warn "${node} cleanup failed"; diagnose_node "${node}"; }
      ;;
    inactive)
      do_set "${node}" cleanup || { log_warn "${node} cleanup failed"; diagnose_node "${node}"; }
      ;;
    unconfigured)
      log_info "${node} already unconfigured, skip."
      ;;
    finalized)
      log_info "${node} already finalized, skip."
      ;;
    *)
      log_warn "${node} state=${s}, try deactivate->cleanup"
      do_set "${node}" deactivate || true
      do_set "${node}" cleanup || true
      ;;
  esac
}

status_node() {
  local node="$1"
  log_info "${node}: $(get_state "${node}")"
}

main() {
  if [[ $# -lt 1 ]]; then
    usage
    exit 1
  fi

  local cmd="$1"
  shift || true

  mapfile -t entries < <(load_nodes "$@")
  [[ ${#entries[@]} -eq 0 ]] && { log_err "Empty node list."; exit 1; }

  local sorted_up sorted_down
  sorted_up="$(sort_entries asc "${entries[@]}")"
  sorted_down="$(sort_entries desc "${entries[@]}")"

  case "${cmd}" in
    up)
      while IFS=$'\t' read -r _ node; do [[ -n "${node:-}" ]] && up_node "${node}"; done <<< "${sorted_up}"
      ;;
    down)
      while IFS=$'\t' read -r _ node; do [[ -n "${node:-}" ]] && down_node "${node}"; done <<< "${sorted_down}"
      ;;
    restart)
      while IFS=$'\t' read -r _ node; do [[ -n "${node:-}" ]] && down_node "${node}"; done <<< "${sorted_down}"
      while IFS=$'\t' read -r _ node; do [[ -n "${node:-}" ]] && up_node "${node}"; done <<< "${sorted_up}"
      ;;
    configure|activate|status)
      while IFS=$'\t' read -r _ node; do
        [[ -z "${node:-}" ]] && continue
        if [[ "${cmd}" == "status" ]]; then status_node "${node}"; else do_set "${node}" "${cmd}" || true; fi
      done <<< "${sorted_up}"
      ;;
    deactivate|cleanup|shutdown)
      while IFS=$'\t' read -r _ node; do [[ -n "${node:-}" ]] && do_set "${node}" "${cmd}" || true; done <<< "${sorted_down}"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"