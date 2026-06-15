#!/usr/bin/env bash
set -euo pipefail

# 脚本目录（tools）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 工作区根目录（cangyirobot）
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# 部署目录与产物目录
DEPLOY_DIR="${WORKSPACE_DIR}/deploy"
ARTIFACT_DIR="${DEPLOY_DIR}/artifacts"

usage() {
  cat <<'EOF'
Usage:
  tools/ros2_ci_deploy.sh build [--select "pkg1 pkg2"] [--up-to "pkg"] [--clean]
  tools/ros2_ci_deploy.sh pack [--name artifact_name]
  tools/ros2_ci_deploy.sh deploy --host HOST --user USER --dest /path [--name artifact_name]
  tools/ros2_ci_deploy.sh all --host HOST --user USER --dest /path [--up-to "pkg"] [--name artifact_name]

Notes:
  1) Run this script at any path; it auto-detects workspace root.
  2) build: compiles workspace into install/.
  3) pack: creates deploy/artifacts/<name>.tar.gz from install/.
  4) deploy: copies artifact and extracts it on target.
EOF
}

ensure_ros_env() {
  # 如果已设置 ROS_DISTRO（如 humble），自动 source ROS 环境
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  fi
}

build_workspace() {
  local select_pkgs=""
  local up_to_pkgs=""
  local clean_first="false"

  # 解析 build 子命令参数
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --select)
        # 仅构建指定包（空格分隔）
        select_pkgs="${2:-}"
        shift 2
        ;;
      --up-to)
        # 构建到某个包（包含依赖）
        up_to_pkgs="${2:-}"
        shift 2
        ;;
      --clean)
        # 构建前清理 build/install/log
        clean_first="true"
        shift 1
        ;;
      *)
        echo "Unknown option for build: $1"
        usage
        exit 1
        ;;
    esac
  done

  ensure_ros_env
  cd "${WORKSPACE_DIR}"

  if [[ "${clean_first}" == "true" ]]; then
    rm -rf build install log
  fi

  # 组装 colcon 命令
  local cmd=(colcon build --symlink-install)
  if [[ -n "${select_pkgs}" ]]; then
    # shellcheck disable=SC2206
    local select_arr=(${select_pkgs})
    cmd+=(--packages-select "${select_arr[@]}")
  elif [[ -n "${up_to_pkgs}" ]]; then
    # shellcheck disable=SC2206
    local up_to_arr=(${up_to_pkgs})
    cmd+=(--packages-up-to "${up_to_arr[@]}")
  fi

  echo "[build] ${cmd[*]}"
  "${cmd[@]}"
}

pack_install() {
  # 未指定名称时，按时间戳生成
  local name="${1:-cangyirobot_$(date +%Y%m%d_%H%M%S)}"
  local artifact="${ARTIFACT_DIR}/${name}.tar.gz"

  cd "${WORKSPACE_DIR}"
  if [[ ! -d "install" ]]; then
    echo "install/ not found, run build first."
    exit 1
  fi

  mkdir -p "${ARTIFACT_DIR}"
  # 将 install 打包，便于远端直接解压运行
  tar -C "${WORKSPACE_DIR}" -czf "${artifact}" install
  echo "[pack] ${artifact}"
}

deploy_artifact() {
  # 部署流程：
  # 1) 参数解析与校验
  # 2) ssh 创建远端目录
  # 3) scp 上传 tar.gz
  # 4) 远端解压到 --dest
  local host=""
  local user=""
  local dest=""
  local name=""

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --host)
        host="${2:-}"
        shift 2
        ;;
      --user)
        user="${2:-}"
        shift 2
        ;;
      --dest)
        dest="${2:-}"
        shift 2
        ;;
      --name)
        name="${2:-}"
        shift 2
        ;;
      *)
        echo "Unknown option for deploy: $1"
        usage
        exit 1
        ;;
    esac
  done

  if [[ -z "${host}" || -z "${user}" || -z "${dest}" ]]; then
    echo "deploy requires --host --user --dest"
    exit 1
  fi

  # 未指定 --name 时，默认取最新产物
  if [[ -z "${name}" ]]; then
    name="$(ls -1t "${ARTIFACT_DIR}"/*.tar.gz 2>/dev/null | head -n1 || true)"
    if [[ -z "${name}" ]]; then
      echo "No artifact found. Run pack first or pass --name."
      exit 1
    fi
  else
    name="${ARTIFACT_DIR}/${name}.tar.gz"
  fi

  if [[ ! -f "${name}" ]]; then
    echo "Artifact not found: ${name}"
    exit 1
  fi

  local artifact_basename
  artifact_basename="$(basename "${name}")"

  echo "[deploy] uploading ${artifact_basename} to ${user}@${host}:${dest}"
  ssh "${user}@${host}" "mkdir -p '${dest}'"
  scp "${name}" "${user}@${host}:${dest}/${artifact_basename}"
  ssh "${user}@${host}" "tar -xzf '${dest}/${artifact_basename}' -C '${dest}'"

  echo "[deploy] done"
  echo "[deploy] target start command example: source ${dest}/install/setup.bash && ros2 launch cangyi_bringup bringup.launch.py mode:=control"
}

main() {
  local subcmd="${1:-help}"
  shift || true

  case "${subcmd}" in
    build)
      build_workspace "$@"
      ;;
    pack)
      local name=""
      while [[ $# -gt 0 ]]; do
        case "$1" in
          --name)
            name="${2:-}"
            shift 2
            ;;
          *)
            echo "Unknown option for pack: $1"
            usage
            exit 1
            ;;
        esac
      done
      pack_install "${name}"
      ;;
    deploy)
      # 仅执行部署（不触发 build/pack）
      deploy_artifact "$@"
      ;;
    all)
      # 串行执行：build -> pack -> deploy
      local host=""
      local user=""
      local dest=""
      local up_to_pkgs=""
      local artifact_name=""

      while [[ $# -gt 0 ]]; do
        case "$1" in
          --host)
            host="${2:-}"
            shift 2
            ;;
          --user)
            user="${2:-}"
            shift 2
            ;;
          --dest)
            dest="${2:-}"
            shift 2
            ;;
          --up-to)
            up_to_pkgs="${2:-}"
            shift 2
            ;;
          --name)
            artifact_name="${2:-}"
            shift 2
            ;;
          *)
            echo "Unknown option for all: $1"
            usage
            exit 1
            ;;
        esac
      done

      if [[ -n "${up_to_pkgs}" ]]; then
        build_workspace --up-to "${up_to_pkgs}"
      else
        build_workspace
      fi

      if [[ -n "${artifact_name}" ]]; then
        pack_install "${artifact_name}"
        deploy_artifact --host "${host}" --user "${user}" --dest "${dest}" --name "${artifact_name}"
      else
        pack_install
        deploy_artifact --host "${host}" --user "${user}" --dest "${dest}"
      fi
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      echo "Unknown subcommand: ${subcmd}"
      usage
      exit 1
      ;;
  esac
}

main "$@"