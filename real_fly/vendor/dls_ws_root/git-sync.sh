#!/usr/bin/env bash
# git-sync.sh — 局域网内多台无人机代码同步脚本（纯 SSH，不依赖外网）
#
# 背景
#   多台 Jetson Orin NX 部署机在同一个局域网，无外网。用 git 通过局域网 SSH
#   同步 dls_ws 代码。每台飞机需要: IP、用户名、仓库路径（默认 /home/<user>/dls_ws）。
#
# 两种同步模式
#   A. 直推模式 (默认, 最方便): 在当前机器一条命令把代码推到所有飞机工作区。
#      - 飞机端一次性配置 receive.denyCurrentBranch=updateInstead（setup 自动做）
#      - 适合「开发机/主控机 -> 各飞机」单向分发
#   B. 中央裸仓库模式 (hub): 任选一台机器建 ~/dls_ws.git 中央仓库，所有飞机把它当
#      origin 拉取/推送，支持双向与飞机间互相同步。
#
# 使用
#   1) 按 scripts/drones.conf.example 创建 scripts/drones.conf（建议保留在 .gitignore，
#      避免各机配置被同步覆盖）
#   2) ./git-sync.sh setup      # 一次性: SSH 免密 + 飞机端 updateInstead + 本机 remote
#   3) ./git-sync.sh            # 等价 push，推当前分支到所有飞机
#   4) ./git-sync.sh status     # 查看各机分支/commit/工作区状态
#
# hub 模式（可选）
#   ./git-sync.sh hub init <user@ip>     # 在指定机建中央裸仓库并推送 main
#   ./git-sync.sh hub push [branch]      # 推送本地到中央
#   ./git-sync.sh hub pull [--hard]      # 让所有飞机从中央拉取（默认 --ff-only）
#   ./git-sync.sh hub status             # 查看中央仓库版本
#
# 环境变量
#   HOSTS_CONF  主机清单路径（默认 scripts/drones.conf）
#   SSH_OPTS    附加 ssh 参数（默认 -o ConnectTimeout=5 -o BatchMode=yes）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOSTS_CONF="${HOSTS_CONF:-${SCRIPT_DIR}/scripts/drones.conf}"
SSH_OPTS="${SSH_OPTS:--o ConnectTimeout=5 -o BatchMode=yes}"
HUB_REPO="dls_ws.git"

# ---------- 主机清单 ----------
HOST_NAMES=(); HOST_SSHS=(); HOST_PATHS=()
load_hosts() {
  if [[ ! -f "${HOSTS_CONF}" ]]; then
    echo "[sync] 未找到主机清单 ${HOSTS_CONF}" >&2
    echo "[sync] 请先复制示例并填写: cp scripts/drones.conf.example scripts/drones.conf" >&2
    exit 1
  fi
  local line name ssh path
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line%%#*}"                          # 去掉行内注释
    [[ -z "${line//[[:space:]]/}" ]] && continue # 跳过空行
    read -r name ssh path <<< "${line}"
    [[ -n "${name}" && -n "${ssh}" && -n "${path}" ]] || continue
    HOST_NAMES+=("${name}")
    HOST_SSHS+=("${ssh}")
    HOST_PATHS+=("${path}")
  done < "${HOSTS_CONF}"
  ((${#HOST_NAMES[@]} > 0)) || { echo "[sync] 主机清单为空: ${HOSTS_CONF}" >&2; exit 1; }
}

# ssh_run <idx> <remote-cmd...>  —— 免密 ssh 到第 idx 台飞机执行命令
ssh_run() {
  local idx="$1"; shift
  ssh ${SSH_OPTS} "${HOST_SSHS[$idx]}" "$@"
}

# ---------- 工具 ----------
in_arr() { local e; for e in "${@:2}"; do [[ "$e" == "$1" ]] && return 0; done; return 1; }

# 识别清单中的本机条目（push/setup 时跳过自己，避免把自己当远端再推一遍）
LOCAL_IPS=""
get_local_ips() {
  if [[ -z "${LOCAL_IPS}" ]]; then
    LOCAL_IPS="$(hostname -I 2>/dev/null || true) $(ip -o -4 addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
    # 过滤回环地址（127.x / ::1），避免 127.0.0.1 被误判为本机
    LOCAL_IPS="$(echo ${LOCAL_IPS} | tr ' ' '\n' | grep -vE '^(127\.|::1$)' | tr '\n' ' ')"
  fi
}
is_self() { # <idx>：清单第 idx 台的 IP 若在本机网卡上，返回 0（真）
  local idx="$1"
  local ip="${HOST_SSHS[$idx]##*@}"
  get_local_ips
  [[ -n "${ip}" ]] || return 1
  local x
  for x in ${LOCAL_IPS}; do
    [[ "${x}" == "${ip}" ]] && return 0
  done
  return 1
}

# ---------- status ----------
cmd_status() {
  load_hosts
  local online=0 offline=0
  printf "%-12s %-22s %-10s %-9s %s\n" "NAME" "SSH" "BRANCH" "COMMIT" "STATE"
  for i in "${!HOST_NAMES[@]}"; do
    local name="${HOST_NAMES[$i]}" ssh="${HOST_SSHS[$i]}" path="${HOST_PATHS[$i]}"
    local branch="-" commit="-" state="unreachable"
    if out="$(ssh_run "$i" "test -d '${path}/.git' && echo repo || echo norepo")"; then
      online=$((online + 1))
      if [[ "${out}" == "repo" ]]; then
        branch="$(ssh_run "$i" "git -C '${path}' symbolic-ref --short -q HEAD 2>/dev/null || echo detached")" || branch="?"
        commit="$(ssh_run "$i" "git -C '${path}' rev-parse --short HEAD 2>/dev/null")" || commit="?"
        if dirty="$(ssh_run "$i" "git -C '${path}' status --porcelain 2>/dev/null")"; then
          [[ -n "${dirty}" ]] && state="DIRTY" || state="clean"
        else
          state="git-err"
        fi
      else
        state="no-repo"
      fi
    else
      offline=$((offline + 1))
    fi
    printf "%-12s %-22s %-10s %-9s %s\n" "${name}" "${ssh}" "${branch}" "${commit}" "${state}"
  done
  echo "[status] 在线 ${online} / 离线 ${offline} / 共 ${#HOST_NAMES[@]}"
}

# ---------- setup（一次性配置） ----------
cmd_setup() {
  load_hosts
  # 1) 本机 SSH 密钥
  if [[ ! -f "${HOME}/.ssh/id_ed25519" && ! -f "${HOME}/.ssh/id_rsa" ]]; then
    echo "[setup] 本机无 SSH 密钥，生成 ed25519 ..."
    mkdir -p "${HOME}/.ssh"; chmod 700 "${HOME}/.ssh"
    ssh-keygen -t ed25519 -N "" -f "${HOME}/.ssh/id_ed25519" >/dev/null
  fi

  for i in "${!HOST_NAMES[@]}"; do
    local name="${HOST_NAMES[$i]}" ssh="${HOST_SSHS[$i]}" path="${HOST_PATHS[$i]}"
    if is_self "$i"; then
      echo "[skip] ${name} 是本机，无需 setup（setup 只针对远端飞机）"
      continue
    fi
    echo "==== ${name} (${ssh}) ===="
    # 2) SSH 免密
    if ssh -o ConnectTimeout=5 -o BatchMode=yes "${ssh}" true >/dev/null 2>&1; then
      echo "  [ok] SSH 免密已就绪"
    else
      echo "  SSH 免密未配置，执行 ssh-copy-id（将提示输入 ${ssh} 的密码）..."
      if command -v ssh-copy-id >/dev/null 2>&1; then
        ssh-copy-id -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "${ssh}" \
          || echo "  [warn] ssh-copy-id 失败：${ssh} 可能离线或密码错误，跳过该机（下次上线后重跑 setup 补齐）" >&2
      else
        echo "  本机无 ssh-copy-id，请手动执行: ssh-copy-id ${ssh}" >&2
      fi
    fi
    # 3) 飞机端仓库检查 + 直推支持
    if out="$(ssh -o ConnectTimeout=5 "${ssh}" "test -d '${path}/.git' && echo repo || echo norepo" 2>/dev/null)"; then
      if [[ "${out}" == "repo" ]]; then
        ssh -o ConnectTimeout=5 "${ssh}" "git -C '${path}' config receive.denyCurrentBranch updateInstead"
        echo "  [ok] ${path} receive.denyCurrentBranch=updateInstead"
      else
        echo "  [warn] ${path} 不是 git 仓库。直推模式要求飞机上已有 dls_ws，可用 hub 模式 clone 或手动拷贝。" >&2
      fi
    else
      echo "  [warn] ${ssh} 不可达，跳过仓库配置（下次上线后重跑 setup 补齐）" >&2
    fi
    # 4) 本机 remote
    git remote remove "${name}" >/dev/null 2>&1 || true
    git remote add "${name}" "${ssh}:${path}"
    echo "  [ok] 本机 remote ${name} -> ${ssh}:${path}"
  done
  echo "[setup] 完成。可运行 ./git-sync.sh status 检查，或 ./git-sync.sh push 同步代码。"
}

# ---------- push（直推模式） ----------
cmd_push() {
  load_hosts
  local branch="" only=() skip=()
  while (($# > 0)); do
    case "$1" in
      --only)
        shift
        while (($# > 0)) && [[ "$1" != -* ]]; do only+=("$1"); shift; done
        ;;
      --skip)
        shift
        while (($# > 0)) && [[ "$1" != -* ]]; do skip+=("$1"); shift; done
        ;;
      -h|--help) usage; exit 0 ;;
      -*) echo "[push] 未知选项: $1" >&2; usage >&2; exit 1 ;;
      *)
        [[ -z "${branch}" ]] && branch="$1" || { echo "[push] 分支参数重复: $1" >&2; exit 1; }
        shift
        ;;
    esac
  done
  [[ -n "${branch}" ]] || branch="$(git branch --show-current 2>/dev/null || true)"
  [[ -n "${branch}" ]] || branch="main"

  if [[ -n "$(git status --porcelain)" ]]; then
    echo "[push] 警告: 本机有未提交改动，推送的只是已提交内容；如需包含请先 git add/commit。" >&2
  fi
  echo "[push] 分支=${branch} -> ${#HOST_NAMES[@]} 台飞机"

  local ok=0 offline=0 skipped=0 fail=0
  for i in "${!HOST_NAMES[@]}"; do
    local name="${HOST_NAMES[$i]}" ssh="${HOST_SSHS[$i]}" path="${HOST_PATHS[$i]}"
    if is_self "$i"; then
      echo "[skip] ${name} 是本机（无需推给自己）"
      skipped=$((skipped + 1)); continue
    fi
    if in_arr "${name}" "${skip[@]}"; then
      echo "[skip] ${name} 在 --skip 列表"
      skipped=$((skipped + 1)); continue
    fi
    if ((${#only[@]} > 0)) && ! in_arr "${name}" "${only[@]}"; then
      echo "[skip] ${name} 不在 --only 列表"
      skipped=$((skipped + 1)); continue
    fi
    echo "==== push -> ${name} (${ssh}) ===="
    # 先探测可达性，离线机明确跳过，不误报"失败"
    if ! ssh_run "$i" true >/dev/null 2>&1; then
      echo "  [offline] ${name} 不可达，跳过（下次上线后重跑本命令，或 --only ${name} 单独补）" >&2
      offline=$((offline + 1)); continue
    fi
    # updateInstead 只在飞机当前 checkout 的目标分支被推送时更新工作区，先校验
    local rbranch=""
    rbranch="$(ssh_run "$i" "git -C '${path}' symbolic-ref --short -q HEAD 2>/dev/null || true")" || true
    if [[ "${rbranch}" != "${branch}" ]]; then
      echo "  [skip] 飞机当前分支=${rbranch:-?}，需要 ${branch}（避免工作区不被更新）。" >&2
      echo "        在飞机上执行: git -C ${path} checkout ${branch}" >&2
      skipped=$((skipped + 1)); continue
    fi
    if git push "${name}" "${branch}:refs/heads/${branch}"; then
      echo "  [ok] ${name} 已更新到 $(git rev-parse --short HEAD)"
      ok=$((ok + 1))
    else
      echo "  [fail] ${name} 推送失败：可能飞机工作区有未提交改动（updateInstead 要求干净）或分支有分叉。" >&2
      fail=$((fail + 1))
    fi
  done
  echo "[push] 完成: 成功 ${ok} / 离线 ${offline} / 跳过 ${skipped} / 失败 ${fail} / 共 ${#HOST_NAMES[@]}"
}

# ---------- hub（中央裸仓库模式） ----------
cmd_hub() {
  local action="${1:-}"; shift || true
  case "${action}" in
    init)   hub_init "$@" ;;
    push)   hub_push "$@" ;;
    pull)   hub_pull "$@" ;;
    status) hub_status ;;
    *) echo "用法: ./git-sync.sh hub {init <user@ip>|push [branch]|pull [--hard]|status}" >&2; exit 1 ;;
  esac
}

hub_remote() { # 输出 hub remote URL（若配置）
  git remote get-url hub 2>/dev/null || true
}

hub_init() {
  local target="${1:-}"
  [[ -n "${target}" ]] || { echo "用法: ./git-sync.sh hub init <user@ip>" >&2; exit 1; }
  echo "[hub] 在 ${target} 初始化裸仓库 ~/${HUB_REPO} ..."
  ssh -o ConnectTimeout=5 "${target}" "git init --bare -q '~/${HUB_REPO}' && echo '[hub] bare repo ready: ~/${HUB_REPO}'"
  git remote remove hub >/dev/null 2>&1 || true
  git remote add hub "${target}:${HUB_REPO}"   # scp 语法，相对 home
  echo "[hub] 本机 remote hub -> ${target}:${HUB_REPO}"
  hub_push main
}

hub_push() {
  local branch="${1:-}"
  [[ -n "${branch}" ]] || branch="$(git branch --show-current 2>/dev/null || true)"
  [[ -n "${branch}" ]] || branch="main"
  local hub
  hub="$(hub_remote)"
  [[ -n "${hub}" ]] || { echo "[hub] 未配置 hub remote，先执行 ./git-sync.sh hub init <user@ip>" >&2; exit 1; }
  git push hub "${branch}:refs/heads/${branch}"
  echo "[hub] 已推送 ${branch} -> ${hub}"
}

hub_pull() {
  load_hosts
  local hard=0
  [[ "${1:-}" == "--hard" ]] && hard=1
  local hub
  hub="$(hub_remote)"
  [[ -n "${hub}" ]] || { echo "[hub] 未配置 hub remote，先执行 ./git-sync.sh hub init <user@ip>" >&2; exit 1; }

  for i in "${!HOST_NAMES[@]}"; do
    local name="${HOST_NAMES[$i]}" ssh="${HOST_SSHS[$i]}" path="${HOST_PATHS[$i]}"
    echo "==== pull <- ${name} (${ssh}) ===="
    if ! out="$(ssh_run "$i" "test -d '${path}/.git' && echo repo || echo norepo")"; then
      echo "  [skip] 无法连接" >&2; continue
    fi
    [[ "${out}" == "repo" ]] || { echo "  [skip] ${path} 不是 git 仓库" >&2; continue; }
    # 飞机 origin 指向 hub
    ssh_run "$i" "git -C '${path}' remote set-url origin '${hub}' 2>/dev/null || git -C '${path}' remote add origin '${hub}'"
    if dirty="$(ssh_run "$i" "git -C '${path}' status --porcelain 2>/dev/null")" && [[ -n "${dirty}" ]]; then
      echo "  [skip] ${path} 有未提交改动（如需强制覆盖用 --hard）" >&2
      continue
    fi
    if (( hard )); then
      ssh_run "$i" "git -C '${path}' fetch -q origin && git -C '${path}' reset --hard origin/main"
    else
      ssh_run "$i" "git -C '${path}' fetch -q origin && git -C '${path}' pull --ff-only origin main"
    fi
    echo "  [ok] ${name} 已同步到 $(ssh_run "$i" "git -C '${path}' rev-parse --short HEAD")"
  done
}

hub_status() {
  local hub
  hub="$(hub_remote)"
  [[ -n "${hub}" ]] || { echo "[hub] 未配置 hub remote" >&2; exit 1; }
  local host="${hub%%:*}" path="${hub#*:}"
  echo "[hub] ${hub}"
  ssh -o ConnectTimeout=5 "${host}" "git -C '~/${HUB_REPO}' log -1 --oneline 2>/dev/null || echo '(无法读取 ~/${HUB_REPO})'"
}

# ---------- usage ----------
usage() {
  sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

部分飞机离线时:
  ./git-sync.sh status                      # 先看哪些在线/离线
  ./git-sync.sh                             # 在线机正常同步，离线机自动跳过
  ./git-sync.sh push main --only drone05    # 只同步指定机（离线机恢复后单独补）
  ./git-sync.sh push main --skip drone02    # 跳过指定机

本机识别:
  push/setup 会自动跳过清单里 IP 与本机匹配的条目（例：在 uav1 上跑，drone01 被跳过），
  不会把代码推给自己。status 和 hub pull 不过滤本机（hub pull 时本机也需要从中央拉取）。
EOF
}

main() {
  local cmd="${1:-}"; shift || true
  case "${cmd}" in
    "" | push) cmd_push "$@" ;;
    status)    cmd_status ;;
    setup)     cmd_setup ;;
    hub)       cmd_hub "$@" ;;
    help|-h|--help) usage; exit 0 ;;
    *) echo "未知命令: ${cmd}" >&2; usage >&2; exit 1 ;;
  esac
}

main "$@"
