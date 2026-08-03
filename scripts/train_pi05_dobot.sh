#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${DOBOT_WORKSPACE_DIR:-$(cd "$(dirname "$0")/.." && pwd)}"
CONFIG_FILE="${PI05_TRAIN_CONFIG:-${WORKSPACE_DIR}/config/pi05_pipeline.env}"
if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "ERROR: training config not found: ${CONFIG_FILE}" >&2
  exit 2
fi

# The config file is the intended operator interface for a repeatable experiment.
set -a
source "${CONFIG_FILE}"
set +a

OPENPI_REPO_DIR="${OPENPI_REPO_DIR:-/home/ps/DZK_repos/openpi}"
OPENPI_DOBOT_DATASET_DIR="${OPENPI_DOBOT_DATASET_DIR:?set OPENPI_DOBOT_DATASET_DIR in ${CONFIG_FILE}}"
OPENPI_POLICY_CONFIG="${OPENPI_POLICY_CONFIG:?set OPENPI_POLICY_CONFIG in ${CONFIG_FILE}}"
OPENPI_EXP_NAME="${OPENPI_EXP_NAME:?set OPENPI_EXP_NAME in ${CONFIG_FILE}}"
OPENPI_TRAIN_STEPS="${OPENPI_TRAIN_STEPS:-1000000}"
OPENPI_TRAIN_RESUME="${OPENPI_TRAIN_RESUME:-false}"
OPENPI_TRAIN_OVERWRITE="${OPENPI_TRAIN_OVERWRITE:-false}"

case "${OPENPI_TRAIN_RESUME}:${OPENPI_TRAIN_OVERWRITE}" in
  false:false|false:true|true:false) ;;
  *) echo "ERROR: resume and overwrite cannot both be true" >&2; exit 2 ;;
esac

cd "${OPENPI_REPO_DIR}"
export OPENPI_DOBOT_DATASET_DIR
compose=(docker compose -f compose.yaml -f compose.gpu.yaml -f compose.dobot.yaml)
"${compose[@]}" up -d openpi

train_args=("--exp-name=${OPENPI_EXP_NAME}" "--num_train_steps=${OPENPI_TRAIN_STEPS}")
if [[ "${OPENPI_TRAIN_RESUME}" == true ]]; then
  train_args+=(--resume)
else
  [[ "${OPENPI_TRAIN_OVERWRITE}" == true ]] && train_args+=(--overwrite)
fi

printf 'Training config: %s\nDataset: %s\nOutput: %s\n' \
  "${OPENPI_POLICY_CONFIG}" "${OPENPI_DOBOT_DATASET_DIR}" \
  "${OPENPI_REPO_DIR}/../openpi-docker-data/checkpoints/${OPENPI_POLICY_CONFIG}/${OPENPI_EXP_NAME}"

if [[ "${OPENPI_TRAIN_RESUME}" == true ]]; then
  norm_command="true"
else
  norm_command="opic-norm --config-name ${OPENPI_POLICY_CONFIG}"
fi
train_command="opic-train ${OPENPI_POLICY_CONFIG} ${train_args[*]}"
log_path="/workspace/wandb/${OPENPI_EXP_NAME}.log"
"${compose[@]}" exec -d -T openpi zsh -lc \
  "source /usr/local/share/openpi/functions.zsh && ${norm_command} && ${train_command} > ${log_path} 2>&1"
echo "Training started. Log: ${OPENPI_REPO_DIR}/../openpi-docker-data/wandb/${OPENPI_EXP_NAME}.log"
