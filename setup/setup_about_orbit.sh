apt update
apt install git

git clone http://183.107.37.79:13003/kaeri_ert/third_party/orbit.git /isaac-sim/../orbit

export ISAACSIM_PATH="/isaac-sim"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

ln -s ${ISAACSIM_PATH} ${ISAACSIM_PATH}/../orbit/_isaac_sim

echo -e "alias orbit=${ISAACSIM_PATH}/../orbit/orbit.sh" >> ${HOME}/.bashrc
source ${HOME}/.bashrc

apt install cmake build-essential
bash ${ISAACSIM_PATH}/../orbit/orbit.sh --install
bash ${ISAACSIM_PATH}/../orbit/orbit.sh --extra rsl_rl