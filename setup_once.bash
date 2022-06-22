SCRIPTDIR="$( cd "$( dirname "$0" 2>/dev/null || pwd)" && pwd )"
echo $SCRIPTDIR

sudo apt-get update && \
    sudo apt-get -y install \
            git \
            libceres-dev \
            libglew-dev \
            libopencv-dev \
            libsm-dev \
            libsuitesparse-dev \
            libxrender-dev \
            python3-dev \
            python3-pip \
            rsync \
            stow \
            unzip \
            virtualenv

{
    export INSTALL_PREFIX=${SCRIPTDIR}/dependencies
    mkdir -p $INSTALL_PREFIX
    export SUDO=sudo
    [ -d $INSTALL_PREFIX/share/Ceres/ ] || \
        make -f ${SCRIPTDIR}/ros_wrapper/install-deps/install-ceres.mk
    [ -d $INSTALL_PREFIX/share/Pangolin/ ] || \
        make -f ${SCRIPTDIR}/ros_wrapper/install-deps/install-pangolin.mk
    [ -f $INSTALL_PREFIX/include/sophus/se3.hpp ] || \
        make -f ${SCRIPTDIR}/ros_wrapper/install-deps/install-sophus.mk
}
export CMAKE_PREFIX_PATH=$INSTALL_PREFIX

PYVER=3.8
virtualenv --python=python${PYVER} $SCRIPTDIR/.tox/py${PYVER/./}
source $SCRIPTDIR/.tox/py${PYVER/./}/bin/activate
pip install --no-cache -r $SCRIPTDIR/ros_wrapper/install-deps/pip-requirements.txt
sed -e "s#%ROOT_DIR%#$INSTALL_PREFIX#g" "$SCRIPTDIR/ros_wrapper/install-deps/activate.sh.template" > "$INSTALL_PREFIX/activate.sh"

source $INSTALL_PREFIX/activate.sh

