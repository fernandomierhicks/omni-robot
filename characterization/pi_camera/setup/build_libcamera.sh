#!/usr/bin/env bash
# Raspberry Pi's libcamera fork + rpicam-apps on Ubuntu 24.04 (Pi 4 / Pi 5).
# Ubuntu's own libcamera 0.2 has the rpi pipeline but not its IPA, so a Pi
# camera is seen and then refused ("Failed to load a suitable IPA library").
# Installs to /usr/local; Ubuntu's libcamera packages stay but are shadowed.
# Usage (on the Pi): bash build_libcamera.sh   (~30-45 min on a Pi 4)
set -euo pipefail

SRC=${SRC:-$HOME/src}
LIBCAMERA_REF=${LIBCAMERA_REF:-main}
RPICAM_REF=${RPICAM_REF:-main}
# vc4 = Pi 4 and earlier; pisp = Pi 5 (pulls libpisp as a meson subproject)
PIPELINES=${PIPELINES:-rpi/vc4,rpi/pisp}

sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q \
    git meson ninja-build pkg-config cmake g++ \
    python3-yaml python3-ply python3-jinja2 python3-pybind11 pybind11-dev \
    libyaml-dev libssl-dev libgnutls28-dev openssl libudev-dev libdw-dev libunwind-dev \
    nlohmann-json3-dev libevent-dev libcap-dev \
    libboost-program-options-dev libdrm-dev libexif-dev libjpeg-dev libpng-dev libtiff-dev
# Ubuntu's `cam` would load its IPA-less libcamera: remove the tools only
sudo apt-get remove -y -q libcamera-tools || true

mkdir -p "$SRC"
cd "$SRC"
[ -d libcamera ] || git clone https://github.com/raspberrypi/libcamera.git
git -C libcamera fetch -q origin && git -C libcamera checkout -q "$LIBCAMERA_REF" && git -C libcamera pull -q --ff-only || true
cd libcamera
meson setup build --wipe --buildtype=release --prefix=/usr/local \
    -Dpipelines="$PIPELINES" -Dipas="$PIPELINES" \
    -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
    -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build -j"$(nproc)"
sudo ninja -C build install
sudo ldconfig
# its Python bindings land in /usr/local/lib/python3/dist-packages, which Ubuntu's python3 doesn't search
PYV=$(python3 -c 'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')
echo /usr/local/lib/python3/dist-packages | sudo tee /usr/local/lib/$PYV/dist-packages/libcamera-local.pth >/dev/null
python3 -c "import libcamera; print('libcamera python:', libcamera.__file__)"

cd "$SRC"
[ -d rpicam-apps ] || git clone https://github.com/raspberrypi/rpicam-apps.git
git -C rpicam-apps fetch -q origin && git -C rpicam-apps checkout -q "$RPICAM_REF" && git -C rpicam-apps pull -q --ff-only || true
cd rpicam-apps
PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig \
meson setup build --wipe --buildtype=release --prefix=/usr/local \
    -Denable_libav=disabled -Denable_drm=disabled -Denable_egl=disabled \
    -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled \
    -Denable_hailo=disabled -Denable_imx500=false
ninja -C build -j"$(nproc)"
sudo ninja -C build install
sudo ldconfig

echo "built: $(PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig pkg-config --modversion libcamera)"
rpicam-hello --list-cameras
