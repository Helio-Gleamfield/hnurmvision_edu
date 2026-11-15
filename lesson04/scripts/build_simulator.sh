#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
if [[ -f /opt/ros/humble/setup.bash ]];then source /opt/ros/humble/setup.bash;else source "$HOME/.config/ros2/humble.bash";fi
source simulation/install_interfaces/setup.bash
if [[ -f "$HOME/RM_Week1_Tools/rust_env.sh" ]];then source "$HOME/RM_Week1_Tools/rust_env.sh";fi
deps="$HOME/RM_Week1_Tools/deps/ubuntu22_humble"
if [[ -d "$deps/usr/include" ]];then
 export CPATH="$deps/usr/include${CPATH:+:$CPATH}"
 export BINDGEN_EXTRA_CLANG_ARGS="-I/usr/lib/gcc/x86_64-linux-gnu/11/include -I$deps/usr/include${BINDGEN_EXTRA_CLANG_ARGS:+ $BINDGEN_EXTRA_CLANG_ARGS}"
 export LIBRARY_PATH="$deps/usr/lib/x86_64-linux-gnu:$deps/usr/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
fi
export CARGO_NET_OFFLINE=${CARGO_NET_OFFLINE:-true}
python3 simulation/tools/make_training_ground.py
cd simulation/simulator
cargo +1.95.0 build --release --frozen --no-default-features --features ros2 -j2
mkdir -p bin/lib
cp target/release/daedalus bin/daedalus.new
mv bin/daedalus.new bin/daedalus
for lib in target/release/deps/libbevy_dylib-*.so "$(rustc +1.95.0 --print sysroot)"/lib/rustlib/x86_64-unknown-linux-gnu/lib/libstd-*.so;do
 name=$(basename "$lib")
 cp "$lib" "bin/lib/$name.new"
 mv "bin/lib/$name.new" "bin/lib/$name"
done
