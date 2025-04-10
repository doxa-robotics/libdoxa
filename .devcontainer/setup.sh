## update and install some things we should probably have
apt-get update
apt-get install -y \
  curl \
  git \
  gnupg2 \
  jq \
  sudo \
  vim \
  build-essential \
  openssl

## Install rustup and common components
RUST_TOOLCHAIN="nightly-2025-02-18"
curl https://sh.rustup.rs -sSf | sh -s -- -y
rustup install $RUST_TOOLCHAIN
rustup component add rustfmt
rustup component add rustfmt --toolchain $RUST_TOOLCHAIN
rustup component add clippy 
rustup component add clippy --toolchain $RUST_TOOLCHAIN
# vexide-specific things
rustup component add rust-src --toolchain $RUST_TOOLCHAIN
rustup component add llvm-tools --toolchain $RUST_TOOLCHAIN
