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
curl https://sh.rustup.rs -sSf | sh -s -- -y 
export RUST_TOOLCHAIN = "nightly-2025-02-18"
rustup install $RUST_TOOLCHAIN
rustup component add rustfmt
rustup component add rustfmt --toolchain $RUST_TOOLCHAIN
rustup component add clippy 
rustup component add clippy --toolchain $RUST_TOOLCHAIN

cargo install cargo-expand
cargo install cargo-edit