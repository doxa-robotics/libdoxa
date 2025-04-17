FROM ubuntu:22.04

COPY ./.devcontainer/setup.sh /tmp/scripts/setup.sh
RUN apt-get update && bash /tmp/scripts/setup.sh
ENV PATH="/root/.cargo/bin:$PATH"