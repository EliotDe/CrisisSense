#syntax=docker/dockerfile:1
#escape=\
#check=error=true

# COMMENT
FROM ubuntu:24.04
LABEL description="CrisisSense embedded firmware CI environment"

RUN echo 'APT::Install-Suggests "0";' >> /etc/apt/apt.conf.d/00-docker
RUN echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/00-docker
RUN DEBIAN_FRONTEND=noninteractive \
  apt-get update \
  && apt-get install -y --no-install-recommends \   
  gcc-arm-none-eabi \
  libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib \
  openocd \
  cppcheck \
  git \
  make \
  && rm -rf /var/lib/apt/lists/*


WORKDIR /workspace
