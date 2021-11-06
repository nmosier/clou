FROM ubuntu:20.04

CMD bash

RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-suggests --no-install-recommends \
    autoconf automake autotools-dev libtool \
    clang-12 cmake git libgoogle-perftools-dev libprotobuf-dev llvm-12-dev make pkg-config protobuf-compiler wget \
    apt-utils apt-transport-https ca-certificates gnupg dialog \
    libz3-dev \
    lldb-12 \
    tmux

ENV LLVM_DIR="/usr/lib/llvm-12"
ENV CC="/usr/bin/clang-12"
ENV CXX="/usr/bin/clang++-12"
ENV LCM_DIR="$HOME/lcm"
ENV LCM_BUILD="$LCM_DIR/build"

COPY . "$LCM_DIR"

# Build lcm tool
WORKDIR "$LCM_DIR/build"
RUN cmake -DCMAKE_BUILD_TYPE=Release -DLLVM_DIR="$LLVM_DIR" -DCMAKE_CXX_FLAGS="-fPIC" ..
RUN make -j$(nproc)

# Configure libsodium
RUN git clone https://github.com/jedisct1/libsodium.git libsodium-v1
RUN cp -r libsodium-v1 libsodium-v4
RUN cp -r libsodium-v1 libsodium-ll

WORKDIR "$LCM_BUILD/libsodium-v1"
RUN autoreconf -i
RUN ./configure CFLAGS="-Wno-cpp -Xclang -load -Xclang $LCM_BUILD/src/liblcm.so"
RUN mkdir lcm

WORKDIR "$LCM_BUILD/libsodium-v4"
RUN autoreconf -i
RUN ./configure CFLAGS="-Wno-cpp -Xclang -load -Xclang $LCM_BUILD/src/liblcm.so"
RUN mkdir lcm

WORKDIR "$LCM_BUILD/libsodium-ll"
RUN autoreconf -i
RUN ./configure CC="$LCM_DIR/scripts/mycc.sh" CFLAGS="-Wno-cpp"
RUN make -j$(nproc)

# Set up debugserver
EXPOSE 11100/tcp

WORKDIR "$LCM_DIR"