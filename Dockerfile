FROM ubuntu:20.04

CMD bash

ARG build_type="Debug"

RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-suggests --no-install-recommends \
    autoconf automake autotools-dev libtool \
    clang-12 cmake git libgoogle-perftools-dev libprotobuf-dev llvm-12-dev make pkg-config protobuf-compiler wget libboost-dev libgdbm-dev \
    apt-utils apt-transport-https ca-certificates gnupg dialog \
    lldb-12 \
    tmux \
    less emacs-nox htop valgrind psmisc \
    unzip python3-setuptools

ENV LLVM_DIR="/usr/lib/llvm-12"
ENV CC="/usr/bin/clang-12"
ENV CXX="/usr/bin/clang++-12"
ENV LCM_DIR="$HOME/lcm"
ENV LCM_BUILD="$LCM_DIR/build"
ENV LCM_SCRIPTS="$LCM_DIR/scripts"
ENV SRC="$LCM_DIR"
RUN echo "export CMAKE_BUILD_PARALLEL_LEVEL=$(nproc)" >> ~/.bashrc

WORKDIR "$LCM_SCRIPTS"
COPY scripts/cloucc.sh .

# Set up custom z3
ENV Z3_NAME=z3
ENV Z3_DIR="$LCM_BUILD/$Z3_NAME/install"
WORKDIR "$LCM_BUILD"
RUN git clone --depth=1 https://github.com/nmosier/$Z3_NAME.git
WORKDIR "$LCM_BUILD/$Z3_NAME/build"
RUN mkdir -p ../install
RUN cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$Z3_DIR" ..
RUN make -j$(nproc)
RUN make install

# Set up dummy libclou.so
WORKDIR "$LCM_BUILD"
RUN mkdir -p src
RUN echo 'static int i;' > dummy.c
RUN clang-12 -shared -o src/libclou.so dummy.c
RUN rm -f dummy.c

# Configure libsodium
RUN git clone --depth=1 https://github.com/jedisct1/libsodium.git libsodium
ENV LIBSODIUM_CPPFLAGS="-UHAVE_INLINE_ASM -UHAVE_EMMINTRIN_H -UHAVE_C_VARARRAYS -UHAVE_ALLOCA"
WORKDIR "$LCM_BUILD/libsodium"
RUN autoreconf -i
RUN ./configure --disable-asm CC="$LCM_SCRIPTS/cloucc.sh" CPPFLAGS="$LIBSODIUM_CPPFLAGS -g -Wno-cpp -Xclang -load -Xclang $LCM_BUILD/src/libclou.so" CFLAGS="-O0" || ( cat config.log; exit 1)
RUN mkdir lcm

# Configure openssl
WORKDIR "$LCM_BUILD"
RUN git clone --depth=1 https://github.com/openssl/openssl.git openssl
WORKDIR openssl
RUN ./Configure CC="$LCM_SCRIPTS/cloucc.sh" CFLAGS="-g -Xclang -load -Xclang $LCM_BUILD/src/libclou.so"
RUN mkdir lcm


# WORKDIR "$LCM_BUILD/libsodium-v4"
# RUN autoreconf -i
# RUN ./configure --disable-asm CFLAGS="-Wno-cpp -Xclang -load -Xclang $LCM_BUILD/src/libclou.so" CPPFLAGS="$LIBSODIUM_CPPFLAGS"
# RUN mkdir lcm

# WORKDIR "$LCM_BUILD/libsodium-ll"
# RUN autoreconf -i
# RUN ./configure --disable-asm CC="$LCM_DIR/scripts/mycc.sh" CFLAGS="-Wno-cpp"
# RUN make -j$(nproc)

RUN ulimit -c unlimited
RUN mkdir -p /tmp/cores

# Build lcm tool
WORKDIR "$LCM_BUILD"
# Uncomment to use newer Z3 version
# ENV Z3_DIR "${LCM_BUILD}/z3-${z3_version}-x64-glibc-2.31"
COPY CMakeLists.txt $LCM_DIR/

ENV CXXFLAGS -fPIC
# RUN cmake -DCMAKE_BUILD_TYPE="${build_type}" -DLLVM_DIR="$LLVM_DIR" -DCMAKE_CXX_FLAGS="-fPIC" ..
# RUN make -j$(nproc)

ENV LD_LIBRARY_PATH="${Z3_DIR}/bin:$LD_LIBRARY_PATH"

ENV CLOU_CFLAGS="-Xclang -load -Xclang ${LCM_BUILD}/src/libclou.so"
