.SUFFIXES:

CLANG ?= clang
LLVM_DIS ?= llvm-dis
CFLAGS += -fdeclspec

%.ll: %.c
	$(CLANG) $(CFLAGS) -c -S -emit-llvm -o $@ $<

LIBSODIUM = ../../libsodium/src/libsodium

poly1305_donna.bc: $(LIBSODIUM)/crypto_onetimeauth/poly1305/donna/poly1305_donna.c
	$(CLANG) $(CFLAGS) -c -emit-llvm -o $@ $< -I $(LIBSODIUM)/include/sodium

