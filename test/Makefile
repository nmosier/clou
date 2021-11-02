.SUFFIXES:

CLANG = clang
CFLAGS += -fdeclspec


%.bc: %.c
	$(CLANG) $(CFLAGS) -c -emit-llvm -o $@ $<

%.ll: %.bc
	llvm-dis < $< > $@


LIBSODIUM = ../../libsodium/src/libsodium

poly1305_donna.bc: $(LIBSODIUM)/crypto_onetimeauth/poly1305/donna/poly1305_donna.c
	$(CLANG) $(CFLAGS) -c -emit-llvm -o $@ $< -I $(LIBSODIUM)/include/sodium

