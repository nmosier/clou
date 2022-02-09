#include "aeg/address.h"
#include "aeg/context.h"
#include "aeg/node.h"

namespace aeg {

Address::Address(Context& ctx): addr(ctx.make_int("addr")) {}
Address::Address(Context& ctx, const z3::expr& addr): addr(addr) {}

std::ostream& operator<<(std::ostream& os, const Address& x) {
    os << "addr(" << x.addr << ")";
    return os;
}

}
