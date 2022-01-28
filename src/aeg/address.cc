#include "aeg/address.h"
#include "aeg/context.h"
#include "aeg/node.h"

namespace aeg {

Address::Address(Context& ctx): arch(ctx.make_int("arch_addr")), trans(ctx.make_int("trans_addr")) {}
Address::Address(Context& ctx, const z3::expr& arch): arch(arch), trans(ctx.make_int("trans_addr")) {}

std::ostream& operator<<(std::ostream& os, const Address& x) {
    return os << "addr(.arch=" << x.arch << ", .trans=" << x.trans << ")";
}

}
