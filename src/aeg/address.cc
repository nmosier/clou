#include "aeg/address.h"
#include "aeg/context.h"

namespace aeg {

Address::Address(Context& ctx): addr(ctx.make_int("addr")) {}


}
