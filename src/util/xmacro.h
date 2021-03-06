#pragma once

#ifndef STR
# define STR(x) #x
#endif

#ifndef XSTR
# define XSTR(x) STR(x)
#endif

#define XM_ENUM_DEF_ELT(x) x,
#define XM_ENUM_DEF(name, XM)                   \
   enum name {                                  \
      XM(XM_ENUM_DEF_ELT)                       \
   }

#define XM_ENUM_TOSTR_ELT(x) case x: return XSTR(x);
#define XM_ENUM_TOSTR(XM, var, dfl)                 \
   switch (var) {                                   \
      XM(XM_ENUM_TOSTR_ELT)                         \
   default: return dfl;                             \
   }

#define XM_ENUM_CLASS_DEF(name, XM) \
enum class name {\
    XM(XM_ENUM_DEF_ELT)\
}

#define XM_ENUM_CLASS_TOSTR_ELT(x, enum_name) case enum_name::x: return XSTR(x);
#define XM_ENUM_CLASS_TOSTR(XM, enum_name, var, dfl) \
switch(var) { \
XM(XM_ENUM_CLASS_TOSTR_ELT, enum_name) \
default: return dfl; \
}
