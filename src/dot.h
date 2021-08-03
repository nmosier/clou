#pragma once

#include <string>

namespace dot {


   template <typename OS>
   OS& quote(OS& os, const std::string& s) {
      for (char c : s) {
         switch (c) {
         case '\n':
            os << "\\l";
            break;
         case '"':
            os << "\\\"";
            break;
         default:
            os << c;
            break;
         }
      }
      return os;
   }

   template <typename OS>
   OS& emit_kv(OS& os, const std::string& key, const std::string& value) {
      os << key << "=\"";
      quote(os, value);
      os << "\"";
      return os;
   }

   template <typename OS>
   OS& emit_kvs(OS& os, const std::string& key, const std::string& value) {
      os << "[";
      emit_kv(os, key, value);
      os << "]";
      return os;
   }
      
}
