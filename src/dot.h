#pragma once

#include <string>
#include <vector>
#include <utility>

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

template <typename OS, typename InputIt>
OS& emit_kvs(OS& os, InputIt begin, InputIt end) {
    os << "[";
    for (InputIt it = begin; it != end; ++it) {
        if (it != begin) {
            os << ",";
        }
        emit_kv(os, it->first, it->second);
    }
    os << "]";
    return os;
}

using kv_vec = std::vector<std::pair<std::string, std::string>>;

template <typename Container, typename OS>
OS& emit_kvs(OS& os, const Container& container) {
    return emit_kvs(os, container.begin(), container.end());
}

}
