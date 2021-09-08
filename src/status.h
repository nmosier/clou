#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <initializer_list>
#include <type_traits>

namespace status {

class bar;

class element {
public:
    void update(std::ostream& os) = 0;
    void done() {}
    
protected:
    bar& parent;
    
    element(bar& parent): parent(parent) {}
    
    friend class bar;
}

class bar {
public:
    bar(): os(nullptr) {}
    
    template <typename Arg, typename... Args>
    bar(const Arg& arg, Args&&... args): os(&std::cerr) {
        add(arg, std::forward<Args>(args)...);
    }
    
    template <typename... Args>
    bar(std::ostream& os, Args&&... args): os(&os) {
        add(std::forward<Args>(args)...);
    }
    
    void add() {}
    
    template <typename Arg, typename... Args>
    void add(const Arg& arg, Args&&... args) {
        static_assert(std::is_base_of<element, Arg>());
        elements.push_back(std::make_unique(arg));
        add(std::forward<Args>(args)...);
    }
    
    void update() {
        os << "\r";
        for (const auto& element : elements) {
            element->update(*os);
        }
    }
    
private:
    std::ostream *os;
    std::vector<std::unique_ptr<element>> elements;
};


class trigger_element: public element {
public:
    
protected:
};

}
