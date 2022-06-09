#!/bin/bash

getpids() {
    pgrep clang-12 | tr "\n" ","
}

htop --pid=$(getpids)
