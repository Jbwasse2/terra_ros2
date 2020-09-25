#!/usr/bin/env bash
cat "$1" | grep 'average' | grep -Eo '[+-]?[0-9]+([.][0-9]+)?'
