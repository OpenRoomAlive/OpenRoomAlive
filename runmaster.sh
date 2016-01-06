#!/usr/bin/env bash

cd `dirname "${BASH_SOURCE[0]}"`/tools
./runmaster.py

kill -9 $PPID