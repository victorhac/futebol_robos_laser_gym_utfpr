#!/bin/bash

kill -9 $(ps aux | egrep  $1.py | head -n 1 | awk '{print $2}')