#!/bin/bash

kill -9 $(ps aux | egrep teste.py | head -n 1 | awk '{print $2}')