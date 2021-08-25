#!/bin/bash

>&2 echo "Available bags:"
ls $1 | >&2 sed -e 's/\.bag//g'
>&2 echo "---------------------"