#!/bin/bash
RXROS=`dirname $0`
(cd "$RXROS/src/rxros/src"; sudo cp -f rxros.h /usr/local/include)
