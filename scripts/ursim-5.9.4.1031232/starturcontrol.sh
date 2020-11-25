#!/bin/bash
SCRIPT_DIR=$(dirname $(readlink -f $0))
HOME=$SCRIPT_DIR $SCRIPT_DIR/URControl -r &>$SCRIPT_DIR/URControl.log &
