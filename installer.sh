#!/bin/bash

#
# © 2022,2023 Harald Barth
# 
# This file is part of CommandStation-EX
#
# This is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# It is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
#
#
# Usage: mkdir DIRNAME ; cd DIRNAME ; ../installer.sh
# or from install directory ./installer.sh
#

DCCEXGITURL="https://github.com/DCC-EX/CommandStation-EX"
ACLIINSTALL="https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh"
ACLI="./bin/arduino-cli"

function need () {
    type -p $1 > /dev/null && return
    dpkg -l $1 2>&1 | egrep ^ii >/dev/null && return
    sudo apt-get install $1
    type -p $1 > /dev/null && return
    echo "Could not install $1, abort"
    exit 255
}

need git

if cat /etc/issue | egrep '^Raspbian' 2>&1 >/dev/null ; then
    # we are on a raspi where we do not support graphical
    unset DISPLAY
fi

if [ x$DISPLAY != x ] ; then
    # we have DISPLAY, do the graphic thing
    need python3-tk
    need python3.8-venv
    mkdir -p ~/ex-installer/venv
    python3 -m venv ~/ex-installer/venv
    cd ~/ex-installer/venv || exit 255
    source ./bin/activate
    git clone https://github.com/DCC-EX/EX-Installer
    cd EX-Installer || exit 255
    pip3 install -r requirements.txt
    exec python3 -m ex_installer
fi
if test -d `basename "$DCCEXGITURL"` ; then
    : assume we are almost there
    cd `basename "$DCCEXGITURL"` || exit 255
fi
if test -d .git ; then
    : assume we are right here
    git pull
else
    git clone "$DCCEXGITURL"
    cd `basename "$DCCEXGITURL"` || exit 255
fi

# prepare versions
VERSIONS=/tmp/versions.$$
git tag --sort=v:refname | grep Prod  | tail -1  >  $VERSIONS
echo master                                      >> $VERSIONS
git tag --sort=v:refname | grep Devel | tail -1  >> $VERSIONS
echo devel                                       >> $VERSIONS

# ask user what version to use
echo "What version to use? (give line number) If in doubt, use 1"
cat -n $VERSIONS
echo -n "> "
LINE=`awk 'BEGIN {getline A < "/dev/tty"} ; A == NR {print}' $VERSIONS`
git checkout $LINE

if test -f config.h ; then
    : all well
else
    # need to do this config better
    cp -p config.example.h config.h
fi
if test -x "$ACLI" ; then
    : all well
else
    need curl
    curl "$ACLIINSTALL" > acliinstall.sh
    chmod +x acliinstall.sh
    ./acliinstall.sh
fi

$ACLI core update-index || exit 255

# Board discovery
BOARDS=/tmp/boards.$$
$ACLI board list > /dev/null               # download missing components
$ACLI board list | grep serial > $BOARDS   # real run
if test -s $BOARDS ; then
    : all well
else
    echo "$ACLI: No boards found"
    exit 255
fi
if test x`< $BOARDS wc -l` = 'x1' ; then
    LINE=`cat $BOARDS`
else
    # ask user
    echo "What board to use? (give line number)"
    cat -n $BOARDS
    echo -n "> "
    LINE=`awk 'BEGIN {getline A < "/dev/tty"} ; A == NR {print}' $BOARDS`
fi
rm $BOARDS
PORT=`echo $LINE | cut -d" " -f1`
echo Will use port: $PORT

# FQBN discovery
FQBN=`echo $LINE | egrep 'arduino:avr:[a-z][a-z]*' | sed 's/.*\(arduino:avr:[a-z][a-z]*\) .*/\1/1'`
if test x$FQBN = x ; then
    # ask user
    cat > /tmp/fqbn.$$ <<EOF
arduino:avr:uno
arduino:avr:mega
esp32:esp32:esp32
EOF
    echo "What board type? (give line number)"
    cat -n /tmp/fqbn.$$
    echo -n "> "
    FQBN=`awk 'BEGIN {getline A < "/dev/tty"} ; A == NR {print}' /tmp/fqbn.$$`
fi
rm /tmp/fqbn.$$
echo FQBN is $FQBN

# Install phase
$ACLI core install `echo $FQBN | sed 's,:[^:]*$,,1'` # remove last component to get package
$ACLI board attach -p $PORT --fqbn $FQBN "$PWD"
$ACLI compile --fqbn $FQBN "$PWD"
$ACLI upload -v -t -p $PORT "$PWD"
