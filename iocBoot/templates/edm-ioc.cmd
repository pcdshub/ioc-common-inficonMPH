#! /bin/bash

# Setup the common directory env variables
if [ -e      /reg/g/pcds/pyps/config/common_dirs.sh ]; then
        source   /reg/g/pcds/pyps/config/common_dirs.sh
elif [ -e    /afs/slac/g/pcds/pyps/config/common_dirs.sh ]; then
        source   /afs/slac/g/pcds/pyps/config/common_dirs.sh
fi

# Setup pydm environment
source /reg/g/pcds/pyps/conda/py36env.sh

pushd $$IOCTOP/inficonMPHScreens

$$LOOP(INFICON)
pydm -m "DEV=$$BASE,IOC=$$IOC_PV" inficon.ui &
$$ENDLOOP(INFICON)
