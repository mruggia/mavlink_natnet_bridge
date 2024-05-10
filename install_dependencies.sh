#!/bin/sh


# mavlink library
rm -rf mavlink
git clone https://github.com/mavlink/mavlink.git --recursive
cd mavlink
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/common.xml
cmake -Bbuild -H. -DCMAKE_INSTALL_PREFIX=install -DMAVLINK_DIALECT=common -DMAVLINK_VERSION=2.0
cmake --build build --target install
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_common message_definitions/v1.0/common.xml
cd ..
# -> copy file ~/mavlink/mavlink_2_common.lua to wireshark plugin directory (for debugging)

# natnet library
rm -rf natnet
curl -O https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.1_ubuntu.tar
tar -xf NatNet_SDK_4.1_ubuntu.tar --one-top-level
mv NatNet_SDK_4.1_ubuntu natnet
rm NatNet_SDK_4.1_ubuntu.tar