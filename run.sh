#!/bin/sh

NAME="gba-test"

mkdir target
arm-none-eabi-as crt0.s -o target/crt0.o && \
cargo xbuild --release --target thumbv4-none-agb.json && \
arm-none-eabi-objcopy -O binary "target/thumbv4-none-agb/release/$NAME" "target/$NAME.gba" && \
gbafix "target/$NAME.gba" && \
#visualboyadvance-m "target/$NAME.gba"
mgba-qt -l 4095 "target/$NAME.gba"
