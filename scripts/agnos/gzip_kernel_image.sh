#!/usr/bin/env bash
# Package the AGNOS kernel gzip compressed instead of uncompressed.
#
# comma's tici_defconfig builds Image-dtb: the uncompressed kernel with every
# device tree concatenated onto it. At AGNOS 19.7 that image is 46897152 bytes,
# which is exactly the size of the boot partition, so appending a fourth device
# tree for the comma three overruns the partition by 372736 bytes. agnos.py
# writes straight to the block device, so the tail of the kernel is dropped and
# the device hangs on the bootloader splash.
#
# Every AGNOS that ever booted a comma three shipped a gzip image instead, and so
# does FrogPilot's. The comma three's boot partition was also only 18515968 bytes
# until comma grew it for the uncompressed images, so a gzip image is the only one
# that also fits a device whose partition table predates AGNOS 13.
#
# This edits comma's checkout in place. Run it after the DTS patch and before
# build_kernel.sh.
set -euo pipefail

KERNEL_DIR=${1:?usage: gzip_kernel_image.sh <agnos-kernel-sdm845 dir> <build_kernel.sh>}
BUILD_SH=${2:?usage: gzip_kernel_image.sh <agnos-kernel-sdm845 dir> <build_kernel.sh>}
DEFCONFIG="$KERNEL_DIR/arch/arm64/configs/tici_defconfig"

test -f "$DEFCONFIG" || { echo "no such file: $DEFCONFIG" >&2; exit 1; }
test -f "$BUILD_SH"  || { echo "no such file: $BUILD_SH" >&2; exit 1; }

# Fail loudly rather than silently no-op if comma has already moved these. The
# whole point of the script is that the build stops producing Image-dtb.
for want in \
  '# CONFIG_IMG_GZ_DTB is not set' \
  'CONFIG_IMG_DTB=y' \
  'CONFIG_BUILD_ARM64_APPENDED_KERNEL_IMAGE_NAME="Image-dtb"' \
  '# CONFIG_BUILD_ARM64_KERNEL_COMPRESSION_GZIP is not set' \
  'CONFIG_BUILD_ARM64_UNCOMPRESSED_KERNEL=y'
do
  grep -qxF "$want" "$DEFCONFIG" || {
    echo "tici_defconfig no longer contains: $want" >&2
    echo "comma changed how the kernel image is packaged; re-derive this script" >&2
    exit 1
  }
done

# BUILD_ARM64_KERNEL_COMPRESSION_GZIP is the one that picks the target in
# arch/arm64/Makefile. IMG_GZ_DTB only feeds the NAME string, but leaving the two
# disagreeing would be a trap for the next person reading the config.
sed -i.bak \
  -e 's/^# CONFIG_IMG_GZ_DTB is not set$/CONFIG_IMG_GZ_DTB=y/' \
  -e 's/^CONFIG_IMG_DTB=y$/# CONFIG_IMG_DTB is not set/' \
  -e 's/^CONFIG_BUILD_ARM64_APPENDED_KERNEL_IMAGE_NAME="Image-dtb"$/CONFIG_BUILD_ARM64_APPENDED_KERNEL_IMAGE_NAME="Image.gz-dtb"/' \
  -e 's/^# CONFIG_BUILD_ARM64_KERNEL_COMPRESSION_GZIP is not set$/CONFIG_BUILD_ARM64_KERNEL_COMPRESSION_GZIP=y/' \
  -e 's/^CONFIG_BUILD_ARM64_UNCOMPRESSED_KERNEL=y$/# CONFIG_BUILD_ARM64_UNCOMPRESSED_KERNEL is not set/' \
  "$DEFCONFIG"
rm -f "$DEFCONFIG.bak"

for want in \
  'CONFIG_IMG_GZ_DTB=y' \
  '# CONFIG_IMG_DTB is not set' \
  'CONFIG_BUILD_ARM64_APPENDED_KERNEL_IMAGE_NAME="Image.gz-dtb"' \
  'CONFIG_BUILD_ARM64_KERNEL_COMPRESSION_GZIP=y' \
  '# CONFIG_BUILD_ARM64_UNCOMPRESSED_KERNEL is not set'
do
  grep -qxF "$want" "$DEFCONFIG" || { echo "defconfig edit did not take: $want" >&2; exit 1; }
done

# build_kernel.sh names Image-dtb twice, to copy it out of the kernel tree and to
# hand it to mkbootimg. Image.gz-dtb does not contain the string Image-dtb, so
# this is idempotent.
sed -i.bak 's/Image-dtb/Image.gz-dtb/g' "$BUILD_SH"
rm -f "$BUILD_SH.bak"

grep -q -- '--kernel Image\.gz-dtb' "$BUILD_SH" || { echo "mkbootimg is not being given Image.gz-dtb" >&2; exit 1; }
grep -q 'boot/Image\.gz-dtb' "$BUILD_SH"        || { echo "build_kernel.sh does not copy out Image.gz-dtb" >&2; exit 1; }
if grep -v 'Image\.gz-dtb' "$BUILD_SH" | grep -q 'Image-dtb'; then
  echo "build_kernel.sh still refers to Image-dtb somewhere" >&2
  exit 1
fi

echo "kernel image will be built and packaged as Image.gz-dtb"
