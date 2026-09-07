# The AGNOS boot image for the comma three

`.github/workflows/zoompilot-agnos-tici-boot.yaml` builds the one AGNOS partition a comma three
cannot take from comma, publishes it as a release asset, and fills in the `boot` entry of
`openpilot/common/hardware/comma/tici_agnos.json`.

**Nothing in this document has been run on a comma three.** The image has never been built, never
been flashed and never been booted. Read [Recovery](#recovery) before you flash anything.

## Why

comma deleted `arch/arm64/boot/dts/qcom/comma_tici.dts` and its `Makefile` line from
`commaai/agnos-kernel-sdm845` in `b90695f101ba` (2025-08-26). That commit changed exactly two files
and nothing else.

One kernel image serves all three SDM845 boards. `tici_defconfig` sets
`CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES=""`, which makes `arch/arm64/boot/Makefile` append
*every* dtb it can find to `Image-dtb`, and the bootloader picks one by matching `qcom,msm-id` and
`qcom,board-id`. Delete the comma three's dtb and the bootloader has nothing to match, so the
kernel never gets a device tree and the board does not come up. AGNOS 12.8 is the last release
built before the deletion and the last one that boots a comma three.

Only `boot` is affected. `xbl`, `xbl_config`, `abl`, `aop`, `devcfg` and `system` are the same
images on all three boards, so `tici_agnos.json` is comma's official manifest for the pinned AGNOS
version with our `boot` entry swapped in. `AGNOS_VERSION` in `launch_env.sh` stays one value
because `/VERSION` is written by the shared system image.

## What the image actually is

The AGNOS kernel at a pinned commit, with the deleted DTS restored, built and packaged exactly the
way `agnos-builder/build_kernel.sh` does it:

- cross compiled with `DEFCONFIG=tici_defconfig` inside comma's `Dockerfile.builder` container
  (Ubuntu 20.04, python2, the Linaro `aarch64-linux-gnu` toolchain from `tools/`)
- `Image-dtb` is the uncompressed `Image` with `comma_tici.dtb`, `comma_tizi.dtb`,
  `comma_mici.dtb` and `comma_ultimate_provisioning.dtb` concatenated onto it
- wrapped by `tools/mkbootimg` with comma's cmdline, 4096 byte pages, base `0x80000000`
- signed with `vble-qti.key`, which is committed in the clear in agnos-builder, and the padded
  signature appended

The restoration lives in `scripts/agnos/restore-comma_tici-dts.patch`. It is version controlled
rather than fetched at build time so a reviewer can read exactly what goes into the kernel. The
`.dts` in it is byte-identical to the deleted file.

### It is not comma's image, and it cannot be

agnos-builder's public master last bumped its kernel submodule in May 2026. The kernel repo has
commits through 2026-09-04, and openpilot pins AGNOS 19.7, cut 2026-09-01. comma therefore builds
19.7 from something that is not fully reflected in public agnos-builder, and **we cannot reproduce
comma's official 19.7 boot image bit for bit**. Ours is the AGNOS kernel at a pinned commit plus
the tici DTB, paired with comma's official 19.7 userspace.

Kernel to userspace coupling on AGNOS is loose (the system image ships no out-of-tree modules built
against a specific `vermagic`), so the pairing is expected to work. Expected, not demonstrated.

Both pins are workflow inputs. Raising `kernel_commit` is the intended way to track comma; the
patch's `Makefile` hunk carries three lines of context and will need refreshing if a kernel bump
adds or removes a board there.

## Running the build

Actions > *zoompilot agnos tici boot* > Run workflow, from the branch whose manifest you want
updated.

| input | default | what it does |
|---|---|---|
| `kernel_commit` | `c368754c26c7b9659de187addc6cccedc6cfb0a0` | the agnos-kernel-sdm845 commit to build. This is what agnos-builder master pins. |
| `agnos_builder_commit` | `8f7207a4f083ce3215f893a01b74613c507cc4bc` | supplies `build_kernel.sh`, the container, `mkbootimg`, the toolchain and the signing key |
| `runner` | `ubuntu-latest` | x86_64, cross compiles with the bundled Linaro toolchain. `ubuntu-24.04-arm` builds natively, which is what comma's own CI does. |
| `release_tag` | blank | blank means `agnos-tici-boot-<run number>` |
| `open_pr` | true | commit the manifest update on a branch and open a PR against the branch the run was dispatched from |

The job clones the kernel shallow at `kernel_commit`, applies the patch, runs `./build_kernel.sh`
unmodified, checks the tici device tree made it into `boot.img`, packages, publishes, and opens the
PR. It takes the better part of an hour cold and rather less with a ccache hit.

### Reproducing it by hand

Any Linux box with Docker. macOS needs a case-sensitive APFS volume, which is why this is a
workflow and not something you run locally by default.

```bash
git clone https://github.com/commaai/agnos-builder
cd agnos-builder
git checkout 8f7207a4f083ce3215f893a01b74613c507cc4bc
git submodule update --init agnos-kernel-sdm845
git -C agnos-kernel-sdm845 checkout c368754c26c7b9659de187addc6cccedc6cfb0a0
git -C agnos-kernel-sdm845 apply /path/to/zoompilot/scripts/agnos/restore-comma_tici-dts.patch
./build_kernel.sh                                  # produces output/boot.img
```

Then package it the way `agnos-builder/scripts/package_ota.py` would:

```bash
cd /path/to/zoompilot
NAME=$(scripts/agnos/manifest_entry.py /path/to/agnos-builder/output/boot.img --xz-name)
xz -9 -T1 -c /path/to/agnos-builder/output/boot.img > "$NAME"
scripts/agnos/manifest_entry.py /path/to/agnos-builder/output/boot.img \
  --url "https://github.com/zoompilot/zoompilot/releases/download/<tag>/$NAME" \
  --verify-xz "$NAME" -o entry.json
scripts/agnos/update_manifest.py openpilot/common/hardware/comma/tici_agnos.json entry.json
```

`boot.img` is not reproducible byte for byte between runs. `mkbootimg` output is deterministic, but
the kernel records a build timestamp, so two builds of the same commit give different hashes. Every
build therefore needs its own manifest entry; do not hand-edit a URL to point at a different build.

## The manifest entry

`openpilot/common/hardware/comma/agnos.py` is the flasher, and every field means something specific
to it. Getting one wrong is how you brick a device, so:

| field | value for `boot` | what reads it |
|---|---|---|
| `name` | `boot` | `get_partition_path()` builds `/dev/disk/by-partlabel/boot_a` or `_b` from it |
| `url` | the release asset | streamed and lzma-decompressed by `StreamingDecompressor` |
| `hash` | sha256 of the **decompressed stream** | checked after flashing against `downloader.sha256` |
| `hash_raw` | sha256 of the bytes **written to the partition** | checked after flashing, and re-checked on every `verify_partition()` |
| `size` | length in bytes of the raw image | the flash fails if the stream is not exactly this long; `verify_partition()` hashes exactly this many bytes off the partition |
| `sparse` | `false` | picks `noop()` over `unsparsify()`, i.e. write the stream straight through |
| `full_check` | `true` | verify by re-hashing the partition rather than by reading a hash string stored just past the image |
| `has_ab` | `true` | append the `_a` / `_b` slot suffix to the partition path |
| `ondevice_hash` | sha256 of the image zero padded to a 4096 byte sector | nothing in openpilot. It is provenance for comma's tooling; we emit it so the entry has the same shape as comma's. |

`hash` and `hash_raw` are equal here and must be. They differ only for sparse images, where `hash`
covers the sparse container and `hash_raw` covers the unpacked result. openpilot's own manifest test
asserts `hash == hash_raw` for every non-sparse entry.

`full_check: true` matters for what happens after a bad flash. With `full_check` false the updater
writes the expected hash as ASCII into the partition just past the image and trusts it later; with
it true, `verify_partition()` re-reads and re-hashes `size` bytes off the block device every time,
so a partition that was corrupted after the fact is caught rather than trusted. `boot` is 45-ish MB,
which is cheap enough to re-hash.

`size` must be the raw image length, not the partition length. `verify_partition()` reads exactly
`size` bytes, so whatever the previous image left beyond that is ignored.

`manifest_entry.py` refuses an image over 64 MiB. That bound is inferred from the size of comma's
own boot images, **not** read off a comma three's GPT. Confirm it against the real partition before
trusting it:

```bash
# on a device
sudo sgdisk -p /dev/disk/by-id/... | grep -i boot
lsblk -b -o NAME,SIZE /dev/disk/by-partlabel/boot_a
```

While the entry is still `REPLACED_BY_BUILD`, `verify_partition()` returns `False` because `size`
is not an `int`, so a half-filled manifest fails safe rather than flashing a placeholder.

## Verifying a build

The workflow already does the first three. Do the rest before you put it on a device.

1. **The dtb was built.** `out/arch/arm64/boot/dts/qcom/comma_tici.dtb` exists.
2. **The dtb reached the boot image.** `strings -a output/boot.img | grep -x 'comma tici'`. The
   workflow fails if this misses, and warns if `comma tizi` or `comma mici` went missing, which
   would mean the append list changed.
3. **The published `.img.xz` round trips.** `manifest_entry.py --verify-xz` decompresses it through
   the same single `LZMADecompressor` that `agnos.py` uses and checks the length and hash. This
   catches the one packaging footgun that matters: `xz -T0` can emit concatenated streams, and
   `agnos.py` stops at the first stream end, which would silently truncate the flash. The workflow
   compresses single threaded for that reason. Compression level does not affect any manifest
   field, since every hash is of the plain image.
4. **Decompile the dtb and read it.** `dtc -I dtb -O dts out/.../comma_tici.dtb` and confirm
   `model = "comma tici"` and that `qcom,msm-id` / `qcom,board-id` match what the deleted file had.
5. **Compare against a known-good image.** Pull AGNOS 12.8's `boot` image, split the appended dtbs
   out of both and diff the tici dtb. Differences should be confined to what changed in
   `comma_common.dtsi` (one PCIe disable) and `sda845-v2.1.dtsi` between 12.8 and the pinned commit.
   This is the strongest check available without hardware and it has not been done.
6. **On the device**, after flashing and before swapping slots:
   `openpilot/common/hardware/comma/agnos.py --verify openpilot/system/hardware/comma/tici_agnos.json`.

The `Content-Type` of a GitHub release asset is not necessarily `application/x-xz`.
`openpilot/common/hardware/comma/tests/test_agnos_updater.py` asserts that header, but only for
`agnos.json`. If that test is ever extended to `tici_agnos.json` it may fail on our asset even
though the asset is fine.

## Flashing

The safe path is the updater's: it marks the **inactive** slot unbootable, flashes it, verifies it,
and only then calls `abctl --set_active`. The slot you are running from is never touched, so a bad
image costs a failed boot rather than a device.

To try an image without going through the updater, do the same thing by hand and keep the fallback
slot intact:

```bash
# on the device. Derive the inactive slot, never hardcode it: writing the slot you
# are running from destroys the fallback this whole procedure depends on.
live=$(abctl --boot_slot | tr -d '[:space:]')   # a or b
case "$live" in
  a) target=b; idx=1 ;;
  b) target=a; idx=0 ;;
  *) echo "could not read the live slot, stop here"; exit 1 ;;
esac
echo "live slot $live, writing boot_$target"

sudo dd if=boot.img of=/dev/disk/by-partlabel/boot_$target
sudo abctl --set_active $idx
sudo reboot
```

This is what `agnos.py` does: `get_target_slot_number()` picks the inactive slot, and the updater
verifies the write before it ever calls `abctl --set_active`.

agnos-builder's own `load_kernel.sh` dds to **both** `boot_a` and `boot_b`. Do not use it here.
That destroys the fallback slot, which is the only thing standing between a bad kernel and a QDL
session.

## Which partitions come from where

This is the part that is easy to get wrong, and getting it wrong looks like
`Unsupported firmware detected` on a black screen at boot, before openpilot runs at all.

| partition | source | why |
|---|---|---|
| `xbl`, `xbl_config`, `aop`, `devcfg` | comma, current AGNOS | byte identical between 18.4 and 19.7, not device specific |
| `abl` | **comma, AGNOS 12.8** | the last bootloader comma shipped for the comma three. A newer `abl` rejects the board |
| `boot` | **ours** | comma's kernel has no `comma_tici.dtb` from AGNOS 13 on |
| `system` | comma, current AGNOS | the rootfs, shared across devices |

Both field validated comma three forks, sunnypilot's `sync-20251218-tici` and opgm's `master-c3`,
independently pin 12.8's `abl` alongside a much newer everything else. That combination looked like
an unexplained hybrid at first and it is not: `abl` is the one bootloader partition that is device
specific, and it is the piece that decides whether the board is allowed to boot at all.

If you ever bump the AGNOS version here, carry `abl` forward unchanged.

## Recovery

Flashing a bad boot partition does not brick the device permanently, but it can leave it unable to
boot at all, and the usual escape hatch is worse for a comma three than for other devices.

- **Slot fallback.** If only one slot is bad the bootloader falls back to the other after its retry
  count runs out. This is the reason for flashing the inactive slot only.
- **QDL always survives, but not for the reason you might assume.** This workflow only ever
  produces `boot`, however `tici_agnos.json` is a whole manifest: taking an AGNOS update through
  the updater also writes comma's `xbl`, `xbl_config`, `abl`, `aop` and `devcfg` to the target
  slot, exactly as it would on a 3X. A comma three therefore does run a bootloader newer than any
  comma shipped for it, which is the same arrangement the third party 18.4 images already in the
  field use. What makes QDL safe is that EDL lives in the SoC boot ROM, not in a partition, so it
  is reachable however badly `boot` or even `xbl` is written. Recovery is
  `agnos-builder/tools/qdl flash boot <known-good boot.img>`, which is what
  `agnos-builder/flash_kernel.sh` does. Getting the device into QDL mode is documented at
  <https://flash.comma.ai>.
- **Keep a known-good image on the host before you flash.** This is the part that bites. Current
  AGNOS releases, and therefore <https://flash.comma.ai>, contain no comma three device tree, so
  reflashing to "factory" does **not** get a comma three booting again. The only boot images that
  work are AGNOS 12.8's and ones built like this. Download 12.8's `boot` image, or keep the
  previous working build, before you flash a new one.

## If the device shows "Unsupported firmware detected"

A black screen reading `Unsupported firmware detected` with a link to
`commaai/hardware/tree/master/comma_three` is the bootloader refusing the board. It happens before
openpilot runs, so nothing in openpilot can fix it. The cause was a manifest carrying a newer `abl`
than 12.8; see the table above.

The device is not bricked. The bootloader is running, which means `xbl` and the boot ROM are fine.

1. **Try the other slot first.** The updater flashes the inactive slot and switches to it, so the
   slot you were on before the update is still intact. Let it fail its retry count and the
   bootloader should fall back on its own. From a working shell, `abctl --set_active` back to the
   previous slot.
2. **Otherwise reflash AGNOS 12.8 over QDL.** 12.8 is the last release comma built for the comma
   three and it is still on the CDN. Get the device into QDL mode as described at
   <https://flash.comma.ai>, then flash 12.8's images with `agnos-builder/tools/qdl`. Do **not** use
   flash.comma.ai's own bundle: it serves current AGNOS, which has neither a `comma_tici` device
   tree nor a compatible `abl`, so it puts the device back into exactly this state.
3. Then install `develop-tici` again. With 12.8's `abl` pinned, the update no longer replaces the
   bootloader with one that rejects the board.

## Files

| path | what |
|---|---|
| `.github/workflows/zoompilot-agnos-tici-boot.yaml` | the build, package, publish and PR |
| `scripts/agnos/restore-comma_tici-dts.patch` | the deleted DTS and the one `Makefile` line |
| `scripts/agnos/manifest_entry.py` | computes and verifies a manifest entry for a flat image |
| `scripts/agnos/update_manifest.py` | swaps one entry into a manifest |
| `openpilot/common/hardware/comma/tici_agnos.json` | the comma three's manifest |
| `docs/comma-three-port.md` | the rest of the port |
