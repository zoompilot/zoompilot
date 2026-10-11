# zoo private S3 archive

Keep a copy of your driving data in your own S3-compatible bucket. No
server to run, no new account: the device uploads each file with a
put-only credential. Encryption is optional and off by default: on, each
file is encrypted to your PGP public key on the device and stock `gpg`
decrypts it anywhere; off, files store readable in the bucket, private
only as far as your bucket access control goes.

Validated on tizi. mici gets the same behavior through the sunnylink
settings surface; mici hardware itself is untested.

## One-time setup (on a laptop)

1. Create a bucket on any S3-compatible service (AWS S3, Cloudflare R2,
   Backblaze B2, MinIO). Create a credential that can only `PutObject`
   under your prefix. Keep separate read/list credentials for yourself:
   the device credential cannot read anything back. Consider bucket
   versioning (a leaked put-only credential can still overwrite objects)
   and a billing alert. Planning an archive ring budget? The device key
   needs list and delete too — see Archive ring below.
2. Optional, for encryption: generate a PGP key and export the public
   key. Skipping this keeps uploads unencrypted.
   ```
   gpg --quick-generate-key "zoo archive" ed25519 cert never
   gpg --quick-add-key <FPR> cv25519 encr never
   gpg --export -a <FPR> > pubkey.asc
   ```
   Back up the private key first: losing it means an unreadable archive,
   with no recovery. A second (backup) recipient key is a good idea; both
   keys encrypt every file.
3. From the repo root:
   ```
   python tools/zoo/configure.py --self-test
   ```
   It validates the credentials and the PGP key when you supply one,
   suggests an archive prefix, PUTs one probe object, and prints the
   exact params to apply on the device over SSH. With a key it also
   writes `zoo-key-check.asc` next to your public key: decrypt it once
   with `gpg -d zoo-key-check.asc`. The output names your key, proving
   the private half really opens what the device will write — do this
   especially for a backup recipient key. Plain-`http`
   endpoints get a warning: contents stay safe (they are ciphertext), but
   the signed request is a short-lived bearer credential on that wire —
   fine for a trusted LAN, not for production.
4. Apply the printed lines over SSH and restart the device. Uploads start
   from that moment (no backlog of old drives; the route/boot counters are
   the marker, not the clock).

## Everyday control

Settings → Device in the sunnylink app shows the settings on any device,
mici included: the archive on/off switch, the encryption toggle, the
upload scope (minimal = qlog + compressed camera + crash/boot; full =
everything; selected = marked routes only), and what an upload request
sends. The device settings panels also carry the upload buttons; see
Uploading a route below. String settings (endpoint, bucket, credentials,
prefix, and the key when encrypting) are one-time entry through
`configure.py` or SSH.

## Uploading a route

Two buttons under Settings → Device queue one drive for upload, no
laptop needed: **Upload Last Drive** and **Upload a Route...**, which
asks for the drive number in hex (like `1a3`). The full route ID also
works. Only drives still stored on the device can upload; a wrong or
gone number is rejected with the recent numbers shown.

A queued drive uploads qlog and the compressed camera by default; the
**Uploaded Route Contents** setting widens that to road camera + rlog,
or everything (rlog and all cameras, driver camera when available). The
request works in any scope, persists across reboots, and clears once the
route is fully uploaded. If the drive is deleted from the device before
anything of it was uploaded, the request is dropped.

During a drive, the on-road bookmark queues the drive in progress: swipe
left on mici, or tap the flag on tizi. The whole drive uploads, including
segments recorded after the swipe; uploads wait for the drive to end and
run offroad unless on-road uploads are on. The "Bookmark Saved" banner is
the only confirmation on the road.

Routes can also be marked from a laptop. Set the Upload Scope to
**Selected routes only** and the device uploads crash and boot logs plus
only the marked routes:

```
python tools/zoo/fetch.py keep 000001a3--c20ba54385
python tools/zoo/fetch.py kept
python tools/zoo/fetch.py unkeep 000001a3--c20ba54385
```

`keep` and `unkeep` write marker objects into a `keep/` folder in the
bucket, so they need a write-capable credential. The read-only
credential can list what is kept with `kept`. Making a folder named
after the route inside `keep/` in the provider's console does the same
thing. A finished upload request writes its own marker, so the device
queue and the laptop list agree.

Marked routes upload even when they were recorded before the archive
was enabled, as long as they are still on the device. The device key
needs list permission for Selected scope, the same key shape as the
ring below.

## Archive ring

Without a ring, the archive grows forever. Set `ZooS3ArchiveBudget`
(for example `8G`) and the device keeps the bucket under that size:
once per boot, and again about once per GiB uploaded, it lists the
prefix and deletes the oldest routes, whole routes only, until the
archive fits. Boot and crash files are never pruned — they are tiny,
and a crash payload cannot be regenerated. Marked routes (see Uploading
a route) are never pruned, and neither is a route with a pending
upload request, or any route younger than a day, so the ring never
fights the uploader for the same files. When
protected data alone is over the budget, the ring deletes nothing
further. Dashcam rules: newest data survives longest. Nothing still on
the device is touched — pending uploads are invisible to the ring, and
completed uploads never repeat.

The ring needs more of the credential: with a budget set, the device
key requires `listFiles` and `deleteFiles` next to `PutObject`. Keep
the budget unset and the credential can stay put-only.

A deleted object is gone unless the bucket keeps versions. The matching
protection is bucket versioning with an old-version expiry window (on
Backblaze: keep versions, expire them after 30 days). Anything the ring
deletes stays recoverable for that window, and old versions age out on
their own. Old versions also consume storage past the budget, so size
the budget with that in mind.

On Backblaze's 10 GB free tier, `8G` leaves room to grow between ring
passes, so a no-card account never crosses into paid storage.

## Reading the archive back

```
python tools/zoo/fetch.py list
python tools/zoo/fetch.py route 000001a3--c20ba54385
```

`fetch.py` lists, downloads, and decompresses; it decrypts too when the
archive is encrypted. Original file names come back. Raw `aws s3 cp` +
`gpg` works on an encrypted archive.

Full-quality video comes back as raw HEVC segments. To watch a route,
join the segments and remux to MP4:

```
cat 000001a3--c20ba54385/fcamera.hevc* > all.hevc
ffmpeg -framerate 20 -i all.hevc -c copy fcam.mp4
```

`qcamera.ts` from the Minimal scope plays directly in any player.

## What the bucket provider sees

With encryption on, file contents are end-to-end encrypted: object
names, file types, sizes, and upload timing are visible, nothing else.
A corrupted or tampered object is detected at decryption (the MDC
check), but encryption does not stop someone with write access from
replacing objects wholesale — that is what bucket versioning is for.

With encryption off, everything in the bucket is readable: to the
provider, to anyone who gets bucket access, and to a public-bucket
mistake.

## Changing things later

- Fixing a typo in the credentials or bucket: pending files flow to the
  corrected destination; nothing resets.
- Changing the bucket or prefix **after** uploads succeeded: a new epoch
  starts from that moment; old files stay where they are.
- Replacing the PGP key (encryption on): new files use the new key, old
  files stay readable with the old one. Keep old private keys as long as
  their files must stay readable.
- Disabling and re-enabling: files recorded while off upload on
  re-enable.
- Changing the budget: the next ring pass enforces the new number.
  Raising it deletes nothing by itself; lowering it prunes on the next
  pass.
