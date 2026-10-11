# zoo private S3 archive

Keep an encrypted copy of your driving data in your own S3-compatible
bucket. No server to run, no new account: the device encrypts each file to
your PGP public key and uploads it with a put-only credential. Stock `gpg`
decrypts the archive anywhere.

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
2. Generate a PGP key and export the public key:
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
   It validates the key and credentials, suggests an archive prefix, PUTs
   one encrypted probe object, and prints the exact params to apply on the
   device over SSH. It also writes `zoo-key-check.asc` next to your public
   key: decrypt it once with `gpg -d zoo-key-check.asc`. The output names
   your key, proving the private half really opens what the device will
   write — do this especially for a backup recipient key. Plain-`http`
   endpoints get a warning: contents stay safe (they are ciphertext), but
   the signed request is a short-lived bearer credential on that wire —
   fine for a trusted LAN, not for production.
4. Apply the printed lines over SSH and restart the device. Uploads start
   from that moment (no backlog of old drives; the route/boot counters are
   the marker, not the clock).

## Everyday control

Settings → Device in the sunnylink app shows the toggles on any device,
mici included: the archive on/off switch and the upload scope (minimal =
qlog + compressed camera + crash/boot; full = everything; selected =
kept routes only). String settings (endpoint, bucket, credentials, key,
prefix) are one-time entry through `configure.py` or SSH.

## Selecting routes

Set the Upload Scope to **Selected routes only** and the device uploads
crash and boot logs plus only the routes you keep. Keep a route from a
laptop:

```
python tools/zoo/fetch.py keep 000001a3--c20ba54385
python tools/zoo/fetch.py kept
python tools/zoo/fetch.py unkeep 000001a3--c20ba54385
```

`keep` and `unkeep` write marker objects into a `keep/` folder in the
bucket, so they need a write-capable credential. The read-only
credential can list what is kept with `kept`. Making a folder named
after the route inside `keep/` in the provider's console does the same
thing.

A kept route uploads even when it was recorded before the archive was
enabled, as long as it is still on the device. The archive ring never
deletes a kept route. The device key needs list permission for Selected
scope, the same key shape as the ring below.

## Archive ring

Without a ring, the archive grows forever. Set `ZooS3ArchiveBudget`
(for example `8G`) and the device keeps the bucket under that size:
once per boot, and again about once per GiB uploaded, it lists the
prefix and deletes the oldest routes, whole routes only, until the
archive fits. Boot and crash files are never pruned — they are tiny,
and a crash payload cannot be regenerated. Kept routes (see Selecting
routes) are never pruned, and neither is any route younger than a day,
so the ring never fights the uploader for the same files. When
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

`fetch.py` lists, downloads, decrypts, and decompresses; original file
names come back. Raw `aws s3 cp` + `gpg` works too.

Full-quality video comes back as raw HEVC segments. To watch a route,
join the segments and remux to MP4:

```
cat 000001a3--c20ba54385/fcamera.hevc* > all.hevc
ffmpeg -framerate 20 -i all.hevc -c copy fcam.mp4
```

`qcamera.ts` from the Minimal scope plays directly in any player.

## What the bucket provider sees

File contents are end-to-end encrypted. Object names, file types, sizes,
and upload timing are visible. A corrupted or tampered object is detected
at decryption (the MDC check), but encryption does not stop someone with
write access from replacing objects wholesale — that is what bucket
versioning is for.

## Changing things later

- Fixing a typo in the credentials or bucket: pending files flow to the
  corrected destination; nothing resets.
- Changing the bucket or prefix **after** uploads succeeded: a new epoch
  starts from that moment; old files stay where they are.
- Replacing the PGP key: new files use the new key, old files stay
  readable with the old one. Keep old private keys as long as their files
  must stay readable.
- Disabling and re-enabling: files recorded while off upload on
  re-enable.
- Changing the budget: the next ring pass enforces the new number.
  Raising it deletes nothing by itself; lowering it prunes on the next
  pass.
