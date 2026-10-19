# zoo private S3 archive

Keep a copy of your driving data in your own S3-compatible bucket. No
server to run, no new account: the device uploads each file with a
put-only credential. Encryption is optional and off by default: on, each
file is encrypted to your PGP public key on the device and stock `gpg`
decrypts it anywhere; off, files store readable in the bucket, private
only as far as your bucket access control goes.

Validated on tizi. mici gets the same behavior through the sunnylink
settings surface; mici hardware itself is untested.

## What you need

- A laptop with Python 3 and `requests`: `pip install requests`.
  `fetch.py` also wants `zstandard` when you read `.zst` files back,
  and `gpg` when the archive is encrypted.
- An account on any S3-compatible service: AWS S3, Cloudflare R2,
  Backblaze B2, MinIO.
- SSH to the device for the one-time setup. Everyday control needs no
  SSH: it lives in the settings panels.
- Optional: a PGP key pair for encryption.

## Install on the device

The feature ships as the `zoo-s3-backend` branch of the mzdnick fork.
Install it the way you normally switch builds: the on-device installer
with the fork URL and the branch name, or over SSH with a
`git checkout zoo-s3-backend` in `/data/openpilot` and a restart.

Install first, configure second. The archive switch and the upload
buttons appear once the branch runs. They do nothing until the setup
below is applied.

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
4. Apply the printed lines over SSH, in the printed order, and restart
   the device after the last line. The enable line comes last on
   purpose: the daemon starts only after the rest is in place. The
   printed lines include the secret access key, so run this where
   terminal scrollback and shell history are acceptable, and clear both
   after.

   Without an archive budget, uploads start from that moment and
   nothing older comes along (the route/boot counters are the marker,
   not the clock). With a budget set, the uploader also walks backward:
   the newest budget-worth of routes on the device uploads as the
   starting archive.

## Turn it on and verify

After the restart, Settings → Device in the sunnylink app shows the
**zoo Private Archive** section on any device, mici included:

- **Private S3 Archive** — the on/off switch.
- **Encryption** — encrypt every upload to your PGP key. Needs the key
  from the setup above.
- **Upload Scope** — Minimal (qlog + compressed camera + crash/boot),
  Full archive, or Selected routes only.
- **Uploaded Route Contents** — what an upload request sends: Default
  (qlog + qcam), Road cam + rlog, or Everything.

Verify the first upload:

1. Drive, or replay a drive already on the device when a budget is set.
2. Offroad, with network up, wait a few minutes. Uploads run one file
   at a time; a full-quality road-camera segment is about 72 MB, so
   Full scope takes a while on a slow uplink.
3. On the laptop: `python tools/zoo/fetch.py list` shows objects under
   your prefix.
4. Over SSH, `cat /data/params/d/ZooS3LastError`: empty means no error
   was recorded since boot. Troubleshooting below maps the values.

A short demo path: Minimal scope, no ring. Drive ten minutes, go
offroad, `fetch.py list`, `fetch.py route <dir>`, and play the
`qcamera.ts` file in any player.

## Everyday control

Settings → Device in the sunnylink app shows the settings on any device,
mici included: the archive on/off switch, the encryption toggle, the
upload scope, and what an upload request sends. The device settings
panels also carry the upload buttons; see Uploading a route below.
String settings (endpoint, bucket, credentials, prefix, and the key when
encrypting) are one-time entry through `configure.py` or SSH.

## Uploading a route

Two buttons under Settings → Device queue one drive for upload, no
laptop needed: **Upload Last Drive** and **Upload a Route...**, which
asks for the drive number in hex (like `1a3`). The full route ID also
works. Only drives still stored on the device can upload. A wrong or
gone number is rejected; the rejection on tizi lists the recent drive
numbers, mici shows the plain message.

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

The budget doubles as the upload window. With it set, the uploader
considers only the newest budget-worth of routes on the device: counted
per segment in the files your scope uploads, whole routes, newest first.
The newest drive always makes it in, and the walk stops at the first
older route that would push the total over the budget, so automatic
uploads stay at or under it — the ring then trims anything older that
requested uploads added. A pending upload request or a kept route rides
past the window: anything you asked for uploads no matter its age. The
window never deletes anything — it only decides what is a candidate; the
ring is still the only deleter, and boot and crash files keep their own
rules. With no budget set, the enable moment stays the only gate.

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

## Sizing and cost

Measured means on the development car (Mazda routes, one-minute
segments): `qlog.zst` about 0.4 MB, `qcamera.ts` about 1.9 MB,
`rlog.zst` about 10 MB, `fcamera.hevc` about 72 MB per minute. Your
numbers move with scene detail and lead activity.

Rough guide per hour of driving: Minimal scope writes about 150 MB;
Road (adds rlog and the road camera) about 5 GB; Everything adds the
driver camera on top. Size the ring budget for the scope you run, and
keep headroom for versioned objects when bucket versioning is on.

During an upload the device briefly holds one extra copy of the file in
a hidden spool (`.zoo-spool`) next to the segments. Keep about 100 MB
free on the data partition.

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
  their files must stay readable. The daemon reads the key once at
  start: after replacing it, toggle the archive switch off and on, or
  restart.
- Disabling and re-enabling: files recorded while off upload on
  re-enable.
- Changing the budget: the next ring pass enforces the new number.
  Raising it deletes nothing by itself; lowering it prunes on the next
  pass.

## Troubleshooting

`cat /data/params/d/ZooS3LastError` over SSH names the last failure;
it clears on each manager start.

- Nothing uploads, error empty: was the switch turned on before the
  params were applied? Apply the printed configure.py lines and the
  daemon picks them up within about half a minute; a toggle of
  **Private S3 Archive**, or a restart, makes it immediate. Check that
  the device has network.
- `auth`: the credential was rejected. Check the secret, the key, that
  the credential may `PutObject` under your prefix, and that path-style
  matches the service (1 for MinIO/R2 and most S3-compatible services,
  0 for AWS virtual-host).
- `endpoint`: the bucket is not where the request went. Check
  `ZooS3Endpoint`, `ZooS3Region`, and `ZooS3PathStyle`.
- `clock_skew`: the device clock is off. Request signing needs correct
  time; GPS time on the road fixes it.
- `bad_key`: `ZooS3PublicKey` is unreadable. Re-apply the key block from
  configure.py and toggle the switch.
- `upload_failed` that persists: check the uplink, then the provider's
  status page. A single object over the provider's size limit fails the
  same way; the file stays pending and other routes wait behind it.

## Turning it off

Toggle **Private S3 Archive** off. The daemon stops and nothing new
uploads; the bucket is untouched. Files recorded while off upload on
re-enable.

To remove the feature: toggle off, delete the zoo params over SSH
(`rm /data/params/d/ZooS3*`), and switch back to your normal branch.
Re-installing later with a fresh `configure.py` run suggests a new
prefix, which starts a new, empty archive while the old objects stay in
the bucket and keep billing. Reuse the old prefix, or delete the old
objects with your admin credentials first.

## Params reference

All zoo params are marked do-not-log: their values never appear in
device logs or uploads.

| Param | Meaning | Default |
| --- | --- | --- |
| `ZooS3Enabled` | Master switch | 0 |
| `ZooS3Encryption` | Encrypt uploads to the PGP key | 0 |
| `ZooS3Endpoint` | Service endpoint URL | — |
| `ZooS3Region` | Region string | — |
| `ZooS3Bucket` | Bucket name | — |
| `ZooS3PathStyle` | 1 = path-style, 0 = virtual-host | 1 |
| `ZooS3AccessKeyId` | Device credential id | — |
| `ZooS3SecretAccessKey` | Device credential secret | — |
| `ZooS3PublicKey` | Armored PGP public key | — |
| `ZooS3Prefix` | Archive prefix in the bucket | `zoo` |
| `ZooS3Scope` | `minimal` / `full` / `selected` | `minimal` |
| `ZooS3UploadContents` | `default` / `road` / `everything` | `default` |
| `ZooS3UploadRoutes` | Queued upload requests (managed by the buttons) | — |
| `ZooS3ArchiveBudget` | Ring budget like `8G`; unset = ring off | — |

Internal state, managed by the daemon, safe to delete but not to edit:
`ZooS3DestinationFp`, `ZooS3EpochRouteCount`, `ZooS3EpochBootCount`,
`ZooS3UploadsDone`, `ZooS3LastUploadTime`, `ZooS3LastError`,
`ZooS3LastPruneTime`.
