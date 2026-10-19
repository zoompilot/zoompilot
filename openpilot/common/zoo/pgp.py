"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Minimal OpenPGP encryptor for the zoo S3 archive.

Encrypts to user-supplied PGP public keys and produces a message that stock
gpg decrypts: OpenPGP v4 keys only (RFC 4880), PKESK v3 + SEIPD v1 with
AES-256, Curve25519 ECDH per RFC 6637 or RSA at 3072 bits or more. Definite
packet lengths everywhere. No decryption and no signature verification: the
user supplies the key, so the user is the trust root.
"""

import base64
import hashlib
import os
import struct
import time

from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding as asym_padding
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey, X25519PublicKey
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives.kdf.concatkdf import ConcatKDFHash
from cryptography.hazmat.primitives.keywrap import aes_key_wrap

AES_256 = 9
RSA_ENCRYPT_ALGOS = (1, 2)
ECDH_ALGO = 18
MIN_RSA_BITS = 3072
# OID 1.3.6.1.4.1.3029.1.5.1, the OpenPGP Curve25519 curve
CV25519_OID = bytes.fromhex('2B060104019755010501')
KEY_FLAG_ENCRYPT = 0x04 | 0x08
KDF_HASHES = {8: 'sha256', 9: 'sha384', 10: 'sha512'}
KDF_SYMS = {7: 16, 8: 24, 9: 32}
ANON_SENDER = b'Anonymous Sender    '


class PgpError(Exception):
  """Key material or message does not fit the supported subset."""


class _Key:
  def __init__(self, packet: bytes, created: int, algo: int, is_subkey: bool):
    self.packet = packet
    self.created = created
    self.algo = algo
    self.is_subkey = is_subkey
    self.flags: int | None = None
    self.expires_s: int | None = None
    self.revoked = False
    self.skip_reason: str | None = None

  def fingerprint(self) -> bytes:
    return hashlib.sha1(b'\x99' + struct.pack('>H', len(self.packet)) + self.packet).digest()

  def key_id(self) -> bytes:
    return self.fingerprint()[-8:]

  def expired(self, now: int) -> bool:
    return self.expires_s is not None and self.created + self.expires_s <= now


def _read_mpi(buf: bytes, off: int) -> tuple[bytes, int]:
  if off + 2 > len(buf):
    raise PgpError('truncated MPI')
  bits = struct.unpack('>H', buf[off : off + 2])[0]
  nbytes = (bits + 7) // 8
  if off + 2 + nbytes > len(buf):
    raise PgpError('truncated MPI body')
  return buf[off + 2 : off + 2 + nbytes], off + 2 + nbytes


def _mpi(data: bytes) -> bytes:
  data = data.lstrip(b'\x00') or b'\x00'
  bits = (len(data) - 1) * 8 + data[0].bit_length()
  return struct.pack('>H', bits) + data


def _iter_packets(data: bytes):
  off = 0
  while off < len(data):
    first = data[off]
    if first & 0x80 == 0:
      raise PgpError('invalid packet header')
    if first & 0x40:  # new format
      tag = first & 0x3F
      off += 1
      if off >= len(data):
        raise PgpError('truncated packet header')
      b1 = data[off]
      if b1 < 192:
        length, off = b1, off + 1
      elif b1 < 224:
        if off + 1 >= len(data):
          raise PgpError('truncated packet header')
        length = ((b1 - 192) << 8) + data[off + 1] + 192
        off += 2
      elif b1 == 255:
        if off + 5 > len(data):
          raise PgpError('truncated packet header')
        length = struct.unpack('>I', data[off + 1 : off + 5])[0]
        off += 5
      else:
        raise PgpError('partial packet lengths are not supported')
    else:  # old format
      tag = (first >> 2) & 0x0F
      ltype = first & 0x03
      off += 1
      if ltype == 0:
        if off >= len(data):
          raise PgpError('truncated packet header')
        length, off = data[off], off + 1
      elif ltype == 1:
        if off + 2 > len(data):
          raise PgpError('truncated packet header')
        length, off = struct.unpack('>H', data[off : off + 2])[0], off + 2
      elif ltype == 2:
        if off + 4 > len(data):
          raise PgpError('truncated packet header')
        length, off = struct.unpack('>I', data[off : off + 4])[0], off + 4
      else:
        raise PgpError('indeterminate packet length is not supported')
    if off + length > len(data):
      raise PgpError('truncated packet')
    yield tag, data[off : off + length]
    off += length


def _packet_header(tag: int, body_len: int) -> bytes:
  out = bytes([0xC0 | tag])
  length = body_len
  if length < 192:
    out += bytes([length])
  elif length < 8384:
    length -= 192
    out += bytes([192 + (length >> 8), length & 0xFF])
  else:
    out += b'\xff' + struct.pack('>I', length)
  return out


def _packet(tag: int, body: bytes) -> bytes:
  return _packet_header(tag, len(body)) + body


def _parse_public_key(packet: bytes) -> tuple[int, int, dict]:
  if len(packet) < 6 or packet[0] != 4:
    raise PgpError(f'unsupported public key version {packet[0] if packet else "none"}, v4 only')
  created = struct.unpack('>I', packet[1:5])[0]
  algo = packet[5]
  fields: dict = {}
  if algo in RSA_ENCRYPT_ALGOS or algo == 3:
    n, off = _read_mpi(packet, 6)
    e, _ = _read_mpi(packet, off)
    fields['rsa'] = rsa.RSAPublicNumbers(int.from_bytes(e, 'big'), int.from_bytes(n, 'big')).public_key()
  elif algo == ECDH_ALGO:
    fields['point'], fields['hash_id'], fields['sym_id'] = _read_ecdh_fields(packet)
  return created, algo, fields


def _read_ecdh_fields(packet: bytes) -> tuple[bytes, int, int]:
  """Return (point, kdf_hash_id, kdf_sym_id) from an ECDH v4 key packet.

  Layout (RFC 6637): oid_len || OID || MPI(point 0x40||32) || kdf field,
  where the kdf field is 03 || 01 || hash_id || sym_id.
  """
  if packet[7 : 7 + len(CV25519_OID)] != CV25519_OID:
    raise PgpError('not a Curve25519 key')
  point, off = _read_mpi(packet, 7 + packet[6])
  tail = packet[off:]
  if len(point) != 33 or point[0] != 0x40:
    raise PgpError('not a native Curve25519 point')
  if len(tail) != 4 or tail[:2] != b'\x03\x01':
    raise PgpError('unsupported KDF parameters')
  return point, tail[2], tail[3]


def _parse_signature(packet: bytes) -> tuple[int, dict]:
  """Return (signature type, selected subpackets). Signatures are not verified."""
  if len(packet) < 6 or packet[0] != 4:
    raise PgpError('unsupported signature version')
  sig_type = packet[1]
  hashed_len = struct.unpack('>H', packet[4:6])[0]
  subpackets: dict = {}
  off, end = 6, 6 + hashed_len
  while off < end:
    b1 = packet[off]
    if b1 < 192:
      slen, off = b1, off + 1
    elif b1 < 255:
      slen = ((b1 - 192) << 8) + packet[off + 1] + 192
      off += 2
    else:
      slen = struct.unpack('>I', packet[off + 1 : off + 5])[0]
      off += 5
    stype = packet[off] & 0x7F
    body = packet[off + 1 : off + slen]
    if stype == 27 and len(body) >= 1:
      subpackets['flags'] = body[0]
    elif stype == 9 and len(body) >= 4:
      subpackets['expires'] = struct.unpack('>I', body)[0]
    off += slen
  return sig_type, subpackets


def _parse_keyring(data: bytes) -> list[_Key]:
  keys: list[_Key] = []
  primary: _Key | None = None
  last_key: _Key | None = None
  for tag, packet in _iter_packets(data):
    if tag in (6, 14):
      # an unparseable key (unsupported curve, bad MPIs) is skipped with a
      # reason, never fatal to the rest of the ring
      try:
        created, algo, fields = _parse_public_key(packet)
        parse_error = None
      except PgpError as e:
        created = struct.unpack('>I', packet[1:5])[0] if len(packet) >= 5 else 0
        algo = packet[5] if len(packet) > 5 else 0
        fields, parse_error = {}, str(e)
      last_key = _Key(packet, created, algo, tag == 14)
      last_key.fields = fields
      last_key.parse_error = parse_error
      if tag == 6:
        primary = last_key
      keys.append(last_key)
    elif tag == 2 and last_key is not None:
      sig_type, sub = _parse_signature(packet)
      if sig_type == 0x20:
        if primary is not None:
          primary.revoked = True
      elif sig_type == 0x28:
        last_key.revoked = True
      elif sig_type in (0x18, 0x13, 0x10, 0x19, 0x1F):
        if last_key.flags is None and 'flags' in sub:
          last_key.flags = sub['flags']
        if last_key.expires_s is None and 'expires' in sub:
          last_key.expires_s = sub['expires']
  return keys


def _usable_candidate(key: _Key, now: int, strict_expiry: bool) -> _Key | None:
  reason = key.parse_error
  if reason is None and key.revoked:
    reason = 'revoked'
  elif reason is None and strict_expiry and key.expired(now):
    reason = 'expired'
  elif reason is None and key.flags is not None and not key.flags & KEY_FLAG_ENCRYPT:
    reason = 'key flags do not allow encryption'
  elif reason is None and key.algo not in (ECDH_ALGO,) + RSA_ENCRYPT_ALGOS:
    reason = f'algorithm {key.algo} is not supported'

  if reason is None and key.algo == ECDH_ALGO:
    if key.fields.get('hash_id') not in KDF_HASHES or key.fields.get('sym_id') not in KDF_SYMS:
      reason = 'unsupported KDF parameters'

  if reason is None and key.algo in RSA_ENCRYPT_ALGOS:
    if key.fields['rsa'].key_size < MIN_RSA_BITS:
      reason = f'RSA {key.fields["rsa"].key_size} is below {MIN_RSA_BITS} bits'

  key.skip_reason = reason
  return None if reason else key


def load_recipients(armored: str, strict_expiry: bool = True) -> list:
  """Parse armored public key blocks into one best encryption key per block.

  Unusable keys are skipped with a reason; a block with no usable key raises
  PgpError. With strict_expiry=False (runtime recheck), expired keys stay
  usable so a wrong device clock never stops uploads.
  """
  # key expiry is wall-clock by definition, not a monotonic interval
  now = int(time.time())  # noqa: TID251
  blocks = [b for b in armored.split('-----END PGP PUBLIC KEY BLOCK-----') if 'BEGIN PGP PUBLIC KEY BLOCK' in b]
  if not blocks:
    raise PgpError('no PGP public key block found')

  recipients = []
  for block in blocks:
    body = block.split('-----BEGIN PGP PUBLIC KEY BLOCK-----')[1]
    lines = [ln for ln in body.splitlines() if ln and not ln.startswith('=')]
    try:
      data = base64.b64decode(''.join(lines))
    except Exception as e:
      raise PgpError(f'invalid base64 in public key block: {e}') from e

    keys = _parse_keyring(data)
    primary_revoked = any(k.revoked and not k.is_subkey for k in keys)
    candidates = []
    skipped = []
    for key in keys:
      if key.is_subkey and primary_revoked:
        key.skip_reason = 'primary key is revoked'
        skipped.append(key)
        continue
      cand = _usable_candidate(key, now, strict_expiry)
      if cand is not None:
        candidates.append(cand)
      else:
        skipped.append(key)
    if not candidates:
      detail = '; '.join(f'{k.key_id().hex()}: {k.skip_reason}' for k in skipped) or 'no keys in block'
      raise PgpError(f'no usable encryption key in block ({detail})')
    best = max(candidates, key=lambda k: k.created)
    recipients.append({'key': best, 'fields': best.fields})
  return recipients


def _session_key_checksum(sk: bytes) -> bytes:
  return struct.pack('>H', sum(sk) & 0xFFFF)


def _ecdh_encrypt(recipient, sk: bytes, eph: X25519PrivateKey) -> tuple[bytes, bytes]:
  fields = recipient['fields']
  shared = eph.exchange(X25519PublicKey.from_public_bytes(fields['point'][1:]))
  # RFC 6637 section 8: NIST concatenation KDF over the shared point, with
  # the KDF parameter ending in "Anonymous Sender    " and the fingerprint
  param = (
    bytes([len(CV25519_OID)]) + CV25519_OID + bytes([ECDH_ALGO, 0x03, 0x01, fields['hash_id'], fields['sym_id']]) + ANON_SENDER + recipient['key'].fingerprint()
  )
  hash_alg = {'sha256': hashes.SHA256, 'sha384': hashes.SHA384, 'sha512': hashes.SHA512}[KDF_HASHES[fields['hash_id']]]()
  kek = ConcatKDFHash(algorithm=hash_alg, length=KDF_SYMS[fields['sym_id']], otherinfo=param).derive(shared)

  m = bytes([AES_256]) + sk + _session_key_checksum(sk)
  pad_len = 8 - (len(m) % 8)
  m += bytes(range(1, pad_len + 1))
  wrapped = aes_key_wrap(kek, m)

  key = recipient['key']
  eph_mpi = _mpi(b'\x40' + eph.public_key().public_bytes_raw())
  return struct.pack('>B', ECDH_ALGO) + eph_mpi + bytes([len(wrapped)]) + wrapped, key.key_id()


def _rsa_encrypt(recipient, sk: bytes) -> tuple[bytes, bytes]:
  m = bytes([AES_256]) + sk + _session_key_checksum(sk)
  ct = recipient['fields']['rsa'].encrypt(m, asym_padding.PKCS1v15())
  key = recipient['key']
  return struct.pack('>B', recipient['key'].algo) + _mpi(ct), key.key_id()


def _pkesk_packets(sk: bytes, recipients: list) -> bytes:
  out = b''
  for recipient in recipients:
    if recipient['key'].algo == ECDH_ALGO:
      body, key_id = _ecdh_encrypt(recipient, sk, X25519PrivateKey.generate())
    else:
      body, key_id = _rsa_encrypt(recipient, sk)
    out += _packet(1, b'\x03' + key_id + body)
  return out


def _literal_packet(data: bytes) -> bytes:
  # binary format, empty filename, zero timestamp
  return _packet(11, b'b\x00' + b'\x00' * 4 + data)


def encrypt(data: bytes, recipients: list) -> bytes:
  """Encrypt bytes to loaded recipients; returns a binary OpenPGP message."""
  if not recipients:
    raise PgpError('no recipients')
  sk = os.urandom(32)
  out = _pkesk_packets(sk, recipients)

  literal = _literal_packet(data)
  # SEIPD v1: version, then OpenPGP-CFB of prefix || literal || MDC
  prefix = os.urandom(16)
  prefix += prefix[14:16]
  plaintext = prefix + literal
  mdc = b'\xd3\x14' + hashlib.sha1(plaintext + b'\xd3\x14').digest()
  plaintext += mdc
  cfb = _CfbStream(sk)
  out += _packet(18, b'\x01' + cfb.update(plaintext) + cfb.finalize())
  return out


class _CfbStream:
  """Streaming AES-256 CFB-128 with a zero IV, block-buffered for chunks.

  SEIPD v1 does NOT resynchronize after the 18-byte prefix (that is the
  legacy tag-9 behavior), so encrypt() and encrypt_file() push the whole
  message, MDC included, through this one stream. Built on ECB primitives
  to stay compatible with any cryptography release.
  """

  def __init__(self, sk: bytes):
    self.ecb = Cipher(algorithms.AES(sk), modes.ECB()).encryptor()
    self.fre = bytes(16)
    self.buf = bytearray()

  def update(self, chunk: bytes) -> bytes:
    self.buf += chunk
    nblocks = len(self.buf) // 16
    out = bytearray()
    for i in range(nblocks):
      block = bytes(self.buf[i * 16 : (i + 1) * 16])
      ct = bytes(a ^ b for a, b in zip(block, self.ecb.update(self.fre), strict=False))
      out += ct
      self.fre = ct
    del self.buf[: nblocks * 16]
    return bytes(out)

  def finalize(self) -> bytes:
    if not self.buf:
      return b''
    ct = bytes(a ^ b for a, b in zip(bytes(self.buf), self.ecb.update(self.fre), strict=False))
    self.buf.clear()
    return ct


def encrypt_file(src_path: str, out_path: str, recipients: list, chunk_size: int = 1 << 20) -> None:
  """Stream-encrypt a file to a binary OpenPGP message file.

  Same message layout as encrypt(): PKESK per recipient, then SEIPD v1 with
  definite lengths computed from the source size, so an archive segment
  never sits in memory twice.
  """
  if not recipients:
    raise PgpError('no recipients')
  file_size = os.path.getsize(src_path)
  literal_head = _packet_header(11, 6 + file_size) + b'b\x00' + b'\x00' * 4
  seipd_body_len = 1 + 18 + len(literal_head) + file_size + 22

  sk = os.urandom(32)
  header = _pkesk_packets(sk, recipients) + _packet_header(18, seipd_body_len) + b'\x01'

  prefix = os.urandom(16)
  prefix += prefix[14:16]
  sha = hashlib.sha1()
  cfb = _CfbStream(sk)
  with open(src_path, 'rb') as src, open(out_path, 'wb') as dst:
    dst.write(header)
    plaintext_head = prefix + literal_head
    sha.update(plaintext_head)
    dst.write(cfb.update(plaintext_head))
    while chunk := src.read(chunk_size):
      sha.update(chunk)
      dst.write(cfb.update(chunk))
    # the MDC must flow through the same CFB stream: no mid-stream finalize,
    # a partial block cannot be resumed in CFB
    sha.update(b'\xd3\x14')
    dst.write(cfb.update(b'\xd3\x14' + sha.digest()))
    dst.write(cfb.finalize())


_CRC24_INIT = 0xB704CE
_CRC24_POLY = 0x1864CFB


def _crc24(data: bytes) -> bytes:
  crc = _CRC24_INIT
  for byte in data:
    crc ^= byte << 16
    for _ in range(8):
      crc <<= 1
      if crc & 0x1000000:
        crc ^= _CRC24_POLY
  return bytes([crc >> 16, (crc >> 8) & 0xFF, crc & 0xFF])


def armor(data: bytes) -> str:
  body = base64.b64encode(data).decode()
  lines = [body[i : i + 64] for i in range(0, len(body), 64)]
  crc = base64.b64encode(_crc24(data)).decode()
  return '-----BEGIN PGP MESSAGE-----\n\n' + '\n'.join(lines) + '\n=' + crc + '\n-----END PGP MESSAGE-----\n'
